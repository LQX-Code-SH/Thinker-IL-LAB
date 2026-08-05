# -*- coding: utf-8 -*-
import argparse
import json
import logging
import shutil
from pathlib import Path

import h5py
import numpy as np
from tqdm import tqdm

from lerobot.datasets import LeRobotDataset
from lerobot.configs.video import VideoEncoderConfig

def load_config(config_path: str) -> tuple[dict, dict]:
    """Load config JSON, extracting hdf5_mapping and returning (features, mapping).

    Feature shapes are normalized from lists to tuples so LeRobot 0.5.x
    list-vs-tuple comparisons work without monkey-patching.
    """
    with open(config_path, "r") as f:
        config = json.load(f)

    mapping = config.pop("hdf5_mapping", None)
    features = config  # remaining keys are the LeRobot feature schema

    # Normalize shape lists to tuples (avoids LeRobot 0.5.x list!=tuple bug)
    for key, spec in features.items():
        if isinstance(spec, dict) and isinstance(spec.get("shape"), list):
            spec["shape"] = tuple(spec["shape"])

    if mapping is None:
        raise ValueError(
            f"Config '{config_path}' is missing 'hdf5_mapping'. "
            "Add an hdf5_mapping section or use an updated config file."
        )

    logging.info(f"Loaded features config with {len(features)} feature keys")
    logging.info(f"Loaded HDF5 mapping for {len(mapping)} fields")
    return features, mapping


def _decode_hdf5_cell(cell) -> str:
    """Decode a single HDF5 object/bytes cell to str."""
    if isinstance(cell, np.ndarray):
        if cell.shape:
            cell = cell.reshape(-1)[0]
        else:
            cell = cell.item()
    if isinstance(cell, bytes):
        return cell.decode("utf-8")
    return str(cell)


def _read_hdf5_part(file, spec) -> np.ndarray:
    """Read a single part from HDF5 according to a mapping spec element.

    spec can be:
      - str: direct HDF5 key, read full array
      - dict: {"hdf5_key": ..., "indices": [...], "expand_dims": true, "repeat": N, "pad": [...], "invert": true,
                "extract": "position_by_name" | "field", ...}
        "indices" (optional): slice array along last axis, e.g. [7,8,9,10,11,12,13]
        "extract" (optional): how to parse JSON-list data —
          "position_by_name" + "names": [...] → extract joint positions by name
          "field" + "field": "pos"            → extract a scalar field from each JSON object
        "expand_dims" (optional): if true and data is 1D, expand to (T, 1)
        "repeat" (optional): repeat the value N times along last axis
        "pad" (optional): append constant values along last axis
        "invert" (optional): if true, apply 1 - value to flip the range
    """
    if isinstance(spec, str):
        return np.array(file[spec])

    hdf5_key = spec["hdf5_key"]
    data = np.array(file[hdf5_key])

    # --- index-based slicing (for sim data without joint names) ---
    if "indices" in spec:
        data = data[..., spec["indices"]]

    # --- JSON-list extraction (for real robot data) ---
    if "extract" in spec:
        extract_type = spec["extract"]
        raw_list = [
            json.loads(_decode_hdf5_cell(cell))
            for cell in data.reshape(-1)
        ]

        if extract_type == "position_by_name":
            names = spec["names"]
            frames = []
            for msg in raw_list:
                name_to_pos = dict(zip(msg["name"], msg["position"]))
                frames.append([name_to_pos[n] for n in names])
            data = np.asarray(frames, dtype=np.float32)

        elif extract_type == "field":
            field = spec["field"]
            data = np.asarray(
                [float(msg[field]) for msg in raw_list], dtype=np.float32
            )

        else:
            raise ValueError(
                f"Unknown extract type '{extract_type}'. "
                f"Supported: position_by_name, field"
            )

    # --- transforms (applied after extraction, if any) ---
    if spec.get("expand_dims", False) and data.ndim == 1:
        data = data[:, None]

    if "invert" in spec and spec["invert"]:
        data = 1.0 - data

    if "repeat" in spec:
        n = spec["repeat"]
        data = np.repeat(data, n, axis=-1)

    if "pad" in spec:
        pad_vals = np.array(spec["pad"], dtype=data.dtype)
        pad_shape = list(data.shape)
        pad_shape[-1] = len(pad_vals)
        pad_arr = np.broadcast_to(pad_vals, pad_shape)
        data = np.concatenate([data, pad_arr], axis=-1)

    return data


def _read_mp4_via_value_list(
    raw_value_list: h5py.Dataset,
    hdf5_path: Path,
    image_size: tuple[int, int],
) -> list[np.ndarray]:
    """Read MP4 video frames from a sidecar file referenced by HDF5 value_list.

    The value_list column contains relative paths (e.g. ``camera_data/.../xxx_aligned.mp4``).
    All rows typically reference the same MP4; the first non-empty entry is used.
    """
    values = [
        _decode_hdf5_cell(cell)
        for cell in np.asarray(raw_value_list[()]).reshape(-1)
    ]
    candidates = [v for v in values if v]
    if not candidates:
        raise ValueError("Camera value_list is empty — no MP4 path found.")
    rel_path = candidates[0]
    mp4_path = (hdf5_path.parent / rel_path).resolve()
    if not mp4_path.is_file():
        raise FileNotFoundError(
            f"Camera MP4 not found: {mp4_path}\n"
            f"  value_list relative path: {rel_path}\n"
            f"  Check the camera_data directory alongside the HDF5."
        )

    import imageio.v3 as iio
    from PIL import Image

    width, height = int(image_size[0]), int(image_size[1])
    frames = []
    for frame_rgb in iio.imiter(mp4_path):
        if frame_rgb.ndim == 2:
            frame_rgb = np.stack([frame_rgb, frame_rgb, frame_rgb], axis=-1)
        elif frame_rgb.shape[-1] == 4:
            frame_rgb = frame_rgb[..., :3]
        img = Image.fromarray(frame_rgb.astype(np.uint8))
        frames.append(
            np.asarray(img.resize((width, height), getattr(Image, "Resampling", Image).BILINEAR))
        )
    return frames


def validate_mapping(mapping: dict, features: dict) -> None:
    """Validate hdf5_mapping against the feature schema."""
    for lerobot_key, field_spec in mapping.items():
        if lerobot_key not in features:
            raise ValueError(
                f"hdf5_mapping key '{lerobot_key}' not found in feature schema"
            )
        if isinstance(field_spec, list):
            if "shape" not in features[lerobot_key]:
                raise ValueError(
                    f"Feature '{lerobot_key}' missing 'shape' in feature schema"
                )
            for item in field_spec:
                if isinstance(item, dict):
                    if "hdf5_key" not in item:
                        raise ValueError(
                            f"Dict entry in '{lerobot_key}' missing 'hdf5_key'"
                        )
        elif isinstance(field_spec, dict):
            required = {"hdf5_key", "encoding", "image_size"}
            missing = required - set(field_spec.keys())
            if missing:
                raise ValueError(
                    f"Image mapping for '{lerobot_key}' missing keys: {missing}"
                )
        else:
            raise TypeError(
                f"hdf5_mapping['{lerobot_key}'] must be a list or dict, "
                f"got {type(field_spec).__name__}"
            )


def validate_features(features: dict) -> None:
    """Warn about feature-schema issues lerobot silently ignores or mis-handles.

    - lerobot 0.5.x ignores a ``video_info`` key on video features; only the
      ``info`` field (auto-filled after encoding) is used. Flag it so configs
      get cleaned up.
    - lerobot expects image/video feature shape as CHW ``[C, H, W]``. A common
      mistake is HWC ``[H, W, C]``; flag it so stored metadata is correct.
    """
    for name, spec in features.items():
        if not isinstance(spec, dict):
            continue
        dtype = spec.get("dtype")
        if dtype not in ("video", "image"):
            continue
        if "video_info" in spec:
            logging.warning(
                f"Feature '{name}' has a 'video_info' field which lerobot "
                "ignores (it auto-fills 'info' after encoding). Remove "
                "'video_info' from the config."
            )
        shape = spec.get("shape")
        if isinstance(shape, list) and len(shape) == 3 and shape[0] != 3 and shape[-1] == 3:
            logging.warning(
                f"Feature '{name}' shape {shape} looks HWC [H,W,C]; lerobot "
                "expects CHW [C,H,W] (e.g. [3,360,640]). Stored metadata would "
                "be wrong otherwise."
            )


def _read_timestamps(ds: h5py.Dataset):
    """Read HDF5 timestamps, auto-detect ns vs seconds, return float64 relative seconds.

    Returns None on empty dataset, (N,) float64 relative seconds otherwise
    (first frame always 0.0).
    """
    arr = np.array(ds).ravel().astype(np.float64)
    if len(arr) == 0:
        return None
    if arr.max() > 1e15:       # uint64 nanoseconds → seconds
        arr /= 1e9
    arr -= arr[0]               # zero-base to first frame
    return arr


def _detect_source_timestamps(
    file: h5py.File,
    mapping: dict,
    user_key: str | None = None,
):
    """Auto-detect source timestamps from HDF5 with three-level fallback.

    1. *user_key* (if given)
    2. Known simulation keys (``observation/timestamp/data``, ``observations/timestamp``)
    3. Sibling ``timestamp_list`` in the same HDF5 group as any data key in *mapping*

    Returns (N,) float64 array of relative seconds, or None if no timestamps found.
    """
    # Level 1: user-specified key
    if user_key and user_key in file:
        ts = _read_timestamps(file[user_key])
        if ts is not None:
            return ts

    # Level 2: simulation-format keys
    for key in ("observation/timestamp/data", "observations/timestamp"):
        if key in file:
            ts = _read_timestamps(file[key])
            if ts is not None:
                return ts

    # Level 3: real-robot format — timestamp_list sibling
    for field_spec in mapping.values():
        items = field_spec if isinstance(field_spec, list) else [field_spec]
        for item in items:
            hdf5_key = item if isinstance(item, str) else item.get("hdf5_key")
            if not hdf5_key:
                continue
            ts_key = str(Path(hdf5_key).parent / "timestamp_list")
            if ts_key in file:
                ts = _read_timestamps(file[ts_key])
                if ts is not None:
                    return ts

    return None


def _resample_frames(compose_fields, image_fields, src_ts, target_fps):
    """Resample compose and image fields to *target_fps* using *src_ts* (relative seconds).

    compose_fields:  {key: (parts_list, dtype)}  — each part shape (N, D) or (N,)
    image_fields:    {key: ndarray (N, H, W, C)}

    Returns ``(new_compose, new_images, M)`` where *M* is the target frame count.
    State/action vectors use per-dimension linear interpolation (``np.interp``);
    images use nearest-neighbour frame selection.
    """
    duration = src_ts[-1]                     # relative seconds, src_ts[0]==0.0
    M = max(2, round(duration * target_fps) + 1)
    target_ts = np.linspace(0.0, duration, M)

    # --- state / action vectors → per-dimension linear interpolation ---
    new_compose = {}
    for key, (parts, dtype) in compose_fields.items():
        new_parts = []
        for part in parts:                    # (N, D)  or  (N,)
            is_1d = (part.ndim == 1)
            src = part if not is_1d else part[:, None]      # → (N, D)
            out = np.empty((M, src.shape[1]), dtype=np.float64)
            for d in range(src.shape[1]):
                out[:, d] = np.interp(target_ts, src_ts,
                                      src[:, d].astype(np.float64))
            new_parts.append(out.astype(dtype) if not is_1d else out[:, 0].astype(dtype))
        new_compose[key] = (new_parts, dtype)  # preserve structure

    # --- images → nearest-neighbour selection ---
    new_images = {}
    indices = np.abs(src_ts[:, None] - target_ts[None, :]).argmin(axis=0)
    for key, arr in image_fields.items():     # (N, H, W, C)
        new_images[key] = arr[indices]        # → (M, H, W, C)

    return new_compose, new_images, M


def initialize_dataset(
    repo_id: str, tgt_path: str, fps: int, robot_type: str, features: dict,
    vcodec: str = "h264",
    image_writer_processes: int = 4, image_writer_threads: int = 4,
) -> LeRobotDataset:
    """Initialize dataset instance, removing existing data if present."""
    dataset_path = Path(tgt_path) / repo_id

    if dataset_path.exists():
        shutil.rmtree(dataset_path)
        logging.warning(f"Removed existing dataset: {dataset_path}")

    # Pick_up_tiangong_all uses mp4v; lerobot's vcodec whitelist rejects mpeg4,
    # so h264 is the closest compatible default. Override via --vcodec.
    camera_encoder = VideoEncoderConfig(vcodec=vcodec, pix_fmt="yuv420p")
    logging.info(f"Creating new dataset: {dataset_path} (vcodec={vcodec})")
    return LeRobotDataset.create(
        repo_id=repo_id,
        root=str(dataset_path),
        fps=fps,
        robot_type=robot_type,
        features=features,
        camera_encoder=camera_encoder,
        image_writer_processes=image_writer_processes,
        image_writer_threads=image_writer_threads,
    )


def _load_label_segments(ep_dir_name: str, label_root: Path) -> list[tuple[int, int]] | None:
    """Load segment ranges from label.json if it exists.

    Returns a list of (start_frame, end_frame) tuples (end_frame is exclusive),
    or None if no label file exists.
    """
    label_path = label_root / ep_dir_name / "label.json"
    if not label_path.is_file():
        logging.warning("No label.json found at %s — treating entire episode as one segment", label_path)
        return None

    with open(label_path, "r") as f:
        label_data = json.load(f)

    if label_data.get("result") != "pass":
        logging.warning("Label result is '%s' (not 'pass') for %s — skipping segments",
                        label_data.get("result"), ep_dir_name)
        return []

    datas = label_data.get("datas", [])
    if not datas:
        logging.warning("Label has no 'datas' entries for %s — treating entire episode as one segment", ep_dir_name)
        return None

    ranges = [(int(d["start_frame"]), int(d["end_frame"])) for d in datas]
    # Sort by start_frame for safety; label.json is typically already sorted
    ranges.sort(key=lambda r: r[0])
    return ranges


def process_episode(
    episode_path: Path,
    dataset: LeRobotDataset,
    task_name: str,
    mapping: dict,
    features: dict,
    resample_fps: float | None = None,
    timestamp_hdf5_key: str | None = None,
    source_fps: float = 30,
    segment_ranges: list[tuple[int, int]] | None = None,
) -> int:
    """Process single episode HDF5 into one or more LeRobot dataset episodes.

    If *segment_ranges* is provided, each (start, end) pair produces a separate
    episode (end is exclusive).  Otherwise the whole recording is saved as one
    episode.

    Returns the number of episodes saved (0 on failure).
    """
    try:
        with h5py.File(episode_path, "r") as file:
            compose_fields = {}   # lerobot_key -> (parts_list, dtype)
            image_fields = {}     # lerobot_key -> numpy array (T, H, W, C)

            for lerobot_key, field_spec in mapping.items():
                if isinstance(field_spec, list):
                    # Numeric compose field: read and concatenate per-list-order
                    parts = [_read_hdf5_part(file, item) for item in field_spec]
                    dtype = np.dtype(features[lerobot_key]["dtype"])
                    compose_fields[lerobot_key] = (parts, dtype)
                elif isinstance(field_spec, dict):
                    # Image field
                    hdf5_key = field_spec["hdf5_key"]
                    encoding = field_spec["encoding"]
                    image_size = tuple(field_spec["image_size"])  # (W, H)
                    raw = file[hdf5_key]

                    if encoding == "jpeg":
                        import cv2
                        images = []
                        for buf in raw:
                            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
                            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            images.append(cv2.resize(img, image_size))
                    elif encoding == "png_depth":
                        import cv2
                        depth_clip_mm = field_spec.get("depth_clip_mm", 8000)
                        images = []
                        for buf in raw:
                            d16 = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_UNCHANGED)
                            d16 = cv2.resize(d16, image_size, interpolation=cv2.INTER_NEAREST)
                            d8 = np.clip(d16, 0, depth_clip_mm).astype(np.float32)
                            d8 = (d8 * (255.0 / depth_clip_mm)).astype(np.uint8)
                            images.append(np.stack([d8, d8, d8], axis=-1))
                    elif encoding == "raw":
                        images = [
                            cv2.resize(img, image_size)
                            for img in raw
                        ]
                    elif encoding == "mp4_value_list":
                        # Real robot: value_list → MP4 path → video frames
                        images = _read_mp4_via_value_list(
                            raw, episode_path, image_size
                        )
                    else:
                        raise ValueError(
                            f"Unknown encoding '{encoding}' for '{lerobot_key}'"
                        )

                    image_fields[lerobot_key] = np.stack(images)

            # --- peek timestamps while file is still open ---
            _src_ts = None
            if resample_fps is not None:
                _src_ts = _detect_source_timestamps(
                    file, mapping, timestamp_hdf5_key,
                )

        num_frames = None
        for parts, _ in compose_fields.values():
            for p in parts:
                if num_frames is None:
                    num_frames = len(p)
                elif len(p) != num_frames:
                    logging.error(
                        f"Frame count mismatch in {episode_path}: "
                        f"expected {num_frames}, got {len(p)}"
                    )
                    return 0

        for arr in image_fields.values():
            if num_frames is None:
                num_frames = len(arr)
            elif len(arr) != num_frames:
                logging.error(
                    f"Frame count mismatch in {episode_path}: "
                    f"expected {num_frames}, got {len(arr)} for image"
                )
                return 0

        # --- frame-rate resampling (optional) ---
        if resample_fps is not None:
            if num_frames < 2:
                logging.info(
                    f"Skipping resampling for {episode_path.name}: "
                    f"only {num_frames} frame(s), need at least 2"
                )
            else:
                src_ts = _src_ts

                if src_ts is not None:
                    if len(src_ts) != num_frames:
                        logging.warning(
                            f"Timestamp count ({len(src_ts)}) != frame count "
                            f"({num_frames}) in {episode_path.name}. "
                            f"Falling back to uniform --fps={source_fps} spacing."
                        )
                        src_ts = None

                if src_ts is None:
                    src_ts = np.arange(num_frames, dtype=np.float64) / source_fps
                    logging.info(
                        f"Using --fps={source_fps} uniform spacing for "
                        f"{episode_path.name}"
                    )

                compose_fields, image_fields, num_frames = _resample_frames(
                    compose_fields, image_fields, src_ts, resample_fps)

    except (FileNotFoundError, OSError, KeyError) as e:
        logging.error(f"Skipped {episode_path}: {e}")
        return 0

    # --- determine segment ranges ---
    if segment_ranges is None:
        ranges = [(0, num_frames)]
    else:
        # Validate and clamp ranges
        ranges = []
        for start, end in segment_ranges:
            if start < 0 or end > num_frames or start >= end:
                logging.warning(
                    "Invalid segment range [%d, %d) for %d-frame episode %s — skipped",
                    start, end, num_frames, episode_path.name,
                )
                continue
            ranges.append((start, end))
        if not ranges:
            logging.warning("No valid segments for %s — skipping", episode_path.name)
            return 0

    # --- emit frames per segment ---
    saved_count = 0
    try:
        for seg_idx, (start, end) in enumerate(ranges):
            seg_label = f"{episode_path.name}" if len(ranges) == 1 else f"{episode_path.name}/seg{seg_idx:02d}"
            for i in tqdm(range(start, end), desc=f"Processing {seg_label}"):
                frame = {}
                for lerobot_key, (parts, dtype) in compose_fields.items():
                    frame[lerobot_key] = np.concatenate(
                        [p[i] for p in parts]
                    ).astype(dtype)
                for lerobot_key, arr in image_fields.items():
                    frame[lerobot_key] = arr[i]
                frame["task"] = task_name
                dataset.add_frame(frame)
            dataset.save_episode()
            saved_count += 1
            logging.info("Saved segment %d/%d: frames [%d, %d) (%d frames)",
                         seg_idx + 1, len(ranges), start, end, end - start)
    except Exception as e:
        logging.error(f"Skipped {episode_path} during frame processing: {e}")
        dataset.clear_episode_buffer()
        return saved_count

    return saved_count


def str2bool(v: str) -> bool:
    """Parse boolean from string, replacing deprecated distutils.util.strtobool."""
    if v.lower() in ("yes", "true", "t", "1"):
        return True
    elif v.lower() in ("no", "false", "f", "0"):
        return False
    raise argparse.ArgumentTypeError(f"Boolean value expected, got '{v}'")


def main():
    parser = argparse.ArgumentParser(description="HDF5 -> LeRobot Dataset Conversion Tool")
    parser.add_argument("--config", type=str, required=True, help="Path to config JSON file")
    parser.add_argument("--repo_id", type=str, required=True, help="Dataset repository ID")
    parser.add_argument("--src_root", type=str, required=True, help="Source data directory")
    parser.add_argument("--tgt_path", type=str, required=True, help="Target output directory")
    parser.add_argument("--task_name", type=str, default="default_task", help="Task name identifier")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument("--robot_type", type=str, default="tiangong", help="Robot type identifier")
    parser.add_argument("--save_one", type=str2bool, default=False, help="Save only one episode for testing")
    parser.add_argument("--hdf5_rel_path", type=str, default="trajectory.hdf5", help="Relative path to HDF5 file within each episode dir")
    parser.add_argument("--image_writer_processes", type=int, default=4, help="Number of image writer processes")
    parser.add_argument("--image_writer_threads", type=int, default=4, help="Number of image writer threads")
    parser.add_argument("--overwrite", action="store_true",
                        help="Overwrite existing output dataset (default: always overwrite)")
    parser.add_argument("--vcodec", type=str, default="h264",
                        help="Video codec: h264 (default, closest to Pick_up's mp4v), libsvtav1 (av1), hevc, auto. "
                             "Note: mpeg4/mp4v is rejected by lerobot's codec whitelist.")
    parser.add_argument("--stream-video", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--resample-fps", type=float, default=None,
                        help="Target FPS for resampling. When set, source frames are "
                             "interpolated to this frame rate. State/action use linear "
                             "interpolation; images use nearest-neighbour selection.")
    parser.add_argument("--timestamp-hdf5-key", type=str, default=None,
                        help="Explicit HDF5 key for per-frame timestamps. "
                             "Auto-detected from known keys if omitted.")
    parser.add_argument("--label-root", type=str, default=None,
                        help="Root directory containing per-episode label.json files. "
                             "Labels are looked up as <label-root>/<episode_dir_name>/label.json. "
                             "When provided, each labeled segment becomes a separate episode; "
                             "unlabeled frames between segments are dropped.")
    args = parser.parse_args()

    # Load configuration
    features, mapping = load_config(args.config)
    validate_mapping(mapping, features)
    validate_features(features)

    # Initialize dataset
    source_fps = args.fps                              # fallback when HDF5 lacks timestamps
    dataset_fps = round(args.resample_fps) if args.resample_fps else args.fps
    dataset = initialize_dataset(
        repo_id=args.repo_id,
        tgt_path=args.tgt_path,
        fps=dataset_fps,
        robot_type=args.robot_type,
        features=features,
        vcodec=args.vcodec,
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
    )

    # Discover episodes: supports both flat and nested layouts.
    #   flat:   <top_dir>/trajectory.hdf5
    #   nested: <top_dir>/<episode_name>/trajectory.hdf5
    # Skips sibling LeRobot datasets / output dirs / empty dirs that happen to
    # live under src_root (avoids noisy "Skipped" errors).
    src_root = Path(args.src_root)
    top_dirs = sorted([d for d in src_root.iterdir() if d.is_dir()])

    episode_pairs = []  # list of (ep_dir, hdf5_path)
    skipped = []
    for top_dir in top_dirs:
        flat_hdf5 = top_dir / args.hdf5_rel_path
        if flat_hdf5.exists():
            episode_pairs.append((top_dir, flat_hdf5))
            continue

        # Check one level deeper for nested layout
        sub_dirs = sorted([d for d in top_dir.iterdir() if d.is_dir()])
        found = False
        for sub_dir in sub_dirs:
            hdf5_path = sub_dir / args.hdf5_rel_path
            if hdf5_path.exists():
                episode_pairs.append((sub_dir, hdf5_path))
                found = True
        if not found:
            skipped.append(top_dir.name)

    if skipped:
        logging.info(f"Skipping {len(skipped)} dir(s) without '{args.hdf5_rel_path}': {skipped}")

    # --- load per-episode labels if --label-root is provided ---
    label_root = Path(args.label_root) if args.label_root else None

    success_count = 0
    total_segments = 0
    logging.info(f"Found {len(episode_pairs)} episodes to process...")
    for ep_dir, ep_path in episode_pairs:
        # Resolve segment ranges from label.json (if available)
        segment_ranges = None  # None = treat whole episode as one segment
        if label_root is not None:
            segment_ranges = _load_label_segments(ep_dir.name, label_root)
            if segment_ranges == []:
                logging.warning("Skipping %s: label result is not 'pass'", ep_dir.name)
                continue
            if segment_ranges is not None:
                logging.info("Label %s: %d segment(s)", ep_dir.name, len(segment_ranges))

        n_saved = process_episode(
            ep_path, dataset, args.task_name, mapping, features,
            resample_fps=args.resample_fps,
            timestamp_hdf5_key=args.timestamp_hdf5_key,
            source_fps=source_fps,
            segment_ranges=segment_ranges,
        )
        if n_saved > 0:
            success_count += 1
            total_segments += n_saved
            logging.info(f"Saved episode: {ep_dir.name} → {n_saved} segment(s) ({success_count}/{len(episode_pairs)} source episodes, {total_segments} total segments)")
        else:
            logging.warning(f"No segments saved for {ep_dir.name}")

        if args.save_one:
            break

    dataset.finalize()
    if label_root is not None:
        logging.info(f"Conversion complete: {success_count}/{len(episode_pairs)} source episodes → {total_segments} labeled segments.")
    else:
        logging.info(f"Conversion complete: {success_count}/{len(episode_pairs)} episodes saved.")


if __name__ == "__main__":
    # force=True: lerobot's import pre-configures the root logger, which makes a
    # plain basicConfig() a no-op and silently drops INFO-level progress logs.
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", force=True)
    main()
