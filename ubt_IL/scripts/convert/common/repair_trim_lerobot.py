#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Repair & trim an existing LeRobot v3 dataset by config — no HDF5 source required.

与 `convert_to_lerobot.py` 的分工：
- 转换器：HDF5 -> LeRobot，config 里写 `hdf5_mapping`（从 HDF5 哪一段取数）；
- 本脚本：LeRobot -> LeRobot，config 里写 `lerobot_mapping`（从源数据集哪几维/哪几路取数），
  并额外支持静止段裁剪、缺失维回填、视频重编码。

config 结构（与转换配置同风格）：
{
  "<输出特征名>": {"dtype": ..., "shape": [...], "names": [...]},   # 输出 LeRobot 特征表
  "lerobot_mapping": {
      "<输出特征名>": {
          "source": "<源数据集特征名>",          # 缺省同名
          "indices": [7, 8, ...] | "all",       # 数值列：按序号取
          "names": ["...", ...],                # 或按维度名取（与 indices 二选一）
          "image_size": [W, H]                  # 视觉特征：可选缩放
      }
  },
  "repair": {
      "trim": "lead_tail" | "cap" | "none",
      "stationary_window": 0,        # 0 = 0.3*fps
      "stationary_thresh": 0.03,
      "range_eps": 1e-3,
      "keep_head": 0, "keep_tail": 0,
      "min_frames": 45,
      "cap": 8, "min_run": 3,
      "fill_state_from_action": "auto" | "none",
      "fill_eps": 1e-6,
      "task": null
  }
}

示例：
    python ubt_IL/scripts/convert/common/repair_trim_lerobot.py \
        --config ubt_IL/scripts/convert/tienkung_dex/configs/tienkung_dex_real_pick_r13_1RGB.json \
        --repo_id tienkung_dex_real_pick \
        --root ubt_IL/dataset/tienkung_dex_real_pick \
        --new_repo_id tienkung_dex_real_pick_r13 \
        --new_root ubt_IL/dataset/tienkung_dex_real_pick_r13
"""

from __future__ import annotations

import argparse
import json
import logging
import shutil
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
from tqdm import tqdm

from lerobot.datasets import LeRobotDataset

try:  # 新版 lerobot：编码参数走 VideoEncoderConfig
    from lerobot.configs.video import VideoEncoderConfig
except ImportError:  # 0.5.x：create(vcodec=...)
    VideoEncoderConfig = None

try:  # 新版 lerobot
    from lerobot.utils.constants import DEFAULT_FEATURES
except ImportError:  # 0.5.x
    from lerobot.datasets.utils import DEFAULT_FEATURES

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("repair_trim_lerobot")

STATE_KEY = "observation.state"
ACTION_KEY = "action"
VISUAL_DTYPES = ("video", "image")


# ─────────────────────────── 配置解析 ────────────────────────────
def load_repair_config(path: Path) -> tuple[dict, dict, dict]:
    """返回 (features, mapping, repair)。features 里去掉 mapping/repair 两个辅助段。"""
    with open(path, "r") as f:
        cfg = json.load(f)
    mapping = cfg.pop("lerobot_mapping", None)
    repair = cfg.pop("repair", None) or {}
    if mapping is None:
        raise ValueError(f"配置 {path} 缺少 lerobot_mapping 段")
    features = cfg

    for key, spec in features.items():
        if "shape" in spec:
            spec["shape"] = tuple(spec["shape"])
    for key in mapping:
        if key not in features:
            raise ValueError(f"lerobot_mapping 中的 '{key}' 不在特征表中")
    return features, mapping, repair


def parse_dim_indices(spec, names: list[str] | None, dim: int) -> list[int]:
    """把 indices 段解析成序号列表：'all' / [7,8,20] / '7-13,20-25' / ['维度名', ...]。"""
    if spec is None or (isinstance(spec, str) and spec.strip().lower() == "all"):
        return list(range(dim))

    out: list[int] = []
    if isinstance(spec, str):
        tokens = [t.strip() for t in spec.split(",") if t.strip()]
    elif isinstance(spec, (list, tuple)):
        tokens = list(spec)
    else:
        raise ValueError(f"无法解析 indices: {spec!r}")

    for token in tokens:
        if isinstance(token, int):
            out.append(token)
            continue
        token = str(token).strip()
        if "-" in token and not token.lstrip("-").isdigit():
            lo, hi = token.split("-", 1)
            out.extend(range(int(lo), int(hi) + 1))
        elif token.lstrip("-").isdigit():
            out.append(int(token))
        elif names and token in names:
            out.append(names.index(token))
        else:
            raise ValueError(f"无法解析维度 '{token}'：既不是序号也不是源维度名")

    out = sorted(set(out))
    if not out or min(out) < 0 or max(out) >= dim:
        raise ValueError(f"维度选择越界：需在 [0, {dim}) 内，实际 {out}")
    return out


def resolve_mapping(cfg_features: dict, mapping: dict, src_features: dict) -> dict:
    """校验并补齐 mapping，返回 {输出特征: {"source","indices","image_size"}}。"""
    resolved: dict = {}
    for out_key, spec in cfg_features.items():
        m = dict(mapping.get(out_key) or {})
        src_key = m.get("source", out_key)
        if src_key not in src_features:
            raise ValueError(f"输出特征 '{out_key}' 的源特征 '{src_key}' 在数据集中不存在")
        src_spec = src_features[src_key]

        if spec["dtype"] in VISUAL_DTYPES:
            if src_spec["dtype"] not in VISUAL_DTYPES:
                raise ValueError(f"'{out_key}' 需要视觉特征，但源 '{src_key}' 是 {src_spec['dtype']}")
            image_size = m.get("image_size")
            if image_size is not None:
                image_size = (int(image_size[0]), int(image_size[1]))
                _, h, w = spec["shape"]
                if (h, w) != (image_size[1], image_size[0]):
                    raise ValueError(
                        f"'{out_key}' 的 image_size={image_size} 与特征 shape={spec['shape']} 不一致"
                    )
            resolved[out_key] = {"source": src_key, "image_size": image_size}
            continue

        dim = int(src_spec["shape"][0])
        names = src_spec.get("names")
        if "names" in m and m["names"] is not None:
            indices = parse_dim_indices(m["names"], names, dim)
        else:
            indices = parse_dim_indices(m.get("indices", "all"), names, dim)
        expect = int(spec["shape"][0])
        if len(indices) != expect:
            raise ValueError(f"'{out_key}' 选了 {len(indices)} 维，但特征 shape 要求 {expect} 维")
        if spec.get("names"):
            src_names = [names[i] if names else f"dim{i}" for i in indices]
            if list(spec["names"]) != src_names:
                logger.warning("'%s' 的 names 与所选源维度名不一致，按配置顺序写入：%s", out_key, spec["names"])
        resolved[out_key] = {"source": src_key, "indices": indices, "image_size": None}
    return resolved


# ─────────────────────────── 静止判据 ────────────────────────────
def normalized_move(action: np.ndarray, window: int, range_eps: float = 1e-3):
    """与 common/convert_to_lerobot.py 同语义：前向窗口归一化最大位移。"""
    a = np.asarray(action, dtype=np.float64)
    if a.ndim == 1:
        a = a[:, None]
    T, D = a.shape
    if T < 2:
        return np.zeros(T), np.zeros(D, dtype=bool)
    rng = a.max(axis=0) - a.min(axis=0)
    active = rng > range_eps
    denom = np.where(active, np.maximum(rng, range_eps), 1.0)
    j = np.minimum(np.arange(T) + window, T - 1)
    disp = np.abs(a[j] - a) / denom[None, :]
    disp[:, ~active] = 0.0
    return disp.max(axis=1), active


def stationary_mask(action: np.ndarray, window: int, thr: float, range_eps: float):
    move, active = normalized_move(action, window, range_eps)
    stat = move < thr
    if len(stat) >= 2:
        stat[-1] = stat[-2]  # 末帧没有前向窗口，继承前一帧
    return stat, active


def plan_lead_tail(action: np.ndarray, opt: dict) -> tuple[int, int, dict]:
    T = len(action)
    stat, active = stationary_mask(action, opt["stationary_window"], opt["stationary_thresh"], opt["range_eps"])
    lead = 0
    while lead < T and stat[lead]:
        lead += 1
    tail = 0
    while tail < T and stat[T - 1 - tail]:
        tail += 1

    start = max(0, lead - opt["keep_head"])
    end = min(T, T - tail + opt["keep_tail"])
    if end - start < opt["min_frames"]:
        logger.warning("episode 裁后仅 %d 帧 (< %d)，保留整集", max(0, end - start), opt["min_frames"])
        start, end = 0, T
    return start, end, {"lead": lead, "tail": tail, "active_dims": int(active.sum()), "length": T}


def plan_cap_internal(action: np.ndarray, opt: dict) -> tuple[np.ndarray, int]:
    T = len(action)
    stat, _ = stationary_mask(action, opt["stationary_window"], opt["stationary_thresh"], opt["range_eps"])
    keep = np.ones(T, dtype=bool)
    k_head = opt["cap"] // 2
    k_tail = opt["cap"] - k_head
    dropped = 0
    i = 0
    while i < T:
        if not stat[i]:
            i += 1
            continue
        j0 = i
        while i < T and stat[i]:
            i += 1
        run = i - j0
        if run >= opt["min_run"] and run > opt["cap"]:
            keep[j0 + k_head : i - k_tail] = False
            dropped += run - opt["cap"]
    return keep, dropped


# ─────────────────────────── 数据读取 ────────────────────────────
def load_numeric(root: Path, meta, episodes: list[int], keys: list[str]) -> dict[int, dict[str, np.ndarray]]:
    """按 episode 读取数值特征（同一 parquet 文件只读一次）。"""
    cache: dict[Path, object] = {}
    out: dict[int, dict[str, np.ndarray]] = {}
    for ep in episodes:
        path = root / meta.get_data_file_path(ep)
        if path not in cache:
            table = pq.read_table(path, columns=["episode_index", "frame_index", *keys])
            cache[path] = table.to_pandas()
        df = cache[path]
        sub = df[df["episode_index"] == ep].sort_values("frame_index")
        length = int(meta.episodes[ep]["length"])
        if len(sub) != length:
            raise RuntimeError(f"episode {ep}: parquet 帧数 {len(sub)} 与 meta.length {length} 不一致")
        out[ep] = {k: np.stack(sub[k].values).astype(np.float32) for k in keys}
    return out


def episode_task(meta, ep: int, fallback: str | None) -> str:
    ep_meta = meta.episodes[ep]
    tasks = ep_meta.get("tasks")
    if tasks is None:
        return fallback or "default_task"
    if isinstance(tasks, str):
        return tasks
    tasks = list(tasks)
    return str(tasks[0]) if tasks else (fallback or "default_task")


def iter_video_frames(path: Path, start_s: float, count: int, fps: float):
    """从 mp4 顺序解码 count 帧，起点为 start_s（同一文件内多 episode 拼接时用）。"""
    import av

    tol = 0.5 / fps
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        if start_s > 0:
            # 指定 stream 时 PyAV 的 offset 单位是流的 time_base，不是微秒
            tb = float(stream.time_base)
            container.seek(int(round(start_s / tb)), stream=stream, backward=True)
        yielded = 0
        for frame in container.decode(stream):
            t = frame.time
            if t is None or t < start_s - tol:
                continue
            yield frame.to_ndarray(format="rgb24")
            yielded += 1
            if yielded >= count:
                break
        if yielded < count:
            raise RuntimeError(f"{path.name}: 期望 {count} 帧，只解出 {yielded} 帧（起点 {start_s:.3f}s）")


def resize_frame(frame: np.ndarray, image_size, is_depth: bool) -> np.ndarray:
    import cv2

    w, h = image_size
    if frame.shape[0] == h and frame.shape[1] == w:
        return frame
    return cv2.resize(frame, (w, h), interpolation=cv2.INTER_NEAREST if is_depth else cv2.INTER_AREA)


# ─────────────────────────── 主流程 ────────────────────────────
def main() -> int:
    p = argparse.ArgumentParser(
        description="按转换配置修复 + 裁剪已有 LeRobot v3 数据集（无需源 HDF5）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--config", required=True, help="修复/裁剪配置 JSON")
    p.add_argument("--repo_id", required=True, help="源数据集 repo_id")
    p.add_argument("--root", required=True, help="源数据集根目录（含 meta/ data/ videos/）")
    p.add_argument("--new_repo_id", default=None, help="输出 repo_id（默认 <repo_id>_fixed）")
    p.add_argument("--new_root", default=None, help="输出根目录（默认 <root>_fixed）")
    p.add_argument("--episodes", default="all", help="要处理的 episode：all / 0,2,5 / 0-4")
    p.add_argument("--vcodec", default="h264", help="视频编码器")
    p.add_argument("--streaming", action=argparse.BooleanOptionalAction, default=True,
                   help="流式编码（视频帧直接进编码器，不落临时 PNG；后台批处理必须开启，"
                        "否则每集编码后清理数百个 PNG 会触发批量删除保护）")
    p.add_argument("--image-writer-processes", type=int, default=4)
    p.add_argument("--image-writer-threads", type=int, default=4)
    p.add_argument("--overwrite", action="store_true", help="输出目录已存在时删除重建")
    p.add_argument("--dry-run", action="store_true", help="只打印裁剪/修复计划，不写盘")
    args = p.parse_args()

    cfg_features, cfg_mapping, cfg_repair = load_repair_config(Path(args.config))
    opt = {
        "trim": cfg_repair.get("trim", "lead_tail"),
        "stationary_window": int(cfg_repair.get("stationary_window", 0)),
        "stationary_thresh": float(cfg_repair.get("stationary_thresh", 0.03)),
        "range_eps": float(cfg_repair.get("range_eps", 1e-3)),
        "keep_head": int(cfg_repair.get("keep_head", 0)),
        "keep_tail": int(cfg_repair.get("keep_tail", 0)),
        "min_frames": int(cfg_repair.get("min_frames", 45)),
        "cap": int(cfg_repair.get("cap", 8)),
        "min_run": int(cfg_repair.get("min_run", 3)),
        "fill": cfg_repair.get("fill_state_from_action", "auto"),
        "fill_eps": float(cfg_repair.get("fill_eps", 1e-6)),
        "task": cfg_repair.get("task"),
    }

    src_root = Path(args.root).expanduser().resolve()
    if not (src_root / "meta" / "info.json").is_file():
        logger.error("%s 不是 LeRobot 数据集（缺少 meta/info.json）", src_root)
        return 2

    new_repo = args.new_repo_id or f"{args.repo_id}_fixed"
    out_root = Path(args.new_root).expanduser().resolve() if args.new_root else src_root.with_name(src_root.name + "_fixed")
    out_dir = out_root if out_root.name == new_repo else out_root / new_repo

    src = LeRobotDataset(args.repo_id, root=str(src_root))
    meta = src.meta
    fps = meta.fps
    if opt["stationary_window"] <= 0:
        opt["stationary_window"] = max(1, round(0.3 * fps))

    mapping = resolve_mapping(cfg_features, cfg_mapping, meta.features)

    episodes = (
        list(range(meta.total_episodes))
        if args.episodes.strip().lower() == "all"
        else parse_episode_list(args.episodes, meta.total_episodes)
    )

    numeric_out = [k for k, m in mapping.items() if "indices" in m]
    for key in numeric_out:
        if key not in (STATE_KEY, ACTION_KEY):
            raise ValueError(f"暂只支持数值特征 {STATE_KEY}/{ACTION_KEY}，收到 '{key}'")
    if STATE_KEY not in mapping:
        raise ValueError(f"配置必须包含 {STATE_KEY}")
    src_numeric_keys = sorted({m["source"] for m in mapping.values() if "indices" in m})
    numeric = load_numeric(src_root, meta, episodes, src_numeric_keys)

    # ── 维度裁剪 ──
    for ep in episodes:
        raw = {k: numeric[ep][k] for k in src_numeric_keys}  # 先快照，避免输出名与源同名时被覆盖
        for out_key in numeric_out:
            m = mapping[out_key]
            numeric[ep][out_key] = raw[m["source"]][:, m["indices"]].copy()

    # ── 缺失维回填（全局判据：跨所有 episode 判定，保证各集特征语义一致）──
    fill_dims: list[tuple[int, str]] = []
    if STATE_KEY in numeric_out and ACTION_KEY in numeric_out and opt["fill"] == "auto":
        state_all = np.concatenate([numeric[ep][STATE_KEY] for ep in episodes], axis=0)
        action_all = np.concatenate([numeric[ep][ACTION_KEY] for ep in episodes], axis=0)
        s_std = state_all.std(axis=0)
        a_std = action_all.std(axis=0)
        for d in range(min(state_all.shape[1], action_all.shape[1])):
            if s_std[d] <= opt["fill_eps"] and a_std[d] > opt["fill_eps"]:
                for ep in episodes:
                    numeric[ep][STATE_KEY][:, d] = numeric[ep][ACTION_KEY][:, d]
                names = cfg_features[STATE_KEY].get("names")
                fill_dims.append((d, names[d] if names else f"dim{d}"))

    # ── 帧裁剪计划 ──
    plan: dict[int, dict] = {}
    for ep in episodes:
        length = int(meta.episodes[ep]["length"])
        action = numeric[ep][ACTION_KEY] if ACTION_KEY in numeric_out else numeric[ep][src_numeric_keys[0]]
        if opt["trim"] == "lead_tail":
            start, end, info = plan_lead_tail(action, opt)
            plan[ep] = {"start": start, "end": end, "length": length, **info}
        elif opt["trim"] == "cap":
            keep, capped = plan_cap_internal(action, opt)
            plan[ep] = {"start": 0, "end": length, "length": length, "keep": keep, "capped": capped}
        else:
            plan[ep] = {"start": 0, "end": length, "length": length}

    cameras = [k for k, spec in cfg_features.items() if spec["dtype"] in VISUAL_DTYPES]
    if fill_dims:
        logger.warning("state 回填：%s（原本常量，用 action 同维数值替代，属近似修复）",
                       ", ".join(f"dim{d}={n}" for d, n in fill_dims))

    print("\n=== 计划 ===")
    print(f"配置: {args.config}")
    print(f"源: {src_root}  ({meta.total_episodes} ep, {meta.total_frames} frames, fps={fps})")
    print(f"出: {out_dir}  repo_id={new_repo}")
    print(f"特征: " + ", ".join(
        f"{k}{tuple(v['shape'])}" for k, v in cfg_features.items()))
    print(f"裁剪: {opt['trim']}  W={opt['stationary_window']} thr={opt['stationary_thresh']}")
    print(f"{'ep':>3} {'原始':>6} {'首静止':>6} {'尾静止':>6} {'保留':>6} {'丢弃':>6}")
    total_in = total_out = 0
    for ep in episodes:
        info = plan[ep]
        n_in = info["length"]
        n_out = int(info["keep"].sum()) if "keep" in info else info["end"] - info["start"]
        total_in += n_in
        total_out += n_out
        print(f"{ep:>3} {n_in:>6} {info.get('lead', '-'):>6} {info.get('tail', '-'):>6} "
              f"{n_out:>6} {n_in - n_out:>6}")
    print(f"合计: {total_in} -> {total_out} 帧（丢弃 {total_in - total_out}, "
          f"{100.0 * (total_in - total_out) / max(total_in, 1):.1f}%）\n")

    if args.dry_run:
        logger.info("--dry-run：未写盘")
        return 0

    # ── 建新数据集 ──
    if out_dir.exists():
        if not args.overwrite:
            logger.error("输出目录已存在：%s（如需重建请加 --overwrite）", out_dir)
            return 2
        shutil.rmtree(out_dir)
    out_dir.parent.mkdir(parents=True, exist_ok=True)

    features = {k: dict(v, shape=tuple(v["shape"])) for k, v in cfg_features.items()}
    create_kwargs = dict(
        repo_id=new_repo,
        root=str(out_dir),
        fps=fps,
        robot_type=meta.robot_type,
        features=features,
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
    )
    if VideoEncoderConfig is not None:
        create_kwargs["camera_encoder"] = VideoEncoderConfig(vcodec=args.vcodec, pix_fmt="yuv420p")
    else:
        create_kwargs["vcodec"] = args.vcodec
    if args.streaming:
        create_kwargs["streaming_encoding"] = True
    try:
        dst = LeRobotDataset.create(**create_kwargs)
    except TypeError:
        # 该版本 create 不支持 streaming_encoding
        create_kwargs.pop("streaming_encoding", None)
        dst = LeRobotDataset.create(**create_kwargs)

    written = 0
    try:
        for ep in tqdm(episodes, desc="episodes"):
            info = plan[ep]
            length = info["length"]
            start, end = info["start"], info["end"]
            keep = info.get("keep")
            indices = [i for i in range(start, end) if keep is None or keep[i]]
            if not indices:
                logger.warning("episode %d 裁剪后为空，跳过", ep)
                dst.clear_episode_buffer()
                continue

            cams = []
            for out_key in cameras:
                m = mapping[out_key]
                path = src_root / meta.get_video_file_path(ep, m["source"])
                t0 = float(meta.episodes[ep][f"videos/{m['source']}/from_timestamp"])
                is_depth = bool((meta.features[m["source"]].get("info") or {}).get("is_depth_map"))
                cams.append((out_key, iter_video_frames(path, t0, length, fps), m.get("image_size"), is_depth))

            keep_mask = np.zeros(length, dtype=bool)
            keep_mask[indices] = True
            task = opt["task"] or episode_task(meta, ep, None)

            for i in tqdm(range(length), desc=f"  ep{ep}", leave=False):
                frames = {out_key: next(it) for out_key, it, _, _ in cams}
                if not keep_mask[i]:
                    continue  # 解码器只能顺序推进，被丢弃的帧读完即弃
                frame = {"task": task}
                for key in numeric_out:
                    frame[key] = numeric[ep][key][i]
                for out_key, _, image_size, is_depth in cams:
                    img = frames[out_key]
                    if image_size is not None:
                        img = resize_frame(img, image_size, is_depth)
                    frame[out_key] = img
                dst.add_frame(frame)
            dst.save_episode()
            written += 1
    finally:
        dst.finalize()

    logger.info("完成：%d 条 episode，%d 帧 -> %s", written, total_out, out_dir)

    # ── 自检 ──
    check = LeRobotDataset(new_repo, root=str(out_dir))
    ok = True
    if check.num_episodes != written:
        logger.error("自检失败：episode 数 %d != %d", check.num_episodes, written)
        ok = False
    if check.num_frames != total_out:
        logger.error("自检失败：帧数 %d != %d", check.num_frames, total_out)
        ok = False
    for key, spec in cfg_features.items():
        if tuple(check.features[key]["shape"]) != tuple(spec["shape"]):
            logger.error("自检失败：'%s' 维度 %s != %s", key, check.features[key]["shape"], spec["shape"])
            ok = False
    print(f"\n自检: {check.num_episodes} ep / {check.num_frames} frames; "
          f"features={ {k: tuple(v['shape']) for k, v in check.features.items() if k not in DEFAULT_FEATURES} }  "
          f"-> {'通过' if ok else '未通过'}")
    return 0 if ok else 1


def parse_episode_list(spec: str, total: int) -> list[int]:
    out: list[int] = []
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token and not token.startswith("-"):
            lo, hi = token.split("-", 1)
            out.extend(range(int(lo), int(hi) + 1))
        else:
            out.append(int(token))
    out = sorted(set(out))
    bad = [e for e in out if e < 0 or e >= total]
    if bad:
        raise ValueError(f"episode 越界：{bad}（总数 {total}）")
    return out


if __name__ == "__main__":
    raise SystemExit(main())
