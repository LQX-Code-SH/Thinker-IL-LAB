#!/usr/bin/env python3
"""Preview Walker S2 camera frames from ROS2 image topics.

Two modes:
  --topic    Single camera preview (legacy mode)
  --robot    Multi-camera preview from robot config (reads camera_topics)

Usage:
  python3 preview_camera.py --robot walker_s2_31d
  python3 preview_camera.py --robot walker_s2_10d
  python3 preview_camera.py --topic /sensor/camera/stereo/color/raw
  python3 preview_camera.py --topic /sensor/camera/stereo/color/raw --width 640 --height 480
  python3 preview_camera.py --robot walker_s2_10d --once --save-frame /tmp/tiled.jpg
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import struct
import threading
import time
from pathlib import Path

import numpy as np

logger = logging.getLogger("preview_walker_camera")


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--robot", default=None,
        choices=["walker_s2_31d", "walker_s2_19d", "walker_s2_10d"],
        help="Robot model name — preview all cameras from its camera_topics",
    )
    source.add_argument(
        "--topic", default=None,
        help="Single ROS2 camera topic (default if --robot not given: /sensor/camera/stereo/color/raw)",
    )
    parser.add_argument(
        "--msg-type", default="Image2m",
        choices=["Image8k", "Image512k", "Image1m", "Image2m", "Image4m", "Image6m", "Image8m",
                 "sensor_msgs/Image"],
        help="Image message type (default: Image2m / shm_msgs.msg.Image2m)",
    )
    parser.add_argument("--width", type=int, default=0,
                        help="Per-camera cell width; 0 keeps native size")
    parser.add_argument("--height", type=int, default=0,
                        help="Per-camera cell height; 0 keeps native size")
    parser.add_argument("--window", default="Walker camera", help="OpenCV preview window title")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Wait timeout for first frame in seconds")
    parser.add_argument("--print-fps", action="store_true", help="Periodically print display FPS")
    parser.add_argument("--save-frame", default=None,
                        help="Optional path to save the latest frame (tiled in multi-camera mode)")
    parser.add_argument("--once", action="store_true",
                        help="Receive one frame from each camera, optionally save, then exit")
    parser.add_argument("--headless", action="store_true",
                        help="Skip GUI preview (useful with --print-fps or --save-frame)")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Camera registry (populated by _setup_cameras)
# ---------------------------------------------------------------------------

class CameraEntry:
    """A named camera node + its metadata."""
    __slots__ = ("name", "node", "topic")

    def __init__(self, name: str, node, topic: str):
        self.name = name
        self.node = node
        self.topic = topic


# ---------------------------------------------------------------------------
# Camera setup
# ---------------------------------------------------------------------------

def _resolve_msg_type(name: str):
    """Resolve a --msg-type string to a ROS2 message class."""
    if name == "sensor_msgs/Image":
        from sensor_msgs.msg import Image
        return Image
    import shm_msgs.msg
    return getattr(shm_msgs.msg, name)


def _resolve_camera_topics(args: argparse.Namespace) -> dict[str, str]:
    """Return {camera_name: ros2_topic} from --robot or --topic."""
    if args.robot:
        import importlib.util
        _constants_path = "/ubt_IL/walker/lerobot_robot_walker/lerobot_robot_walker/constants.py"
        spec = importlib.util.spec_from_file_location("_walker_constants", _constants_path)
        _mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(_mod)
        cfg = _mod.ROBOT_MODELS[args.robot]
        return dict(cfg.camera_topics)

    # Single-topic mode
    topic = args.topic or "/sensor/camera/stereo/color/raw"
    return {"camera": topic}


def _topic_to_default_msg_type(topic: str):
    """Heuristic fallback when DDS type discovery fails.

    Returns the most likely shm_msgs type class for a given camera topic.
    """
    import shm_msgs.msg

    if "wrist" in topic:
        return shm_msgs.msg.Image1m
    if "stereo_left" in topic or "stereo_right" in topic:
        return shm_msgs.msg.Image6m
    if "stereo" in topic:
        return shm_msgs.msg.Image2m
    return shm_msgs.msg.Image2m


def _auto_detect_msg_types(topics: dict[str, str]) -> dict[str, object]:
    """Auto-detect the best shm_msgs/Image* type for each topic via ROS2 discovery.

    Creates a temporary node, queries get_topic_names_and_types(), then destroys
    the node. Falls back to a topic-name-based heuristic if discovery fails.
    """
    import rclpy
    import shm_msgs.msg

    result: dict[str, object] = {}

    try:
        temp_node = rclpy.create_node("_preview_type_discovery")
    except Exception:
        return {name: _topic_to_default_msg_type(topic) for name, topic in topics.items()}

    try:
        # Retry discovery a few times (DDS may need time to settle)
        discovered = []
        for _ in range(5):
            time.sleep(0.5)
            discovered = temp_node.get_topic_names_and_types()
            if discovered:
                break

        # Build lookup: topic → set of shm_msgs type names
        type_lookup: dict[str, set[str]] = {}
        for t, types in discovered:
            for typ in types:
                if typ.startswith("shm_msgs/msg/Image"):
                    type_lookup.setdefault(t, set()).add(typ.split("/")[-1])

        for name, topic in topics.items():
            candidates = type_lookup.get(topic, set())
            if not candidates:
                fallback = _topic_to_default_msg_type(topic)
                logger.warning(
                    "Cannot auto-detect msg_type for '%s' (%s), falling back to %s",
                    topic, name, fallback.__name__,
                )
                result[name] = fallback
            else:
                choice = sorted(candidates)[-1]  # lexicographic: Image6m > Image2m > Image1m
                result[name] = getattr(shm_msgs.msg, choice)
                logger.info("  auto-detected %s for '%s' (%s)", choice, topic, name)
    finally:
        temp_node.destroy_node()

    return result


def _setup_cameras(
    topics: dict[str, str],
    msg_types: dict[str, object],
) -> list[CameraEntry]:
    """Create one Camera node per topic.  Must be called after rclpy.init()."""
    import sys
    _walker_sdk_path = "/ubt_IL/walker/walker_sdk_ros2/robot_control"
    if _walker_sdk_path not in sys.path:
        sys.path.insert(0, _walker_sdk_path)
    from camera import Camera

    entries: list[CameraEntry] = []
    for name, topic in topics.items():
        msg_type = msg_types.get(name, None)
        node = Camera(topic=topic, msg_type=msg_type, node_name=f"cam_{name}")
        entries.append(CameraEntry(name=name, node=node, topic=topic))
    return entries


# ---------------------------------------------------------------------------
# Tiling
# ---------------------------------------------------------------------------

def _tile_grid(n: int) -> tuple[int, int]:
    """Return (cols, rows) for n cameras — prefer 3-column layout for 5 cams."""
    if n <= 2:
        return (n, 1)
    if n == 4:
        return (2, 2)
    # 3 or 5 → single row of 3, or 3×2
    return (3, (n + 2) // 3)


def _build_tile(
    frames: list[tuple[str, np.ndarray | None]],
    cell_w: int,
    cell_h: int,
    cv2,
) -> np.ndarray:
    """Tile frames into a single BGR canvas.  Missing frames → black placeholder."""
    n = len(frames)
    cols, rows = _tile_grid(n)
    canvas = np.zeros((rows * cell_h, cols * cell_w, 3), dtype=np.uint8)

    for idx, (name, frame) in enumerate(frames):
        r, c = divmod(idx, cols)
        y0, x0 = r * cell_h, c * cell_w

        if frame is not None:
            h, w = frame.shape[:2]
            # Scale to fit cell, preserving aspect ratio
            scale = min(cell_w / w, cell_h / h)
            fh, fw = int(h * scale), int(w * scale)
            if fh != h or fw != w:
                display = cv2.resize(frame, (fw, fh))
            else:
                display = frame
            # Center in cell
            dy, dx = (cell_h - fh) // 2, (cell_w - fw) // 2
            canvas[y0 + dy:y0 + dy + fh, x0 + dx:x0 + dx + fw] = display

        # Label
        cv2.putText(canvas, name, (x0 + 5, y0 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return canvas


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _is_headless() -> bool:
    """Check whether the X11 display is actually usable (no OpenCV GUI calls).

    Does a full authenticated X11 connection handshake — reading the
    MIT-MAGIC-COOKIE-1 from Xauthority files and including it in the setup
    request.  Without auth, SSH-X11-forwarded servers send status 0 (failed).
    """
    display = os.environ.get("DISPLAY", "")
    if not display:
        return True

    # Parse display:  host:D.S  →  host, D, S
    # ":0" → unix socket,  "localhost:11.0" → TCP port 6011
    try:
        host, rest = display.split(":", 1) if ":" in display else ("", display)
        parts = rest.split(".")
        d = int(parts[0])
        if host:
            port = 6000 + d
            sock = socket.create_connection((host, port), timeout=1.0)
            family = b"\x00\x00"       # FamilyInternet (0)
            conn_addr = host.encode()
        else:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            sock.connect(f"/tmp/.X11-unix/X{d}")
            family = b"\x01\x00"       # FamilyLocal (256)
            conn_addr = socket.gethostname().encode()

        disp_bytes = str(d).encode()

        # Read auth from both $XAUTHORITY and ~/.Xauthority.
        # SSH X11 forwarding stores cookies under the machine hostname
        # (e.g. "vision/unix:10"), not under "localhost:10.0".
        auth_name, auth_data = b"", b""
        auth_name, auth_data = _read_xauth(family, conn_addr, disp_bytes)
        if not auth_data:
            auth_name, auth_data = _read_xauth_any_display(disp_bytes)

        # Build X11 connection setup with auth.
        # All multi-byte fields follow byte 0 (0x6c = little-endian).
        # Auth name and data must be padded to 4-byte boundaries.
        def _pad4(data):
            return data + b"\x00" * ((4 - len(data) % 4) % 4)

        setup = bytearray()
        setup += b"\x6c"                # byte-order: little-endian
        setup += b"\x00"                # unused
        setup += struct.pack("<HH", 11, 0)          # protocol major 11, minor 0
        setup += struct.pack("<HH", len(auth_name), len(auth_data))
        setup += b"\x00\x00"            # padding
        setup += _pad4(auth_name)
        setup += _pad4(auth_data)

        sock.sendall(setup)
        sock.settimeout(2.0)
        resp = sock.recv(1)
        sock.close()
        # Status 1 = Success, 2 = Authenticate — the server is alive.
        return not (len(resp) == 1 and resp[0] in (1, 2))
    except Exception:
        return True


def _read_xauth_any_display(disp: bytes) -> tuple[bytes, bytes]:
    """Read the FIRST MIT-MAGIC-COOKIE-1 with a matching display number.

    SSH X11 forwarding stores cookies as ``hostname/unix:N`` while DISPLAY
    is ``localhost:N.0`` — family and address differ, but display number matches.
    """
    for xauth_path in _xauth_paths():
        try:
            with open(xauth_path, "rb") as f:
                data = f.read()
        except (FileNotFoundError, PermissionError):
            continue
        for fam, entry_addr, entry_disp, entry_name, entry_data, _ in _parse_xauth(data):
            if entry_disp == disp and entry_data:
                return entry_name, entry_data
    return b"", b""


def _xauth_paths():
    """Xauthority file paths to try, in priority order."""
    paths = []
    if os.environ.get("XAUTHORITY"):
        paths.append(os.environ["XAUTHORITY"])
    paths.append(os.path.join(os.path.expanduser("~"), ".Xauthority"))
    return paths


def _parse_xauth(data: bytes):
    """Yield (family, addr, disp, name, data, next_pos) for each Xauthority entry."""
    pos = 0
    while pos + 4 <= len(data):
        fam = data[pos:pos + 2]
        pos += 2
        addr_len = struct.unpack(">H", data[pos:pos + 2])[0]
        pos += 2
        if pos + addr_len > len(data):
            break
        entry_addr = data[pos:pos + addr_len]
        pos += addr_len
        if pos + 2 > len(data):
            break
        disp_len = struct.unpack(">H", data[pos:pos + 2])[0]
        pos += 2
        if pos + disp_len > len(data):
            break
        entry_disp = data[pos:pos + disp_len]
        pos += disp_len
        if pos + 2 > len(data):
            break
        name_len = struct.unpack(">H", data[pos:pos + 2])[0]
        pos += 2
        if pos + name_len > len(data):
            break
        entry_name = data[pos:pos + name_len]
        pos += name_len
        if pos + 2 > len(data):
            break
        data_len = struct.unpack(">H", data[pos:pos + 2])[0]
        pos += 2
        if pos + data_len > len(data):
            break
        entry_data = data[pos:pos + data_len]
        pos += data_len
        yield fam, entry_addr, entry_disp, entry_name, entry_data, pos


def _read_xauth(family: bytes, addr: bytes, disp: bytes) -> tuple[bytes, bytes]:
    """Exact-match (family + address + display) MIT-MAGIC-COOKIE-1 lookup."""
    for xauth_path in _xauth_paths():
        try:
            with open(xauth_path, "rb") as f:
                data = f.read()
        except (FileNotFoundError, PermissionError):
            continue
        for fam, entry_addr, entry_disp, entry_name, entry_data, _ in _parse_xauth(data):
            if fam == family and entry_addr == addr and entry_disp == disp:
                return entry_name, entry_data
    return b"", b""


def _save_frame(path: str, frame: np.ndarray, cv2) -> None:
    output = Path(path)
    ok = cv2.imwrite(str(output), frame)
    if not ok:
        raise RuntimeError(f"failed to write frame to {output}")
    logger.info("Saved frame to %s", output)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    try:
        import cv2
    except ImportError as exc:
        logger.error("Missing dependency: %s. Install opencv-python-headless.", exc)
        return 1

    # ── Resolve topics & message types ────────────────────────────────────
    camera_topics = _resolve_camera_topics(args)

    # ── ROS2 init ────────────────────────────────────────────────────────
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()

    # Always auto-detect per-camera msg_type; --msg-type serves as manual fallback.
    camera_msg_types = _auto_detect_msg_types(camera_topics)

    entries = _setup_cameras(camera_topics, camera_msg_types)

    executor = MultiThreadedExecutor(num_threads=max(2, len(entries) + 1))
    for e in entries:
        executor.add_node(e.node)

    # Wrap executor.spin: on shutdown the signal handler invalidates the rclpy
    # context asynchronously, and the spin thread's next spin_once() raises
    # RCLError ("rcl_shutdown() was called"). Swallow it so the daemon thread
    # exits quietly instead of printing "Exception in thread Thread-1 (spin)".
    def _spin(exec: MultiThreadedExecutor) -> None:
        try:
            exec.spin()
        except Exception:
            pass

    spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spin_thread.start()

    # ── Main loop ────────────────────────────────────────────────────────
    last_tile: np.ndarray | None = None
    last_fps_log = time.perf_counter()
    frames = 0
    return_code = 0

    try:
        # ── Wait for all cameras ─────────────────────────────────────────
        logger.info("Waiting for %d camera(s) (timeout=%.1fs) ...", len(entries), args.timeout)
        all_ready = True
        for e in entries:
            if not e.node.wait_for_image(timeout=args.timeout):
                logger.error("Timeout: no image on '%s' (%s)", e.topic, e.name)
                all_ready = False
        if not all_ready:
            logger.error("Not all cameras are publishing. Check topics and RMW_IMPLEMENTATION.")

        # Print info for ready cameras
        for e in entries:
            if e.node.is_available():
                info = e.node.get_image_info()
                if info:
                    logger.info("  %-20s → %dx%d %s", e.name,
                                info["width"], info["height"], info["encoding"])

        if not args.once and any(e.node.is_available() for e in entries):
            # Detect headless environment — test X11 connectivity WITHOUT
            # touching OpenCV GUI (cv2.namedWindow triggers a C abort on
            # broken X11 connections, which cannot be caught by try/except).
            headless = args.headless
            if not headless:
                headless = _is_headless()
                if headless:
                    logger.warning("X11 display '%s' not reachable — switching to headless mode.",
                                   os.environ.get("DISPLAY", "(unset)"))
                    logger.warning("Use --once --save-frame /tmp/frame.jpg for a single capture.")
            if not headless:
                cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
                logger.info("Press 'q' or Esc in the preview window to exit.")
            else:
                logger.info("Headless mode — Ctrl+C to stop. Use --once to capture a single frame.")

        # ── Determine cell size ──────────────────────────────────────────
        cell_w = args.width
        cell_h = args.height
        if cell_w == 0 or cell_h == 0:
            # Use first available camera's native resolution
            for e in entries:
                info = e.node.get_image_info()
                if info:
                    cell_w = cell_w or info["width"]
                    cell_h = cell_h or info["height"]
                    break
            cell_w = cell_w or 640
            cell_h = cell_h or 480

        while rclpy.ok():
            # Collect latest frame from each camera
            frame_list: list[tuple[str, np.ndarray | None]] = []
            for e in entries:
                img = e.node.get_latest_image(encoding="bgr8")
                frame_list.append((e.name, img))

            if all(f is None for _, f in frame_list):
                time.sleep(0.01)
                continue

            # In single-camera mode, just use the raw frame
            if len(entries) == 1:
                display_frame = frame_list[0][1]
                if display_frame is None:
                    time.sleep(0.01)
                    continue
                if cell_w > 0 and cell_h > 0:
                    display_frame = cv2.resize(display_frame, (cell_w, cell_h))
                last_tile = display_frame
            else:
                display_frame = _build_tile(frame_list, cell_w, cell_h, cv2)
                last_tile = display_frame

            frames += 1

            # --once
            if args.once:
                if args.save_frame and last_tile is not None:
                    _save_frame(args.save_frame, last_tile, cv2)
                else:
                    logger.info("Received one frame from %d camera(s)", len(entries))
                return 0

            # Show (or skip in headless mode)
            if headless:
                time.sleep(0.03)  # ~30 fps cap
            else:
                try:
                    cv2.imshow(args.window, display_frame)
                    key = cv2.waitKey(1) & 0xFF
                except cv2.error as exc:
                    logger.error(
                        "OpenCV preview failed: %s. "
                        "If running headless, use --once --save-frame /tmp/frame.jpg.",
                        exc,
                    )
                    return_code = 1
                    break
                if key in (ord("q"), 27):
                    break

            # FPS
            if args.print_fps:
                now = time.perf_counter()
                elapsed = now - last_fps_log
                if elapsed >= 5.0:
                    logger.info("Preview FPS: %.1f", frames / elapsed)
                    frames = 0
                    last_fps_log = now

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        if args.save_frame and last_tile is not None and not args.once:
            _save_frame(args.save_frame, last_tile, cv2)
        cv2.destroyAllWindows()

        # Ctrl+C triggers rclpy's default SIGINT handler which calls
        # rclpy.shutdown() asynchronously — the context may already be
        # invalid by the time we reach this block.  Guard every ROS2 call.
        if not rclpy.ok():
            # Already shut down by signal handler. _spin swallows the RCLError
            # raised on the invalid context, so join lets the daemon thread
            # exit cleanly before we return (avoids a stray stderr trace).
            spin_thread.join(timeout=2.0)
            return return_code

        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        for e in entries:
            try:
                e.node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
