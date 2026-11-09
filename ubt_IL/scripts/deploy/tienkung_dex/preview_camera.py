#!/usr/bin/env python3
"""Preview the TienKung Dex head camera (ZMQ JPEG).

Dex 的相机不走 ROS 话题，只有一条 ZMQ JPEG 流：

  * 仿真：ubt_sim 的 ``AsyncJpegCameraSender`` 直接在 **5758** 发 JPEG，
    每帧带 12 字节 ``struct.pack('dI', ts, frame_id)`` 头；
  * 真机：Bridge2（``--enable-image``）把 ``/ob_camera_head/color/image_raw``
    编成 JPEG 发到 **5558**，没有头。

本脚本只订阅这条流：不连机器人、不发指令，可以和 ``rollout.sh`` 同时跑
（对端是 PUB，多订阅者互不影响）。解码与水平裁剪的逻辑与插件里的
``dex_camera``（``ZMQImageReceiver``）一致，窗口里看到的就是策略看到的画面。

Usage:
  # 仿真（容器内）
  /lerobot/.venv/bin/python preview_camera.py
  # 真机
  /lerobot/.venv/bin/python preview_camera.py --port 5558
  # 只看策略那一段（拼接帧里取 0~640）
  /lerobot/.venv/bin/python preview_camera.py --width 640
  # 抓一帧存图（没有 DISPLAY 时自动走这条）
  /lerobot/.venv/bin/python preview_camera.py --once --save-frame /tmp/dex_head.jpg
  # ubt_sim 在另一台机器
  /lerobot/.venv/bin/python preview_camera.py --host 192.168.1.20
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from collections import deque

try:
    import cv2
    import numpy as np
    import zmq
except ImportError as exc:  # pragma: no cover - 依赖缺失时给一句人话
    sys.exit(f"[preview] 缺少依赖 {exc.name}：请用 /lerobot/.venv/bin/python 运行本脚本。")

# 仿真 ubt_sim 的 'dI' 头：double 时间戳 + uint32 帧号
HEADER_SIZE = struct.calcsize("dI")


def _decode(buf: bytes):
    """JPEG bytes -> (frame, header_size, meta)。

    发送端有两种：ubt_sim 带 12 字节头、Bridge2 不带。靠"原样能解 / 剥头能解"
    判断，与 ``ZMQImageReceiver._recv_loop`` 同一套启发式。
    """
    frame = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    if frame is not None:
        return frame, 0, None
    if len(buf) > HEADER_SIZE:
        stripped = cv2.imdecode(
            np.frombuffer(buf[HEADER_SIZE:], np.uint8), cv2.IMREAD_COLOR
        )
        if stripped is not None:
            ts, frame_id = struct.unpack("dI", buf[:HEADER_SIZE])
            return stripped, HEADER_SIZE, (ts, frame_id)
    return None, 0, None


def _recv_latest(sock, block: bool) -> bytes | None:
    """收一条消息；队列里有积压时只保留最新一帧（预览不追旧帧）。"""
    try:
        buf = sock.recv(0 if block else zmq.NOBLOCK)
    except zmq.Again:
        return None
    while True:
        try:
            buf = sock.recv(zmq.NOBLOCK)
        except zmq.Again:
            return buf


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--host", default=os.environ.get("ZMQ_HOST", "127.0.0.1"),
        help="发送端地址（默认 127.0.0.1，跨机时填 ubt_sim 所在机器 IP）",
    )
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("CAMERA_PORT", "5758")),
        help="相机端口：仿真 5758 / 真机 5558（默认 5758）",
    )
    parser.add_argument(
        "--offset-x", type=int, default=0,
        help="拼接帧里本相机画面的起始列（默认 0）",
    )
    parser.add_argument(
        "--width", type=int, default=0,
        help="水平裁剪宽度（0 = 不裁剪；策略配置是 640）",
    )
    parser.add_argument(
        "--height", type=int, default=0,
        help="期望高度，仅用于校验告警（0 = 不校验）",
    )
    parser.add_argument("--once", action="store_true", help="收到一帧就退出")
    parser.add_argument(
        "--save-frame", default=None,
        help="把首帧存成图片（无 DISPLAY 时默认 /tmp/tienkung_dex_head.jpg）",
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0, help="等待首帧的秒数（默认 10）",
    )
    parser.add_argument("--window", default="TienKung Dex head camera", help="cv2 窗口标题")
    parser.add_argument("--print-fps", action="store_true", help="每秒往终端打一行 fps")
    args = parser.parse_args()

    headless = not os.environ.get("DISPLAY")
    if headless and args.save_frame is None:
        args.save_frame = "/tmp/tienkung_dex_head.jpg"
        print(f"[preview] DISPLAY 未设置：抓一帧存到 {args.save_frame}")

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVTIMEO, 1000)
    sock.connect(f"tcp://{args.host}:{args.port}")

    where = {5758: "仿真（ubt_sim 直发）", 5558: "真机（Bridge2 转码）"}.get(args.port, "")
    print(f"[preview] 订阅 tcp://{args.host}:{args.port} {where}".rstrip(), flush=True)

    # ── 等首帧 ────────────────────────────────────────────────────────────
    deadline = time.time() + args.timeout
    first = None
    while time.time() < deadline:
        buf = _recv_latest(sock, block=True)
        if buf is None:
            continue
        frame, header, meta = _decode(buf)
        if frame is not None:
            first = (frame, header, meta)
            break

    if first is None:
        print(
            f"[preview] {args.timeout:g}s 没收到可解码的帧。检查：\n"
            f"          仿真 → ubt_sim 是否在跑、5758 是否有流\n"
            f"          真机 → Bridge2 是否带 --enable-image、端口是否 5558\n"
            f"          跨机 → --host/--port 是否正确",
            file=sys.stderr,
        )
        sock.close()
        ctx.term()
        return 1

    frame, header, meta = first
    h, w = frame.shape[:2]
    note = f"已剥 {header} 字节 dI 头" if header else "无头（Bridge2）"
    print(f"[preview] 首帧 {w}x{h}，{note}")
    if args.height and args.height != h:
        print(f"[preview][WARN] 帧高 {h} != --height {args.height}", file=sys.stderr)

    # ── 裁剪：与 ZMQImageReceiver 一致，只做水平切分 ──────────────────────
    warned = {"crop": False}

    def crop(img):
        if args.width <= 0:
            return img
        x1 = args.offset_x + args.width
        if x1 > img.shape[1]:
            if not warned["crop"]:
                print(
                    f"[preview][WARN] 帧宽 {img.shape[1]} 不够切 "
                    f"[{args.offset_x}:{x1}]，保持原图",
                    file=sys.stderr,
                )
                warned["crop"] = True
            return img
        return np.ascontiguousarray(img[:, args.offset_x:x1])

    frame = crop(frame)
    if args.save_frame:
        cv2.imwrite(args.save_frame, frame)
        print(f"[preview] 已保存 {frame.shape[1]}x{frame.shape[0]} → {args.save_frame}")

    if args.once or headless:
        sock.close()
        ctx.term()
        return 0

    # ── 实时窗口 ──────────────────────────────────────────────────────────
    # dI 头里的时间戳是发送端时钟：跨机时两边时钟不同步，延迟没意义，只在本地显示。
    show_latency = args.host in ("127.0.0.1", "localhost", "::1")
    times: deque[float] = deque(maxlen=120)
    latency_ms: float | None = None
    last_report = 0.0
    shown = 0

    try:
        while True:
            buf = _recv_latest(sock, block=False)
            if buf is None:
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                continue

            frame, _, meta = _decode(buf)
            if frame is None:
                continue
            if meta is not None and show_latency:
                latency_ms = max(0.0, (time.time() - meta[0]) * 1000.0)

            frame = crop(frame)
            shown += 1
            times.append(time.perf_counter())
            fps = 0.0
            if len(times) >= 2 and times[-1] > times[0]:
                fps = (len(times) - 1) / (times[-1] - times[0])

            label = f"{fps:4.1f} fps  {frame.shape[1]}x{frame.shape[0]}"
            if latency_ms is not None:
                label += f"  {latency_ms:.0f} ms"

            hud = frame.copy()
            cv2.putText(
                hud, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 1, cv2.LINE_AA,
            )
            cv2.imshow(args.window, hud)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

            if args.print_fps and time.time() - last_report >= 1.0:
                last_report = time.time()
                print(f"[preview] {label}")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        sock.close()
        ctx.term()

    print(f"[preview] 共显示 {shown} 帧，退出")
    return 0


if __name__ == "__main__":
    sys.exit(main())
