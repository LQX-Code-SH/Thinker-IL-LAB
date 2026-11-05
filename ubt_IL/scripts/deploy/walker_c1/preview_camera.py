#!/usr/bin/env python3
"""Preview the Walker C1 head camera from ROS2.

C1 只有一路头部单目 RGB，但仿真与真机的消息类型不同：
  - 仿真（ubt_sim 桥接）：`/sensor/camera/head/color/raw`（sensor_msgs/Image）
  - 真机（Thor 版机器人系统）：`/sensor/camera/head_stereo_left/color/compressed`
    （sensor_msgs/CompressedImage，JPEG）

`--msg-type auto`（默认）按话题名推断：以 `/compressed` 结尾即为压缩图。

Usage:
  python3 preview_camera.py                          # 实时预览（需 DISPLAY）
  python3 preview_camera.py --once --save-frame /tmp/c1_head.jpg
  python3 preview_camera.py --once --save-frame /tmp/c1_real.jpg \
      --topic /sensor/camera/head_stereo_left/color/compressed
  python3 preview_camera.py --topic /sensor/camera/head/color/raw --width 640 --height 480
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


def _encoding_str(msg) -> str:
    """取 encoding 字段：sensor_msgs/Image 是 str，shm_msgs 是定长 char[256]+size。"""
    raw = getattr(msg, "encoding", "")
    if hasattr(raw, "data"):
        size = int(getattr(raw, "size", 0)) or len(raw.data)
        return bytes(raw.data)[:size].decode("utf-8", "replace")
    return str(raw)


def _decode(msg) -> np.ndarray | None:
    """sensor_msgs/Image 或 shm_msgs/Image6m -> BGR ndarray（不依赖 cv_bridge）。"""
    import cv2

    h, w = int(msg.height), int(msg.width)
    step = int(getattr(msg, "step", 0) or 0)
    enc = _encoding_str(msg).strip().lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    # Walker C1 真机：yuv422（UYVY），step = 2*width
    if enc in ("yuv422", "uyvy", "uyvy422", "yuyv", "yuyv422"):
        if step <= 0:
            step = w * 2
        # OpenCV 的 YUV422 转换要 (h, w, 2) 的打包字节视图（UYVY = U Y V Y…），
        # 每行前 w*2 字节；reshape 成 (h, w//2, 2) 会把宽度砍半。
        rows = buf[: h * step].reshape(h, step)[:, : w * 2]
        yuv = rows.reshape(h, w, 2)
        code = (
            cv2.COLOR_YUV2BGR_YUYV if enc.startswith(("yuyv", "yuyv422"))
            else cv2.COLOR_YUV2BGR_UYVY
        )
        return cv2.cvtColor(yuv, code)

    if enc in ("rgb8", "bgr8"):
        channels = 3
    elif enc in ("rgba8", "bgra8"):
        channels = 4
    elif enc in ("mono8", "8uc1"):
        channels = 1
    else:
        print(f"[preview] unsupported encoding: {enc}", file=sys.stderr)
        return None

    if step <= 0:
        step = w * channels
    if step != w * channels:
        # 行有 padding：按 step 截断每一行
        rows = [buf[i * step: i * step + w * channels] for i in range(h)]
        buf = np.concatenate(rows)
    img = buf.reshape(h, w, channels) if channels > 1 else buf.reshape(h, w)
    if enc == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif enc == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif channels == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _decode_compressed(msg) -> np.ndarray | None:
    """sensor_msgs/CompressedImage -> BGR ndarray."""
    import cv2

    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if buf.size == 0:
        print("[preview] empty compressed payload", file=sys.stderr)
        return None
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        print(f"[preview] JPEG/PNG decode failed (format={getattr(msg, 'format', '?')})", file=sys.stderr)
    return img


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/sensor/camera/head/color/raw", help="Image topic")
    parser.add_argument(
        "--msg-type",
        default="auto",
        choices=["auto", "image", "compressed", "image6m"],
        help="消息类型：auto=按话题名推断（/compressed→压缩图，其余→sensor_msgs/Image）；"
        "真机 shm_msgs/Image6m 需显式指定 image6m",
    )
    parser.add_argument("--width", type=int, default=0, help="Resize width (0 = keep)")
    parser.add_argument("--height", type=int, default=0, help="Resize height (0 = keep)")
    parser.add_argument("--once", action="store_true", help="Grab one frame and exit")
    parser.add_argument("--save-frame", default=None, help="Save the first frame to this path")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for first frame")
    parser.add_argument("--window", default="Walker C1 camera", help="cv2 window title")
    parser.add_argument("--print-fps", action="store_true", help="Print receive FPS")
    args = parser.parse_args()

    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image

    msg_type = args.msg_type
    if msg_type == "auto":
        msg_type = "compressed" if args.topic.rstrip("/").endswith("/compressed") else "image"
    if msg_type == "compressed":
        sub_type, decode = CompressedImage, _decode_compressed
    elif msg_type == "image6m":
        from shm_msgs.msg import Image6m

        sub_type, decode = Image6m, _decode
    else:
        sub_type, decode = Image, _decode

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )

    rclpy.init()
    node = Node("walker_c1_preview_camera")
    state = {"img": None, "count": 0, "t0": time.time()}

    def cb(msg):
        img = decode(msg)
        if img is not None:
            state["img"] = img
            state["count"] += 1

    node.create_subscription(sub_type, args.topic, cb, qos)
    print(f"[preview] waiting for {args.topic} ({msg_type}, timeout {args.timeout}s)...")

    headless = not os.environ.get("DISPLAY")
    if headless and not args.save_frame:
        args.save_frame = "/tmp/walker_c1_head.jpg"
        print("[preview] DISPLAY 未设置：改为抓一帧保存到", args.save_frame)

    deadline = time.time() + args.timeout
    while time.time() < deadline and state["img"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["img"] is None:
        print("[preview] 超时：没有收到图像。检查 ROS_DOMAIN_ID / RMW / 话题名。", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    def render(img):
        if args.width and args.height:
            return cv2.resize(img, (args.width, args.height))
        return img

    frame = render(state["img"])
    if args.save_frame:
        cv2.imwrite(args.save_frame, frame)
        print(f"[preview] saved {frame.shape[1]}x{frame.shape[0]} → {args.save_frame}")

    if args.once or headless:
        node.destroy_node()
        rclpy.shutdown()
        return 0

    last = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["img"] is not None:
                frame = render(state["img"])
                cv2.imshow(args.window, frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            if args.print_fps:
                now = time.time()
                if now - last >= 1.0:
                    print(f"[preview] fps≈{state['count'] / (now - state['t0']):.1f} shape={frame.shape}")
                    last = now
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
