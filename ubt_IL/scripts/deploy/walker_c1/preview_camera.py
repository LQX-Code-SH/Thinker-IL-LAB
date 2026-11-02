#!/usr/bin/env python3
"""Preview the Walker C1 head camera (`sensor_msgs/Image`) from ROS2.

C1 has a single head RGB camera (`/sensor/camera/head/color/raw`), published by
ubt_sim (simulation) or the robot SDK (real). This script subscribes, optionally
resizes, and shows/saves frames.

Usage:
  python3 preview_camera.py                          # 实时预览（需 DISPLAY）
  python3 preview_camera.py --once --save-frame /tmp/c1_head.jpg
  python3 preview_camera.py --topic /sensor/camera/head/color/raw --width 640 --height 480
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


def _decode(msg) -> np.ndarray | None:
    """sensor_msgs/Image -> BGR ndarray (no cv_bridge dependency)."""
    import cv2

    h, w = int(msg.height), int(msg.width)
    enc = str(msg.encoding).lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        channels = 3
    elif enc in ("rgba8", "bgra8"):
        channels = 4
    elif enc in ("mono8", "8uc1"):
        channels = 1
    else:
        print(f"[preview] unsupported encoding: {msg.encoding}", file=sys.stderr)
        return None

    if msg.step and int(msg.step) != w * channels:
        # 行有 padding：按 step 截断每一行
        rows = [
            buf[i * int(msg.step): i * int(msg.step) + w * channels]
            for i in range(h)
        ]
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/sensor/camera/head/color/raw", help="Image topic")
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
    from sensor_msgs.msg import Image

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
        img = _decode(msg)
        if img is not None:
            state["img"] = img
            state["count"] += 1

    node.create_subscription(Image, args.topic, cb, qos)
    print(f"[preview] waiting for {args.topic} (timeout {args.timeout}s)...")

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
