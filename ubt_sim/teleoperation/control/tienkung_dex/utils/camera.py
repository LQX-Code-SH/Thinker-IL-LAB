#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 相机工具：RGB + 深度订阅、预览、存盘。

话题从桥接配置读：``/ob_camera_head/color/image_raw`` +
``/ob_camera_head/depth/image_raw``（与真机一致，仿真发同一套，靠
``ROS_DOMAIN_ID`` 分流）。

消息类型都是 ``sensor_msgs/Image``：RGB ``rgb8``，深度 ``16UC1``（毫米）。
C++ 图像桥（``teleoperation/bridges/zmq_image_bridge``）负责发，本模块不碰 ZMQ。

用法::

    /usr/bin/python3 utils/camera.py --preview
    /usr/bin/python3 utils/camera.py --save --count 10 --interval 0.5
    /usr/bin/python3 utils/camera.py --info
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from typing import Any

import numpy as np
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

_UTILS_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROL_DIR = os.path.dirname(_UTILS_DIR)
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

BRIDGE_DIR = os.path.abspath(
    os.path.join(_CONTROL_DIR, os.pardir, os.pardir, "bridges", "tienkung_dex")
)
CONFIG_PATH = os.path.join(BRIDGE_DIR, "bridge_config.yaml")

# 配置文件读不到时的兜底（与 bridge_config.yaml 保持一致）
FALLBACK_TOPICS = ("/ob_camera_head/color/image_raw", "/ob_camera_head/depth/image_raw")


def image_topics() -> tuple[str, str]:
    """返回 ``(rgb_topic, depth_topic)``；优先读桥接配置文件。"""
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH) as f:
                cfg = yaml.safe_load(f)
            pub = cfg["topics"]["pub"]
            return pub["image_rgb"]["topic"], pub["image_depth"]["topic"]
        except Exception:  # noqa: BLE001 - 配置缺字段时退回兜底
            pass
    return FALLBACK_TOPICS


def image_to_array(msg: Image) -> np.ndarray:
    """``sensor_msgs/Image`` -> numpy（按 ``step`` 处理行对齐）。"""
    h, w = int(msg.height), int(msg.width)
    encoding = (msg.encoding or "").lower()
    if encoding in ("rgb8", "bgr8"):
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, int(msg.step))
        arr = row[:, : w * 3].reshape(h, w, 3)
        return arr[:, :, ::-1].copy() if encoding == "bgr8" else arr.copy()
    if encoding in ("16uc1", "mono16"):
        row = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, int(msg.step) // 2)
        return row[:, :w].copy()
    if encoding == "32fc1":
        row = np.frombuffer(msg.data, dtype=np.float32).reshape(h, int(msg.step) // 4)
        return row[:, :w].copy()
    if encoding in ("mono8",):
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, int(msg.step))
        return row[:, :w].copy()
    raise ValueError(f"不支持的图像编码: {msg.encoding}")


class DexCamera(Node):
    """RGB + 深度订阅节点。"""

    def __init__(
        self,
        rgb_topic: str | None = None,
        depth_topic: str | None = None,
    ):
        super().__init__("tienkung_dex_camera")
        default_rgb, default_depth = image_topics()
        self.rgb_topic = rgb_topic or default_rgb
        self.depth_topic = depth_topic or default_depth

        self.lock = threading.Lock()
        self.rgb: np.ndarray | None = None
        self.depth: np.ndarray | None = None
        self.rgb_msg: Image | None = None
        self.depth_msg: Image | None = None
        self.rgb_count = 0
        self.depth_count = 0
        self.rgb_stamp: float | None = None
        self.depth_stamp: float | None = None
        self._warned: set[str] = set()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        # 图像话题通常只有 publisher 用默认（reliable）QoS 时，BEST_EFFORT sub 也能收
        self.create_subscription(Image, self.rgb_topic, self._rgb_cb, qos)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, qos)
        self.get_logger().info(f"RGB: {self.rgb_topic}  深度: {self.depth_topic}")

    def _rgb_cb(self, msg: Image) -> None:
        try:
            arr = image_to_array(msg)
        except ValueError as exc:
            self._warn_once(f"rgb:{msg.encoding}", str(exc))
            return
        with self.lock:
            self.rgb = arr
            self.rgb_msg = msg
            self.rgb_count += 1
            self.rgb_stamp = time.time()

    def _depth_cb(self, msg: Image) -> None:
        try:
            arr = image_to_array(msg)
        except ValueError as exc:
            self._warn_once(f"depth:{msg.encoding}", str(exc))
            return
        with self.lock:
            self.depth = arr
            self.depth_msg = msg
            self.depth_count += 1
            self.depth_stamp = time.time()

    def _warn_once(self, key: str, message: str) -> None:
        if key in self._warned:
            return
        self._warned.add(key)
        self.get_logger().warning(message)

    # ------------------------------------------------------------------ 查询
    def wait_for_rgb(self, timeout: float = 10.0) -> bool:
        return self._wait(lambda: self.rgb_count > 0, timeout, "RGB")

    def wait_for_depth(self, timeout: float = 10.0) -> bool:
        return self._wait(lambda: self.depth_count > 0, timeout, "深度")

    def _wait(self, predicate, timeout: float, label: str) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if predicate():
                return True
            time.sleep(0.02)
        self.get_logger().error(f"等待{label}图像超时（{timeout}s）")
        return False

    def snapshot(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        with self.lock:
            rgb = None if self.rgb is None else self.rgb.copy()
            depth = None if self.depth is None else self.depth.copy()
        return rgb, depth

    def depth_stats(self) -> dict[str, float] | None:
        with self.lock:
            if self.depth is None:
                return None
            depth = self.depth
        valid = depth[(depth > 0) & (depth < 65535)]  # 65535 = 无穷远/无回波
        if valid.size == 0:
            return {"valid_ratio": 0.0}
        return {
            "min_mm": float(valid.min()),
            "max_mm": float(valid.max()),
            "mean_mm": float(valid.mean()),
            "valid_ratio": float(valid.size / depth.size),
        }


# ============================================================================
# 打印 / 保存
# ============================================================================
def print_image_info(cam: DexCamera, with_depth: bool = True) -> None:
    with cam.lock:
        rgb, depth = cam.rgb, cam.depth
        rgb_count, depth_count = cam.rgb_count, cam.depth_count
        rgb_stamp, depth_stamp = cam.rgb_stamp, cam.depth_stamp
    if rgb is None:
        print("RGB: 无数据")
    else:
        age = "—" if rgb_stamp is None else f"{time.time() - rgb_stamp:.2f}s"
        print(f"RGB:   {rgb.shape} {rgb.dtype}  帧数={rgb_count}  距上帧={age}")
    if not with_depth:
        return
    if depth is None:
        print("深度: 无数据")
        return
    age = "—" if depth_stamp is None else f"{time.time() - depth_stamp:.2f}s"
    stats = cam.depth_stats() or {}
    stat_str = " ".join(f"{k}={v:.1f}" for k, v in stats.items() if k != "valid_ratio")
    print(
        f"深度:  {depth.shape} {depth.dtype}  帧数={depth_count}  距上帧={age}  "
        f"有效={stats.get('valid_ratio', 0.0) * 100:.1f}%  {stat_str}"
    )


def save_snapshot(
    cam: DexCamera, out_dir: str, prefix: str = "frame", with_depth: bool = True
) -> str | None:
    """保存一帧：``<prefix>.png``（RGB）+ ``<prefix>_depth.png/.npy``（深度）。"""
    import cv2

    rgb, depth = cam.snapshot()
    if rgb is None:
        print("没有 RGB 帧，跳过保存")
        return None
    os.makedirs(out_dir, exist_ok=True)
    rgb_path = os.path.join(out_dir, f"{prefix}.png")
    cv2.imwrite(rgb_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if with_depth and depth is not None:
        cv2.imwrite(os.path.join(out_dir, f"{prefix}_depth.png"), depth)
        np.save(os.path.join(out_dir, f"{prefix}_depth.npy"), depth)
    print(f"已保存 {rgb_path}" + ("" if depth is None else f" + {prefix}_depth.png/.npy"))
    return rgb_path


def preview_loop(cam: DexCamera, with_depth: bool = True, show_depth: bool = False) -> None:
    """cv2 实时预览：``q``/``ESC`` 退出，``s`` 存一帧到当前目录。"""
    import cv2

    print("预览中：q/ESC 退出，s 保存一帧")
    saved = 0
    try:
        while rclpy.ok():
            rgb, depth = cam.snapshot()
            if rgb is not None:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cv2.imshow("tienkung_dex RGB", bgr)
                if show_depth and depth is not None and with_depth:
                    # 16UC1 毫米 -> 0-255 灰度（0.2~4 m）
                    scaled = np.clip((depth.astype(np.float32) - 200.0) / (4000.0 - 200.0), 0, 1)
                    cv2.imshow("tienkung_dex depth", (scaled * 255).astype(np.uint8))
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                save_snapshot(cam, os.getcwd(), prefix=f"snapshot_{int(time.time())}", with_depth=with_depth)
                saved += 1
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if saved:
            print(f"共保存 {saved} 帧")


# ============================================================================
# CLI
# ============================================================================
def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="TienKung Dex 相机订阅 / 预览 / 存盘")
    parser.add_argument("--rgb-topic", default=None, help="覆盖 RGB 话题")
    parser.add_argument("--depth-topic", default=None, help="覆盖深度话题")
    parser.add_argument("--info", action="store_true", help="周期性打印帧信息（默认模式）")
    parser.add_argument("--preview", action="store_true", help="cv2 预览窗口")
    parser.add_argument("--show-depth", action="store_true", help="预览时同时显示深度")
    parser.add_argument("--save", action="store_true", help="保存图像")
    parser.add_argument("--out-dir", default="camera_frames", help="保存目录（默认 ./camera_frames）")
    parser.add_argument("--count", type=int, default=0, help="保存/预览帧数（0=无限）")
    parser.add_argument("--interval", type=float, default=1.0, help="保存/打印间隔（秒）")
    parser.add_argument("--no-depth", action="store_true", help="不处理深度")
    cli = parser.parse_args(args)

    rclpy.init()
    cam = DexCamera(rgb_topic=cli.rgb_topic, depth_topic=cli.depth_topic)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(cam)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        print(f"等待 RGB 图像：{cam.rgb_topic}")
        if not cam.wait_for_rgb(timeout=10.0):
            print("检查：C++ 图像桥是否在跑（桥接日志里应有 Starting C++ image bridge）")
            return
        if not cli.no_depth:
            cam.wait_for_depth(timeout=5.0)

        if cli.preview:
            preview_loop(cam, with_depth=not cli.no_depth, show_depth=cli.show_depth)
            return

        saved = 0
        printed = 0
        start = time.monotonic()
        while rclpy.ok():
            if cli.count and (saved if cli.save else printed) >= cli.count:
                break
            if cli.save:
                prefix = f"frame_{saved:04d}"
                save_snapshot(cam, cli.out_dir, prefix=prefix, with_depth=not cli.no_depth)
                saved += 1
            else:
                print_image_info(cam, with_depth=not cli.no_depth)
                printed += 1
            if cli.interval > 0:
                time.sleep(cli.interval)
            elif time.monotonic() - start > 60.0:
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown(timeout_sec=1.0)
        except TypeError:
            executor.shutdown()
        spin_thread.join(timeout=2.0)
        cam.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
