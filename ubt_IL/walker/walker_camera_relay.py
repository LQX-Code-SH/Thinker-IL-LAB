#!/usr/bin/env python3
"""Walker Camera Relay — ROS2 shm_msgs → ZMQ JPEG bridge (standalone process).

Extracted from ros2_walker_bridge.py (CameraRelay class).
Subscribes to ROS2 shm_msgs/Image* topics, resizes to target resolution,
JPEG-encodes, and publishes over ZMQ for LeRobot WalkerCamera consumption.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import signal
import threading
import time

import cv2
import numpy as np
import zmq

logger = logging.getLogger("walker_camera_relay")


# ── Process cleanup (same pattern as kill_existing_bridge) ──────────────────


def _is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def kill_existing_camera_relay() -> None:
    """Find and kill any already-running walker_camera_relay.py processes."""
    current_pid = os.getpid()
    parent_pid = os.getppid()

    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in (current_pid, parent_pid):
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                parts = f.read().split(b"\x00")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if len(parts) < 2:
            continue
        if parts[1].decode("utf-8", "replace").endswith("walker_camera_relay.py"):
            pids.append(pid)

    if not pids:
        return

    logger.info("Found existing camera relay processes (PIDs: %s), terminating ...", pids)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        alive = [p for p in pids if _is_alive(p)]
        if not alive:
            break
        time.sleep(0.1)
    else:
        for pid in alive:
            logger.warning("Force killing camera relay process %d", pid)
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(0.1)

    time.sleep(0.5)
    logger.info("Previous camera relay instances terminated.")


# ── Camera Relay ───────────────────────────────────────────────────────────


class CameraRelay:
    """Relays Walker camera images from ROS2 shm_msgs to ZMQ (standalone)."""

    def __init__(self, camera_topics: dict, zmq_image_port: int):
        self._camera_topics = {}
        for cam_name, value in camera_topics.items():
            if isinstance(value, dict):
                self._camera_topics[cam_name] = {
                    "topic": value.get("topic"),
                    "msg_type": value.get("msg_type", "shm_msgs/Image2m"),
                    "width": int(value.get("width", 640)),
                    "height": int(value.get("height", 360)),
                }
            else:
                self._camera_topics[cam_name] = {
                    "topic": value,
                    "msg_type": "shm_msgs/Image2m",
                    "width": 640,
                    "height": 360,
                }

        self._running = True
        self._latest_images: dict[str, tuple] = {}
        self._image_lock = threading.Lock()
        self._logged_first: set[str] = set()

        # ZMQ image publisher
        self._zmq_context = zmq.Context()
        self._image_socket = self._zmq_context.socket(zmq.PUB)
        self._image_socket.bind(f"tcp://*:{zmq_image_port}")
        self._image_socket.setsockopt(zmq.SNDHWM, 1)
        logger.info("Camera relay ZMQ image PUB: port=%d", zmq_image_port)

        # ROS2 Node
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.callback_groups import ReentrantCallbackGroup

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("walker_camera_relay")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Reentrant 组：允许各相机回调在 MultiThreadedExecutor 的多线程上并发执行，
        # 避免 head(1920x1536 yuv422 重解码) 占满默认 MutuallyExclusive 组饿死其它相机。
        cb_group = ReentrantCallbackGroup()

        # Dynamically resolve shm_msgs/Image* or sensor_msgs/Image type strings
        def _resolve_msg_type(msg_type_name: str):
            if msg_type_name == "sensor_msgs/Image":
                from sensor_msgs.msg import Image
                return Image
            pkg, sep, msg_name = msg_type_name.partition("/")
            if not sep:
                return None
            try:
                import importlib
                msg_module = importlib.import_module(f"{pkg}.msg")
                return getattr(msg_module, msg_name)
            except (ImportError, AttributeError):
                return None

        for cam_name, cam_cfg in self._camera_topics.items():
            topic = cam_cfg.get("topic")
            msg_type_name = cam_cfg.get("msg_type", "shm_msgs/Image2m")
            msg_type = _resolve_msg_type(msg_type_name)
            if msg_type is None:
                logger.warning(
                    "Camera relay: unsupported/unavailable msg_type %s for %s",
                    msg_type_name, cam_name,
                )
                continue
            self._node.create_subscription(
                msg_type, topic,
                lambda msg, name=cam_name: self._camera_callback(name, msg),
                qos_sensor,
                callback_group=cb_group,
            )
            target_w = cam_cfg.get("width", 640)
            target_h = cam_cfg.get("height", 360)
            logger.info(
                "Camera relay: subscribed %s (%s) → %s (resize→%dx%d)",
                cam_name, msg_type_name, topic, target_w, target_h,
            )

        # Executor + thread
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="camera_relay_executor"
        )
        self._executor_thread.start()

        # Publish thread
        self._pub_thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="camera_relay_pub"
        )
        self._pub_thread.start()

    # ── Camera callback ─────────────────────────────────────────────────

    def _camera_callback(self, cam_name: str, msg) -> None:
        try:
            height = msg.height
            width = msg.width
            step = msg.step
            encoding = self._resolve_encoding(msg)
            img_data = bytes(msg.data)

            byte_count = height * step
            if encoding == "bgr8":
                img = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, width, 3))
            elif encoding == "rgb8":
                img = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif encoding == "yuv422":
                img = self._yuv422_to_bgr(img_data[:byte_count], width, height)
            elif encoding == "mono8":
                img = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, width))
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                logger.debug("Camera relay: unsupported encoding %s for %s", encoding, cam_name)
                return

            if cam_name not in self._logged_first:
                self._logged_first.add(cam_name)
                logger.info("Camera relay: first frame for %s, %dx%d %s", cam_name, width, height, encoding)

            # Resize to target resolution before JPEG encode (bandwidth optimization)
            cam_cfg = self._camera_topics.get(cam_name, {})
            target_w = cam_cfg.get("width", 640)
            target_h = cam_cfg.get("height", 360)
            if img.shape[1] != target_w or img.shape[0] != target_h:
                img = cv2.resize(img, (target_w, target_h))

            success, jpeg_buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not success:
                return

            with self._image_lock:
                self._latest_images[cam_name] = (jpeg_buf.tobytes(), time.time())

        except Exception as e:
            logger.warning("Camera relay: callback error for %s: %s", cam_name, e)

    def _publish_loop(self) -> None:
        while self._running:
            try:
                with self._image_lock:
                    if not self._latest_images:
                        time.sleep(0.01)
                        continue
                    images_b64 = {
                        cam_name: base64.b64encode(jpeg_bytes).decode('ascii')
                        for cam_name, (jpeg_bytes, _ts) in self._latest_images.items()
                    }
                self._image_socket.send_string(
                    json.dumps({"images": images_b64, "ts": time.time()}),
                    flags=zmq.NOBLOCK,
                )
                time.sleep(0.033)
            except zmq.Again:
                logger.debug("Camera relay: image send dropped (ZMQ buffer full)")
            except Exception as e:
                logger.warning("Camera relay: publish error: %s", e)
                time.sleep(0.1)

    def stop(self) -> None:
        self._running = False
        if self._pub_thread is not None and self._pub_thread.is_alive():
            self._pub_thread.join(timeout=2.0)
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        if self._executor_thread is not None and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
        self._image_socket.close()
        self._zmq_context.term()

    # ── Static helpers ──────────────────────────────────────────────────

    @staticmethod
    def _resolve_encoding(msg) -> str:
        raw = msg.encoding
        if hasattr(raw, 'data'):
            encoding = ''.join(chr(c) for c in raw.data if c != 0)
        else:
            encoding = str(raw)
        known = ["bgr8", "rgb8", "bgra8", "rgba8", "mono8", "mono16",
                 "yuv422", "yuyv422", "uyvy422", "16UC1", "32FC1"]
        for k in known:
            if encoding.startswith(k):
                return k
        return encoding

    @staticmethod
    def _yuv422_to_bgr(yuv_data, width, height) -> np.ndarray:
        yuv = np.frombuffer(yuv_data, dtype=np.uint8).reshape((height, width // 2, 4))
        u = yuv[:, :, 0]
        y0 = yuv[:, :, 1]
        v = yuv[:, :, 2]
        y1 = yuv[:, :, 3]
        y = np.zeros((height, width), dtype=np.uint8)
        y[:, 0::2] = y0
        y[:, 1::2] = y1
        u_full = np.repeat(u, 2, axis=1)
        v_full = np.repeat(v, 2, axis=1)
        yuv_img = cv2.merge((y, u_full, v_full))
        return cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR)


# ── CLI ─────────────────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Walker Camera Relay — ROS2 shm_msgs → ZMQ JPEG")
    parser.add_argument("--config", type=str, default=None, required=True,
                       help="JSON config with camera_topics, zmq_image_port, ros_namespace")
    return parser.parse_args()


def main():
    args = _parse_args()

    try:
        cfg = json.loads(args.config)
    except json.JSONDecodeError as e:
        logger.error("Failed to parse --config JSON: %s", e)
        return

    zmq_image_port = int(cfg.get("zmq_image_port", 5563))
    camera_topics = cfg.get("camera_topics", {})

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    if not camera_topics:
        logger.warning("No camera_topics configured, exiting.")
        return

    kill_existing_camera_relay()

    relay = CameraRelay(camera_topics, zmq_image_port)

    stop_event = threading.Event()

    def signal_handler(sig, frame):
        logger.info("Received signal %s, shutting down...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Camera relay running (%d cameras on port %d). Press Ctrl+C to stop.",
                len(camera_topics), zmq_image_port)
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass

    logger.info("Shutting down camera relay...")
    relay.stop()
    import rclpy
    if rclpy.ok():
        rclpy.shutdown()
    logger.info("Camera relay stopped.")


if __name__ == "__main__":
    main()
