"""ROS2 node that publishes Intel RealSense D405 frames as image messages."""

import logging
import signal
import threading
import time
from typing import Any, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .driver import RealSenseD405Driver

logger = logging.getLogger(__name__)

# QoS profile matching Walker S2 camera topics (BEST_EFFORT + VOLATILE).
# Compatible with walker_camera_relay.py subscriptions.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# Max length for shm_msgs/String char arrays
_SHM_STRING_SIZE = 256


def _write_shm_string(field, value: str) -> None:
    """Write a Python string into an shm_msgs/String char[256] field.

    shm_msgs uses fixed-size ``char[256]`` arrays instead of ``std::string``.
    This writes each character's ASCII code into the underlying ``.data`` array
    (if present), zero-filling the rest.  For fields that are already plain
    strings (e.g. sensor_msgs), assigns directly.
    """
    data = field.data if hasattr(field, "data") else field
    for i in range(_SHM_STRING_SIZE):
        data[i] = ord(value[i]) if i < len(value) else 0


class _CameraSlot:
    """Internal holder for a single camera's driver + publisher + thread."""

    def __init__(
        self,
        name: str,
        driver: RealSenseD405Driver,
        publisher,
        frame_id: str,
        msg_type: str,
    ):
        self.name = name
        self.driver = driver
        self.publisher = publisher
        self.frame_id = frame_id
        self.msg_type = msg_type
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()


class RealSenseWristCameraNode(Node):
    """ROS2 node that captures and publishes frames from Intel RealSense D405 cameras.

    Each camera runs in its own capture+ publish thread. Supports both
    ``sensor_msgs/Image`` (standard ROS2) and ``shm_msgs/Image*`` (Walker S2
    zero-copy shared memory) message types.

    Parameters
    ----------
    cameras : list[dict]
        Each dict:
        - serial (str): camera serial number
        - topic (str): ROS2 topic to publish on
        - msg_type (str): "sensor_msgs/Image", "shm_msgs/Image1m", etc.
        - frame_id (str): optical frame_id in message header
        - width (int): frame width (default 640)
        - height (int): frame height (default 480)
        - fps (int): frame rate (default 15)
    """

    def __init__(self, cameras: list[dict[str, Any]], **kwargs):
        node_name = kwargs.pop("node_name", "realsense_wrist_camera")
        super().__init__(node_name, **kwargs)

        self._slots: list[_CameraSlot] = []
        self._shutdown_event = threading.Event()

        # Resolve message types once
        self._msg_type_cache: dict[str, type] = {}

        for cam_cfg in cameras:
            self._add_camera(cam_cfg)

        if not self._slots:
            raise ValueError("No cameras configured; at least one camera is required.")

        # Register signal handlers for graceful shutdown
        self._install_signal_handlers()

        logger.info(
            "RealSenseWristCameraNode initialized with %d camera(s)", len(self._slots),
        )

    # ------------------------------------------------------------------
    # Camera management
    # ------------------------------------------------------------------

    def _add_camera(self, cfg: dict[str, Any]) -> None:
        """Create driver + publisher for a single camera config entry."""
        serial = str(cfg["serial"])
        topic = cfg["topic"]
        msg_type_name = cfg.get("msg_type", "sensor_msgs/Image")
        frame_id = cfg.get("frame_id", "realsense_camera")
        width = int(cfg.get("width", 640))
        height = int(cfg.get("height", 480))
        fps = int(cfg.get("fps", 60))
        name = cfg.get("name", serial[:8])

        # Resolve message type
        msg_type = self._resolve_msg_type(msg_type_name)

        # Create driver
        driver = RealSenseD405Driver(
            serial=serial,
            width=width,
            height=height,
            fps=fps,
        )

        # Create publisher
        pub = self.create_publisher(msg_type, topic, SENSOR_QOS)

        slot = _CameraSlot(
            name=name,
            driver=driver,
            publisher=pub,
            frame_id=frame_id,
            msg_type=msg_type_name,
        )
        self._slots.append(slot)

        logger.info(
            "Added camera '%s': SN=%s topic=%s msg=%s %dx%d@%dFPS",
            name, serial, topic, msg_type_name, width, height, fps,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start all cameras and their publish loops."""
        for slot in self._slots:
            slot.driver.start()
            slot._stop_event.clear()
            slot._thread = threading.Thread(
                target=self._publish_loop,
                args=(slot,),
                daemon=True,
                name=f"rs_{slot.name}",
            )
            slot._thread.start()

        logger.info("All cameras started.")

    def stop(self) -> None:
        """Stop all cameras and join threads."""
        self._shutdown_event.set()
        for slot in self._slots:
            slot._stop_event.set()
        for slot in self._slots:
            if slot._thread is not None and slot._thread.is_alive():
                slot._thread.join(timeout=3.0)
            slot.driver.stop()
        logger.info("All cameras stopped.")

    def spin_forever(self) -> None:
        """Block until shutdown event is set (call from main thread)."""
        try:
            while rclpy.ok() and not self._shutdown_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # Publish loop (per camera)
    # ------------------------------------------------------------------

    def _publish_loop(self, slot: _CameraSlot) -> None:
        """Capture and publish frames from a single camera."""
        logger.info("Capture loop started for camera '%s'", slot.name)
        target_interval = 1.0 / slot.driver.fps

        while not slot._stop_event.is_set() and rclpy.ok():
            loop_start = time.time()

            img = slot.driver.get_frame(timeout_ms=5000)
            if img is None:
                # No frame — small sleep to avoid busy-waiting on persistent errors
                time.sleep(0.1)
                continue

            try:
                msg = self._build_msg(slot, img)
                slot.publisher.publish(msg)
            except Exception as e:
                logger.warning(
                    "Failed to build/publish message for camera '%s': %s",
                    slot.name, e,
                )
                continue

            # Rate limiting: sleep to maintain target FPS
            elapsed = time.time() - loop_start
            sleep_time = target_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("Capture loop stopped for camera '%s'", slot.name)

    # ------------------------------------------------------------------
    # Message construction
    # ------------------------------------------------------------------

    def _build_msg(self, slot: _CameraSlot, img: np.ndarray) -> Any:
        """Build the appropriate image message from a numpy array."""
        now = self.get_clock().now().to_msg()
        height, width = img.shape[:2]

        if slot.msg_type.startswith("shm_msgs/"):
            return self._build_shm_msg(slot, img, now, height, width)
        else:
            return self._build_sensor_msg(slot, img, now, height, width)

    @staticmethod
    def _build_sensor_msg(
        slot: _CameraSlot, img: np.ndarray, stamp, height: int, width: int,
    ):
        """Build standard sensor_msgs/Image."""
        from sensor_msgs.msg import Image

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = slot.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = img.tobytes()
        return msg

    @staticmethod
    def _build_shm_msg(
        slot: _CameraSlot, img: np.ndarray, stamp, height: int, width: int,
    ):
        """Build shm_msgs/Image* message with fixed-size char[256] + uint8[]."""
        # Resolve and import the shm_msgs type dynamically
        msg_type = RealSenseWristCameraNode._resolve_shm_msg_type(slot.msg_type)
        msg = msg_type()

        # Header — shm_msgs uses custom String type (char[256] arrays)
        msg.header.stamp = stamp
        _write_shm_string(msg.header.frame_id, slot.frame_id)
        msg.height = height
        msg.width = width
        msg.is_bigendian = False
        msg.step = width * 3  # BGR8 = 3 bytes/pixel

        # Encoding: char[256] array — must write ASCII codes individually
        _write_shm_string(msg.encoding, "bgr8")

        # Data: uint8[N] fixed-size numpy array — slice assignment (C-level copy)
        # Avoid Python-level per-element loops: .ravel() is O(1), [:] = is O(n) in C.
        flat = img.ravel()
        msg.data[:flat.size] = flat

        return msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_shm_msg_type(msg_type_name: str):
        """Dynamically import shm_msgs/Image* type."""
        pkg, _, msg_name = msg_type_name.partition("/")
        if not msg_name:
            raise ValueError(f"Invalid msg_type: {msg_type_name!r}")

        try:
            import importlib
            msg_module = importlib.import_module(f"{pkg}.msg")
            return getattr(msg_module, msg_name)
        except (ImportError, AttributeError) as e:
            raise ImportError(
                f"Cannot import {msg_type_name}: {e}. "
                f"Ensure the ROS2 package '{pkg}' is installed and sourced."
            ) from e

    def _resolve_msg_type(self, msg_type_name: str):
        """Resolve message type string to Python class (with caching)."""
        if msg_type_name in self._msg_type_cache:
            return self._msg_type_cache[msg_type_name]

        if msg_type_name == "sensor_msgs/Image":
            from sensor_msgs.msg import Image
            msg_type = Image
        elif msg_type_name.startswith("shm_msgs/"):
            msg_type = self._resolve_shm_msg_type(msg_type_name)
        else:
            raise ValueError(f"Unsupported msg_type: {msg_type_name!r}")

        self._msg_type_cache[msg_type_name] = msg_type
        return msg_type

    # ------------------------------------------------------------------
    # Signal handling
    # ------------------------------------------------------------------

    def _install_signal_handlers(self) -> None:
        """Register signal handlers for graceful shutdown."""
        def _handler(signum, frame):
            logger.info("Received signal %d, shutting down...", signum)
            self._shutdown_event.set()

        for sig in (signal.SIGTERM, signal.SIGINT):
            try:
                signal.signal(sig, _handler)
            except (ValueError, OSError):
                pass  # Not in main thread — the spin_forever loop handles KeyboardInterrupt
