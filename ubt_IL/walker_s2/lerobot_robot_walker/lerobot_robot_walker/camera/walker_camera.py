"""WalkerCamera — LeRobot Camera backed by Walker Bridge2 ZMQ image relay.

Receives JPEG frames from Walker Bridge2 (ros2_walker_bridge.py) via ZMQ SUB,
decodes them, and presents them through the LeRobot Camera interface.

The Bridge2 process handles shm_msgs subscription, deep-copy, decode, and
JPEG re-encoding on the Python 3.10 side. This camera runs in the Python 3.12
LeRobot process and only needs ZMQ + OpenCV.

ZMQ message format from Bridge2:
    JSON: {"images": {"<camera_name>": "<base64-encoded-JPEG>"}, "ts": <float>}

Inherits:
    - Camera: LeRobot standard camera interface
"""

from __future__ import annotations

import base64
import json
import logging
import threading
import time
from typing import Any

import cv2
import numpy as np
import zmq
from numpy.typing import NDArray

from lerobot.cameras.camera import Camera

from .config_walker_camera import WalkerCameraConfig

logger = logging.getLogger(__name__)


class WalkerCamera(Camera):
    """Camera that receives frames from Walker Bridge2 via ZMQ.

    Architecture:
        Walker HW → shm_msgs (ROS2) → Bridge2 (decode + JPEG encode)
            → ZMQ PUB :5563 (JSON with base64 JPEG)
                → WalkerCamera (ZMQ SUB + JPEG decode)
                    → Camera interface
    """

    def __init__(self, config: WalkerCameraConfig):
        Camera.__init__(self, config)
        self.config = config
        self.color_mode = config.color_mode
        self.timeout_ms = config.timeout_ms
        self._connected = False

        # ZMQ state (initialized in connect())
        self._zmq_context: zmq.Context | None = None
        self._zmq_socket: zmq.Socket | None = None

        # Thread-safe frame buffer
        self._frame_lock = threading.Lock()
        self._latest_frame: NDArray[Any] | None = None
        self._latest_timestamp: float | None = None
        self._new_frame_event = threading.Event()

        # Receive thread
        self._recv_thread: threading.Thread | None = None
        self._running = False

    def __str__(self) -> str:
        return (
            f"WalkerCamera(zmq://{self.config.server_address}:"
            f"{self.config.port}, name={self.config.camera_name})"
        )

    # ------------------------------------------------------------------
    # Camera interface
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        raise NotImplementedError(
            "Walker cameras require manual configuration "
            "(server_address, port, camera_name)."
        )

    def connect(self, warmup: bool = True) -> None:
        # Start ZMQ SUB connection
        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_socket.connect(f"tcp://{self.config.server_address}:{self.config.port}")
        self._zmq_socket.setsockopt(zmq.RCVHWM, 1)
        self._zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True, name=f"{self}_recv"
        )
        self._recv_thread.start()

        self._connected = True
        logger.info("%s connected.", self)

        if warmup:
            start_time = time.time()
            while time.time() - start_time < self.config.warmup_s:
                try:
                    self.async_read(timeout_ms=1000)
                    break
                except (TimeoutError, RuntimeError):
                    time.sleep(0.1)

            with self._frame_lock:
                frame = self._latest_frame
            if frame is None:
                logger.warning("%s: no frames received during warmup.", self)

    def read(self, color_mode: Any = None) -> NDArray[Any]:
        if color_mode is not None:
            logger.warning("%s: read() color_mode parameter is deprecated.", self)

        self._new_frame_event.clear()
        frame = self.async_read(timeout_ms=10000)
        return frame

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        if self._new_frame_event.wait(timeout=timeout_ms / 1000.0):
            with self._frame_lock:
                frame = self._latest_frame
                self._new_frame_event.clear()
            if frame is None:
                raise RuntimeError(f"{self} has no frame available.")
            return frame

        raise TimeoutError(f"{self} async_read timeout after {timeout_ms}ms")

    def read_latest(self, max_age_ms: int = 1000) -> NDArray[Any]:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        with self._frame_lock:
            frame = self._latest_frame
            timestamp = self._latest_timestamp

        if frame is None or timestamp is None:
            raise RuntimeError(f"{self} has not captured any frames yet.")

        age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            raise TimeoutError(
                f"{self} latest frame is too old: {age_ms:.1f}ms (max: {max_age_ms}ms)."
            )

        return frame

    def disconnect(self) -> None:
        if not self._connected:
            return

        # Stop receive thread
        self._running = False
        if self._recv_thread is not None and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=3.0)
            self._recv_thread = None

        # Close ZMQ
        if self._zmq_socket is not None:
            self._zmq_socket.close()
            self._zmq_socket = None
        if self._zmq_context is not None:
            self._zmq_context.term()
            self._zmq_context = None

        with self._frame_lock:
            self._latest_frame = None
            self._latest_timestamp = None
            self._new_frame_event.clear()

        self._connected = False
        logger.info("%s disconnected.", self)

    # ------------------------------------------------------------------
    # Background receive loop
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        """Background loop: receive JSON with base64 JPEG → decode → buffer."""
        while self._running:
            try:
                msg = self._zmq_socket.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(msg)

                # Extract this camera's frame
                images = data.get("images", {})
                jpeg_b64 = images.get(self.config.camera_name)
                if jpeg_b64 is None:
                    continue

                # Decode base64 → JPEG bytes → numpy image
                jpeg_bytes = base64.b64decode(jpeg_b64)
                np_img = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
                if frame is None:
                    logger.warning("%s: failed to decode JPEG frame.", self)
                    continue

                # Match the configured LeRobot camera shape.
                if frame.shape[1] != self.config.width or frame.shape[0] != self.config.height:
                    frame = cv2.resize(frame, (self.config.width, self.config.height))

                # Store in thread-safe buffer
                capture_time = time.perf_counter()
                with self._frame_lock:
                    self._latest_frame = frame
                    self._latest_timestamp = capture_time
                self._new_frame_event.set()

            except zmq.Again:
                time.sleep(0.001)
            except (json.JSONDecodeError, KeyError) as e:
                logger.warning("%s: message parse error: %s", self, e)
                time.sleep(0.01)
            except Exception as e:
                logger.warning("%s: recv error: %s", self, e)
                time.sleep(0.01)
