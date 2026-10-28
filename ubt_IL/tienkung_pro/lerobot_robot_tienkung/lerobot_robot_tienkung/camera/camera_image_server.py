"""ImageServerCamera — LeRobot Camera backed by a ZMQ ImageServer.

Receives JPEG frames from ImageServer (scripts/deploy/tienkung_pro/image_server.py) via
ZMQ SUB, decodes them, and extracts this camera's portion by offset.

Inherits:
    - Camera: LeRobot standard camera interface
    - ZMQImageReceiver: ZMQ SUB + JPEG decode + offset split + display
      (extracted from scripts/deploy/tienkung_pro/image_client.py)
"""

from __future__ import annotations

import logging
import time
from typing import Any

from numpy.typing import NDArray

from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import ColorMode

from .config_image_server import ImageServerCameraConfig
from .zmq_image_receiver import ZMQImageReceiver

logger = logging.getLogger(__name__)


class ImageServerCamera(Camera, ZMQImageReceiver):
    """Camera that receives frames from an ImageServer via ZMQ.

    Architecture:
        ImageServer (ZMQ PUB :5555, JPEG)
            → ZMQImageReceiver (ZMQ SUB, decode, offset split, display)
                → ImageServerCamera (Camera interface)

    Multi-part ZMQ message format is NOT used — ImageServer sends a single
    JPEG byte buffer per frame. The full concatenated frame is decoded, and
    this camera extracts its portion using offset_x and width.
    """

    def __init__(self, config: ImageServerCameraConfig):
        Camera.__init__(self, config)
        ZMQImageReceiver.__init__(
            self,
            server_address=config.server_address,
            port=config.port,
            offset_x=config.offset_x,
            width=config.width or 640,
            height=config.height or 480,
            display=config.display,
        )
        self.config = config
        self.color_mode = config.color_mode
        self.timeout_ms = config.timeout_ms
        self._connected = False

    def __str__(self) -> str:
        return (
            f"ImageServerCamera(zmq://{self.config.server_address}:"
            f"{self.config.port}, offset={self.config.offset_x})"
        )

    # ------------------------------------------------------------------
    # Camera interface — delegates to ZMQImageReceiver
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        raise NotImplementedError(
            "ImageServer cameras require manual configuration "
            "(server_address, port, offset_x)."
        )

    def connect(self, warmup: bool = True) -> None:
        # Start ZMQ SUB receiver
        self.start()
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

            frame, _ = self.get_latest_frame()
            if frame is None:
                logger.warning("%s: no frames received during warmup.", self)

    def read(self, color_mode: ColorMode | None = None) -> NDArray[Any]:
        if color_mode is not None:
            logger.warning("%s: read() color_mode parameter is deprecated.", self)

        self._new_frame_event.clear()
        frame = self.async_read(timeout_ms=10000)
        return frame

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        frame, _ = self.wait_for_new_frame(timeout_ms)
        if frame is None:
            raise TimeoutError(f"{self} async_read timeout after {timeout_ms}ms")

        return frame

    def read_latest(self, max_age_ms: int = 1000) -> NDArray[Any]:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected.")

        frame, timestamp = self.get_latest_frame()
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

        self.stop()
        self._connected = False
        logger.info("%s disconnected.", self)
