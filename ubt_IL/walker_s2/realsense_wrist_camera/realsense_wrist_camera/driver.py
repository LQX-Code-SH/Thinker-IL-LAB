"""RealSense D405 camera driver — pyrealsense2 pipeline wrapper.

Usage:
    driver = RealSenseD405Driver(serial="...", width=640, height=480, fps=15)
    driver.start()
    img = driver.get_frame()       # numpy BGR array or None
    driver.stop()
"""

import logging
from typing import Optional

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None  # handled at start() time

logger = logging.getLogger(__name__)


class RealSenseD405Driver:
    """Single Intel RealSense D405 camera driver.

    Parameters
    ----------
    serial : str
        Camera serial number (found via find-realsense-cameras or rs.context()).
    width : int
        Requested frame width (default 640).
    height : int
        Requested frame height (default 480).
    fps : int
        Requested frame rate (default 15).
    """

    def __init__(
        self,
        serial: str,
        width: int = 640,
        height: int = 480,
        fps: int = 60,
    ):
        if rs is None:
            raise ImportError(
                "pyrealsense2 is required for RealSenseD405Driver. "
                "Install it with: pip install pyrealsense2"
            )

        self._serial = serial
        self._width = width
        self._height = height
        self._fps = fps

        self._pipeline: Optional[rs.pipeline] = None
        self._profile = None
        self._is_running = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the device and start streaming."""
        if self._is_running:
            return

        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self._serial)
        config.enable_stream(
            rs.stream.color,
            self._width,
            self._height,
            rs.format.bgr8,
            self._fps,
        )

        logger.info(
            "Starting RealSense D405 SN=%s at %dx%d@%d FPS",
            self._serial, self._width, self._height, self._fps,
        )

        self._profile = self._pipeline.start(config)

        # Verify the device is accessible
        device = self._profile.get_device()
        if device is None:
            self._pipeline.stop()
            self._pipeline = None
            raise RuntimeError(
                f"Failed to get device for RealSense D405 SN={self._serial}. "
                f"Check USB connection and permissions."
            )

        self._is_running = True

    def stop(self) -> None:
        """Stop streaming and release the device."""
        self._is_running = False
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as e:
                logger.debug("Error stopping pipeline SN=%s: %s", self._serial, e)
            self._pipeline = None
            self._profile = None

    # ------------------------------------------------------------------
    # Frame capture
    # ------------------------------------------------------------------

    def get_frame(self, timeout_ms: int = 5000) -> Optional[np.ndarray]:
        """Wait for and return the next color frame.

        Parameters
        ----------
        timeout_ms : int
            Maximum time to wait for a frame (milliseconds).

        Returns
        -------
        numpy.ndarray or None
            BGR image (height, width, 3), dtype uint8. None on timeout or error.
        """
        if not self._is_running or self._pipeline is None:
            return None

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=timeout_ms)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            return np.asanyarray(color_frame.get_data())
        except Exception as e:
            logger.warning(
                "Failed to capture frame from RealSense SN=%s: %s",
                self._serial, e,
            )
            return None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_running(self) -> bool:
        """Whether the pipeline is currently streaming."""
        return self._is_running

    @property
    def serial(self) -> str:
        return self._serial

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def fps(self) -> int:
        return self._fps

    # ------------------------------------------------------------------
    # Static: device discovery
    # ------------------------------------------------------------------

    @staticmethod
    def discover() -> list[dict]:
        """Discover all connected RealSense devices.

        Returns
        -------
        list[dict]
            Each entry: {'serial': str, 'name': str, 'usb_type': str,
                         'firmware': str, 'product_line': str}
        """
        if rs is None:
            logger.warning("pyrealsense2 not available; cannot discover devices.")
            return []

        devices = []
        for dev in rs.context().query_devices():
            devices.append({
                "serial": dev.get_info(rs.camera_info.serial_number),
                "name": dev.get_info(rs.camera_info.name),
                "usb_type": dev.get_info(rs.camera_info.usb_type_descriptor),
                "firmware": dev.get_info(rs.camera_info.firmware_version),
                "product_line": dev.get_info(rs.camera_info.product_line),
            })
        return devices
