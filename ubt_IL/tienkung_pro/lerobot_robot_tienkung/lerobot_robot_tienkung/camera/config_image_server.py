"""Configuration for ImageServerCamera.

Registers as CameraConfig type "image_server" so that
--robot.cameras.<key>.type=image_server works on the CLI.
"""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig, ColorMode


@CameraConfig.register_subclass("image_server")
@dataclass
class ImageServerCameraConfig(CameraConfig):
    """Configuration for a camera that receives frames from ImageServer via ZMQ.

    The ImageServer (scripts/deploy/tienkung_pro/image_server.py) publishes concatenated
    JPEG frames over ZMQ PUB. Each ImageServerCamera extracts its own portion
    of the frame using offset_x + width.

    Attributes:
        server_address: ZMQ server address (ImageServer PUB endpoint).
        port: ZMQ server port (default 5558, matches ImageServer deployment port).
        offset_x: Horizontal pixel offset in the concatenated frame where
                  this camera's image begins.
        display: Whether to show a real-time cv2.imshow popup window.
        color_mode: Output color format (BGR by default, matching cv2.imdecode).
        timeout_ms: Timeout for async reads in milliseconds.
        warmup_s: Seconds to wait for the first frame during warmup.
    """

    server_address: str = "127.0.0.1"
    port: int = 5558
    offset_x: int = 0
    display: bool = False
    color_mode: ColorMode = ColorMode.BGR
    timeout_ms: int = 5000
    warmup_s: int = 3
