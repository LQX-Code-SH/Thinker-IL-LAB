"""Configuration for WalkerCamera.

Registers as CameraConfig type "walker_camera" so that
--robot.cameras.<key>.type=walker_camera works on the CLI.
"""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig, ColorMode


@CameraConfig.register_subclass("walker_camera")
@dataclass
class WalkerCameraConfig(CameraConfig):
    """Configuration for a camera that receives frames from Walker Bridge2 via ZMQ.

    The Walker Bridge2 process subscribes to shm_msgs camera topics on ROS2,
    decodes images, re-encodes as JPEG, and publishes them over ZMQ PUB.
    WalkerCamera connects to this ZMQ endpoint and receives JPEG frames.

    Attributes:
        server_address: ZMQ server address (Bridge2 PUB endpoint).
        port: ZMQ server port (default 5563, matches Walker Bridge2 image port).
        camera_name: Camera identifier key used in the ZMQ JSON message
                     to select this camera's frame from multi-camera messages.
        color_mode: Output color format (BGR by default, matching cv2.imdecode).
        timeout_ms: Timeout for async reads in milliseconds.
        warmup_s: Seconds to wait for the first frame during warmup.
    """
    server_address: str = "127.0.0.1"
    port: int = 5563
    camera_name: str = "stereo_color"
    color_mode: ColorMode = ColorMode.BGR
    timeout_ms: int = 5000
    warmup_s: int = 3
