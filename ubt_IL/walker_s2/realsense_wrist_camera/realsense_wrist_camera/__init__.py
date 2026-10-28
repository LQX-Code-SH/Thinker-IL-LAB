"""realsense_wrist_camera — Intel RealSense D405 wrist camera ROS2 publisher.

Provides:
- RealSenseD405Driver: single camera capture driver (pyrealsense2 wrapper)
- RealSenseWristCameraNode: ROS2 node that publishes frames from one or more cameras
- CLI entry points: realsense-wrist-camera, find-realsense-cameras
"""

__version__ = "0.1.0"

from .driver import RealSenseD405Driver
from .node import RealSenseWristCameraNode

__all__ = ["RealSenseD405Driver", "RealSenseWristCameraNode"]
