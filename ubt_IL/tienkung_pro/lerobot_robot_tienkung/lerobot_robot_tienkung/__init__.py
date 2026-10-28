"""TienKung robot plugin for LeRobot.

Provides TienKungRobot (dual-arm + Inspire dexterous hands) and ImageServerCamera
for deployment through the lerobot-rollout framework.

When this package is imported, the config classes register themselves with
LeRobot's ChoiceRegistry so that `--robot.type=tienkung` and
`--robot.cameras.<key>.type=image_server` work on the CLI.
"""

from .camera import ImageServerCamera, ImageServerCameraConfig, ZMQImageReceiver
from .config_tienkung import TienKungRobotConfig
from .tienkung import TienKungRobot

__all__ = [
    "TienKungRobot",
    "TienKungRobotConfig",
    "ImageServerCamera",
    "ImageServerCameraConfig",
    "ZMQImageReceiver",
]
