"""Walker S2 robot plugin for LeRobot.

Provides WalkerRobot (dual-arm + V4 dexterous hands humanoid) and WalkerCamera
for deployment through the lerobot-rollout framework.

When this package is imported, the config classes register themselves with
LeRobot's ChoiceRegistry so that `--robot.type=walker` and
`--robot.cameras.<key>.type=walker_camera` work on the CLI.
"""

from .camera import WalkerCamera, WalkerCameraConfig
from .config_walker import WalkerRobotConfig
from .walker import WalkerRobot

__all__ = [
    "WalkerRobot",
    "WalkerRobotConfig",
    "WalkerCamera",
    "WalkerCameraConfig",
]
