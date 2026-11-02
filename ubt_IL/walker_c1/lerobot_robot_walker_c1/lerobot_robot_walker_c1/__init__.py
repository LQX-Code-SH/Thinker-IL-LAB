"""Walker C1 (Astron) robot plugin for LeRobot.

Provides WalkerC1Robot (dual-arm + 6-channel dexterous hands, 26-DOF policy
interface) and WalkerC1Camera for deployment through lerobot-rollout.

Importing this package registers the config classes with LeRobot's
ChoiceRegistry, so ``--robot.type=walker_c1`` and
``--robot.cameras.<key>.type=walker_c1_camera`` work on the CLI.
"""

from .camera import WalkerC1Camera, WalkerC1CameraConfig
from .config_walker_c1 import WalkerC1RobotConfig
from .walker_c1 import WalkerC1Robot

__all__ = [
    "WalkerC1Robot",
    "WalkerC1RobotConfig",
    "WalkerC1Camera",
    "WalkerC1CameraConfig",
]
