"""TienKung Dex robot plugin for LeRobot.

Provides TienKungDexRobot (right arm + right dexterous hand) and DexCamera
for deployment through the lerobot-rollout framework.

When this package is imported, the config classes register themselves with
LeRobot's ChoiceRegistry so that `--robot.type=tienkung_dex` and
`--robot.cameras.<key>.type=dex_camera` work on the CLI.
"""

from .camera import DexCamera, DexCameraConfig, ZMQImageReceiver
from .config_tienkung_dex import TienKungDexRobotConfig
from .tienkung_dex import TienKungDexRobot

__all__ = [
    "TienKungDexRobot",
    "TienKungDexRobotConfig",
    "DexCamera",
    "DexCameraConfig",
    "ZMQImageReceiver",
]
