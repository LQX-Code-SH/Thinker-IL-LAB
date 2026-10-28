from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig

from .constants import (
    DOF_HAND_VARIANT,
    GROUP_CMD_LIMITS,
    HOME_POSE,
    JOINT_INDEX_ENUMS,
    RIGHT_ARM_JOINTS,
    RIGHT_HAND_JOINTS_BY_VARIANT,
    TOPIC_ARM_CMD,
    TOPIC_ESTOP,
    TOPIC_HAND_CMD_BY_VARIANT,
    TOPIC_IMAGE_HEAD,
    TOPIC_ROBOT_STATE,
    hand_open_pose,
    inactive_fill_for,
    joint_names_with_pos,
)

# 右臂站立位（7 维），= HOME_POSE 的右臂子集
RIGHT_ARM_HOME = [HOME_POSE[name] for name in RIGHT_ARM_JOINTS]


def _pos(names: list[str]) -> list[str]:
    """Append LeRobot-required ``.pos`` suffix to semantic joint names."""
    return [f"{n}.pos" for n in names]


@RobotConfig.register_subclass("tienkung_dex")
@dataclass
class TienKungDexRobotConfig(RobotConfig):
    # ZMQ configuration (LeRobot ↔ Bridge2 internal communication)
    zmq_host: str = "127.0.0.1"
    zmq_cmd_port: int = 5559       # LeRobot PUB → Bridge2 SUB
    zmq_status_port: int = 5560    # Bridge2 PUB → LeRobot SUB
    bridge_enabled: bool = True     # Auto-start Bridge2 subprocess
    bridge_script: str = "/ubt_IL/tienkung_dex/ros2_deploy_bridge.py"  # Path to Bridge2 script (bind-mounted)

    # ROS2 namespaces（真机多机共存时用；仿真留空）
    ros_namespace: str = ""
    cmd_namespace: str = ""

    # DOF 配置名（取自 JOINT_INDEX_ENUMS）。决定 all_joints 的维度与顺序（= 数据集顺序），
    # 同时决定手型：tienkung_dex_20 → inspire，tienkung_dex_brainco_13 → brainco。
    joint_config: str = "tienkung_dex_20"

    # 手型，留空表示跟随 joint_config（推荐）。显式填写时必须与 joint_config 一致，
    # 防止"用因时数据的模型去驱动强脑手"这类配错。
    hand_variant: str = ""

    # Joint group definitions (固定物理通道顺序，bridge 按位寻址，严禁重排)
    right_arm_joints: list[str] = field(default_factory=lambda: _pos(RIGHT_ARM_JOINTS))
    right_hand_joints: list[str] = field(default_factory=list)  # 由 hand_variant 派生

    # Full joint ordering (determines model action/observation tensor dimension mapping).
    # __post_init__ 会按 joint_config 从对应 DOF 枚举派生覆盖。
    all_joints: list[str] = field(default_factory=list)

    # Motor IDs (1:1 mapping with right_arm_joints, used by Bridge2 to address hardware)
    right_arm_motor_ids: list[int] = field(default_factory=lambda: list(range(21, 28)))

    # Arm motion parameters（spd/cur 必须填，不填真机会走 SDK 默认值，行为不可控）
    arm_speed: float = GROUP_CMD_LIMITS["arm"]["spd"]
    arm_current: float = GROUP_CMD_LIMITS["arm"]["cur"]

    # Hand open position（rad，张开/伸直）
    hand_open_position: list[float] = field(default_factory=list)

    # ROS2 topic names (command side / publish)
    topic_arm_cmd: str = TOPIC_ARM_CMD
    topic_right_hand_cmd: str = ""  # 留空 = 按 hand_variant 选（因时/强脑话题名不同）

    # ROS2 topic names (status side / subscribe)
    topic_robot_state: str = TOPIC_ROBOT_STATE
    topic_estop: str = TOPIC_ESTOP

    # 头相机。仿真里 ubt_sim 的 AsyncJpegCameraSender 直接在 image_port 发 JPEG，
    # 相机侧直连即可（image_enabled=False）；真机没有这条链路，由 Bridge2 把
    # /ob_camera_head/color/image_raw 编成 JPEG 转到 image_port（image_enabled=True）。
    image_enabled: bool = False
    image_port: int = 5558
    topic_image_head: str = TOPIC_IMAGE_HEAD

    # Safety
    max_relative_target: float | None = None
    # 推理结束 disconnect() 时是否回 home_position（默认 False：不归位）。
    # 与 tienkung 同名同义：tienkung_dex 无下力矩逻辑，此字段仅控制回 home。
    return_home_on_disconnect: bool = False

    # Home position (7-dim: right arm only)
    home_position: list[float] = field(default_factory=lambda: list(RIGHT_ARM_HOME))

    # Cameras (keyed by name matching policy's expected image key, e.g. "camera_head")
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        # Resolve DOF enum and derive all_joints (policy 维度与顺序 = 数据集顺序)
        # + inactive_fill (非激活关节的静态填充值)。
        if self.joint_config not in JOINT_INDEX_ENUMS:
            raise ValueError(
                f"joint_config {self.joint_config!r} not registered. "
                f"Available: {list(JOINT_INDEX_ENUMS)}. "
                f"新增 DOF 请在 constants.py 定义 IntEnum 并注册到 JOINT_INDEX_ENUMS。"
            )
        enum_cls = JOINT_INDEX_ENUMS[self.joint_config]
        self.all_joints = joint_names_with_pos(enum_cls)

        # 手型：留空跟随 joint_config，显式填写则校验一致
        expected_variant = DOF_HAND_VARIANT[self.joint_config]
        if not self.hand_variant:
            self.hand_variant = expected_variant
        elif self.hand_variant != expected_variant:
            raise ValueError(
                f"hand_variant={self.hand_variant!r} 与 joint_config={self.joint_config!r} "
                f"不匹配（应为 {expected_variant!r}）。留空即自动跟随 joint_config。"
            )
        if self.hand_variant not in RIGHT_HAND_JOINTS_BY_VARIANT:
            raise ValueError(
                f"hand_variant must be one of {sorted(RIGHT_HAND_JOINTS_BY_VARIANT)}, "
                f"got {self.hand_variant!r}"
            )

        if not self.right_hand_joints:
            self.right_hand_joints = _pos(RIGHT_HAND_JOINTS_BY_VARIANT[self.hand_variant])
        if not self.topic_right_hand_cmd:
            self.topic_right_hand_cmd = TOPIC_HAND_CMD_BY_VARIANT[self.hand_variant]
        if not self.hand_open_position:
            hand = hand_open_pose(self.hand_variant)
            self.hand_open_position = [
                hand[name.removesuffix(".pos")] for name in self.right_hand_joints
            ]

        self._inactive_fill = inactive_fill_for(self.hand_variant, self.joint_config, enum_cls)

        # Validate all_joints is a subset of the hardware groups (no extra joints)
        group_set = set(self.right_arm_joints + self.right_hand_joints)
        all_set = set(self.all_joints)
        if not all_set.issubset(group_set):
            raise ValueError(
                f"all_joints contains joints not in the hardware groups. "
                f"Unknown: {all_set - group_set}"
            )
        if len(self.all_joints) != len(all_set):
            raise ValueError(
                f"all_joints has duplicates: {len(self.all_joints)} items vs {len(all_set)} unique"
            )

        # Validate motor ID counts match joint counts
        if len(self.right_arm_motor_ids) != len(self.right_arm_joints):
            raise ValueError(
                f"right_arm_motor_ids count ({len(self.right_arm_motor_ids)}) must match "
                f"right_arm_joints count ({len(self.right_arm_joints)})"
            )

        # Validate hand_open_position matches hand joint count
        if len(self.hand_open_position) != len(self.right_hand_joints):
            raise ValueError(
                f"hand_open_position count ({len(self.hand_open_position)}) must match "
                f"right_hand_joints count ({len(self.right_hand_joints)})"
            )

        # Validate home_position dimension
        if len(self.home_position) != len(self.right_arm_joints):
            raise ValueError(
                f"home_position count ({len(self.home_position)}) must match "
                f"right_arm_joints count ({len(self.right_arm_joints)})"
            )

    def to_bridge_config(self) -> dict:
        """Serialize config fields needed by ros2_deploy_bridge.py (system Python 3.10).

        The bridge runs outside the LeRobot venv and cannot import this class.
        This method produces a JSON-serializable dict passed via --config CLI arg.
        """
        return {
            "zmq_cmd_port": self.zmq_cmd_port,
            "zmq_status_port": self.zmq_status_port,
            "ros_namespace": self.ros_namespace,
            "cmd_namespace": self.cmd_namespace,
            "hand_variant": self.hand_variant,
            "right_arm_motor_ids": self.right_arm_motor_ids,
            # 帧内关节序（物理通道序）。桥接拿它与自己的通道表逐字比对，
            # 不一致直接拒启——两边的常量副本漂移时这是唯一能拦住的检查。
            "right_hand_joints": self.right_hand_joints,
            "arm_speed": self.arm_speed,
            "arm_current": self.arm_current,
            "hand_open_position": self.hand_open_position,
            "topic_arm_cmd": self.topic_arm_cmd,
            "topic_right_hand_cmd": self.topic_right_hand_cmd,
            "topic_robot_state": self.topic_robot_state,
            "topic_estop": self.topic_estop,
            "image_enabled": self.image_enabled,
            "image_port": self.image_port,
            "topic_image_head": self.topic_image_head,
        }
