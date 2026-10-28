from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig

from .constants import (
    ARM_HOME,
    CANONICAL_JOINT_NAMES,
    DEFAULT_ARM_CURRENT,
    DEFAULT_ARM_SPEED,
    DEFAULT_RESET_CURRENT,
    DEFAULT_RESET_SPEED,
    HAND_OPEN,
    ID_ARM_L,
    ID_ARM_R,
    JOINT_INDEX_ENUMS,
    LEFT_ARM_JOINTS,
    LEFT_HAND_JOINTS,
    RIGHT_ARM_JOINTS,
    RIGHT_HAND_JOINTS,
    TOPIC_ARM_CMD,
    TOPIC_ARM_STATUS,
    TOPIC_HEAD_CMD,
    TOPIC_LEFT_HAND_CMD,
    TOPIC_LEFT_HAND_STATUS,
    TOPIC_RIGHT_HAND_CMD,
    TOPIC_RIGHT_HAND_STATUS,
    inactive_fill_for,
    joint_names_with_pos,
)


def _pos(names: list[str]) -> list[str]:
    """Append LeRobot-required ``.pos`` suffix to semantic joint names."""
    return [f"{n}.pos" for n in names]


@RobotConfig.register_subclass("tienkung")
@dataclass
class TienKungRobotConfig(RobotConfig):
    # ZMQ configuration (LeRobot ↔ Bridge2 internal communication)
    zmq_host: str = "127.0.0.1"
    zmq_cmd_port: int = 5559       # LeRobot PUB → Bridge2 SUB
    zmq_status_port: int = 5560    # Bridge2 PUB → LeRobot SUB
    bridge_enabled: bool = True     # Auto-start Bridge2 subprocess
    bridge_script: str = "/ubt_IL/tienkung_pro/ros2_deploy_bridge.py"  # Path to Bridge2 script (bind-mounted)

    # ROS2 topics (real robot defaults)
    ros_namespace: str = ""
    cmd_namespace: str = ""

    # Hand type: "inspire" (currently the only supported type)
    hand_type: str = "inspire"

    # Joint group definitions (固定物理 motor/手指顺序，bridge 按位寻址，严禁重排)
    left_arm_joints: list[str] = field(default_factory=lambda: _pos(LEFT_ARM_JOINTS))
    right_arm_joints: list[str] = field(default_factory=lambda: _pos(RIGHT_ARM_JOINTS))
    left_hand_joints: list[str] = field(default_factory=lambda: _pos(LEFT_HAND_JOINTS))
    right_hand_joints: list[str] = field(default_factory=lambda: _pos(RIGHT_HAND_JOINTS))

    # DOF 配置名（取自 JOINT_INDEX_ENUMS）。决定 all_joints 的维度与顺序（= 数据集顺序）。
    # 默认 "tienkung_26"（全 26，物理序）。部署 13-DOF 模型时设为 "tienkung_13"。
    # 新增 DOF：在 constants.py 定义 IntEnum 并注册到 JOINT_INDEX_ENUMS，然后设此字段即可。
    joint_config: str = "tienkung_26"

    # Full joint ordering (determines model action/observation tensor dimension mapping).
    # 默认全 26（物理序）；__post_init__ 会按 joint_config 从对应 DOF 枚举派生覆盖。
    # 枚举成员顺序须与数据集 action/state 顺序一致（可重排），成员名须取自 4 分组并集。
    all_joints: list[str] = field(default_factory=lambda: _pos(CANONICAL_JOINT_NAMES))

    # Motor IDs (1:1 mapping with arm joint groups, used by Bridge2 to address hardware)
    left_arm_motor_ids: list[int] = field(default_factory=lambda: list(ID_ARM_L))
    right_arm_motor_ids: list[int] = field(default_factory=lambda: list(ID_ARM_R))

    # Arm motion parameters
    arm_speed: float = DEFAULT_ARM_SPEED
    arm_current: float = DEFAULT_ARM_CURRENT
    reset_speed: float = DEFAULT_RESET_SPEED
    reset_current: float = DEFAULT_RESET_CURRENT

    # Hand open position (after Inspire clip, these values open the hand)
    hand_open_position: list[float] = field(default_factory=lambda: list(HAND_OPEN))

    # ROS2 topic names (command side / publish)
    topic_arm_cmd: str = TOPIC_ARM_CMD
    topic_head_cmd: str = TOPIC_HEAD_CMD
    topic_left_hand_cmd: str = TOPIC_LEFT_HAND_CMD
    topic_right_hand_cmd: str = TOPIC_RIGHT_HAND_CMD

    # ROS2 topic names (status side / subscribe)
    topic_arm_status: str = TOPIC_ARM_STATUS
    topic_left_hand_status: str = TOPIC_LEFT_HAND_STATUS
    topic_right_hand_status: str = TOPIC_RIGHT_HAND_STATUS

    # Safety
    max_relative_target: float | None = None
    # 推理结束 disconnect() 时是否回到 home_position（默认 False：不归位）。
    # 注：lerobot 其它机器人的 disable_torque_on_disconnect 语义为"下力矩"，
    # tienkung 无下力矩逻辑，此字段仅控制回零，故改名以正语义。
    return_home_on_disconnect: bool = False

    # Home position (14-dim: left arm 7 + right arm 7)
    home_position: list[float] = field(default_factory=lambda: list(ARM_HOME))

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
        self._inactive_fill = inactive_fill_for(self.joint_config, enum_cls)

        # Validate all_joints is a subset of the union of 4 joint groups (no extra joints)
        group_set = set(self.left_arm_joints + self.right_arm_joints
                        + self.left_hand_joints + self.right_hand_joints)
        all_set = set(self.all_joints)
        if not all_set.issubset(group_set):
            raise ValueError(
                f"all_joints contains joints not in the 4 hardware groups. "
                f"Unknown: {all_set - group_set}"
            )
        if len(self.all_joints) != len(all_set):
            raise ValueError(
                f"all_joints has duplicates: {len(self.all_joints)} items vs {len(all_set)} unique"
            )

        # Validate hand_type
        if self.hand_type not in ("inspire",):
            raise ValueError(
                f"hand_type must be 'inspire', got {self.hand_type!r}"
            )

        # Validate motor ID counts match joint counts
        if len(self.left_arm_motor_ids) != len(self.left_arm_joints):
            raise ValueError(
                f"left_arm_motor_ids count ({len(self.left_arm_motor_ids)}) must match "
                f"left_arm_joints count ({len(self.left_arm_joints)})"
            )
        if len(self.right_arm_motor_ids) != len(self.right_arm_joints):
            raise ValueError(
                f"right_arm_motor_ids count ({len(self.right_arm_motor_ids)}) must match "
                f"right_arm_joints count ({len(self.right_arm_joints)})"
            )

        # Validate hand_open_position matches hand joint count
        if len(self.hand_open_position) != len(self.left_hand_joints):
            raise ValueError(
                f"hand_open_position count ({len(self.hand_open_position)}) must match "
                f"left_hand_joints count ({len(self.left_hand_joints)})"
            )

        # Validate home_position dimension
        expected_home_dim = len(self.left_arm_joints) + len(self.right_arm_joints)
        if len(self.home_position) != expected_home_dim:
            raise ValueError(
                f"home_position count ({len(self.home_position)}) must match "
                f"left_arm + right_arm joint count ({expected_home_dim})"
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
            "left_arm_motor_ids": self.left_arm_motor_ids,
            "right_arm_motor_ids": self.right_arm_motor_ids,
            "arm_speed": self.arm_speed,
            "arm_current": self.arm_current,
            "hand_type": self.hand_type,
            "hand_open_position": self.hand_open_position,
            "topic_arm_cmd": self.topic_arm_cmd,
            "topic_head_cmd": self.topic_head_cmd,
            "topic_left_hand_cmd": self.topic_left_hand_cmd,
            "topic_right_hand_cmd": self.topic_right_hand_cmd,
            "topic_arm_status": self.topic_arm_status,
            "topic_left_hand_status": self.topic_left_hand_status,
            "topic_right_hand_status": self.topic_right_hand_status,
        }
