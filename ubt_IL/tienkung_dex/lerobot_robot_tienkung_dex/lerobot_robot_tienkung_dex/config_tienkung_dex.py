from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig

from .constants import (
    ARM_JOINTS,
    DOF_HAND_ACTION_MODE,
    DOF_HAND_VARIANT,
    GROUP_CMD_LIMITS,
    HEAD_JOINTS,
    HOME_POSE,
    JOINT_INDEX_ENUMS,
    RIGHT_ARM_JOINTS,
    RIGHT_HAND_JOINTS_BY_VARIANT,
    TOPIC_ARM_CMD,
    TOPIC_ESTOP,
    TOPIC_HAND_CMD_BY_VARIANT,
    TOPIC_IMAGE_HEAD,
    TOPIC_ROBOT_STATE,
    hand_channel_joints,
    hand_open_pose,
    inactive_fill_for,
    joint_names_with_pos,
    pico6_action_joints,
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
    # 同时决定手型：tienkung_dex_20 → inspire，tienkung_dex_brainco_13 → brainco，
    # tienkung_dex_inspire_13 → inspire（r13 数据集 6 维归一化，pico6 展开）。
    joint_config: str = "tienkung_dex_20"

    # 手型，留空表示跟随 joint_config（推荐）。显式填写时必须与 joint_config 一致，
    # 防止"用因时数据的模型去驱动强脑手"这类配错。
    hand_variant: str = ""

    # 因时手策略侧动作量纲。留 "auto" = 由 joint_config 推导（推荐）：
    #   rad13 = 20D 老口径，ZMQ right_hand 是 13 维关节角 rad；
    #   pico6 = r13 数据集，ZMQ right_hand 是 6 维归一化 0~1，Bridge2 展开成 13 个角码。
    # 显式填写时必须与 joint_config 一致。
    hand_action_mode: str = "auto"

    # Joint group definitions (固定物理通道顺序，bridge 按位寻址，严禁重排)
    right_arm_joints: list[str] = field(default_factory=lambda: _pos(RIGHT_ARM_JOINTS))
    # 硬件物理通道名（因时 13 / 强脑 6），传给 Bridge2 做 _assert_hand_table 比对。
    right_hand_joints: list[str] = field(default_factory=list)  # 由 hand_variant 派生
    # 策略侧手列名（send_action / get_observation 用，带 .pos）。rad13 时 =
    # right_hand_joints；pico6 时 = r13 数据集 6 个列名。
    right_hand_action_joints: list[str] = field(default_factory=list)
    left_hand_action_joints: list[str] = field(default_factory=list)

    # ---- 任意组组合（可选）：左侧 / 头部默认关闭，13D/20D 行为完全不变 ----
    # 关节级颗粒度：*_joints 与 *_motor_ids 一一对应，可为子集（臂/头部）；
    # 手部通道是定长数组，只能整只手启停。
    enable_left_arm: bool = False
    enable_left_hand: bool = False
    enable_head: bool = False
    left_arm_joints: list[str] = field(default_factory=lambda: _pos(ARM_JOINTS["left"]))
    left_arm_motor_ids: list[int] = field(default_factory=lambda: list(range(11, 18)))
    left_hand_joints: list[str] = field(default_factory=list)   # 由 left_hand_variant 派生
    left_hand_variant: str = ""  # 留空 = 跟随 hand_variant（左右手可不同型）
    left_hand_open_position: list[float] = field(default_factory=list)
    head_joints: list[str] = field(default_factory=lambda: _pos(HEAD_JOINTS))
    head_motor_ids: list[int] = field(default_factory=lambda: [1, 2])
    topic_head_cmd: str = "/head/cmd"

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
    topic_left_hand_cmd: str = ""   # 留空 = 按 left_hand_variant 选

    # ROS2 topic names (status side / subscribe)
    topic_robot_state: str = TOPIC_ROBOT_STATE
    topic_estop: str = TOPIC_ESTOP

    # 头相机。仿真里 ubt_sim 的 AsyncJpegCameraSender 直接在 image_port 发 JPEG，
    # 相机侧直连即可（image_enabled=False）；真机没有这条链路，由 Bridge2 把
    # /ob_camera_head/color/image_raw 编成 JPEG 转到 image_port（image_enabled=True）。
    image_enabled: bool = False
    image_port: int = 5558
    topic_image_head: str = TOPIC_IMAGE_HEAD
    # Bridge2 编码前的缩放目标（0 = 不缩放）。真机 /ob_camera_head 是 1280x720，
    # 而 dex_camera 只切列不缩放，必须在这里让桥接先缩到策略训练分辨率（640x360）。
    # 必须与 cameras[...] 里的 width/height 一致。
    image_width: int = 0
    image_height: int = 0
    # True = Bridge2 订阅 .../image_raw/compressed（真机推荐）。
    # 实测 raw 1280x720 的 rclpy 反序列化 ~35ms/帧 → 只能转出 10~16fps；
    # compressed（~386KB）解码+缩放+重编码 ~11ms/帧 → 稳定 ~20fps。
    image_compressed: bool = False

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

        # 策略侧手动作量纲：auto 时按 joint_config 推导
        # （tienkung_dex_inspire_13 自动走 pico6，无需额外开关）。
        expected_mode = DOF_HAND_ACTION_MODE.get(self.joint_config, "rad13")
        if self.hand_action_mode == "auto":
            self.hand_action_mode = expected_mode
        elif self.hand_action_mode not in ("rad13", "pico6"):
            raise ValueError(
                f"hand_action_mode 须为 'auto' | 'rad13' | 'pico6'，"
                f"got {self.hand_action_mode!r}"
            )
        elif self.hand_action_mode != expected_mode:
            raise ValueError(
                f"hand_action_mode={self.hand_action_mode!r} 与 "
                f"joint_config={self.joint_config!r} 不匹配（应为 {expected_mode!r}）"
            )
        self._pico6 = self.hand_action_mode == "pico6"
        if self._pico6 and self.hand_variant != "inspire":
            raise ValueError("hand_action_mode=pico6 只能用于因时手（hand_variant=inspire）")

        if not self.right_hand_joints:
            self.right_hand_joints = _pos(RIGHT_HAND_JOINTS_BY_VARIANT[self.hand_variant])
        # 策略侧手列名：pico6 是 r13 数据集的 6 个列名，rad13 就是物理通道名。
        # 硬件通道名（right_hand_joints）始终不变，供 Bridge2 逐字比对。
        if not self.right_hand_action_joints:
            self.right_hand_action_joints = (
                _pos(pico6_action_joints("right")) if self._pico6
                else list(self.right_hand_joints)
            )
        if not self.topic_right_hand_cmd:
            self.topic_right_hand_cmd = TOPIC_HAND_CMD_BY_VARIANT[self.hand_variant]
        if not self.hand_open_position:
            if self._pico6:
                # pico6 的"张开"= 6 维全 0（展开后是各通道码上限 = HAND_TARGET）
                self.hand_open_position = [0.0] * len(self.right_hand_action_joints)
            else:
                hand = hand_open_pose(self.hand_variant)
                self.hand_open_position = [
                    hand[name.removesuffix(".pos")] for name in self.right_hand_joints
                ]

        self._inactive_fill = inactive_fill_for(self.hand_variant, self.joint_config, enum_cls)

        # --- 左侧 / 头部（可选组合）：左侧手型留空则跟随右手 ---
        if not self.left_hand_variant:
            self.left_hand_variant = self.hand_variant
        if self.left_hand_variant not in RIGHT_HAND_JOINTS_BY_VARIANT:
            raise ValueError(
                f"left_hand_variant must be one of {sorted(RIGHT_HAND_JOINTS_BY_VARIANT)}, "
                f"got {self.left_hand_variant!r}"
            )
        if self.enable_left_hand and not self.left_hand_joints:
            self.left_hand_joints = _pos([
                joints[0] for joints in hand_channel_joints(self.left_hand_variant, "left")
            ])
        if not self.left_hand_action_joints:
            self.left_hand_action_joints = (
                _pos(pico6_action_joints("left")) if self._pico6
                else list(self.left_hand_joints)
            )
        if not self.topic_left_hand_cmd:
            self.topic_left_hand_cmd = TOPIC_HAND_CMD_BY_VARIANT[self.left_hand_variant].replace(
                "/right_hand/", "/left_hand/"
            )
        if self.enable_left_hand and not self.left_hand_open_position:
            if self._pico6:
                self.left_hand_open_position = [0.0] * len(self.left_hand_action_joints)
            else:
                hand = hand_open_pose(self.left_hand_variant)
                self.left_hand_open_position = [
                    hand[name.removesuffix(".pos")] for name in self.left_hand_joints
                ]

        if len(self.left_arm_joints) != len(self.left_arm_motor_ids):
            raise ValueError(
                f"left_arm_motor_ids count ({len(self.left_arm_motor_ids)}) must match "
                f"left_arm_joints count ({len(self.left_arm_joints)})"
            )
        if len(self.head_joints) != len(self.head_motor_ids):
            raise ValueError(
                f"head_motor_ids count ({len(self.head_motor_ids)}) must match "
                f"head_joints count ({len(self.head_joints)})"
            )
        if self.enable_left_hand and len(self.left_hand_open_position) != len(self.left_hand_action_joints):
            raise ValueError(
                f"left_hand_open_position count ({len(self.left_hand_open_position)}) must match "
                f"left_hand_action_joints count ({len(self.left_hand_action_joints)})"
            )

        # Validate all_joints is a subset of the hardware groups (no extra joints).
        # pico6 下策略手列名是 6 个 r13 列名，不是 13 个物理通道名，必须按策略侧校验。
        group_set = set(self.right_arm_joints + self.right_hand_action_joints)
        if self.enable_left_arm:
            group_set |= set(self.left_arm_joints)
        if self.enable_left_hand:
            group_set |= set(self.left_hand_action_joints)
        if self.enable_head:
            group_set |= set(self.head_joints)
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

        # Validate hand_open_position matches the policy-side hand action dim
        if len(self.hand_open_position) != len(self.right_hand_action_joints):
            raise ValueError(
                f"hand_open_position count ({len(self.hand_open_position)}) must match "
                f"right_hand_action_joints count ({len(self.right_hand_action_joints)})"
            )
        if self._pico6 and len(self.right_hand_action_joints) != 6:
            raise ValueError(
                f"pico6 需要 6 个策略手列名，实际 {len(self.right_hand_action_joints)}"
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
            # 因时手动作量纲：pico6 = 6 维 0~1 展开成 13 码；rad13 = 老 13 维 rad。
            "hand_action_mode": self.hand_action_mode,
            "right_arm_motor_ids": self.right_arm_motor_ids,
            # 帧内关节序（物理通道序）。桥接拿它与自己的通道表逐字比对，
            # 不一致直接拒启——两边的常量副本漂移时这是唯一能拦住的检查。
            "right_hand_joints": self.right_hand_joints,
            # pico6 的 6 维策略列名（r13 数据集顺序），桥接启动逐字比对；
            # rad13 时与 right_hand_joints 相同，桥接不会使用。
            "right_hand_action_joints": self.right_hand_action_joints,
            "left_hand_action_joints": self.left_hand_action_joints,
            # 可选组：左侧 / 头部（组级开关 + 关节级列表，可为子集）
            "enable_left_arm": self.enable_left_arm,
            "enable_left_hand": self.enable_left_hand,
            "enable_head": self.enable_head,
            "left_arm_joints": self.left_arm_joints,
            "left_arm_motor_ids": self.left_arm_motor_ids,
            "left_hand_joints": self.left_hand_joints,
            "left_hand_variant": self.left_hand_variant,
            "head_joints": self.head_joints,
            "head_motor_ids": self.head_motor_ids,
            "topic_left_hand_cmd": self.topic_left_hand_cmd,
            "topic_head_cmd": self.topic_head_cmd,
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
            "image_width": self.image_width,
            "image_height": self.image_height,
            "image_compressed": self.image_compressed,
        }
