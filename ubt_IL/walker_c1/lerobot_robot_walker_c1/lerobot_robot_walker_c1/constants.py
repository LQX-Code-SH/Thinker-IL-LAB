"""Walker C1 (Astron) 机器人硬件常量（walker_c1 插件侧）。

数据来源：
- 关节名/顺序：ubt_sim ``teleoperation/control/walker_c1/constants.py``
  （与 ``/mc/sdk/robot_state`` 的 31 个身体关节顺序一致）
- 臂部限位/复位位姿：ubt_sim ``control/walker_c1/constants.py`` × walker_c1 分支
  ``scripts/deploy/configs/walker/walker_c1_26d.json``
- 手部 6 通道：真机 SDK 压缩通道（仿真侧 11 关节由 ubt_sim action_process 展开）

此模块与 ``ros2_walker_c1_bridge.py`` 独立维护——两边运行在不同 Python 环境
（3.12 vs 3.10），不交叉导入，仅通过 ZMQ + JSON 配置通信。
"""

# ============================================================================
# 1. 身体关节（31，与 /mc/sdk/robot_state 的 joint_states 顺序一致）
# ============================================================================
BODY_JOINT_NAMES = [
    # 左臂 (0-6)
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    # 右臂 (7-13)
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    # 头 (14-15)
    "head_yaw_joint",
    "head_pitch_joint",
    # 腰 (16-18)
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    # 左腿 (19-24)
    "L_hip_pitch_joint",
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
    # 右腿 (25-30)
    "R_hip_pitch_joint",
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]

LEFT_ARM_JOINTS = BODY_JOINT_NAMES[0:7]
RIGHT_ARM_JOINTS = BODY_JOINT_NAMES[7:14]
HEAD_JOINTS = BODY_JOINT_NAMES[14:16]
WAIST_JOINTS = BODY_JOINT_NAMES[16:19]
LEG_JOINT_NAMES = BODY_JOINT_NAMES[19:31]

# 策略实际控制的上肢关节（14 臂）。头/腰/腿不进模型，由机器人自身保持。
ACTIVE_BODY_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

# ============================================================================
# 2. C1 灵巧手（真机 SDK 6 通道；仿真 11 关节在 ubt_sim 侧展开）
# ============================================================================
C1_HAND_LEFT_JOINTS = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

C1_HAND_RIGHT_JOINTS = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

# 手部关节限位（rad），短名（去掉 left_/right_ 前缀）。来源：walker_c1_26d.json
C1_HAND_JOINT_LIMITS = {
    "thumb_swing": (0.0, 0.96),
    "thumb_mcp": (0.0, 1.04),
    "index_mcp": (0.0, 1.57),
    "middle_mcp": (0.0, 1.57),
    "ring_mcp": (0.0, 1.57),
    "little_mcp": (0.0, 1.57),
}

# ============================================================================
# 3. 关节限位（rad）与复位位姿
# ============================================================================
BODY_JOINT_LIMITS = {
    "L_shoulder_pitch_joint": (-3.089, 2.565),
    "L_shoulder_roll_joint": (-0.139, 1.884),
    "L_shoulder_yaw_joint": (-2.844, 2.844),
    "L_elbow_pitch_joint": (-2.2689, 0.104),
    "L_elbow_yaw_joint": (-2.844, 2.844),
    "L_wrist_pitch_joint": (-0.68, 0.68),
    "L_wrist_roll_joint": (-1.57, 1.57),
    "R_shoulder_pitch_joint": (-3.089, 2.565),
    "R_shoulder_roll_joint": (-1.884, 0.139),
    "R_shoulder_yaw_joint": (-2.844, 2.844),
    "R_elbow_pitch_joint": (-2.2689, 0.104),
    "R_elbow_yaw_joint": (-2.844, 2.844),
    "R_wrist_pitch_joint": (-0.68, 0.68),
    "R_wrist_roll_joint": (-1.57, 1.57),
}

# 桌面抓放预备位（ubt_sim TASK_RESET_BODY_POSE，上体 19 关节；腿保持 0）
READY_POSE = {
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0,
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.50,
    "L_shoulder_pitch_joint": -0.291,
    "L_shoulder_roll_joint": 0.30,
    "L_shoulder_yaw_joint": 0.136,
    "L_elbow_pitch_joint": -1.155,
    "L_elbow_yaw_joint": 0.124,
    "L_wrist_pitch_joint": -0.361,
    "L_wrist_roll_joint": -0.194,
    "R_shoulder_pitch_joint": -0.291,
    "R_shoulder_roll_joint": -0.30,
    "R_shoulder_yaw_joint": -0.136,
    "R_elbow_pitch_joint": -1.155,
    "R_elbow_yaw_joint": -0.124,
    "R_wrist_pitch_joint": -0.361,
    "R_wrist_roll_joint": 0.194,
}

# 扁平化 home_position，按 ACTIVE_BODY_JOINTS 顺序
HOME_POSITION = [READY_POSE[name] for name in ACTIVE_BODY_JOINTS]

# 手部张开位置（C1 SDK 以 0 rad 为张开中位）
HAND_OPEN_POSITION = [0.0] * 6

# C1 不锁任何关节：头/腰/腿本就不在 action_order 中，bridge 不会下发。
DEFAULT_LOCK_JOINTS: list[str] = []

# ============================================================================
# 4. 策略维度契约（26 = 左臂 7 + 右臂 7 + 左手 6 + 右手 6）
# ============================================================================
ACTION_ORDER = ACTIVE_BODY_JOINTS + C1_HAND_LEFT_JOINTS + C1_HAND_RIGHT_JOINTS

# inactive_fill 的迭代基准：只含 6 组硬件关节（本机型 = 26，不含头/腰/腿），
# 避免填充键无法归属到空的 head/waist 组。
CANONICAL_JOINT_NAMES = list(ACTION_ORDER)

# ============================================================================
# 5. ROS2 话题（仿真与真机同构，靠 ROS_DOMAIN_ID 隔离）
# ============================================================================
TOPIC_BODY_CMD = "/mc/sdk/robot_command"
TOPIC_BODY_STATE = "/mc/sdk/robot_state"
TOPIC_LEFT_HAND_CMD = "/mc/left_hand/command"
TOPIC_RIGHT_HAND_CMD = "/mc/right_hand/command"
TOPIC_LEFT_HAND_STATE = "/mc/left_hand/joint_states"
TOPIC_RIGHT_HAND_STATE = "/mc/right_hand/joint_states"

# 相机：C1 只有头部单目 RGB（无腕部相机、无深度）
# 仿真：ubt_sim 桥接发布未压缩 sensor_msgs/Image
TOPIC_CAMERA_HEAD = "/sensor/camera/head/color/raw"
# 真机（Thor 版本机器人系统）：头双目的左目，只有 JPEG 压缩话题
TOPIC_CAMERA_HEAD_STEREO_COMPRESSED = "/sensor/camera/head_stereo_left/color/compressed"

# ============================================================================
# 6. 相机默认参数
# ============================================================================
DEFAULT_CAMERA_WIDTH = 320
DEFAULT_CAMERA_HEIGHT = 240
DEFAULT_CAMERA_FPS = 15
DEFAULT_CAMERA_WARMUP_S = 3
DEFAULT_CAMERA_TIMEOUT_MS = 5000
MSG_TYPE_IMAGE = "sensor_msgs/Image"
MSG_TYPE_COMPRESSED_IMAGE = "sensor_msgs/CompressedImage"
DEFAULT_CAMERA_MSG_TYPE = MSG_TYPE_IMAGE

# ============================================================================
# 7. ROBOT_MODELS 注册表（ROBOT_MODEL → 完整机器人规格：关节 + 相机）
# ============================================================================
# 设计要点（两套映射机制解耦）：
#   - policy ↔ 数据集：按位映射。joint_order 的顺序 == 数据集 action/state 维度与顺序。
#   - policy ↔ 硬件：按名散射。硬件分组是固定物理顺序（bridge 按名寻址）。
#
# 新增型号：在 ROBOT_MODELS 注册即可，无需创建外部 JSON 配置文件。


class RobotConfig:
    """机器人型号规格基类（dict-style 访问，与 config_walker_c1.py 兼容）。"""

    joint_order: list[str] = []
    camera_topics: dict[str, str] = {}
    camera_to_image_key: dict[str, str] = {}
    camera_msg_types: dict[str, str] = {}

    def __init__(self, **kwargs):
        cls = type(self)
        for key in ("joint_order", "camera_topics", "camera_to_image_key", "camera_msg_types"):
            val = kwargs.get(key, getattr(cls, key))
            object.__setattr__(self, key, val)
        if "camera_warmup_s" in kwargs:
            object.__setattr__(self, "camera_warmup_s", kwargs["camera_warmup_s"])
        elif "camera_warmup_s" in cls.__dict__:
            object.__setattr__(self, "camera_warmup_s", cls.camera_warmup_s)

    def __getitem__(self, key: str):
        return getattr(self, key)

    def get(self, key: str, default=None):
        return getattr(self, key, default)

    def __repr__(self) -> str:
        n_dof = len(self.joint_order)
        n_cam = len(self.camera_topics)
        return f"{type(self).__name__}({n_dof} DOF, {n_cam} cameras)"


class Walker_C1_26D_1Camera(RobotConfig):
    """26 DOF: 双臂(14) + 左右 C1 灵巧手各 6；1 路头部 RGB。"""

    joint_order = list(ACTION_ORDER)

    camera_topics = {"camera_head": TOPIC_CAMERA_HEAD}

    camera_to_image_key = {"camera_head": "camera_head"}

    camera_msg_types = {"camera_head": DEFAULT_CAMERA_MSG_TYPE}

    camera_sizes = {"camera_head": (DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT)}

    camera_warmup_s = DEFAULT_CAMERA_WARMUP_S


class Walker_C1_13D_1Camera(RobotConfig):
    """13 DOF: 右臂(7) + 右手 6 通道；1 路头部 RGB。

    左臂/左手不在策略输出中。部署 JSON 路径下 body.groups 仍保留 left_arm/left_hand，
    由 ``inactive_fill`` 填 home 位并按名下发（真机位置保持，仿真保持 HOME_POSE）。
    """

    joint_order = list(RIGHT_ARM_JOINTS) + list(C1_HAND_RIGHT_JOINTS)

    camera_topics = {"camera_head": TOPIC_CAMERA_HEAD}

    camera_to_image_key = {"camera_head": "camera_head"}

    camera_msg_types = {"camera_head": DEFAULT_CAMERA_MSG_TYPE}

    camera_sizes = {"camera_head": (DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT)}

    camera_warmup_s = DEFAULT_CAMERA_WARMUP_S


ROBOT_MODELS = {
    "walker_c1_26d": Walker_C1_26D_1Camera(),
    "walker_c1_13d": Walker_C1_13D_1Camera(),
}

# ============================================================================
# 8. 非激活关节填充
# ============================================================================
# 任何不在所选 joint_order 中的硬件关节的默认静态值：
#   臂/腰/头取 READY_POSE 对应位，C1 手取 0.0（张开）。
DEFAULT_INACTIVE_FILL: dict[str, float] = {
    **READY_POSE,
    **{j: 0.0 for j in C1_HAND_LEFT_JOINTS + C1_HAND_RIGHT_JOINTS},
}

# per-DOF 非激活关节填充覆盖（裸关节名 -> 值）。空时全用默认。
INACTIVE_FILL_OVERRIDES: dict[str, dict[str, float]] = {}


def inactive_fill_for(dof_name: str, active_joints: list[str]) -> dict[str, float]:
    """选中 DOF 未包含的关节 -> ``.pos`` 键的静态填充值（按名，与顺序无关）。"""
    active = set(active_joints)
    fill = {
        j: DEFAULT_INACTIVE_FILL.get(j, 0.0)
        for j in CANONICAL_JOINT_NAMES
        if j not in active
    }
    fill.update(INACTIVE_FILL_OVERRIDES.get(dof_name, {}))
    return {f"{j}.pos": v for j, v in fill.items()}


def joint_names_with_pos(joint_order: list[str]) -> list[str]:
    """从 joint_order 列表派生 all_joints（关节名 + .pos 后缀，顺序 = 数据集顺序）。"""
    return [f"{name}.pos" for name in joint_order]
