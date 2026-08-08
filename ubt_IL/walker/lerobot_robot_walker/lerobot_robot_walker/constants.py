"""Walker S2 机器人硬件常量（walker 插件侧）。

数据来源：walker/walker_sdk_ros2/robot_control/robot_control.py
此模块与 ros2_walker_bridge.py 独立维护——两边运行在不同 Python 环境
（3.12 vs 3.10），不交叉导入，仅通过 ZMQ + JSON 配置通信。
"""


# ============================================================================
# 1. 关节名
# ============================================================================

# ── 身体关节名（17: 7左臂 + 7右臂 + head_pitch + head_yaw + waist_yaw）──
BODY_JOINT_NAMES = [
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "waist_yaw_joint",
]

# ── V4 手部关节名（每手 7 关节，含 thumb_pip）──
V4_HAND_LEFT_JOINTS = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_thumb_pip",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

V4_HAND_RIGHT_JOINTS = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_thumb_pip",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]


# ============================================================================
# 2. 关节限位
# ============================================================================

# ── 身体关节限位（rad），来源：Walker S2 硬件规格书 ──
BODY_JOINT_LIMITS = {
    "L_elbow_roll_joint":       (-2.6180, 0.0),
    "L_elbow_yaw_joint":        (-2.9147, 2.9147),
    "L_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "L_shoulder_roll_joint":    (-1.85,   0.0873),
    "L_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "L_wrist_pitch_joint":      (-1.5882, 1.5882),
    "L_wrist_roll_joint":       (-1.9897, 1.9897),
    "R_elbow_roll_joint":       (-2.6180, 0.0),
    "R_elbow_yaw_joint":        (-2.9147, 2.9147),
    "R_shoulder_pitch_joint":   (-2.8274, 2.8274),
    "R_shoulder_roll_joint":    (-1.85,   0.0873),
    "R_shoulder_yaw_joint":     (-2.8972, 2.9147),
    "R_wrist_pitch_joint":      (-1.5882, 1.5882),
    "R_wrist_roll_joint":       (-1.9897, 1.9897),
    "head_pitch_joint":         (-0.6807, 0.5061),
    "head_yaw_joint":           (-1.6406, 1.6406),
    "waist_yaw_joint":          (-2.7925, 2.7925),
}

# ── V4 手部关节限位（rad），短名（去掉 left_/right_ 前缀）──
V4_HAND_JOINT_LIMITS = {
    "thumb_swing":  (0.0, 2.11),
    "thumb_mcp":    (0.0, 1.85),
    "thumb_pip":    (0.0, 1.09),
    "index_mcp":    (0.0, 1.71),
    "middle_mcp":   (0.0, 1.71),
    "ring_mcp":     (0.0, 1.71),
    "little_mcp":   (0.0, 1.71),
}


# ============================================================================
# 3. 位姿常量
# ============================================================================

# ── 归位位姿（17-dim body，按 BODY_JOINT_NAMES 顺序）──
READY_POSE = {
    "L_elbow_roll_joint":       -1.5600,
    "L_elbow_yaw_joint":        2.8800,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.5600,
    "R_elbow_yaw_joint":        -2.8800,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

# 扁平化 home_position，按 BODY_JOINT_NAMES 顺序
HOME_POSITION = [READY_POSE[name] for name in BODY_JOINT_NAMES]

# 手部张开位置（V4: 全 0 = 手指自然伸直）
HAND_OPEN_POSITION = [0.0] * 7


# ============================================================================
# 4. 默认锁定关节
# ============================================================================

# ── 默认锁定关节（不发送控制指令，保持当前位置）──
DEFAULT_LOCK_JOINTS = ["head_pitch_joint", "head_yaw_joint", "waist_yaw_joint"]


# ============================================================================
# 5. ROS2 话题名
# ============================================================================

TOPIC_BODY_CMD = "/mc/sdk/robot_command"
TOPIC_BODY_STATE = "/mc/sdk/robot_state"
TOPIC_LEFT_HAND_CMD = "/mc/left_hand/command"
TOPIC_RIGHT_HAND_CMD = "/mc/right_hand/command"
TOPIC_LEFT_HAND_STATE = "/mc/left_hand/joint_states"
TOPIC_RIGHT_HAND_STATE = "/mc/right_hand/joint_states"

# ── 大寰 PGC-140-50 夹爪话题（ecat_task_msgs/{GripCmd,GripStatus}）──
TOPIC_LEFT_GRIP_CMD = "/ecat/left_grip/cmd"
TOPIC_RIGHT_GRIP_CMD = "/ecat/right_grip/cmd"
TOPIC_LEFT_GRIP_STATE = "/ecat/left_grip/state"
TOPIC_RIGHT_GRIP_STATE = "/ecat/right_grip/state"

# ── 相机话题（默认）──
TOPIC_CAMERA_STEREO = "/sensor/camera/stereo/color/raw"
TOPIC_CAMERA_STEREO_LEFT = "/sensor/camera/stereo_left/image/raw"
TOPIC_CAMERA_STEREO_RIGHT = "/sensor/camera/stereo_right/image/raw"
TOPIC_CAMERA_WRIST_LEFT = "/sensor/camera/wrist_left/color/raw"
TOPIC_CAMERA_WRIST_RIGHT = "/sensor/camera/wrist_right/color/raw"


# ============================================================================
# 6. 固定硬件分组（物理顺序，bridge 按位寻址，严禁重排）
# ============================================================================

LEFT_ARM_JOINTS = [
    "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint", "L_shoulder_yaw_joint", "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]
RIGHT_ARM_JOINTS = [
    "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint", "R_shoulder_yaw_joint", "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]
HEAD_JOINTS = ["head_pitch_joint", "head_yaw_joint"]
WAIST_JOINTS = ["waist_yaw_joint"]

# canonical 全集（物理序），供 inactive_fill 迭代用
CANONICAL_JOINT_NAMES = (
    LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + HEAD_JOINTS + WAIST_JOINTS
    + V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS
    + ["left_grip", "right_grip"]
)  # 33: 17 body + 14 V4 hand + 2 PGC gripper


# =====================================================================
# 7. ROBOT_MODELS 注册表（ROBOT_MODEL → 完整机器人规格：关节 + 相机）
# =====================================================================
#
# 设计要点（两套映射机制解耦）：
#   - policy ↔ 数据集：按位映射。joint_order 列表的顺序 == 数据集 action/state
#     维度与顺序。joint_order 可任意重排以匹配数据集顺序。
#   - policy ↔ 硬件：按名散射。6 个硬件分组是【固定物理 motor/手指/夹爪顺序】
#     （bridge 按位寻址，严禁重排），与 joint_order 顺序无关。
#     send_action 按名从 action 字典取值、按物理序拼 bridge list。
# 二者解耦后，joint_order 可随意重排、可取任意子集，均能正确部署。
#
# ROBOT_MODELS 规格字段：
#   - joint_order：定义 action/state 的关节维度与顺序（不含 .pos 后缀，
#     由 joint_names_with_pos() 统一追加）
#   - camera_topics：相机源 key → ROS2 topic 映射
#   - camera_to_image_key：相机源 key → 模型 observation.images key 映射
#   - camera_msg_types：相机源 key → msg_type 字符串（可选，默认 DEFAULT_CAMERA_MSG_TYPE）
#   - camera_warmup_s：相机预热时间（可选，默认 DEFAULT_CAMERA_WARMUP_S）
#
# 新增型号：在 ROBOT_MODELS 注册即可，无需创建外部 JSON 配置文件。


class RobotConfig:
    """机器人型号规格基类。

    子类通过类属性定义默认值；实例化时无参走默认、传参可覆盖。
    支持 dict-style 访问（spec["key"]、spec.get("key", default)），
    与 config_walker.py 的访问模式兼容。

    camera_warmup_s 仅当子类显式定义时才存在于实例上，否则 spec.get("camera_warmup_s", d)
    返回默认值 d。
    """

    joint_order: list[str] = []
    camera_topics: dict[str, str] = {}
    camera_to_image_key: dict[str, str] = {}
    camera_msg_types: dict[str, str] = {}

    def __init__(self, **kwargs):
        cls = type(self)
        for key in ("joint_order", "camera_topics", "camera_to_image_key", "camera_msg_types"):
            val = kwargs.get(key, getattr(cls, key))
            object.__setattr__(self, key, val)
        # camera_warmup_s 仅当子类显式定义或 kwargs 传入时才设置
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


# ============================================================================
# 8. 相机默认参数
# ============================================================================

DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 15
DEFAULT_CAMERA_WARMUP_S = 3
DEFAULT_CAMERA_TIMEOUT_MS = 5000
DEFAULT_CAMERA_MSG_TYPE = "shm_msgs/Image2m"


# ============================================================================
# 9. 机器人型号规格类（数据定义在类属性中）
# ============================================================================


class Walker_S2_31D_5Camera(RobotConfig):
    """31 DOF: body(17) + left V4 hand(7) + right V4 hand(7)"""

    joint_order = [
        # Left arm (0-6)
        "L_elbow_roll_joint",
        "L_elbow_yaw_joint",
        "L_shoulder_pitch_joint",
        "L_shoulder_roll_joint",
        "L_shoulder_yaw_joint",
        "L_wrist_pitch_joint",
        "L_wrist_roll_joint",
        # Right arm (7-13)
        "R_elbow_roll_joint",
        "R_elbow_yaw_joint",
        "R_shoulder_pitch_joint",
        "R_shoulder_roll_joint",
        "R_shoulder_yaw_joint",
        "R_wrist_pitch_joint",
        "R_wrist_roll_joint",
        # Head (14-15)
        "head_pitch_joint",
        "head_yaw_joint",
        # Waist (16)
        "waist_yaw_joint",
        # Left V4 hand (17-23)
        "left_thumb_swing",
        "left_thumb_mcp",
        "left_thumb_pip",
        "left_index_mcp",
        "left_middle_mcp",
        "left_ring_mcp",
        "left_little_mcp",
        # Right V4 hand (24-30)
        "right_thumb_swing",
        "right_thumb_mcp",
        "right_thumb_pip",
        "right_index_mcp",
        "right_middle_mcp",
        "right_ring_mcp",
        "right_little_mcp",
    ]

    camera_topics = {
        "camera_head":        "/sensor/camera/stereo/color/raw",
        "camera_head_left":   "/sensor/camera/stereo_left/image/raw",
        "camera_head_right":  "/sensor/camera/stereo_right/image/raw",
        "camera_wrist_left":  "/sensor/camera/wrist_left/color/raw",
        "camera_wrist_right": "/sensor/camera/wrist_right/color/raw",
    }

    camera_to_image_key = {
        "camera_head":        "camera_head",
        "camera_head_left":   "camera_head_left",
        "camera_head_right":  "camera_head_right",
        "camera_wrist_left":  "camera_wrist_left",
        "camera_wrist_right": "camera_wrist_right",
    }

    camera_msg_types = {
        "camera_head":        "shm_msgs/Image2m",
        "camera_head_left":   "shm_msgs/Image6m",
        "camera_head_right":  "shm_msgs/Image6m",
        "camera_wrist_left":  "shm_msgs/Image1m",
        "camera_wrist_right": "shm_msgs/Image1m",
    }


class Walker_S2_19D_4Camera(RobotConfig):
    """19 DOF: body(17) + left PGC grip(1) + right PGC grip(1)"""

    joint_order = [
        # Left arm (0-6)
        "L_elbow_roll_joint",
        "L_elbow_yaw_joint",
        "L_shoulder_pitch_joint",
        "L_shoulder_roll_joint",
        "L_shoulder_yaw_joint",
        "L_wrist_pitch_joint",
        "L_wrist_roll_joint",
        # Right arm (7-13)
        "R_elbow_roll_joint",
        "R_elbow_yaw_joint",
        "R_shoulder_pitch_joint",
        "R_shoulder_roll_joint",
        "R_shoulder_yaw_joint",
        "R_wrist_pitch_joint",
        "R_wrist_roll_joint",
        # Head (14-15)
        "head_pitch_joint",
        "head_yaw_joint",
        # Waist (16)
        "waist_yaw_joint",
        # PGC grippers (17-18)
        "left_grip",
        "right_grip",
    ]

    camera_topics = {
        "camera_head_left":   "/sensor/camera/stereo_left/image/raw",
        "camera_head_right":  "/sensor/camera/stereo_right/image/raw",
        "camera_wrist_left":  "/sensor/camera/wrist_left/color/raw",
        "camera_wrist_right": "/sensor/camera/wrist_right/color/raw",
    }

    camera_to_image_key = {
        "camera_head_left":   "camera_head_left",
        "camera_head_right":  "camera_head_right",
        "camera_wrist_left":  "camera_wrist_left",
        "camera_wrist_right": "camera_wrist_right",
    }

    camera_msg_types = {
        "camera_head_left":   "shm_msgs/Image6m",
        "camera_head_right":  "shm_msgs/Image6m",
        "camera_wrist_left":  "shm_msgs/Image1m",
        "camera_wrist_right": "shm_msgs/Image1m",
    }

    camera_warmup_s = 10


class Walker_S2_10D_2Camera(RobotConfig):
    """10 DOF: right arm(7) + head(2) + right PGC grip(1)"""

    joint_order = [
        # Right arm (0-6)
        "R_elbow_roll_joint",
        "R_elbow_yaw_joint",
        "R_shoulder_pitch_joint",
        "R_shoulder_roll_joint",
        "R_shoulder_yaw_joint",
        "R_wrist_pitch_joint",
        "R_wrist_roll_joint",
        # Head (7-8)
        "head_pitch_joint",
        "head_yaw_joint",
        # Right PGC gripper (9)
        "right_grip",
    ]

    camera_topics = {
        "camera_head_right":  "/sensor/camera/stereo_right/image/raw",
        "camera_wrist_right": "/sensor/camera/wrist_right/color/raw",
    }

    camera_to_image_key = {
        "camera_head_right":  "camera_head_right",
        "camera_wrist_right": "camera_wrist_right",
    }

    camera_msg_types = {
        "camera_head_right":  "shm_msgs/Image6m",
        "camera_wrist_right": "shm_msgs/Image1m",
    }

    # 相机交付分辨率 (width, height)，必须与策略训练输入分辨率一致。
    # relay 据此 resize ROS 帧、WalkerCamera 保持同尺寸(no-op)，模型直接收到训练分辨率。
    # head  [3,256,320] -> H=256,W=320；wrist [3,240,320] -> H=240,W=320。
    camera_sizes = {
        "camera_head_right":  (320, 256),
        "camera_wrist_right": (320, 240),
    }

    camera_warmup_s = 10


# ============================================================================
# ROBOT_MODELS 注册表（实例化上面的规格类）
# ============================================================================

ROBOT_MODELS: dict[str, RobotConfig] = {
    "walker_s2_31d": Walker_S2_31D_5Camera(),
    "walker_s2_19d": Walker_S2_19D_4Camera(),
    "walker_s2_10d": Walker_S2_10D_2Camera(),
}


# ============================================================================
# 10. 非激活关节填充
# ============================================================================

# ── 非激活关节的静态填充 ──
# 任何非激活关节（不在所选 joint_order 中）的默认静态值：
#   车身关节取 READY_POSE 对应位，V4 手取 0.0（伸直），夹爪取 0.0（闭合安全位）。
DEFAULT_INACTIVE_FILL: dict[str, float] = {
    **READY_POSE,
    **{j: 0.0 for j in V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS},
    "left_grip": 0.0,
    "right_grip": 0.0,
}

# 可选：per-DOF 非激活关节填充覆盖（裸关节名 -> 值）。空时全用默认。
# 例：让 10-DOF 的非激活左手张到全开而非默认 0.0
#   {"walker_s2_10d": {"left_grip": 0.03, "left_thumb_swing": 0.5, ...}}
INACTIVE_FILL_OVERRIDES: dict[str, dict[str, float]] = {}


def inactive_fill_for(dof_name: str, active_joints: list[str]) -> dict[str, float]:
    """选中 DOF 未包含的关节 -> .pos 键的静态填充值（按名，与顺序无关）。

    先取 DEFAULT_INACTIVE_FILL，再用 INACTIVE_FILL_OVERRIDES[dof_name] 覆盖。
    返回的 dict 键已补 ``.pos`` 后缀，可直接用于 robot 的 action 字典。
    """
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
