"""天工 TienKung 机器人硬件常量（tienkung 插件侧）。

借鉴 ubt_sim/teleoperation/control/constants.py 的模式。
此模块与 ros2_deploy_bridge.py 独立维护——两边运行在不同 Python 环境
（3.12 vs 3.10），不交叉导入，仅通过 ZMQ + JSON 配置通信。
"""

# ── 电机 ID ↔ 关节名映射 ──
ID_TO_NAME = {
    # Left Arm
    11: "shoulder_pitch_l_joint",
    12: "shoulder_roll_l_joint",
    13: "shoulder_yaw_l_joint",
    14: "elbow_pitch_l_joint",
    15: "elbow_yaw_l_joint",
    16: "wrist_pitch_l_joint",
    17: "wrist_roll_l_joint",
    # Right Arm
    21: "shoulder_pitch_r_joint",
    22: "shoulder_roll_r_joint",
    23: "shoulder_yaw_r_joint",
    24: "elbow_pitch_r_joint",
    25: "elbow_yaw_r_joint",
    26: "wrist_pitch_r_joint",
    27: "wrist_roll_r_joint",
}

NAME_TO_ID = {v: k for k, v in ID_TO_NAME.items()}

# ── 分组 ID 列表 ──
ID_ARM_L = [11, 12, 13, 14, 15, 16, 17]
ID_ARM_R = [21, 22, 23, 24, 25, 26, 27]

# ── 手指映射：ROS2 手指索引 → 关节名 ──
HAND_L_MAP = {
    1: "left_little_1_joint",
    2: "left_ring_1_joint",
    3: "left_middle_1_joint",
    4: "left_index_1_joint",
    5: "left_thumb_2_joint",
    6: "left_thumb_1_joint",
}

HAND_R_MAP = {
    1: "right_little_1_joint",
    2: "right_ring_1_joint",
    3: "right_middle_1_joint",
    4: "right_index_1_joint",
    5: "right_thumb_2_joint",
    6: "right_thumb_1_joint",
}

# ── 归位位姿 ──
ARM_HOME = [
    -0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006,  # 左臂 7 关节
    -0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194,  # 右臂 7 关节
]

HAND_OPEN = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]
HAND_CLOSE = [0.3, 0.3, 0.3, 0.3, 0.3, 0.0]

# ── 电机默认参数 ──
DEFAULT_ARM_SPEED = 0.5
DEFAULT_ARM_CURRENT = 5.0
DEFAULT_RESET_SPEED = 0.2
DEFAULT_RESET_CURRENT = 5.0

# ── ROS2 话题名 ──
TOPIC_ARM_CMD = "/arm/cmd_pos"
TOPIC_HEAD_CMD = "/head/cmd_pos"
TOPIC_LEFT_HAND_CMD = "/inspire_hand/ctrl/left_hand"
TOPIC_RIGHT_HAND_CMD = "/inspire_hand/ctrl/right_hand"
TOPIC_ARM_STATUS = "/arm/status"
TOPIC_LEFT_HAND_STATUS = "/inspire_hand/state/left_hand"
TOPIC_RIGHT_HAND_STATUS = "/inspire_hand/state/right_hand"


# =====================================================================
# 关节 DOF 映射配置（支持任意自由度 / 任意关节顺序的模型部署）
# =====================================================================
#
# 设计要点（两套映射机制解耦）：
#   - policy ↔ 数据集：按位映射。枚举成员顺序 == 数据集 action/state 顺序。
#     故枚举成员可任意重排以匹配数据集顺序。
#   - policy ↔ 硬件：按名散射。4 个硬件分组是【固定物理 motor/手指顺序】
#     （bridge 按位寻址 motor ID / 手指索引，严禁重排），与枚举顺序无关。
#     send_action 按名从 action 字典取值、按物理序拼 bridge list。
# 二者解耦后，枚举可随意重排、可取任意子集，均能正确部署。

from enum import IntEnum

# ── 固定硬件分组（物理 motor/手指顺序，bridge 按位寻址，严禁重排）──
# 顺序与数据集默认 26 维布局一致：左臂7 - 右臂7 - 左手6 - 右手6。
LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
]  # motor IDs 11-17
RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
]  # motor IDs 21-27
LEFT_HAND_JOINTS = [
    "left_little_finger_joint", "left_ring_finger_joint", "left_middle_finger_joint",
    "left_index_finger_joint", "left_thumb_bend_joint", "left_thumb_rotation_joint",
]  # Inspire 手指索引 1-6
RIGHT_HAND_JOINTS = [
    "right_little_finger_joint", "right_ring_finger_joint", "right_middle_finger_joint",
    "right_index_finger_joint", "right_thumb_bend_joint", "right_thumb_rotation_joint",
]
# canonical 全集（物理序），供派生填充迭代用
CANONICAL_JOINT_NAMES = (
    LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + LEFT_HAND_JOINTS + RIGHT_HAND_JOINTS
)  # 26


# ── DOF 枚举：定义 all_joints（policy 维度与顺序 = 数据集顺序）──
# 成员名须取自 CANONICAL_JOINT_NAMES；成员顺序须与数据集 action/state 顺序一致（可重排）。
class TienKung26JointIndex(IntEnum):
    """26-DOF（默认物理序：左臂-右臂-左手-右手）。成员顺序可重排以匹配数据集。"""

    # Left arm (0-6)
    left_shoulder_pitch_joint = 0
    left_shoulder_roll_joint = 1
    left_shoulder_yaw_joint = 2
    left_elbow_pitch_joint = 3
    left_wrist_yaw_joint = 4
    left_wrist_pitch_joint = 5
    left_wrist_roll_joint = 6
    # Right arm (7-13)
    right_shoulder_pitch_joint = 7
    right_shoulder_roll_joint = 8
    right_shoulder_yaw_joint = 9
    right_elbow_pitch_joint = 10
    right_wrist_yaw_joint = 11
    right_wrist_pitch_joint = 12
    right_wrist_roll_joint = 13
    # Left hand (14-19)
    left_little_finger_joint = 14
    left_ring_finger_joint = 15
    left_middle_finger_joint = 16
    left_index_finger_joint = 17
    left_thumb_bend_joint = 18
    left_thumb_rotation_joint = 19
    # Right hand (20-25)
    right_little_finger_joint = 20
    right_ring_finger_joint = 21
    right_middle_finger_joint = 22
    right_index_finger_joint = 23
    right_thumb_bend_joint = 24
    right_thumb_rotation_joint = 25


class TienKung13JointIndex(IntEnum):
    """13-DOF（右臂7 + 右手6，顺序：右臂-右手，与 13-DOF 数据集 action 顺序一致）。"""

    # Right arm (0-6)
    right_shoulder_pitch_joint = 0
    right_shoulder_roll_joint = 1
    right_shoulder_yaw_joint = 2
    right_elbow_pitch_joint = 3
    right_wrist_yaw_joint = 4
    right_wrist_pitch_joint = 5
    right_wrist_roll_joint = 6
    # Right hand (7-12)
    right_little_finger_joint = 7
    right_ring_finger_joint = 8
    right_middle_finger_joint = 9
    right_index_finger_joint = 10
    right_thumb_bend_joint = 11
    right_thumb_rotation_joint = 12


# DOF 名 -> 枚举类 注册表（新增 DOF：定义 IntEnum + 在此注册即可）
JOINT_INDEX_ENUMS: dict[str, type] = {
    "tienkung_26": TienKung26JointIndex,
    "tienkung_13": TienKung13JointIndex,
}


# ── 非激活关节的静态填充 ──
# 任何非激活关节（不在所选 DOF 枚举中）的默认静态值：
#   臂取 ARM_HOME 对应位，手取 1.0（张开）。
DEFAULT_INACTIVE_FILL: dict[str, float] = {
    **dict(zip(LEFT_ARM_JOINTS, ARM_HOME[:7])),
    **dict(zip(RIGHT_ARM_JOINTS, ARM_HOME[7:14])),
    **{j: 1.0 for j in LEFT_HAND_JOINTS + RIGHT_HAND_JOINTS},
}

# 可选：per-DOF 非激活关节填充覆盖（裸关节名 -> 值）。空时全用默认。
# 例：14-DOF（仅臂）想让双手闭合而非 1.0，可登记
#   {"tienkung_14": {"left_little_finger_joint": 0.3, ...}}
INACTIVE_FILL_OVERRIDES: dict[str, dict[str, float]] = {}


def inactive_fill_for(dof_name: str, enum_cls: type) -> dict[str, float]:
    """选中 DOF 枚举未包含的关节 -> .pos 键的静态填充值（按名，与枚举顺序无关）。

    先取 DEFAULT_INACTIVE_FILL，再用 INACTIVE_FILL_OVERRIDES[dof_name] 覆盖。
    返回的 dict 键已补 ``.pos`` 后缀，可直接用于 robot 的 action 字典。
    """
    active = {m.name for m in enum_cls}
    fill = {
        j: DEFAULT_INACTIVE_FILL.get(j, 0.0)
        for j in CANONICAL_JOINT_NAMES
        if j not in active
    }
    fill.update(INACTIVE_FILL_OVERRIDES.get(dof_name, {}))
    return {f"{j}.pos": v for j, v in fill.items()}


def joint_names_with_pos(enum_cls: type) -> list[str]:
    """从 DOF 枚举派生 all_joints（成员名 + .pos 后缀，顺序 = 枚举顺序 = 数据集顺序）。"""
    return [f"{m.name}.pos" for m in enum_cls]
