"""tienkung_dex ROS 侧共享常量（纯数据，零依赖）。

桥接进程跑在系统 Python（3.10）里，不能 import 依赖 isaaclab 的
``ubt_sim.devices.tienkung_dex.config``，因此在这里维护一份 ROS 侧使用的
关节名清单；仿真侧的权威定义仍是 ``devices/tienkung_dex/config.py``，
两者由 ``scripts/check_tienkung_dex_usd.py`` + 采集脚本的关节校验兜底。
"""

from __future__ import annotations

BODY_JOINTS = [
    # 腿
    "hip_pitch_l_joint",
    "hip_roll_l_joint",
    "hip_yaw_l_joint",
    "knee_pitch_l_joint",
    "ankle_pitch_l_joint",
    "ankle_roll_l_joint",
    "hip_pitch_r_joint",
    "hip_roll_r_joint",
    "hip_yaw_r_joint",
    "knee_pitch_r_joint",
    "ankle_pitch_r_joint",
    "ankle_roll_r_joint",
    # 腰
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    # 头
    "head_yaw_joint",
    "head_pitch_joint",
    # 左臂
    "shoulder_pitch_l_joint",
    "shoulder_roll_l_joint",
    "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint",
    "elbow_yaw_l_joint",
    "wrist_pitch_l_joint",
    "wrist_roll_l_joint",
    # 右臂
    "shoulder_pitch_r_joint",
    "shoulder_roll_r_joint",
    "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint",
    "elbow_yaw_r_joint",
    "wrist_pitch_r_joint",
    "wrist_roll_r_joint",
]

LEFT_HAND_JOINTS = [
    "left_thumb_yaw_joint",
    "left_thumb_mcp_joint",
    "left_thumb_pip_joint",
    "left_thumb_dip_joint",
    "left_index_yaw_joint",
    "left_index_mcp_joint",
    "left_index_pip_joint",
    "left_index_dip_joint",
    "left_middle_yaw_joint",
    "left_middle_mcp_joint",
    "left_middle_pip_joint",
    "left_middle_dip_joint",
    "left_ring_mcp_joint",
    "left_ring_pip_joint",
    "left_ring_dip_joint",
    "left_pinky_mcp_joint",
    "left_pinky_pip_joint",
    "left_pinky_dip_joint",
]

RIGHT_HAND_JOINTS = [
    "right_thumb_yaw_joint",
    "right_thumb_mcp_joint",
    "right_thumb_pip_joint",
    "right_thumb_dip_joint",
    "right_index_yaw_joint",
    "right_index_mcp_joint",
    "right_index_pip_joint",
    "right_index_dip_joint",
    "right_middle_yaw_joint",
    "right_middle_mcp_joint",
    "right_middle_pip_joint",
    "right_middle_dip_joint",
    "right_ring_mcp_joint",
    "right_ring_pip_joint",
    "right_ring_dip_joint",
    "right_pinky_mcp_joint",
    "right_pinky_pip_joint",
    "right_pinky_dip_joint",
]

ALL_JOINTS = BODY_JOINTS + LEFT_HAND_JOINTS + RIGHT_HAND_JOINTS

BODY_JOINT_SET = set(BODY_JOINTS)
LEFT_HAND_JOINT_SET = set(LEFT_HAND_JOINTS)
RIGHT_HAND_JOINT_SET = set(RIGHT_HAND_JOINTS)

# ---------------------------------------------------------------------------
# BrainCo Revo2 变体：身体关节相同，单手 11 revolute（拇指 3 + 四指各 2）
# ---------------------------------------------------------------------------
BRAINCO_LEFT_HAND_JOINTS = [
    "left_thumb_metacarpal_joint",
    "left_thumb_proximal_joint",
    "left_thumb_distal_joint",
    "left_index_proximal_joint",
    "left_index_distal_joint",
    "left_middle_proximal_joint",
    "left_middle_distal_joint",
    "left_ring_proximal_joint",
    "left_ring_distal_joint",
    "left_pinky_proximal_joint",
    "left_pinky_distal_joint",
]

BRAINCO_RIGHT_HAND_JOINTS = [
    "right_thumb_metacarpal_joint",
    "right_thumb_proximal_joint",
    "right_thumb_distal_joint",
    "right_index_proximal_joint",
    "right_index_distal_joint",
    "right_middle_proximal_joint",
    "right_middle_distal_joint",
    "right_ring_proximal_joint",
    "right_ring_distal_joint",
    "right_pinky_proximal_joint",
    "right_pinky_distal_joint",
]

HAND_JOINTS_BY_VARIANT = {
    "inspire": (LEFT_HAND_JOINTS, RIGHT_HAND_JOINTS),
    "brainco": (BRAINCO_LEFT_HAND_JOINTS, BRAINCO_RIGHT_HAND_JOINTS),
}

ALL_JOINTS_BY_VARIANT = {name: BODY_JOINTS + list(hands[0]) + list(hands[1]) for name, hands in HAND_JOINTS_BY_VARIANT.items()}

ALL_JOINT_SETS_BY_VARIANT = {name: set(joints) for name, joints in ALL_JOINTS_BY_VARIANT.items()}

DEFAULT_VARIANT = "inspire"


def hand_joints(variant: str = DEFAULT_VARIANT) -> tuple[list[str], list[str]]:
    """返回指定变体的 ``(左手关节, 右手关节)``。"""
    if variant not in HAND_JOINTS_BY_VARIANT:
        raise ValueError(f"未知的 tienkung_dex 变体: {variant}（可选 {sorted(HAND_JOINTS_BY_VARIANT)}）")
    return HAND_JOINTS_BY_VARIANT[variant]

# URDF 的 mimic 关系（被动关节由源关节倍率展开，ROS 侧无需单独下发）
MIMIC_JOINTS = {
    "left_thumb_pip_joint": {"joint": "left_thumb_mcp_joint", "multiplier": 1.0853},
    "left_index_dip_joint": {"joint": "left_index_pip_joint", "multiplier": 1.1232},
    "left_middle_dip_joint": {"joint": "left_middle_pip_joint", "multiplier": 1.1232},
    "left_ring_dip_joint": {"joint": "left_ring_pip_joint", "multiplier": 1.1232},
    "left_pinky_dip_joint": {"joint": "left_pinky_pip_joint", "multiplier": 1.1232},
    "right_thumb_pip_joint": {"joint": "right_thumb_mcp_joint", "multiplier": 1.0853},
    "right_index_dip_joint": {"joint": "right_index_pip_joint", "multiplier": 1.1232},
    "right_middle_dip_joint": {"joint": "right_middle_pip_joint", "multiplier": 1.1232},
    "right_ring_dip_joint": {"joint": "right_ring_pip_joint", "multiplier": 1.1232},
    "right_pinky_dip_joint": {"joint": "right_pinky_pip_joint", "multiplier": 1.1232},
}

# ROS 侧名称别名：允许省略 _joint 后缀，或使用不带左右侧标记的写法
_ALIAS = {}
for _name in ALL_JOINTS + BRAINCO_LEFT_HAND_JOINTS + BRAINCO_RIGHT_HAND_JOINTS:
    _ALIAS[_name] = _name
    _ALIAS[_name.removesuffix("_joint")] = _name
_ALIAS.update(
    {
        # 常见简写
        "left_thumb_swing_joint": "left_thumb_yaw_joint",
        "right_thumb_swing_joint": "right_thumb_yaw_joint",
        "left_little": "left_pinky_mcp_joint",
        "right_little": "right_pinky_mcp_joint",
    }
)
ALIAS_TO_SIM = _ALIAS


def normalize_joint_name(name: str) -> str | None:
    """把 ROS 侧关节名规范化为仿真关节名；未知名称返回 None。"""
    if name in ALIAS_TO_SIM:
        return ALIAS_TO_SIM[name]
    return ALIAS_TO_SIM.get(name.removesuffix("_joint"))


def split_status(
    joint_names: list[str], values: list[float], variant: str = DEFAULT_VARIANT
) -> dict[str, dict[str, float]]:
    """按 body/left_hand/right_hand 分组过滤状态，便于分别发布 JointState。"""
    left, right = hand_joints(variant)
    left_set, right_set = set(left), set(right)
    groups: dict[str, dict[str, float]] = {"body": {}, "left_hand": {}, "right_hand": {}}
    for name, value in zip(joint_names, values):
        if name in BODY_JOINT_SET:
            groups["body"][name] = value
        elif name in left_set:
            groups["left_hand"][name] = value
        elif name in right_set:
            groups["right_hand"][name] = value
    return groups
