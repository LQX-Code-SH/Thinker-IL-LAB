"""tienkung_dex ROS 侧共享常量（纯数据，零依赖）。

桥接进程跑在系统 Python（3.10）里，不能 import 依赖 isaaclab 的
``ubt_sim.devices.tienkung_dex.config``，因此在这里维护一份 ROS 侧使用的
关节名清单；仿真侧的权威定义仍是 ``devices/tienkung_dex/config.py``，
两者由 ``scripts/check_tienkung_dex_usd.py`` + 采集脚本的关节校验兜底。
"""

from __future__ import annotations

import math

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


# ===========================================================================
# 控制层（dex_controller.py）用的常量
#
# 下面这些表是仿真侧 ``devices/tienkung_dex/config.py`` 的 ROS 侧镜像：
# 那边依赖 isaaclab 无法 import，这边保持纯数据、零依赖。修改 URDF 后需同步
# （``scripts/check_tienkung_dex_usd.py`` 与本文件的限位校验兜底）。
# ===========================================================================

ARM_JOINTS = {
    "left": [
        "shoulder_pitch_l_joint",
        "shoulder_roll_l_joint",
        "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint",
        "elbow_yaw_l_joint",
        "wrist_pitch_l_joint",
        "wrist_roll_l_joint",
    ],
    "right": [
        "shoulder_pitch_r_joint",
        "shoulder_roll_r_joint",
        "shoulder_yaw_r_joint",
        "elbow_pitch_r_joint",
        "elbow_yaw_r_joint",
        "wrist_pitch_r_joint",
        "wrist_roll_r_joint",
    ],
}

WAIST_JOINTS = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
HEAD_JOINTS = ["head_yaw_joint", "head_pitch_joint"]
LEG_JOINTS = [
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
]

# 与真机 /arm|head|waist|leg/cmd 的分组一一对应
GROUP_JOINTS = {
    "head": HEAD_JOINTS,
    "arm": ARM_JOINTS["left"] + ARM_JOINTS["right"],
    "waist": WAIST_JOINTS,
    "leg": LEG_JOINTS,
}

# URDF 关节限位 (lower, upper)，单位 rad
BODY_JOINT_LIMITS = {
    "hip_pitch_l_joint": (-3.054326, 2.792527),
    "hip_roll_l_joint": (-0.418879, 2.617994),
    "hip_yaw_l_joint": (-1.396263, 4.45059),
    "knee_pitch_l_joint": (-0.087266, 2.530727),
    "ankle_pitch_l_joint": (-1.186824, 0.523599),
    "ankle_roll_l_joint": (-0.523599, 0.523599),
    "hip_pitch_r_joint": (-3.054326, 2.792527),
    "hip_roll_r_joint": (-2.617994, 0.418879),
    "hip_yaw_r_joint": (-4.45059, 1.396263),
    "knee_pitch_r_joint": (-0.087266, 2.530727),
    "ankle_pitch_r_joint": (-1.186824, 0.523599),
    "ankle_roll_r_joint": (-0.523599, 0.523599),
    "waist_yaw_joint": (-2.617994, 3.228859),
    "waist_roll_joint": (-0.436332, 0.436332),
    "waist_pitch_joint": (-0.523599, 0.872665),
    "head_yaw_joint": (-1.047198, 1.047198),
    # 正值 = 低头（URDF 轴向 (0,1,0)）：低头到 +58°、仰头到 -15°。
    # URDF 的 <limit>、对应 USD 的关节限位与这里必须一致。
    "head_pitch_joint": (-0.261799, 1.012291),
    "shoulder_pitch_l_joint": (-2.844887, 2.844887),
    "shoulder_roll_l_joint": (-0.191986, 3.333579),
    "shoulder_yaw_l_joint": (-2.844887, 2.844887),
    "elbow_pitch_l_joint": (-2.548181, 0.191986),
    "elbow_yaw_l_joint": (-2.844887, 2.844887),
    "wrist_pitch_l_joint": (-1.32645, 1.32645),
    "wrist_roll_l_joint": (-1.32645, 1.32645),
    "shoulder_pitch_r_joint": (-2.844887, 2.844887),
    "shoulder_roll_r_joint": (-3.333579, 0.191986),
    "shoulder_yaw_r_joint": (-2.844887, 2.844887),
    "elbow_pitch_r_joint": (-2.548181, 0.191986),
    "elbow_yaw_r_joint": (-2.844887, 2.844887),
    "wrist_pitch_r_joint": (-1.32645, 1.32645),
    "wrist_roll_r_joint": (-1.32645, 1.32645),
}

HAND_JOINT_LIMITS = {
    "inspire": {
        # 拇指
        "thumb_yaw": (0.0, 1.919862),
        "thumb_mcp": (0.0, 0.610865),
        "thumb_pip": (0.0, 1.041888),
        "thumb_dip": (0.0, 0.942478),
        # 四指（ring/pinky 无 yaw）
        "index_yaw": (-0.26118, 0.26118),
        "index_mcp": (0.0, 1.308997),
        "index_pip": (0.0, 1.48353),
        "index_dip": (0.0, 1.568),
        "middle_yaw": (-0.26118, 0.26118),
        "middle_mcp": (0.0, 1.308997),
        "middle_pip": (0.0, 1.48353),
        "middle_dip": (0.0, 1.568),
        "ring_mcp": (0.0, 1.308997),
        "ring_pip": (0.0, 1.48353),
        "ring_dip": (0.0, 1.568),
        "pinky_mcp": (0.0, 1.308997),
        "pinky_pip": (0.0, 1.48353),
        "pinky_dip": (0.0, 1.568),
    },
    "brainco": {
        "thumb_metacarpal": (0.0, 1.55334),
        "thumb_proximal": (0.0, 1.0472),
        "thumb_distal": (0.0, 1.15192),
        "index_proximal": (0.0, 1.41),
        "index_distal": (0.0, 1.69297),
        "middle_proximal": (0.0, 1.41),
        "middle_distal": (0.0, 1.69297),
        "ring_proximal": (0.0, 1.41),
        "ring_distal": (0.0, 1.69297),
        "pinky_proximal": (0.0, 1.41),
        "pinky_distal": (0.0, 1.69297),
    },
}

# 站立位（与 devices/tienkung_dex/config.py 的 TIENKUNG_DEX_HOME_POSE 一致）
#
# waist_pitch 取 0，不用实机标定的 0.0327：那 1.9° 前倾在视觉上像"没站直"，
# 且预备位手臂前伸时还会叠上重力下垂（仿真里实测多 8.5°），两者合起来容易被
# 误判成姿态错误。腰 yaw/roll 仍保留实机值。
HOME_POSE = {
    "head_yaw_joint": 0.0008,
    "head_pitch_joint": 0.0006,
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0067,
    "waist_yaw_joint": 0.0019,
    "shoulder_pitch_l_joint": 0.2219,
    "shoulder_roll_l_joint": 0.1361,
    "shoulder_yaw_l_joint": 0.0231,
    "elbow_pitch_l_joint": -0.4959,
    "elbow_yaw_l_joint": 0.0013,
    "wrist_pitch_l_joint": 0.0035,
    "wrist_roll_l_joint": -0.0006,
    "shoulder_pitch_r_joint": 0.1770,
    "shoulder_roll_r_joint": -0.1469,
    "shoulder_yaw_r_joint": 0.0048,
    "elbow_pitch_r_joint": -0.3612,
    "elbow_yaw_r_joint": 0.0003,
    "wrist_pitch_r_joint": 0.0015,
    "wrist_roll_r_joint": -0.0007,
    "hip_pitch_l_joint": -0.1441,
    "hip_roll_l_joint": -0.0063,
    "hip_yaw_l_joint": -0.0055,
    "knee_pitch_l_joint": 0.3680,
    "ankle_pitch_l_joint": -0.1672,
    "ankle_roll_l_joint": -0.1740,
    "hip_pitch_r_joint": -0.1469,
    "hip_roll_r_joint": 0.0035,
    "hip_yaw_r_joint": 0.0246,
    "knee_pitch_r_joint": 0.3414,
    "ankle_pitch_r_joint": -0.1372,
    "ankle_roll_r_joint": -0.1400,
}

# 上肢站立位（home 默认不动腿——真机站立时腿由行走控制器持有）
HOME_UPPER_POSE = {
    name: value
    for name, value in HOME_POSE.items()
    if name not in LEG_JOINTS
}

# 预备位四段（数值照搬 tiangong2dex 的 reset：同一套身体 URDF、同一组限位，
# 那边已在真机/仿真上调通）。
#
# 机器人正前方有桌面（parlor 场景；本任务把 table 沿 z 缩放到 1.1，桌面
# 0.897 → 0.987 m，见 config/tienkung_dex/parlor.yaml 的 scene.table_z_scale）。
# 站立位手臂自然下垂时手在世界 0.80 m——比桌面还低；直接在体侧屈肘，前臂会
# 从桌面前沿扫过去；只屈肘把手抬到腰椎高度，手仍在桌面以下，肘也收不到位。
# 因此四段走法：先纯肩外展（Shoulder Roll）把整条手臂侧向抬到桌面以上，
# 再在高位折前臂，然后把大臂转回近垂直、前臂保持高位，最后前臂落到水平
# 预备位。全程手部在桌面上方不低于 0.23 m（FK 复核，见下）。
#
# 关节空间插值的 FK 复核（pelvis 系 → 世界，桌面 0.987 m，含腰部下垂
# 0.05 rad 的偏保守估计；只统计落在桌面投影范围内的点，体侧下垂的手臂不算）：
#   CLEARANCE       纯外展，手臂整体还在桌面外侧
#   FOREARM         肘以下最低 +0.30 m
#   UPPER_VERTICAL  肘以下最低 +0.44 m
#   READY           手 1.29 m（+0.30），手 x=8.28 / y=6.51（左）、5.64（右）
# 三段之间插值最低 +0.30 m（CLEARANCE→FOREARM 终点）。
# 注意：从站立位一步到 READY 时手要贴着桌面从体侧扫过去，新桌面（0.987 m）
# 已经彻底挡住这条直线路径，所以分段不是可选优化而是必需（--no-staged
# 只用于真机无桌面或手持物已抬高的场景）。

# 段 1：纯肩外展，手臂伸直侧向抬起（左 roll 正向、右 roll 负向，见 BODY_JOINT_LIMITS）
READY_CLEARANCE_POSE = {
    "shoulder_pitch_l_joint": 0.0,
    "shoulder_roll_l_joint": 1.60,
    "shoulder_yaw_l_joint": 0.0,
    "elbow_pitch_l_joint": 0.0,
    "elbow_yaw_l_joint": 0.0,
    "wrist_pitch_l_joint": 0.0,
    "wrist_roll_l_joint": 0.0,
    "shoulder_pitch_r_joint": 0.0,
    "shoulder_roll_r_joint": -1.60,
    "shoulder_yaw_r_joint": 0.0,
    "elbow_pitch_r_joint": 0.0,
    "elbow_yaw_r_joint": 0.0,
    "wrist_pitch_r_joint": 0.0,
    "wrist_roll_r_joint": 0.0,
}

# 段 2：保持外展，把前臂折到比最终姿态更高的位置，确保手先完全离开桌面
READY_FOREARM_POSE = {
    **READY_CLEARANCE_POSE,
    "elbow_pitch_l_joint": -2.00,
    "elbow_pitch_r_joint": -2.00,
    "wrist_pitch_l_joint": -0.55,
    "wrist_pitch_r_joint": -0.55,
}

# 段 3：大臂转回近垂直，前臂仍保持高位；肩 pitch 保留一点前抬，
# 避免手落回桌沿以下
READY_UPPER_VERTICAL_POSE = {
    **READY_FOREARM_POSE,
    "shoulder_pitch_l_joint": -0.70,
    "shoulder_roll_l_joint": 0.10,
    "shoulder_pitch_r_joint": -0.70,
    "shoulder_roll_r_joint": -0.10,
}

# 段 4（最终预备位）：大臂近垂直，前臂落到约水平；左右腕角略有差异以配合手型。
# 肩外展比段 3 大（±0.6 rad，段 3 是 ±0.10）：两臂张开的预备位，左右手分开
# 到体侧前外侧，给后续单手抓取留出侧向余量。段 3 仍按近垂直走，外展留到最后
# 一段完成，路径中手部离桌面只会更高。
READY_POSE = dict(HOME_UPPER_POSE)
READY_POSE.update(
    {
        **READY_UPPER_VERTICAL_POSE,
        # 低头 45° 看桌面（正值为低头）：相机在 head_pitch_link 上是平装的，
        # 视角完全由这个关节给，低头 45° 后桌面工作区进入画面
        "head_pitch_joint": 0.785398,
        "shoulder_roll_l_joint": 0.60,
        "shoulder_roll_r_joint": -0.60,
        "elbow_pitch_l_joint": -1.10,
        "elbow_pitch_r_joint": -1.10,
        "wrist_pitch_l_joint": -0.35,
        "wrist_pitch_r_joint": -0.20,
    }
)

# 仿真世界坐标系中的机器人基座位姿（config/tienkung_dex/parlor.yaml 的 init_state）
# x=7.85 与 tiangong2dex 抓取标定站位一致；7.80 时苹果心到右肩 0.681 m，
# 超出本臂 0.573 m 的腕可达半径，抓取点够不到。
ROBOT_WORLD_POS = (7.85, 6.08257, 0.998)
ROBOT_WORLD_ROT_WXYZ = (1.0, 0.0, 0.0, 0.0)

# parlor 场景（assets/scenes/parlor/scene_v2.usd）的桌面与苹果尺寸，量自 USD bbox：
#   原始台面 0.897 m；苹果 bbox x∈[8.302, 8.386] y∈[5.786, 5.871] z∈[0.899, 0.984]，
#   取中心、直径约 8.4 cm；盘子中心约 (8.374, 6.046, 0.916)，在苹果左后方 22 cm。
# 本任务（config/tienkung_dex/parlor.yaml）把 table 沿 z 缩放到 1.1：台面升到
# 0.987 m，苹果/盘子随台面同步上移 0.0897 m。下面都是抬高后的值。
APPLE_WORLD_POS = (8.344, 5.8285, 1.0312)
APPLE_DIAMETER_M = 0.084
TABLETOP_WORLD_Z = 0.987

# 右肩（shoulder_pitch_r_joint 原点）在 pelvis 系的位置，量自 URDF FK（两种手型相同）。
# 抓取时手末端沿「肩→苹果」方向摆位，这条线能让手臂在 0.573 m 腕可达半径内
# 取得最大余量：苹果心距肩 0.642 m，手落在 0.48 m 处最舒展。
RIGHT_SHOULDER_PELVIS = (0.0066, -0.1473, 0.3478)

DEFAULT_CONTROL_HZ = 50.0
# 单关节速度上限（rad/s）：轨迹点间超过即拒绝下发，防止用户脚本误发跳变
DEFAULT_MAX_JOINT_SPEED = 1.5
DEFAULT_MOVE_DURATION = 3.0
DEFAULT_JOINT_DURATION = 2.0


def action_limit(joint_name: str, variant: str = DEFAULT_VARIANT) -> float:
    """关节"闭合/动作"方向的极限（绝对值较大的一侧），与仿真 _target_limit 一致。"""
    if joint_name in BODY_JOINT_LIMITS:
        low, high = BODY_JOINT_LIMITS[joint_name]
    else:
        short = joint_name
        for prefix in ("left_", "right_"):
            if short.startswith(prefix):
                short = short[len(prefix):]
                break
        short = short.removesuffix("_joint")
        low, high = HAND_JOINT_LIMITS[variant][short]
    return low if abs(low) > abs(high) else high


def is_hand_abduction_joint(joint_name: str) -> bool:
    """手部关节是否为偏航/外展轴（``*_yaw``、强脑 ``thumb_metacarpal``）。

    这些轴不参与握拳屈曲：握拳时它们保持当前基准姿态，而不是被拉到动作极限。
    """
    short = joint_name
    for prefix in ("left_", "right_"):
        if short.startswith(prefix):
            short = short[len(prefix):]
            break
    short = short.removesuffix("_joint")
    return short.endswith("_yaw") or short == "thumb_metacarpal"


def hand_closed_pose(
    variant: str = DEFAULT_VARIANT,
    base_pose: dict[str, float] | None = None,
) -> dict[str, float]:
    """握拳目标（rad）：屈曲关节到动作极限，偏航/外展关节保持 ``base_pose`` 的值。

    与强脑 6 电机手势表 ``rock``（四指收拢、拇指扣拳）语义一致；因时同理。

    ``base_pose`` 是握拳时的基准姿态（默认全 0）。传 :func:`ready_hand_pose`
    就得到"预备位手型只弯曲其它自由度"的握拳：拇指侧摆留在 850 = 90°，
    不会在合手瞬间甩回全外展。注意只对偏航/外展轴生效，屈曲轴一律到动作极限。
    """
    base = base_pose or {}
    pose: dict[str, float] = {}
    for name in hand_joints(variant)[0] + hand_joints(variant)[1]:
        if is_hand_abduction_joint(name):
            pose[name] = float(base.get(name, 0.0))
        else:
            pose[name] = action_limit(name, variant)
    return pose


def hand_open_pose(variant: str = DEFAULT_VARIANT) -> dict[str, float]:
    """张开手（所有手部关节回 0）。只在回 home 时用。"""
    left, right = hand_joints(variant)
    return {name: 0.0 for name in left + right}


# 预备位手型：因时通道 10（大拇指侧摆）内收到码 850，即 90°。
# 通道行程是 650-1750（110°），110° 全部内收在抓取时过冲，取 90°。
INSPIRE_READY_THUMB_YAW_CODE = 850


def ready_hand_pose(variant: str = DEFAULT_VARIANT) -> dict[str, float]:
    """预备位手型：手指伸直、拇指侧摆内收到位（因时码 850 = 90°）。

    与 :func:`hand_open_pose` 只差拇指侧摆一个关节：**全伸直只在回 ``home`` 时用**，
    预备位和抓取前的手都是"张开 + 拇指内收"。强脑 6 电机里 ``thumb_metacarpal``
    是旋转轴、与因时的侧摆不同轴，没有对应通道，保持 0（等同全伸直）。
    """
    pose = hand_open_pose(variant)
    if variant == "inspire":
        channel = next(
            index for index, (short, *_) in enumerate(INSPIRE_CHANNELS) if short == "thumb_yaw"
        )
        value = inspire_code_to_rad(channel, INSPIRE_READY_THUMB_YAW_CODE)
        for name in pose:
            if name.endswith("thumb_yaw_joint"):
                pose[name] = value
    return pose


# ===========================================================================
# 真机接口（ros2_bridge_msgs / 灵巧手消息）与仿真关节的对照表
#
# 真机按关节 ID 编址（``/robot_state`` 的 ``status[].name``、``/arm/cmd``
# 的 ``ctrl[].name``），仿真按关节名编址，两边换算只在这里定义一次。
# 桥接进程与 ROS 侧控制脚本共用本文件。
#
# 关节 ID 来源：xhumanoid_sdk 的 ``config/joints.yaml``（真机 ``/robot_state``
# 实测核对过 ID 集合：arm 11~17/21~27、head 1/2、waist 31~33、leg 51~56/61~66）。
# 注意 ID 15/25 在真机叫 ``wrist_yaw``，而 URDF/仿真里同一个自由度命名为
# ``elbow_yaw_{l,r}_joint``——这是历史命名差异，不是两条关节。
# ===========================================================================
JOINT_ID_TO_SIM: dict[int, str] = {
    # ---- 头 ----
    1: "head_yaw_joint",
    2: "head_pitch_joint",
    # ---- 腰 ----
    31: "waist_pitch_joint",
    32: "waist_roll_joint",
    33: "waist_yaw_joint",
    # ---- 左臂 ----
    11: "shoulder_pitch_l_joint",
    12: "shoulder_roll_l_joint",
    13: "shoulder_yaw_l_joint",
    14: "elbow_pitch_l_joint",
    15: "elbow_yaw_l_joint",  # 真机命名 left_wrist_yaw
    16: "wrist_pitch_l_joint",
    17: "wrist_roll_l_joint",
    # ---- 右臂 ----
    21: "shoulder_pitch_r_joint",
    22: "shoulder_roll_r_joint",
    23: "shoulder_yaw_r_joint",
    24: "elbow_pitch_r_joint",
    25: "elbow_yaw_r_joint",  # 真机命名 right_wrist_yaw
    26: "wrist_pitch_r_joint",
    27: "wrist_roll_r_joint",
    # ---- 左腿 ----
    51: "hip_pitch_l_joint",
    52: "hip_roll_l_joint",
    53: "hip_yaw_l_joint",
    54: "knee_pitch_l_joint",
    55: "ankle_pitch_l_joint",
    56: "ankle_roll_l_joint",
    # ---- 右腿 ----
    61: "hip_pitch_r_joint",
    62: "hip_roll_r_joint",
    63: "hip_yaw_r_joint",
    64: "knee_pitch_r_joint",
    65: "ankle_pitch_r_joint",
    66: "ankle_roll_r_joint",
}

SIM_TO_JOINT_ID: dict[str, int] = {name: jid for jid, name in JOINT_ID_TO_SIM.items()}

# 分组 -> 该组的话题与消息类型由桥接按真机约定绑定：
#   /arm/cmd (ArmCtrl)、/head/cmd (HeadCtrl)、/waist/cmd (ArmCtrl)、/leg/cmd (ArmCtrl)
# 真机没有 WaistCtrl / LegCtrl，SDK 对这两组同样回落用 ArmCtrl（字段完全一致）。
GROUP_JOINT_IDS: dict[str, list[int]] = {
    "head": [1, 2],
    "arm": [11, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26, 27],
    "waist": [31, 32, 33],
    "leg": [51, 52, 53, 54, 55, 56, 61, 62, 63, 64, 65, 66],
}

JOINT_ID_TO_GROUP: dict[int, str] = {
    jid: group for group, ids in GROUP_JOINT_IDS.items() for jid in ids
}

# 真机 ControlMode（HWI §7.1）：0=位置 1=阻抗 2=速度 3=零位标定 4=距离 5=电流
CONTROL_MODE_POSITION = 0
CONTROL_MODE_IMPEDANCE = 1

# 真机各组位置指令的默认速度/电流限幅（沿用 SDK demo 的实测值：
# 手臂 cur=40 才够扛重力，头/腰 cur=10 即可）
GROUP_CMD_LIMITS = {
    "arm": {"spd": 0.5, "cur": 40.0},
    "head": {"spd": 0.3, "cur": 10.0},
    "waist": {"spd": 0.3, "cur": 10.0},
    "leg": {"spd": 0.5, "cur": 40.0},
}

# ---------------------------------------------------------------------------
# 灵巧手：真机通道 <-> 仿真关节
# ---------------------------------------------------------------------------
# 两个变体的真机通道语义完全不同：
#
# * 强脑 Revo2：6 电机，1~1000（1=伸直，1000=弯曲），按"闭合度"解释；
# * 因时 RH56：13 通道，单位 **0.1°**（1200 = 120°），每通道一个关节，
#   零位（伸直）不是 0 而是该通道的下限。
#
# 仿真侧与 ROS 侧的统一中间量是"关节弧度"：桥接下发 ``{关节名: rad}``，
# devices/tienkung_dex/action_process.py 直接按绝对角执行（旧版的
# ``{关节名}_percent`` 仍然兼容，只在强脑链路上使用）。
BRAINCO_CODE_MIN, BRAINCO_CODE_MAX = 1, 1000

# 强脑 Revo2：6 电机顺序 thumb_flex / thumb_rot / index / middle / ring / pinky。
# 远端关节在 URDF 里是 mimic（distal ← proximal），不需要单独驱动。
BRAINCO_MOTOR_JOINTS: tuple[tuple[str, ...], ...] = (
    ("thumb_proximal",),   # 0 拇指弯曲
    ("thumb_metacarpal",), # 1 拇指旋转
    ("index_proximal",),   # 2 食指
    ("middle_proximal",),  # 3 中指
    ("ring_proximal",),    # 4 无名指
    ("pinky_proximal",),   # 5 小指
)

# 因时 RH56：13 通道，每通道一个关节。厂商接口定义（单位 0.1°，1200 = 120°）：
#
#   0  小拇指指根 900-1650     1  无名指指根 900-1650     2  中指指根  900-1650
#   3  中指侧摆   -150-150     4  食指指根   900-1650     5  食指侧摆  -150-150
#   6  小拇指指中 1150-1900    7  无名指指中 1150-1900    8  中指指中  1150-1900
#   9  食指指中   1150-1900    10 大拇指侧摆 650-1750     11 大拇指一关节 1250-1600
#   12 大拇指二关节 1500-2040
#
# 换算 ``rad = (code - 零位码) × 方向 × 0.1°``。**屈曲通道码值越大越伸直**：
# 伸直在码上限（指根 1650 / 指中 1900 / 拇指侧摆 1750 / 拇指一关节 1600 /
# 拇指二关节 2040），方向 d=-1；侧摆通道零位在 0，方向 d=+1。
# 依据是厂商握拳示例 ``[1051,1051,1051,0,1051,0,1150,1150,1150,1150,1500,1400,1500]``：
# 按"上限伸直"读出来是指根 60°、指中 75°（全弯）、拇指一关节 20°、拇指二关节
# 54°，正是标准握拳；若按下限伸直读，指中会全部停在 1150（0°），不成拳。
# 屈曲行程与 URDF 限位对得上（指根 75°、拇指一关节 35°、拇指侧摆 110°、
# 侧摆 ±15°）；两处对不上、以 URDF 为准并在此记录：指中真机 75° vs URDF 85°，
# 拇指二关节真机 54° vs URDF 的 mimic 1.0853×thumb_mcp=38°。上报时超出真机
# 行程的角度会夹回通道上下限。食指/中指侧摆的正方向没有实测标定，暂按
# "码值增大 = 关节角增大"处理。
INSPIRE_CHANNELS: tuple[tuple[str, int, int, int, int], ...] = (
    # (仿真关节短名, 码下限, 码上限, 0 rad 对应的码, 方向)
    ("pinky_mcp",   900, 1650, 1650, -1),   # 0  小拇指指根
    ("ring_mcp",    900, 1650, 1650, -1),   # 1  无名指指根
    ("middle_mcp",  900, 1650, 1650, -1),   # 2  中指指根
    ("middle_yaw", -150,  150,    0, +1),   # 3  中指侧摆
    ("index_mcp",   900, 1650, 1650, -1),   # 4  食指指根
    ("index_yaw",  -150,  150,    0, +1),   # 5  食指侧摆
    ("pinky_pip",  1150, 1900, 1900, -1),   # 6  小拇指指中
    ("ring_pip",   1150, 1900, 1900, -1),   # 7  无名指指中
    ("middle_pip", 1150, 1900, 1900, -1),   # 8  中指指中
    ("index_pip",  1150, 1900, 1900, -1),   # 9  食指指中
    ("thumb_yaw",   650, 1750, 1750, -1),   # 10 大拇指侧摆
    ("thumb_mcp",  1250, 1600, 1600, -1),   # 11 大拇指一关节弯曲
    ("thumb_pip",  1500, 2040, 2040, -1),   # 12 大拇指二关节弯曲（URDF 里是 mimic，只上报）
)

INSPIRE_CHANNEL_JOINTS: tuple[tuple[str, ...], ...] = tuple(
    (short,) for short, *_ in INSPIRE_CHANNELS
)

HAND_CHANNEL_COUNT = {"brainco": len(BRAINCO_MOTOR_JOINTS), "inspire": len(INSPIRE_CHANNEL_JOINTS)}

# 因时通道码的物理单位：0.1° 一个码
INSPIRE_CODE_DEG = 0.1


def hand_channel_joints(variant: str, side: str) -> tuple[tuple[str, ...], ...]:
    """返回该变体/左右手每个真机通道对应的仿真关节全名（可能为空元组）。"""
    prefix = f"{side}_"
    if variant == "brainco":
        table = BRAINCO_MOTOR_JOINTS
    elif variant == "inspire":
        table = INSPIRE_CHANNEL_JOINTS
    else:
        raise ValueError(f"未知的灵巧手变体: {variant}")
    return tuple(tuple(f"{prefix}{joint}_joint" for joint in entry) for entry in table)


def inspire_code_to_rad(channel: int, code: int) -> float:
    """因时通道码（0.1°，带零位偏置与方向）-> 仿真关节角（rad）。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    clipped = min(max(int(code), low), high)
    return (clipped - zero) * direction * INSPIRE_CODE_DEG * math.pi / 180.0


def inspire_rad_to_code(channel: int, rad: float) -> int:
    """仿真关节角（rad）-> 因时通道码；超出真机行程时夹到通道上下限。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    code = zero + float(rad) * 180.0 / math.pi / (direction * INSPIRE_CODE_DEG)
    return int(round(min(max(code, low), high)))


def hand_code_to_percent(variant: str, code: int) -> float:
    """强脑通道值 -> 闭合度 [0, 1]（因时已改用 :func:`hand_code_to_rad`）。"""
    if variant != "brainco":
        raise ValueError(f"{variant} 通道不是闭合度语义，请用 hand_code_to_rad")
    clipped = min(max(int(code), BRAINCO_CODE_MIN), BRAINCO_CODE_MAX)
    return (clipped - BRAINCO_CODE_MIN) / float(BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)


def hand_percent_to_code(variant: str, percent: float) -> int:
    """闭合度 [0, 1] -> 强脑通道值（取整并夹在值域内）。"""
    if variant != "brainco":
        raise ValueError(f"{variant} 通道不是闭合度语义，请用 hand_rad_to_code")
    ratio = min(max(float(percent), 0.0), 1.0)
    return int(round(BRAINCO_CODE_MIN + ratio * (BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)))


def hand_code_to_rad(variant: str, channel: int, code: int, joint_name: str) -> float:
    """真机通道值 -> 仿真关节角（rad），两个变体的统一入口。"""
    if variant == "brainco":
        return hand_code_to_percent(variant, code) * action_limit(joint_name, variant)
    return inspire_code_to_rad(channel, code)


def hand_rad_to_code(variant: str, channel: int, rad: float, joint_name: str) -> int:
    """仿真关节角（rad）-> 真机通道值，两个变体的统一入口。"""
    if variant == "brainco":
        limit = action_limit(joint_name, variant)
        return hand_percent_to_code(variant, float(rad) / limit if abs(limit) > 1e-6 else 0.0)
    return inspire_rad_to_code(channel, rad)
