"""天工行者 Dex（TienKung Dex V3）硬件常量（tienkung_dex 插件侧）。

关节名 / 限位 / 预备位 / 灵巧手通道表**逐字镜像**自
``ubt_sim/teleoperation/control/tienkung_dex/constants.py``——那边是仿真与 ROS 侧的
权威定义，改 URDF 或重新标定后必须同步本文件（两边的关节名对不上会得到
"能跑但按名映射全部落空"的静默失败）。

本模块不 import ubt_sim、也不 import `lerobot_robot_tienkung`：插件包各自独立，
复用一律是复制源码后各自维护（与 walker 插件自带相机、不碰 tienkung 相机同一做法）。
``ros2_deploy_bridge.py`` 跑在系统 Python 3.10、无法 import 本模块，那边另有一份
运行期副本，改动同样要同步。
"""

from __future__ import annotations

import math

# ===========================================================================
# 关节清单
# ===========================================================================

# 全身 31 个主动关节（腿 12 + 腰 3 + 头 2 + 双臂 14），顺序即 HDF5
# ``joint_observations/joint_position`` 的列序。右臂落在 [24:31]。
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

# ---------------------------------------------------------------------------
# 灵巧手（URDF 关节名，逐字取自 ubt_sim）
# ---------------------------------------------------------------------------
# 因时 Inspire RH56：单手 18 个 revolute，其中 5 个 dip 是 mimic（跟随 pip），
# 实际可驱动的只有 13 个——恰好对应 13 个通道。
INSPIRE_LEFT_HAND_JOINTS = [
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

INSPIRE_RIGHT_HAND_JOINTS = [
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

# 强脑 BrainCo Revo2：单手 11 个 revolute（拇指 3 + 四指各 2），由 6 个电机驱动
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
    "inspire": (INSPIRE_LEFT_HAND_JOINTS, INSPIRE_RIGHT_HAND_JOINTS),
    "brainco": (BRAINCO_LEFT_HAND_JOINTS, BRAINCO_RIGHT_HAND_JOINTS),
}

DEFAULT_VARIANT = "inspire"

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


def hand_joints(variant: str = DEFAULT_VARIANT) -> tuple[list[str], list[str]]:
    """返回指定变体的 ``(左手关节, 右手关节)``。"""
    if variant not in HAND_JOINTS_BY_VARIANT:
        raise ValueError(f"未知的 tienkung_dex 变体: {variant}（可选 {sorted(HAND_JOINTS_BY_VARIANT)}）")
    return HAND_JOINTS_BY_VARIANT[variant]


# ===========================================================================
# 关节限位（URDF，单位 rad）
# ===========================================================================

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

# 短名（去掉 left_/right_ 前缀与 _joint 后缀）
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
    """手部关节是否为偏航/外展轴（``*_yaw``、强脑 ``thumb_metacarpal``）。"""
    short = joint_name
    for prefix in ("left_", "right_"):
        if short.startswith(prefix):
            short = short[len(prefix):]
            break
    short = short.removesuffix("_joint")
    return short.endswith("_yaw") or short == "thumb_metacarpal"


# ===========================================================================
# 姿态（站立位 / 预备位）
# ===========================================================================

# 站立位。waist_pitch 取 0，不用实机标定的 0.0327：那 1.9° 前倾在视觉上像
# "没站直"，且预备位手臂前伸时还会叠上重力下垂。
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
HOME_UPPER_POSE = {name: value for name, value in HOME_POSE.items() if name not in LEG_JOINTS}

# 预备位四段。机器人正前方有桌面（parlor 场景桌面 0.987 m），站立位手臂自然
# 下垂时手在世界 0.80 m——比桌面还低，直接在体侧屈肘会让前臂从桌面前沿扫过去。
# 因此四段走法：先纯肩外展把整条手臂侧向抬到桌面以上，再在高位折前臂，然后把
# 大臂转回近垂直、前臂保持高位，最后前臂落到水平预备位。
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

READY_FOREARM_POSE = {
    **READY_CLEARANCE_POSE,
    "elbow_pitch_l_joint": -2.00,
    "elbow_pitch_r_joint": -2.00,
    "wrist_pitch_l_joint": -0.55,
    "wrist_pitch_r_joint": -0.55,
}

READY_UPPER_VERTICAL_POSE = {
    **READY_FOREARM_POSE,
    "shoulder_pitch_l_joint": -0.70,
    "shoulder_roll_l_joint": 0.10,
    "shoulder_pitch_r_joint": -0.70,
    "shoulder_roll_r_joint": -0.10,
}

# 最终预备位。head_pitch = 0.785398（低头 45°）**是训练视角的前提**：
# 相机在 head_pitch_link 上平装，视角完全由这个关节给，不设它画面分布就与数据集不同。
READY_POSE = dict(HOME_UPPER_POSE)
READY_POSE.update(
    {
        **READY_UPPER_VERTICAL_POSE,
        "head_pitch_joint": 0.785398,
        "shoulder_roll_l_joint": 0.60,
        "shoulder_roll_r_joint": -0.60,
        "elbow_pitch_l_joint": -1.10,
        "elbow_pitch_r_joint": -1.10,
        "wrist_pitch_l_joint": -0.35,
        "wrist_pitch_r_joint": -0.20,
    }
)

# 2026-10-27 仿真侧加了手臂/腰部重力前馈之前，命令 READY_POSE 时手臂实际稳住的
# 关节角（实测）。那时末端比设计位低 133 mm。加补偿后物理姿态会真的落到
# READY_POSE 上（高 13 cm），桌面间隙/相机画幅/示教位姿都得跟着重调，所以把
# **旧版实际姿态**作为新目标：补偿只负责让「指令 = 实际」，姿态本身沿用旧版。
#
# 注意：真机驱动器闭环、本来就没有这段下垂，真机要用 READY_POSE，用这份会站成
# 旧版仿真的姿态（低 13 cm）。
READY_POSE_PRE_GRAV_COMP = {
    "shoulder_pitch_l_joint": -0.5389,
    "shoulder_roll_l_joint": 0.5122,
    "shoulder_yaw_l_joint": -0.0402,
    "elbow_pitch_l_joint": -1.0236,
    "elbow_yaw_l_joint": -0.0311,
    "wrist_pitch_l_joint": -0.3338,
    "wrist_roll_l_joint": -0.0096,
    "shoulder_pitch_r_joint": -0.5366,
    "shoulder_roll_r_joint": -0.5133,
    "shoulder_yaw_r_joint": 0.0343,
    "elbow_pitch_r_joint": -1.0241,
    "elbow_yaw_r_joint": 0.0213,
    "wrist_pitch_r_joint": -0.1818,
    "wrist_roll_r_joint": 0.0087,
    "waist_yaw_joint": 0.0019,
    "waist_roll_joint": 0.0078,
    "waist_pitch_joint": 0.0404,
}

# 拿上表直接当目标时前馈吃不干净（URDF 与 USD 的臂/手惯量不一致，残差随姿态
# 在 0.98~1.38 之间漂，不是固定增益能修好的），这里只把**局部**残差从目标里扣掉，
# 让物理姿态精确落回旧版。**改任何执行器增益都要重标。**
_READY_RESIDUAL_AT_COMP = {
    "shoulder_pitch_l_joint": 0.00684,
    "shoulder_roll_l_joint": -0.00072,
    "shoulder_yaw_l_joint": -0.00026,
    "elbow_pitch_l_joint": 0.00232,
    "elbow_yaw_l_joint": -0.00352,
    "wrist_pitch_l_joint": 0.00017,
    "wrist_roll_l_joint": -0.00025,
    "shoulder_pitch_r_joint": 0.00741,
    "shoulder_roll_r_joint": 0.00047,
    "shoulder_yaw_r_joint": -0.00045,
    "elbow_pitch_r_joint": 0.00196,
    "elbow_yaw_r_joint": 0.00240,
    "wrist_pitch_r_joint": 0.00027,
    "wrist_roll_r_joint": 0.00023,
    "waist_yaw_joint": 0.00000,
    "waist_roll_joint": 0.00000,
    "waist_pitch_joint": 0.00118,
}

# 仿真下发的预备位目标（= 旧版实测姿态 − 前馈残差）。**只覆盖双臂 14 + 腰 3，不含 head**，
# 所以最终目标要 merge：仿真 ``{**READY_POSE, **READY_POSE_TARGET}``，真机 ``READY_POSE``。
READY_POSE_TARGET = {
    name: value - _READY_RESIDUAL_AT_COMP.get(name, 0.0)
    for name, value in READY_POSE_PRE_GRAV_COMP.items()
}

# 四段各自的轨迹时长（墙钟秒），对应 CLEARANCE → FOREARM → UPPER_VERTICAL → TARGET。
READY_STAGE_DURATIONS = (5.0, 5.0, 5.0, 5.0)

# 收尾一步（头部追平 + 预备位手型）的时长，算作预备动作的第 5 段。
READY_TAIL_DURATION = 3.0


def ready_final_pose(real: bool = False) -> dict[str, float]:
    """预备位的最终目标关节角（不含手部）。

    ``real=False``（仿真）：``{**READY_POSE, **READY_POSE_TARGET}`` —— TARGET 只覆盖
    双臂 14 + 腰 3，head 必须从 READY_POSE 取（低头 45° 决定训练视角）。
    ``real=True``（真机）：``READY_POSE`` —— 驱动器闭环，没有仿真那段下垂。
    """
    if real:
        return dict(READY_POSE)
    return {**READY_POSE, **READY_POSE_TARGET}


# ===========================================================================
# 灵巧手：真机通道 <-> 仿真关节
# ===========================================================================
# 强脑 Revo2：6 电机，1~1000（1=伸直，1000=弯曲），按"闭合度"解释；
# 因时 RH56：13 通道，单位 **0.1°**（1200 = 120°），每通道一个关节。
# 仿真侧与 ROS 侧的统一中间量是"关节弧度"：桥接下发的就是 ``{关节名: rad}`` 的
# 位置语义，只是真机线缆上走的是通道码。
BRAINCO_CODE_MIN, BRAINCO_CODE_MAX = 1, 1000

# 强脑 Revo2：6 电机顺序 thumb_flex / thumb_rot / index / middle / ring / pinky。
# 远端关节在 URDF 里是 mimic（distal ← proximal），不需要单独驱动。
BRAINCO_MOTOR_JOINTS: tuple[tuple[str, ...], ...] = (
    ("thumb_proximal",),    # 0 拇指弯曲
    ("thumb_metacarpal",),  # 1 拇指旋转
    ("index_proximal",),    # 2 食指
    ("middle_proximal",),   # 3 中指
    ("ring_proximal",),     # 4 无名指
    ("pinky_proximal",),    # 5 小指
)

# 因时 RH56：13 通道，每通道一个关节。厂商接口定义（单位 0.1°，1200 = 120°）：
#
#   0  小拇指指根 900-1650     1  无名指指根 900-1650     2  中指指根  900-1650
#   3  中指侧摆   -150-150     4  食指指根   900-1650     5  食指侧摆  -150-150
#   6  小拇指指中 1150-1900    7  无名指指中 1150-1900    8  中指指中  1150-1900
#   9  食指指中   1150-1900    10 大拇指侧摆 650-1750     11 大拇指一关节 1250-1600
#   12 大拇指二关节 1500-2040
#
# 换算 ``rad = (code - 零位码) × 方向 × 0.1°``。**屈曲通道码值越大越伸直**：伸直在
# 码上限，方向 d=-1；侧摆通道零位在 0，方向 d=+1。
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

# 预备位手型：因时通道 10（大拇指侧摆）内收到码 850，即 90°。
# 通道行程是 650-1750（110°），110° 全部内收在抓取时过冲，取 90°。
INSPIRE_READY_THUMB_YAW_CODE = 850


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


def hand_percent_to_code(variant: str, percent: float) -> int:
    """闭合度 [0, 1] -> 强脑通道值（取整并夹在值域内）。"""
    if variant != "brainco":
        raise ValueError(f"{variant} 通道不是闭合度语义，请用 hand_rad_to_code")
    ratio = min(max(float(percent), 0.0), 1.0)
    return int(round(BRAINCO_CODE_MIN + ratio * (BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)))


def hand_code_to_rad(variant: str, channel: int, code: int, joint_name: str) -> float:
    """真机通道值 -> 仿真关节角（rad），两个变体的统一入口。"""
    if variant == "brainco":
        percent = (min(max(int(code), BRAINCO_CODE_MIN), BRAINCO_CODE_MAX) - BRAINCO_CODE_MIN) / float(
            BRAINCO_CODE_MAX - BRAINCO_CODE_MIN
        )
        return percent * action_limit(joint_name, variant)
    return inspire_code_to_rad(channel, code)


def hand_rad_to_code(variant: str, channel: int, rad: float, joint_name: str) -> int:
    """仿真关节角（rad）-> 真机通道值，两个变体的统一入口。"""
    if variant == "brainco":
        limit = action_limit(joint_name, variant)
        return hand_percent_to_code(variant, float(rad) / limit if abs(limit) > 1e-6 else 0.0)
    return inspire_rad_to_code(channel, rad)


def hand_open_pose(variant: str = DEFAULT_VARIANT) -> dict[str, float]:
    """张开手（所有手部关节回 0）。只在回 home 时用。"""
    left, right = hand_joints(variant)
    return {name: 0.0 for name in left + right}


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
# ===========================================================================
# 真机按关节 ID 编址（``/robot_state`` 的 ``status[].name``、``/arm/cmd`` 的
# ``ctrl[].name``），仿真/插件按关节名编址，两边换算只在这里定义一次。
#
# 关节 ID 来源：xhumanoid_sdk 的 ``config/joints.yaml``。注意 ID 15/25 在真机叫
# ``wrist_yaw``，而 URDF/仿真里同一个自由度命名为 ``elbow_yaw_{l,r}_joint``——
# 历史命名差异，不是两条关节，桥接必须查表、不能按名猜。
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

# 分组 -> 话题/消息类型由桥接按真机约定绑定：
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

# 真机各组位置指令的默认速度/电流限幅（沿用 SDK demo 的实测值：
# 手臂 cur=40 才够扛重力，头/腰 cur=10 即可）
GROUP_CMD_LIMITS = {
    "arm": {"spd": 0.5, "cur": 40.0},
    "head": {"spd": 0.3, "cur": 10.0},
    "waist": {"spd": 0.3, "cur": 10.0},
    "leg": {"spd": 0.5, "cur": 40.0},
}


# ---------------------------------------------------------------------------
# ROS2 话题名（与 ubt_sim/teleoperation/bridges/tienkung_dex/bridge_config.yaml 一致）
# ---------------------------------------------------------------------------
TOPIC_ARM_CMD = "/arm/cmd"
TOPIC_HEAD_CMD = "/head/cmd"
TOPIC_WAIST_CMD = "/waist/cmd"
TOPIC_LEG_CMD = "/leg/cmd"
TOPIC_ROBOT_STATE = "/robot_state"

# 灵巧手指令/反馈话题两个变体完全不同，必须按 hand_variant 选
TOPIC_HAND_CMD_BY_VARIANT = {
    "inspire": "/right_hand/angle_set",        # inspire_hand_msgs/SetAngle
    "brainco": "/right_hand/set_motor_multi",  # brainco_hand_msgs/SetMotorMulti
}
TOPIC_HAND_STATUS_BY_VARIANT = {
    "inspire": "/right_hand/angle_actual",     # inspire_hand_msgs/GetAngleAct
    "brainco": "/right_hand/motor_status",     # brainco_hand_msgs/MotorStatus
}

TOPIC_IMAGE_HEAD = "/ob_camera_head/color/image_raw"  # sensor_msgs/Image
TOPIC_ESTOP = "/power/board/key_status"              # bodyctrl_msgs/PowerBoardKeyStatus
TOPIC_SIM_RESET = "/sim/cmd_reset"                   # std_msgs/Bool


# ===========================================================================
# 关节 DOF 映射配置（支持任意自由度 / 任意关节顺序的模型部署）
# ===========================================================================
#
# 设计要点（与 lerobot_robot_tienkung 同一套机制，两套映射解耦）：
#   - policy ↔ 数据集：按位映射。枚举成员顺序 == 数据集 action/state 顺序。
#   - policy ↔ 硬件：按名散射。硬件分组是【固定物理通道顺序】（bridge 按位寻址
#     关节 ID / 手部通道，严禁重排），与枚举顺序无关。
# 二者解耦后，枚举可随意重排、可取任意子集，均能正确部署。

from enum import IntEnum  # noqa: E402  （与 tienkung 插件同一位置、同一风格）

# ── 固定硬件分组（物理通道顺序，bridge 按位寻址，严禁重排）──
# 本轮只驱动右臂 + 右手：左手/腰/头由 robot_ready 摆好后保持，不进策略。
RIGHT_ARM_JOINTS = ARM_JOINTS["right"]  # 关节 ID 21-27，与 BODY_JOINTS[24:31] 同序

# 右手：**按真机通道顺序**排列，第 i 个元素就是桥接下发的第 i 个通道。
# 因时 13 通道是 pinky→thumb（与 INSPIRE_CHANNELS 同序），与数据集的
# thumb→pinky（URDF 关节序）**不是同一排列**，靠 send_action 按名散射换算。
RIGHT_HAND_JOINTS_BY_VARIANT: dict[str, list[str]] = {
    variant: [joints[0] for joints in hand_channel_joints(variant, "right")]
    for variant in HAND_JOINTS_BY_VARIANT
}

INACTIVE_FILL_OVERRIDES: dict[str, dict[str, float]] = {}


def canonical_joint_names(variant: str = DEFAULT_VARIANT) -> list[str]:
    """该变体下插件能驱动的全部硬件关节（物理通道序）。"""
    return RIGHT_ARM_JOINTS + RIGHT_HAND_JOINTS_BY_VARIANT[variant]


def default_inactive_fill(variant: str = DEFAULT_VARIANT) -> dict[str, float]:
    """非激活关节的默认静态值：臂取站立位，手取 0（张开/伸直）。"""
    fill = {name: HOME_POSE[name] for name in RIGHT_ARM_JOINTS}
    fill.update({name: 0.0 for name in RIGHT_HAND_JOINTS_BY_VARIANT[variant]})
    return fill


# ── DOF 枚举：定义 all_joints（policy 维度与顺序 = 数据集顺序）──
# 成员名须取自 canonical_joint_names()；成员顺序须与数据集 action/state 顺序一致。
class TienKungDex20JointIndex(IntEnum):
    """20-DOF（右臂 7 + 右手因时 13），顺序 = 数据集 20 维顺序。

    手部按 **URDF 关节序**（thumb→pinky），不是因时通道序——数据集里手列取自
    ``hand_observations/hand_right_position`` 的 [0,1,2,4,5,6,8,9,10,12,13,15,16]，
    剔掉 5 个恒 0 的 dip。改这个顺序必须同步改
    ``scripts/convert/tienkung_dex/configs/tienkung_dex_20d_1RGB.json`` 的 names。
    """

    # Right arm (0-6)
    shoulder_pitch_r_joint = 0
    shoulder_roll_r_joint = 1
    shoulder_yaw_r_joint = 2
    elbow_pitch_r_joint = 3
    elbow_yaw_r_joint = 4
    wrist_pitch_r_joint = 5
    wrist_roll_r_joint = 6
    # Right hand (7-19)，URDF 关节序
    right_thumb_yaw_joint = 7
    right_thumb_mcp_joint = 8
    right_thumb_pip_joint = 9
    right_index_yaw_joint = 10
    right_index_mcp_joint = 11
    right_index_pip_joint = 12
    right_middle_yaw_joint = 13
    right_middle_mcp_joint = 14
    right_middle_pip_joint = 15
    right_ring_mcp_joint = 16
    right_ring_pip_joint = 17
    right_pinky_mcp_joint = 18
    right_pinky_pip_joint = 19


class TienKungDexBrainco13JointIndex(IntEnum):
    """13-DOF（右臂 7 + 右手强脑 6 电机），供 BrainCo 实物采集的数据训练用。

    手部按强脑电机序（thumb_flex / thumb_rot / index / middle / ring / pinky），
    远端关节在 URDF 里是 mimic，不作为动作维度。
    """

    # Right arm (0-6)
    shoulder_pitch_r_joint = 0
    shoulder_roll_r_joint = 1
    shoulder_yaw_r_joint = 2
    elbow_pitch_r_joint = 3
    elbow_yaw_r_joint = 4
    wrist_pitch_r_joint = 5
    wrist_roll_r_joint = 6
    # Right hand (7-12)，强脑电机序
    right_thumb_proximal_joint = 7
    right_thumb_metacarpal_joint = 8
    right_index_proximal_joint = 9
    right_middle_proximal_joint = 10
    right_ring_proximal_joint = 11
    right_pinky_proximal_joint = 12


# DOF 名 -> (枚举类, 手型) 注册表（新增 DOF：定义 IntEnum + 在此注册即可）
JOINT_INDEX_ENUMS: dict[str, type] = {
    "tienkung_dex_20": TienKungDex20JointIndex,
    "tienkung_dex_brainco_13": TienKungDexBrainco13JointIndex,
}

# 每个 DOF 配置绑定的手型（joint_config 决定 hand_variant，避免两者配错）
DOF_HAND_VARIANT: dict[str, str] = {
    "tienkung_dex_20": "inspire",
    "tienkung_dex_brainco_13": "brainco",
}


def inactive_fill_for(variant: str, dof_name: str, enum_cls: type) -> dict[str, float]:
    """选中 DOF 枚举未包含的关节 -> ``.pos`` 键的静态填充值（按名，与枚举顺序无关）。

    先取 default_inactive_fill(variant)，再用 INACTIVE_FILL_OVERRIDES[dof_name] 覆盖。
    返回的 dict 键已补 ``.pos`` 后缀，可直接用于 robot 的 action 字典。
    """
    active = {m.name for m in enum_cls}
    fill = {
        j: default_inactive_fill(variant).get(j, 0.0)
        for j in canonical_joint_names(variant)
        if j not in active
    }
    fill.update(INACTIVE_FILL_OVERRIDES.get(dof_name, {}))
    return {f"{j}.pos": v for j, v in fill.items()}


def joint_names_with_pos(enum_cls: type) -> list[str]:
    """从 DOF 枚举派生 all_joints（成员名 + ``.pos`` 后缀，顺序 = 枚举顺序 = 数据集顺序）。"""
    return [f"{m.name}.pos" for m in enum_cls]
