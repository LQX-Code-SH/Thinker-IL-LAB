"""TienKung Dex + BrainCo Revo2 双手变体的配置。

身体部分与 Inspire 变体完全一致（31 revolute），差异仅在双手：
每只手 11 个 revolute（拇指 3、其余四指各 2），指尖 ``*_distal`` 为 mimic。

与 ``config.py`` 的区别：
* URDF/USD 指向 ``tienkung_dex_brainco.*``；
* 手部关节清单与执行器分组；
* 手部 ``effort/velocity`` 按 Revo2 的较小力矩取值（URDF 已给出，运行时仍以 URDF 为准）。
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from .config import (
    TIENKUNG_DEX_HEAD_JOINTS,
    TIENKUNG_DEX_LEFT_ARM_JOINTS,
    TIENKUNG_DEX_LEFT_LEG_JOINTS,
    TIENKUNG_DEX_RIGHT_ARM_JOINTS,
    TIENKUNG_DEX_RIGHT_LEG_JOINTS,
    TIENKUNG_DEX_ROOT,
    TIENKUNG_DEX_WAIST_JOINTS,
)
from .urdf_utils import joint_limits as _joint_limits
from .urdf_utils import mimic_joints as _mimic_joints
from .urdf_utils import safe_urdf_table

TIENKUNG_DEX_BRAINCO_URDF_PATH = TIENKUNG_DEX_ROOT / "urdf" / "tienkung_dex_brainco.urdf"
TIENKUNG_DEX_BRAINCO_USD_PATH = TIENKUNG_DEX_ROOT / "usd" / "tienkung_dex_brainco.usd"

TIENKUNG_DEX_BRAINCO_LEFT_HAND_JOINTS = [
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

TIENKUNG_DEX_BRAINCO_RIGHT_HAND_JOINTS = [
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

TIENKUNG_DEX_BRAINCO_BODY_JOINTS = (
    TIENKUNG_DEX_LEFT_LEG_JOINTS
    + TIENKUNG_DEX_RIGHT_LEG_JOINTS
    + TIENKUNG_DEX_WAIST_JOINTS
    + TIENKUNG_DEX_HEAD_JOINTS
    + TIENKUNG_DEX_LEFT_ARM_JOINTS
    + TIENKUNG_DEX_RIGHT_ARM_JOINTS
)  # 31

TIENKUNG_DEX_BRAINCO_HAND_JOINTS = (
    TIENKUNG_DEX_BRAINCO_LEFT_HAND_JOINTS + TIENKUNG_DEX_BRAINCO_RIGHT_HAND_JOINTS
)  # 22
TIENKUNG_DEX_BRAINCO_ALL_JOINTS = TIENKUNG_DEX_BRAINCO_BODY_JOINTS + TIENKUNG_DEX_BRAINCO_HAND_JOINTS  # 53

TIENKUNG_DEX_BRAINCO_HOME_POSE = {
    # 头（实机 1/2）
    "head_yaw_joint": 0.0008,
    "head_pitch_joint": 0.0006,
    # 腰（实机 31/32/33）；pitch 取 0，不用实机标定的 0.0327（那 1.9° 前倾
    # 会叠在预备位的重力下垂上，视觉上像没站直）
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0067,
    "waist_yaw_joint": 0.0019,
    # 左臂（实机 11-17；实机 wrist_yaw = URDF elbow_yaw）
    "shoulder_pitch_l_joint": 0.2219,
    "shoulder_roll_l_joint": 0.1361,
    "shoulder_yaw_l_joint": 0.0231,
    "elbow_pitch_l_joint": -0.4959,
    "elbow_yaw_l_joint": 0.0013,
    "wrist_pitch_l_joint": 0.0035,
    "wrist_roll_l_joint": -0.0006,
    # 右臂（实机 21-27）
    "shoulder_pitch_r_joint": 0.1770,
    "shoulder_roll_r_joint": -0.1469,
    "shoulder_yaw_r_joint": 0.0048,
    "elbow_pitch_r_joint": -0.3612,
    "elbow_yaw_r_joint": 0.0003,
    "wrist_pitch_r_joint": 0.0015,
    "wrist_roll_r_joint": -0.0007,
    # 左腿（实机 51-56）
    "hip_pitch_l_joint": -0.1441,
    "hip_roll_l_joint": -0.0063,
    "hip_yaw_l_joint": -0.0055,
    "knee_pitch_l_joint": 0.3680,
    "ankle_pitch_l_joint": -0.1672,
    "ankle_roll_l_joint": -0.1740,
    # 右腿（实机 61-66）
    "hip_pitch_r_joint": -0.1469,
    "hip_roll_r_joint": 0.0035,
    "hip_yaw_r_joint": 0.0246,
    "knee_pitch_r_joint": 0.3414,
    "ankle_pitch_r_joint": -0.1372,
    "ankle_roll_r_joint": -0.1400,
    # 手部（Revo2 张开位）
    **{name: 0.0 for name in TIENKUNG_DEX_BRAINCO_HAND_JOINTS},
}

_URDF_TABLE = safe_urdf_table(TIENKUNG_DEX_BRAINCO_URDF_PATH, label="tienkung_dex brainco")

TIENKUNG_DEX_BRAINCO_JOINT_LIMITS: dict[str, tuple[float, float, float, float]] = _joint_limits(_URDF_TABLE)
TIENKUNG_DEX_BRAINCO_MIMIC_JOINTS: dict[str, dict] = _mimic_joints(_URDF_TABLE)

TIENKUNG_DEX_BRAINCO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(TIENKUNG_DEX_BRAINCO_USD_PATH.resolve()),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos=TIENKUNG_DEX_BRAINCO_HOME_POSE,
    ),
    actuators={
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_LEFT_ARM_JOINTS,
            effort_limit_sim=50.0,
            velocity_limit_sim=10.0,
            stiffness=80.0,
            damping=20.0,
        ),
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_RIGHT_ARM_JOINTS,
            effort_limit_sim=50.0,
            velocity_limit_sim=10.0,
            stiffness=80.0,
            damping=20.0,
        ),
        "left_hand": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_BRAINCO_LEFT_HAND_JOINTS,
            effort_limit_sim=1.5,
            velocity_limit_sim=2.0,
            stiffness=10.0,
            damping=2.0,
        ),
        "right_hand": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_BRAINCO_RIGHT_HAND_JOINTS,
            effort_limit_sim=1.5,
            velocity_limit_sim=2.0,
            stiffness=10.0,
            damping=2.0,
        ),
        # 头部刚度/阻尼与 config.py 一致（600/10）；仿真里真正卡住头部跟踪的是
        # 速度上限与 ~11 Hz 的仿真步率，不是刚度，详见 config.py 的注释
        "head": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_HEAD_JOINTS,
            effort_limit_sim=6.3,
            velocity_limit_sim=1.27,
            stiffness=600.0,
            damping=10.0,
        ),
        # 腰刚度与阻尼说明见 config.py（同为 600/35，压住预备位的重力下垂）
        "waist": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_WAIST_JOINTS,
            effort_limit_sim=200.0,
            velocity_limit_sim=10.0,
            stiffness=600.0,
            damping=35.0,
        ),
        "left_leg": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_LEFT_LEG_JOINTS,
            effort_limit_sim=235.0,
            velocity_limit_sim=16.0,
            stiffness=200.0,
            damping=20.0,
        ),
        "right_leg": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_RIGHT_LEG_JOINTS,
            effort_limit_sim=235.0,
            velocity_limit_sim=16.0,
            stiffness=200.0,
            damping=20.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
