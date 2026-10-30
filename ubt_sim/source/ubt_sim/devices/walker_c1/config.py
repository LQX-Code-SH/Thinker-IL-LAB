"""Configuration for the Walker C1 / Astron robot asset.

关节分组顺序即 action / 录制顺序，改动需同步 ``action_process.py`` 与数据契约。
关节限位/速度/力矩以整机 URDF
(``assets/robots/walker_c1/urdf/walker_c1_with_hands.urdf``)
为唯一来源（覆盖全部 53 个驱动关节，含双手），避免手写常量与模型漂移；
URDF 缺失时 ``WALKER_C1_JOINT_LIMITS`` 为空表，仿真仍靠 USD 软限位运行。
"""

from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from ubt_sim.utils.constant import ASSETS_ROOT
from ubt_sim.utils.urdf_utils import joint_limits as _joint_limits
from ubt_sim.utils.urdf_utils import safe_urdf_table

ROBOTS_ROOT = Path(ASSETS_ROOT) / "robots"
# 仿真主资产：walker_c1_merged.usd
# 由 scripts/import_walker_c1_usd.py 生成（merge_fixed_joints=True），
# 53 个驱动关节 / 0 固定关节 / 1 个 articulation root + 5 路相机 + 2 路 IMU。
# 配色由 scripts/recolor_walker_c1_usd.py 直接写进 configuration/*_base.usd 的
# root layer（复用 tienkung_dex 方案：OmniPBR 模板 + GeomSubset + decal）。
# 原始 sensorKpkd 无手版本保留在 usd/original/ 供回溯。
_WALKER_C1_MERGED_USD = ROBOTS_ROOT / "walker_c1" / "usd" / "walker_c1_merged.usd"
_WALKER_C1_BASE_USD = ROBOTS_ROOT / "walker_c1" / "usd" / "walker_c1_with_hands.usd"
_WALKER_C1_DEFAULT_USD_PATH = (
    _WALKER_C1_MERGED_USD if _WALKER_C1_MERGED_USD.is_file() else _WALKER_C1_BASE_USD
)
WALKER_C1_USD_PATH = Path(
    os.environ.get("UBT_SIM_WALKER_C1_USD_PATH", str(_WALKER_C1_DEFAULT_USD_PATH))
).expanduser()
# 带手整机 URDF（65 个 joint，其中 53 个驱动），mesh 引用为相对路径 ../meshes/
WALKER_C1_URDF_PATH = ROBOTS_ROOT / "walker_c1" / "urdf" / "walker_c1_with_hands.urdf"

# ---------------------------------------------------------------------------
# 关节分组（顺序即 action / 状态 / 录制顺序）
# ---------------------------------------------------------------------------

WALKER_C1_LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]

WALKER_C1_RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]

WALKER_C1_LEFT_HAND_JOINTS = [
    "L_thumb_cmp_joint",
    "L_thumb_mpp_joint",
    "L_thumb_ip_joint",
    "L_index_mpp_joint",
    "L_index_ip_joint",
    "L_middle_mpp_joint",
    "L_middle_ip_joint",
    "L_ring_mpp_joint",
    "L_ring_ip_joint",
    "L_little_mpp_joint",
    "L_little_ip_joint",
]

WALKER_C1_RIGHT_HAND_JOINTS = [
    "R_thumb_cmp_joint",
    "R_thumb_mpp_joint",
    "R_thumb_ip_joint",
    "R_index_mpp_joint",
    "R_index_ip_joint",
    "R_middle_mpp_joint",
    "R_middle_ip_joint",
    "R_ring_mpp_joint",
    "R_ring_ip_joint",
    "R_little_mpp_joint",
    "R_little_ip_joint",
]

WALKER_C1_HEAD_JOINTS = [
    "head_yaw_joint",
    "head_pitch_joint",
]

WALKER_C1_WAIST_JOINTS = [
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
]

WALKER_C1_LEFT_LEG_JOINTS = [
    "L_hip_pitch_joint",
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
]

WALKER_C1_RIGHT_LEG_JOINTS = [
    "R_hip_pitch_joint",
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]

WALKER_C1_ARM_JOINTS = WALKER_C1_LEFT_ARM_JOINTS + WALKER_C1_RIGHT_ARM_JOINTS
WALKER_C1_HAND_JOINTS = WALKER_C1_LEFT_HAND_JOINTS + WALKER_C1_RIGHT_HAND_JOINTS
WALKER_C1_LEG_JOINTS = WALKER_C1_LEFT_LEG_JOINTS + WALKER_C1_RIGHT_LEG_JOINTS

# 桌面操作的首个控制范围：腿在配置中存在，但在引入全身策略前保持 HOME 位。
WALKER_C1_UPPER_BODY_JOINTS = (
    WALKER_C1_LEFT_ARM_JOINTS
    + WALKER_C1_RIGHT_ARM_JOINTS
    + WALKER_C1_LEFT_HAND_JOINTS
    + WALKER_C1_RIGHT_HAND_JOINTS
    + WALKER_C1_HEAD_JOINTS
    + WALKER_C1_WAIST_JOINTS
)

WALKER_C1_HOME_POSE = {
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0,
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.0,
    "L_shoulder_pitch_joint": 0.0,
    "L_shoulder_roll_joint": 0.0,
    "L_shoulder_yaw_joint": 0.0,
    "L_elbow_pitch_joint": 0.0,
    "L_elbow_yaw_joint": 0.0,
    "L_wrist_pitch_joint": 0.0,
    "L_wrist_roll_joint": 0.0,
    "R_shoulder_pitch_joint": 0.0,
    "R_shoulder_roll_joint": 0.0,
    "R_shoulder_yaw_joint": 0.0,
    "R_elbow_pitch_joint": 0.0,
    "R_elbow_yaw_joint": 0.0,
    "R_wrist_pitch_joint": 0.0,
    "R_wrist_roll_joint": 0.0,
    "L_hip_pitch_joint": 0.0,
    "L_hip_roll_joint": 0.0,
    "L_hip_yaw_joint": 0.0,
    # 腿膝下限量位就是 0.080（URDF），留 2e-2 余量避免 Isaac Lab 的越界校验误判
    "L_knee_pitch_joint": 0.10,
    "L_ankle_pitch_joint": 0.0,
    "L_ankle_roll_joint": 0.0,
    "R_hip_pitch_joint": 0.0,
    "R_hip_roll_joint": 0.0,
    "R_hip_yaw_joint": 0.0,
    "R_knee_pitch_joint": 0.10,
    "R_ankle_pitch_joint": 0.0,
    "R_ankle_roll_joint": 0.0,
    "L_thumb_cmp_joint": 0.0,
    "L_thumb_mpp_joint": 0.0,
    "L_thumb_ip_joint": 0.0,
    "L_index_mpp_joint": 0.0,
    "L_index_ip_joint": 0.0,
    "L_middle_mpp_joint": 0.0,
    "L_middle_ip_joint": 0.0,
    "L_ring_mpp_joint": 0.0,
    "L_ring_ip_joint": 0.0,
    "L_little_mpp_joint": 0.0,
    "L_little_ip_joint": 0.0,
    "R_thumb_cmp_joint": 0.0,
    "R_thumb_mpp_joint": 0.0,
    "R_thumb_ip_joint": 0.0,
    "R_index_mpp_joint": 0.0,
    "R_index_ip_joint": 0.0,
    "R_middle_mpp_joint": 0.0,
    "R_middle_ip_joint": 0.0,
    "R_ring_mpp_joint": 0.0,
    "R_ring_ip_joint": 0.0,
    "R_little_mpp_joint": 0.0,
    "R_little_ip_joint": 0.0,
}

# ---------------------------------------------------------------------------
# URDF 限位（身体关节）——UBT_SIM_WALKER_C1_* 之外不做手工维护
# ---------------------------------------------------------------------------

_WALKER_C1_URDF_TABLE = safe_urdf_table(WALKER_C1_URDF_PATH, "walker_c1")
_DRIVEN_JOINTS = set(WALKER_C1_HOME_POSE)
# name -> (lower[rad], upper[rad], velocity[rad/s], effort[Nm])（含手部 22 关节）
WALKER_C1_JOINT_LIMITS = {
    name: limits
    for name, limits in _joint_limits(_WALKER_C1_URDF_TABLE).items()
    if name in _DRIVEN_JOINTS
}

# ---------------------------------------------------------------------------
# 执行器增益（与已验证的 Walker S2 参考对齐：同一 Isaac Lab 栈、同一固定根上体，
# 重力开启）。原始 C1 数值（臂/头 80、腰 120、腿 200）不足以在重力下保持 HOME，
# 会导致腿塌陷、头部下垂。手部沿用 URDF 原点附近的力控值。
# ---------------------------------------------------------------------------

WALKER_C1_ARM_STIFFNESS = {
    name: (500 if ("shoulder_roll" in name or "elbow_pitch" in name) else 600)
    for name in WALKER_C1_ARM_JOINTS
}
WALKER_C1_ARM_DAMPING = {name: 40 for name in WALKER_C1_ARM_JOINTS}
# 手部为力控：URDF 原点指尖力矩约 1.35 N·m。调试期 200/20 是为了对抗错误的加速度驱动；
# 在力控下会让任何指尖接触把轻物体弹飞（100 g 苹果上数百 N）。
WALKER_C1_HAND_STIFFNESS = {name: 10 for name in WALKER_C1_HAND_JOINTS}
WALKER_C1_HAND_DAMPING = {name: 1 for name in WALKER_C1_HAND_JOINTS}
WALKER_C1_HEAD_STIFFNESS = {name: 600 for name in WALKER_C1_HEAD_JOINTS}
WALKER_C1_HEAD_DAMPING = {name: 60 for name in WALKER_C1_HEAD_JOINTS}
WALKER_C1_WAIST_STIFFNESS = {name: 600 for name in WALKER_C1_WAIST_JOINTS}
WALKER_C1_WAIST_DAMPING = {name: 60 for name in WALKER_C1_WAIST_JOINTS}
WALKER_C1_LEG_STIFFNESS = {
    name: (
        1500 if ("hip_pitch" in name or "knee" in name)
        else 1600 if "ankle" in name
        else 1100  # hip_roll, hip_yaw
    )
    for name in WALKER_C1_LEG_JOINTS
}
WALKER_C1_LEG_DAMPING = {
    name: (
        65 if ("hip_pitch" in name or "knee" in name)
        else 70 if "ankle" in name
        else 55  # hip_roll, hip_yaw
    )
    for name in WALKER_C1_LEG_JOINTS
}

WALKER_C1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(WALKER_C1_USD_PATH.resolve()),
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
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=WALKER_C1_HOME_POSE,
    ),
    actuators={
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_LEFT_ARM_JOINTS,
            effort_limit_sim={
                ".*shoulder_pitch.*": 100,
                ".*shoulder_roll.*": 100,
                ".*shoulder_yaw.*": 45,
                ".*elbow_pitch.*": 80,
                ".*elbow_yaw.*": 30,
                ".*wrist_.*": 20,
            },
            velocity_limit_sim=14,
            stiffness={name: WALKER_C1_ARM_STIFFNESS[name] for name in WALKER_C1_LEFT_ARM_JOINTS},
            damping={name: WALKER_C1_ARM_DAMPING[name] for name in WALKER_C1_LEFT_ARM_JOINTS},
        ),
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_RIGHT_ARM_JOINTS,
            effort_limit_sim={
                ".*shoulder_pitch.*": 100,
                ".*shoulder_roll.*": 100,
                ".*shoulder_yaw.*": 45,
                ".*elbow_pitch.*": 80,
                ".*elbow_yaw.*": 30,
                ".*wrist_.*": 20,
            },
            velocity_limit_sim=14,
            stiffness={name: WALKER_C1_ARM_STIFFNESS[name] for name in WALKER_C1_RIGHT_ARM_JOINTS},
            damping={name: WALKER_C1_ARM_DAMPING[name] for name in WALKER_C1_RIGHT_ARM_JOINTS},
        ),
        "left_hand": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_LEFT_HAND_JOINTS,
            # 2 N·m（URDF 原点 1.35）：刚体指尖无软垫，抓握效率低于真手；
            # 2 N·m 时拇指在 80 g 苹果上会饱和并滑脱。
            effort_limit_sim=2,
            velocity_limit_sim=3,
            stiffness={name: WALKER_C1_HAND_STIFFNESS[name] for name in WALKER_C1_LEFT_HAND_JOINTS},
            damping={name: WALKER_C1_HAND_DAMPING[name] for name in WALKER_C1_LEFT_HAND_JOINTS},
        ),
        "right_hand": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_RIGHT_HAND_JOINTS,
            effort_limit_sim=2,
            velocity_limit_sim=3,
            stiffness={name: WALKER_C1_HAND_STIFFNESS[name] for name in WALKER_C1_RIGHT_HAND_JOINTS},
            damping={name: WALKER_C1_HAND_DAMPING[name] for name in WALKER_C1_RIGHT_HAND_JOINTS},
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_HEAD_JOINTS,
            effort_limit_sim=5,
            velocity_limit_sim=5.24,
            stiffness=WALKER_C1_HEAD_STIFFNESS,
            damping=WALKER_C1_HEAD_DAMPING,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_WAIST_JOINTS,
            effort_limit_sim={
                "waist_yaw_joint": 63,
                "waist_pitch_joint": 165,
                "waist_roll_joint": 110,
            },
            velocity_limit_sim=10.47,
            stiffness=WALKER_C1_WAIST_STIFFNESS,
            damping=WALKER_C1_WAIST_DAMPING,
        ),
        "left_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_LEFT_LEG_JOINTS,
            effort_limit_sim={
                ".*hip_pitch.*": 210,
                ".*hip_roll.*": 165,
                ".*hip_yaw.*": 110,
                ".*knee_pitch.*": 210,
                ".*ankle_.*": 60,
            },
            velocity_limit_sim=12.57,
            stiffness={name: WALKER_C1_LEG_STIFFNESS[name] for name in WALKER_C1_LEFT_LEG_JOINTS},
            damping={name: WALKER_C1_LEG_DAMPING[name] for name in WALKER_C1_LEFT_LEG_JOINTS},
        ),
        "right_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_C1_RIGHT_LEG_JOINTS,
            effort_limit_sim={
                ".*hip_pitch.*": 210,
                ".*hip_roll.*": 165,
                ".*hip_yaw.*": 110,
                ".*knee_pitch.*": 210,
                ".*ankle_.*": 60,
            },
            velocity_limit_sim=12.57,
            stiffness={name: WALKER_C1_LEG_STIFFNESS[name] for name in WALKER_C1_RIGHT_LEG_JOINTS},
            damping={name: WALKER_C1_LEG_DAMPING[name] for name in WALKER_C1_RIGHT_LEG_JOINTS},
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
