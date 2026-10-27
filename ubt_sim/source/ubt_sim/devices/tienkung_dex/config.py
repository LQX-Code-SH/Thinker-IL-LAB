"""Configuration for the TienKung Dex (V3 + Inspire RH5DG2) robot asset.

关节分组、限位与 mimic 关系优先从 URDF 读取（``assets/robots/tienkung_dex/urdf/``），
避免手写常量与模型不一致；URDF 不可用时退回内置常量。
"""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from ubt_sim.utils.constant import ASSETS_ROOT

from .urdf_utils import joint_limits as _joint_limits
from .urdf_utils import mimic_joints as _mimic_joints
from .urdf_utils import safe_urdf_table

ROBOTS_ROOT = Path(ASSETS_ROOT) / "robots"
TIENKUNG_DEX_ROOT = ROBOTS_ROOT / "tienkung_dex"
TIENKUNG_DEX_URDF_PATH = TIENKUNG_DEX_ROOT / "urdf" / "tienkung_dex_inspire.urdf"
TIENKUNG_DEX_USD_PATH = TIENKUNG_DEX_ROOT / "usd" / "tienkung_dex_inspire.usd"

# ---------------------------------------------------------------------------
# 关节分组（顺序即 action / 录制顺序，改动需同步 action_process 与数据契约）
# ---------------------------------------------------------------------------

TIENKUNG_DEX_LEFT_LEG_JOINTS = [
    "hip_pitch_l_joint",
    "hip_roll_l_joint",
    "hip_yaw_l_joint",
    "knee_pitch_l_joint",
    "ankle_pitch_l_joint",
    "ankle_roll_l_joint",
]

TIENKUNG_DEX_RIGHT_LEG_JOINTS = [
    "hip_pitch_r_joint",
    "hip_roll_r_joint",
    "hip_yaw_r_joint",
    "knee_pitch_r_joint",
    "ankle_pitch_r_joint",
    "ankle_roll_r_joint",
]

TIENKUNG_DEX_WAIST_JOINTS = [
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
]

TIENKUNG_DEX_HEAD_JOINTS = [
    "head_yaw_joint",
    "head_pitch_joint",
]

TIENKUNG_DEX_LEFT_ARM_JOINTS = [
    "shoulder_pitch_l_joint",
    "shoulder_roll_l_joint",
    "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint",
    "elbow_yaw_l_joint",
    "wrist_pitch_l_joint",
    "wrist_roll_l_joint",
]

TIENKUNG_DEX_RIGHT_ARM_JOINTS = [
    "shoulder_pitch_r_joint",
    "shoulder_roll_r_joint",
    "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint",
    "elbow_yaw_r_joint",
    "wrist_pitch_r_joint",
    "wrist_roll_r_joint",
]

# Inspire RH5DG2：单手 18 个 revolute（拇指/食指/中指 4，无名指/小指 3）
TIENKUNG_DEX_LEFT_HAND_JOINTS = [
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

TIENKUNG_DEX_RIGHT_HAND_JOINTS = [
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

TIENKUNG_DEX_BODY_JOINTS = (
    TIENKUNG_DEX_LEFT_LEG_JOINTS
    + TIENKUNG_DEX_RIGHT_LEG_JOINTS
    + TIENKUNG_DEX_WAIST_JOINTS
    + TIENKUNG_DEX_HEAD_JOINTS
    + TIENKUNG_DEX_LEFT_ARM_JOINTS
    + TIENKUNG_DEX_RIGHT_ARM_JOINTS
)  # 31

TIENKUNG_DEX_HAND_JOINTS = TIENKUNG_DEX_LEFT_HAND_JOINTS + TIENKUNG_DEX_RIGHT_HAND_JOINTS  # 36
TIENKUNG_DEX_ALL_JOINTS = TIENKUNG_DEX_BODY_JOINTS + TIENKUNG_DEX_HAND_JOINTS  # 67

# 站姿：实机 home_standing 标定值（pelvis 原点离地 0.998 m，见 parlor.yaml）。
# 膝关节本身微屈（左 0.368 / 右 0.341 rad），不要以 0 为基准；手部张开为 0。
TIENKUNG_DEX_HOME_POSE = {
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
    # 手部（RH5DG2 张开位）
    **{name: 0.0 for name in TIENKUNG_DEX_HAND_JOINTS},
}


# ---------------------------------------------------------------------------
# URDF 解析：关节限位 / 速度 / 力矩 / mimic（见 urdf_utils.py）
# ---------------------------------------------------------------------------

_URDF_TABLE = safe_urdf_table(TIENKUNG_DEX_URDF_PATH, label="tienkung_dex inspire")

# name -> (lower [rad], upper [rad], velocity [rad/s], effort [Nm])
TIENKUNG_DEX_JOINT_LIMITS: dict[str, tuple[float, float, float, float]] = _joint_limits(_URDF_TABLE)

# name -> {"joint": 源关节, "multiplier": 倍率, "offset": 偏置}
TIENKUNG_DEX_MIMIC_JOINTS: dict[str, dict] = _mimic_joints(_URDF_TABLE)

# 手部通道中的 mimic 关节不单独下发，由 action_process 按倍率展开
TIENKUNG_DEX_INDEPENDENT_HAND_JOINTS = [
    name for name in TIENKUNG_DEX_HAND_JOINTS if name not in TIENKUNG_DEX_MIMIC_JOINTS
]


TIENKUNG_DEX_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(TIENKUNG_DEX_USD_PATH.resolve()),
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
        joint_pos=TIENKUNG_DEX_HOME_POSE,
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
            joint_names_expr=TIENKUNG_DEX_LEFT_HAND_JOINTS,
            effort_limit_sim=1.0,
            velocity_limit_sim=2.0,
            stiffness=10.0,
            damping=2.0,
        ),
        "right_hand": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_RIGHT_HAND_JOINTS,
            effort_limit_sim=1.0,
            velocity_limit_sim=2.0,
            stiffness=10.0,
            damping=2.0,
        ),
        # 头部刚度从 200/20 提到 600/10，压住低头看桌面时的重力下垂与稳态误差。
        # 注意：仿真里限制头部响应速度的不是刚度，而是 velocity_limit_sim=1.27
        # rad/s。主循环只有 ~11 Hz（`--perf_stats` 实测每步 Phys 30 ms + Render
        # 40 ms，RTF≈0.11），2.5 s 墙钟的预备位轨迹换算到仿真时间要 ~5 rad/s，
        # 头被速度上限卡住，走完仍差 ~0.2 rad，之后几秒才追上（误差与
        # 2.5/8/20 s 三种时长实测：0.48 / 0.0001 / 0.0000 rad）。真机上 45° 走
        # 1.27 rad/s 只要 0.62 s，2.5 s 轨迹跟得上，不需要为此降速。
        "head": ImplicitActuatorCfg(
            joint_names_expr=TIENKUNG_DEX_HEAD_JOINTS,
            effort_limit_sim=6.3,
            velocity_limit_sim=1.27,
            stiffness=600.0,
            damping=10.0,
        ),
        # 腰比手臂（stiffness=80）需要硬得多：它要托住整条上半身，手臂前伸时
        # 力矩约 30 N·m。200 时预备位实测下垂 0.149 rad（8.5°），视觉上整个人
        # 前倾；600 把下垂压到约 0.05 rad（3°）。damping 按 sqrt(k) 同比放大，
        # 保持原阻尼比（仿真 dt=0.01 s，ω_n≈20 rad/s，隐式积分稳定）。
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
