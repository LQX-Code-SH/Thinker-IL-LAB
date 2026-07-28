from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from ubt_sim.utils.constant import ASSETS_ROOT

"""Configuration for the Walker S2 robot asset."""
ROBOTS_ROOT = Path(ASSETS_ROOT) / "robots"
WALKER_S2_USD_PATH = ROBOTS_ROOT / "walker_s2" / "s2_v1.usd"

WALKER_S2_LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]

WALKER_S2_RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]

# Two-finger PGC grippers are exposed as active articulation DOFs in the Walker S2
# USD.  ECAT GripCmd.pos is the distance between fingers in meters: 0.0 is
# closed and 0.05 is fully open.  The USD prismatic joints are open at 0.0 and
# close by moving in opposite directions within about +/-0.022 m, so command-space
# closing travel is scaled to a safe per-joint travel.  The signs below map
# command-space positive closing to each USD joint axis.
WALKER_S2_LEFT_HAND_JOINTS = ["L_finger1_joint", "L_finger2_joint"]
WALKER_S2_RIGHT_HAND_JOINTS = ["R_finger1_joint", "R_finger2_joint"]
WALKER_S2_GRIPPER_OPENING_MIN_M = 0.0
WALKER_S2_GRIPPER_OPENING_MAX_M = 0.05
WALKER_S2_GRIPPER_HOME_OPENING_M = 0.05
WALKER_S2_GRIPPER_JOINT_CLOSING_M = 0.02
WALKER_S2_GRIPPER_JOINT_SIGNS = {
    "L_finger1_joint": 1,
    "L_finger2_joint": 1,
    "R_finger1_joint": 1,
    "R_finger2_joint": 1,
}

WALKER_S2_HEAD_JOINTS = [
    "head_pitch_joint",
    "head_yaw_joint",
]

WALKER_S2_WAIST_YAW_JOINTS = ["waist_yaw_joint"]
WALKER_S2_WAIST_PITCH_JOINTS = ["waist_pitch_joint"]
WALKER_S2_WAIST_JOINTS = WALKER_S2_WAIST_YAW_JOINTS + WALKER_S2_WAIST_PITCH_JOINTS

WALKER_S2_LEFT_LEG_JOINTS = [
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_hip_pitch_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
]

WALKER_S2_RIGHT_LEG_JOINTS = [
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_hip_pitch_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]

# Position-control gains from the Walker S2 SDK joint parameter table.  Isaac Lab's
# implicit actuator stiffness is the closest equivalent of the SDK pos_kp.
#
# Damping is set to Kp/5 — matching the stiffness/damping ratio that Tienkung Pro
# (80/20=4:1) empirically validates as stable in PhysX articulation drives.
# Kd=0 causes persistent undamped oscillation; higher Kd ratios reduce overshoot
# at the expense of slightly slower rise time.  effort_limit_sim + velocity_limit_sim
# provide hard caps on top of the PD drive.
WALKER_S2_HEAD_STIFFNESS = {
    "head_yaw_joint": 600,
    "head_pitch_joint": 600,
}
WALKER_S2_HEAD_DAMPING = {name: 120 for name in WALKER_S2_HEAD_STIFFNESS}

WALKER_S2_ARM_STIFFNESS = {
    "L_shoulder_pitch_joint": 600,
    "L_shoulder_roll_joint": 500,
    "L_shoulder_yaw_joint": 600,
    "L_elbow_roll_joint": 500,
    "L_elbow_yaw_joint": 600,
    "L_wrist_pitch_joint": 600,
    "L_wrist_roll_joint": 600,
    "R_shoulder_pitch_joint": 600,
    "R_shoulder_roll_joint": 500,
    "R_shoulder_yaw_joint": 600,
    "R_elbow_roll_joint": 500,
    "R_elbow_yaw_joint": 600,
    "R_wrist_pitch_joint": 600,
    "R_wrist_roll_joint": 600,
}
WALKER_S2_ARM_DAMPING = {
    name: (100 if "roll" in name else 120) for name in WALKER_S2_ARM_STIFFNESS
}

WALKER_S2_WAIST_STIFFNESS = {
    "waist_yaw_joint": 600,
    "waist_pitch_joint": 600,
}
WALKER_S2_WAIST_DAMPING = {name: 120 for name in WALKER_S2_WAIST_STIFFNESS}

WALKER_S2_LEG_STIFFNESS = {
    "L_hip_roll_joint": 1100,
    "L_hip_yaw_joint": 1100,
    "L_hip_pitch_joint": 1500,
    "L_knee_pitch_joint": 1500,
    "L_ankle_pitch_joint": 1600,
    "L_ankle_roll_joint": 1600,
    "R_hip_roll_joint": 1100,
    "R_hip_yaw_joint": 1100,
    "R_hip_pitch_joint": 1500,
    "R_knee_pitch_joint": 1500,
    "R_ankle_pitch_joint": 1600,
    "R_ankle_roll_joint": 1600,
}
WALKER_S2_LEG_DAMPING = {
    name: (220 if "hip_roll" in name or "hip_yaw" in name else 300 if "hip_pitch" in name or "knee" in name else 320)
    for name in WALKER_S2_LEG_STIFFNESS
}
WALKER_S2_GRIPPER_STIFFNESS = {
    name: 150 for name in WALKER_S2_LEFT_HAND_JOINTS + WALKER_S2_RIGHT_HAND_JOINTS
}
WALKER_S2_GRIPPER_DAMPING = {name: 20 for name in WALKER_S2_GRIPPER_STIFFNESS}
WALKER_S2_GRIPPER_HOME_POSE = {
    name: WALKER_S2_GRIPPER_JOINT_SIGNS[name]
    * ((WALKER_S2_GRIPPER_OPENING_MAX_M - WALKER_S2_GRIPPER_HOME_OPENING_M) / WALKER_S2_GRIPPER_OPENING_MAX_M)
    * WALKER_S2_GRIPPER_JOINT_CLOSING_M
    for name in WALKER_S2_LEFT_HAND_JOINTS + WALKER_S2_RIGHT_HAND_JOINTS
}

WALKER_S2_HOME_POSE = {
    # Keep the robot in a neutral standing pose at startup. The previous upper-body
    # seed pose came from a bent manipulation posture and caused the waist/arms to
    # look twisted before any ROS2 command was received.
    "L_shoulder_pitch_joint": 0.0,
    "L_shoulder_roll_joint": 0.0,
    "L_shoulder_yaw_joint": 0.0,
    "L_elbow_roll_joint": 0.0,
    "L_elbow_yaw_joint": 0.0,
    "L_wrist_pitch_joint": 0.0,
    "L_wrist_roll_joint": 0.0,
    "R_shoulder_pitch_joint": 0.0,
    "R_shoulder_roll_joint": 0.0,
    "R_shoulder_yaw_joint": 0.0,
    "R_elbow_roll_joint": 0.0,
    "R_elbow_yaw_joint": 0.0,
    "R_wrist_pitch_joint": 0.0,
    "R_wrist_roll_joint": 0.0,
    "head_pitch_joint": 0.0,
    "head_yaw_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "L_hip_roll_joint": 0.0,
    "L_hip_yaw_joint": 0.0,
    "L_hip_pitch_joint": 0.0,
    "L_knee_pitch_joint": 0.0,
    "L_ankle_pitch_joint": 0.0,
    "L_ankle_roll_joint": 0.0,
    "R_hip_roll_joint": 0.0,
    "R_hip_yaw_joint": 0.0,
    "R_hip_pitch_joint": 0.0,
    "R_knee_pitch_joint": 0.0,
    "R_ankle_pitch_joint": 0.0,
    "R_ankle_roll_joint": 0.0,
    **WALKER_S2_GRIPPER_HOME_POSE,
}

WALKER_S2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(WALKER_S2_USD_PATH.resolve()),
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
        joint_pos=WALKER_S2_HOME_POSE,
    ),
    actuators={
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_LEFT_ARM_JOINTS,
            effort_limit_sim={
                ".*shoulder_pitch.*": 80,
                ".*shoulder_roll.*": 80,
                ".*shoulder_yaw.*": 45,
                ".*elbow_roll.*": 45,
                ".*elbow_yaw.*": 20,
                ".*wrist_.*": 20,
            },
            velocity_limit_sim=3.1,  # 30 rpm (真机手臂最大转速)
            stiffness={name: WALKER_S2_ARM_STIFFNESS[name] for name in WALKER_S2_LEFT_ARM_JOINTS},
            damping={name: WALKER_S2_ARM_DAMPING[name] for name in WALKER_S2_LEFT_ARM_JOINTS},
        ),
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_RIGHT_ARM_JOINTS,
            effort_limit_sim={
                ".*shoulder_pitch.*": 80,
                ".*shoulder_roll.*": 80,
                ".*shoulder_yaw.*": 45,
                ".*elbow_roll.*": 45,
                ".*elbow_yaw.*": 20,
                ".*wrist_.*": 20,
            },
            velocity_limit_sim=3.1,  # 30 rpm (真机手臂最大转速)
            stiffness={name: WALKER_S2_ARM_STIFFNESS[name] for name in WALKER_S2_RIGHT_ARM_JOINTS},
            damping={name: WALKER_S2_ARM_DAMPING[name] for name in WALKER_S2_RIGHT_ARM_JOINTS},
        ),
        "left_gripper": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_LEFT_HAND_JOINTS,
            effort_limit_sim=100,
            velocity_limit_sim=0.2,
            stiffness={name: WALKER_S2_GRIPPER_STIFFNESS[name] for name in WALKER_S2_LEFT_HAND_JOINTS},
            damping={name: WALKER_S2_GRIPPER_DAMPING[name] for name in WALKER_S2_LEFT_HAND_JOINTS},
        ),
        "right_gripper": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_RIGHT_HAND_JOINTS,
            effort_limit_sim=100,
            velocity_limit_sim=0.2,
            stiffness={name: WALKER_S2_GRIPPER_STIFFNESS[name] for name in WALKER_S2_RIGHT_HAND_JOINTS},
            damping={name: WALKER_S2_GRIPPER_DAMPING[name] for name in WALKER_S2_RIGHT_HAND_JOINTS},
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_HEAD_JOINTS,
            effort_limit_sim=4.5,
            velocity_limit_sim=5.2,
            stiffness=WALKER_S2_HEAD_STIFFNESS,
            damping=WALKER_S2_HEAD_DAMPING,
        ),
        "waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_WAIST_YAW_JOINTS,
            effort_limit_sim=85,
            velocity_limit_sim=3.9,
            stiffness={name: WALKER_S2_WAIST_STIFFNESS[name] for name in WALKER_S2_WAIST_YAW_JOINTS},
            damping={name: WALKER_S2_WAIST_DAMPING[name] for name in WALKER_S2_WAIST_YAW_JOINTS},
        ),
        "waist_pitch": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_WAIST_PITCH_JOINTS,
            effort_limit_sim=265,
            velocity_limit_sim=2.1,
            stiffness={name: WALKER_S2_WAIST_STIFFNESS[name] for name in WALKER_S2_WAIST_PITCH_JOINTS},
            damping={name: WALKER_S2_WAIST_DAMPING[name] for name in WALKER_S2_WAIST_PITCH_JOINTS},
        ),
        "left_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_LEFT_LEG_JOINTS,
            effort_limit_sim={
                ".*hip_roll.*": 225,
                ".*hip_yaw.*": 65,
                ".*hip_pitch.*": 225,
                ".*knee_pitch.*": 225,
                ".*ankle_.*": 65,
            },
            velocity_limit_sim=8.4,
            stiffness={name: WALKER_S2_LEG_STIFFNESS[name] for name in WALKER_S2_LEFT_LEG_JOINTS},
            damping={name: WALKER_S2_LEG_DAMPING[name] for name in WALKER_S2_LEFT_LEG_JOINTS},
        ),
        "right_leg": ImplicitActuatorCfg(
            joint_names_expr=WALKER_S2_RIGHT_LEG_JOINTS,
            effort_limit_sim={
                ".*hip_roll.*": 225,
                ".*hip_yaw.*": 65,
                ".*hip_pitch.*": 225,
                ".*knee_pitch.*": 225,
                ".*ankle_.*": 65,
            },
            velocity_limit_sim=8.4,
            stiffness={name: WALKER_S2_LEG_STIFFNESS[name] for name in WALKER_S2_RIGHT_LEG_JOINTS},
            damping={name: WALKER_S2_LEG_DAMPING[name] for name in WALKER_S2_RIGHT_LEG_JOINTS},
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
