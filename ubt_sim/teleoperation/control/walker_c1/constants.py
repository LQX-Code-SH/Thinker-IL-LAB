"""Walker C1 / Astron 遥操作与采集常量（ROS 侧，不 import Isaac 模块）。

关节名与 ``/mc/sdk/robot_state`` 的 31 个身体关节顺序一致；
手部按真机 SDK 的 6 维压缩通道命名（仿真侧 11 个手部关节由 action_process 展开）。
"""

BODY_JOINT_NAMES = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "L_hip_pitch_joint",
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
    "R_hip_pitch_joint",
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]

LEFT_ARM_JOINT_NAMES = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]

RIGHT_ARM_JOINT_NAMES = [
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]

HEAD_JOINT_NAMES = ["head_yaw_joint", "head_pitch_joint"]
WAIST_JOINT_NAMES = ["waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"]
LEG_JOINT_NAMES = [
    "L_hip_pitch_joint",
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
    "R_hip_pitch_joint",
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]

# 上体可动关节（复位/运动学用），顺序与 BODY_JOINT_NAMES 中的上体子集一致
UPPER_BODY_JOINT_NAMES = WAIST_JOINT_NAMES + HEAD_JOINT_NAMES + LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES

# 真机 SDK 的手部 6 维通道（与 /mc/{left,right}_hand/* 话题一致）
LEFT_HAND_JOINT_NAMES = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

RIGHT_HAND_JOINT_NAMES = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

# ---------------------------------------------------------------- 任务复位姿态
# 桌面操作预备位：由天工抓放复位姿态按关节语义映射到 C1 关节名。
TASK_RESET_BODY_POSE = {
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0,
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.50,
    # 左右臂关于矢状面对称：pitch 同号，roll/yaw 反号
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

# C1/Astron SDK 手部指令以 0 rad 为张开的中间位
TASK_RESET_LEFT_HAND_POSE = [0.0] * len(LEFT_HAND_JOINT_NAMES)
TASK_RESET_RIGHT_HAND_POSE = [0.0] * len(RIGHT_HAND_JOINT_NAMES)

# 复位分三段：先抬肩让手臂离开桌面，再收肘，最后落到预备位
TASK_RESET_ARM_CLEAR_POSE = {
    "L_shoulder_roll_joint": 1.5,
    "R_shoulder_roll_joint": -1.5,
}

TASK_RESET_ELBOW_CLEAR_POSE = {
    "L_elbow_pitch_joint": -1.5,
    "R_elbow_pitch_joint": -1.5,
}

DEFAULT_TASK_RESET_HZ = 100.0
DEFAULT_TASK_RESET_DURATION = 1.0
DEFAULT_TASK_RESET_CLEAR_DURATION = 0.6

# 采集常量
DEFAULT_CAPTURE_HZ = 15.0
HDF5_REL_PATH = "trajectory.hdf5"
CAMERA_NAME = "camera_head"
RGB_TOPIC = "/sensor/camera/head/color/raw"
DEPTH_TOPIC = "/sensor/camera/head/depth/raw"
