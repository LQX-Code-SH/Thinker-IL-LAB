"""Walker S2 机器人控制常量 —— 关节定义、限位、话题、预备姿态等。

所有常量集中在此模块中，通过 ``from robot_control.constants import *`` 或
``from robot_control import ...`` 使用。
"""

import numpy as np

__all__ = [
    # 默认参数
    "DEFAULT_COMMAND_TOPIC",
    "DEFAULT_STATE_TOPIC",
    "DEFAULT_CONTROL_HZ",
    "DEFAULT_MAX_JOINT_SPEED",
    "DEFAULT_LOCK_JOINTS",
    # PVT
    "_PVT_DEFAULT_KP",
    "_PVT_DEFAULT_KD",
    "PROFILE_LINEAR",
    "PROFILE_QUINTIC",
    # 身体关节
    "BODY_JOINT_NAMES",
    "BODY_JOINT_LIMITS",
    "LEFT_ARM_JOINTS",
    "RIGHT_ARM_JOINTS",
    # V4 手部
    "V4_HAND_JOINT_LIMITS",
    "V4_HAND_LEFT_JOINTS",
    "V4_HAND_RIGHT_JOINTS",
    "V4_HAND_JOINT_MAP",
    "V4_HAND_OPEN_POSE",
    "V4_HAND_CLOSE_POSE",
    # V4 手部测试
    "V4_HAND_TEST_AMPLITUDE",
    "V4_HAND_TEST_PERIOD",
    "V4_HAND_TEST_PHASE_DIFF",
    "V4_HAND_TEST_DEFAULT_CYCLES",
    "V4_HAND_TEST_HZ",
    "V4_HAND_LEFT_TOPIC",
    "V4_HAND_RIGHT_TOPIC",
    "V4_HAND_LEFT_STATE_TOPIC",
    "V4_HAND_RIGHT_STATE_TOPIC",
    # 夹爪
    "GRIP_LEFT_CMD_TOPIC",
    "GRIP_RIGHT_CMD_TOPIC",
    "GRIP_LEFT_STATE_TOPIC",
    "GRIP_RIGHT_STATE_TOPIC",
    "GRIP_POSITION_LIMIT",
    "GRIP_FORCE_LIMIT",
    "GRIP_VELOCITY_LIMIT",
    "GRIP_ACCELERATION_LIMIT",
    # 头部测试
    "HEAD_TEST_AMPLITUDE",
    "HEAD_TEST_PERIOD",
    "HEAD_TEST_DEFAULT_CYCLES",
    # 预备姿态 / 归零姿态
    "READY_POSE",
    "READY_STAGE_1_POSE",
    "READY_STAGE_2_POSE",
    "HOME_POSE",
    "HOME_STAGE_1_POSE",
    "HOME_STAGE_2_POSE",
    "LEGACY_READY_STAGE_1_PITCH_ROLL_POSE",
    "LEGACY_READY_STAGE_1_ELBOW_YAW_POSE",
    "LEGACY_READY_STAGE_2_POSE",
]

# ============================================================================
# 默认参数
# ============================================================================

DEFAULT_COMMAND_TOPIC = "/mc/sdk/robot_command"
DEFAULT_STATE_TOPIC = "/mc/sdk/robot_state"
DEFAULT_CONTROL_HZ = 500  # 对齐 pub_arm_command / SDK demo 基线（500Hz，2ms/点）
DEFAULT_MAX_JOINT_SPEED = 6.28  # rad/s，安全速度上限
DEFAULT_LOCK_JOINTS = ["head_pitch_joint", "head_yaw_joint", "waist_yaw_joint"]

# ============================================================================
# PVT 力位混合模式（JointCmd.CUSTOM_MODE_1 = 7）参数
# 控制律：tau = Kp·(q_des − q) + Kd·(dq_des − dq) + effort_ff
#   position = q_des（目标位置）
#   velocity = dq_des（速度前馈，规划速度）
#   effort   = effort_ff（力矩前馈，如重力补偿，默认 0）
#   v1 = Kp, v2 = Kd
# ⚠️ 以下 Kp/Kd 是 deliberately soft 的未验证占位值，必须真机调！
#    Kp 太小 → 手臂下垂（重力补偿不足）；Kp 太大 → 振荡。
#    理想基线：容器内 config_mc_walker_s2_v1_sps 的 mode=2 内部增益。
# ============================================================================

# 对齐 pub_arm_command.py 的 PVT 基线增益（kp=50/kd=2，velocity=0 纯阻尼）：
# 原 30/1 过软 → 跟踪误差与振荡（抖动）。真机仍可经 pvt_kp/pvt_kd 覆盖。
_PVT_DEFAULT_KP = 200.0  # 位置增益（真机调试值）
_PVT_DEFAULT_KD = 50.0   # 速度增益（阻尼，真机调试值）

# 轨迹插值 profile
PROFILE_LINEAR = "linear"
PROFILE_QUINTIC = "quintic"   # s(τ)=10τ³−15τ⁴+6τ⁵，起止速度/加速度均为 0 → 无 jerk 阶跃

# ============================================================================
# 关节定义（原 utars_clamp_and_place_large_bio_box_in_test_field.yaml 中的
# actions.joints 段，硬编码以消除对配置文件的依赖）
# ============================================================================

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

# 关节限位（rad），来源：Walker S2 硬件规格书
# 键 = 关节名（与 BODY_JOINT_NAMES 一致），值 = (lower, upper)
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
    "R_shoulder_yaw_joint":     (-2.8972, 2.8972),
    "R_wrist_pitch_joint":      (-1.5882, 1.5882),
    "R_wrist_roll_joint":       (-1.9897, 1.9897),
    "head_pitch_joint":         (-0.6807, 0.5061),
    "head_yaw_joint":           (-1.6406, 1.6406),
    "waist_yaw_joint":          (-2.7925, 2.7925),
}

# V4 手部关节限位（rad），左右手相同
# 键 = 短名（去掉 left_/right_ 前缀），查找时 removeprefix 即可
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
# 头部周期运动测试参数
# 参考：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/example/src/walker_s2/
#       low_level/pub_head_command.cpp
#
# 原 SDK demo：500Hz 发布，position = sin(time_cnt) * 0.5，time_cnt += 0.002
# 对应连续函数：position = sin(2π * t / T) * amplitude
# 其中：振幅 0.5 rad，时间步 0.002s（500Hz），周期 T = 2π ≈ 6.28s
# ============================================================================

HEAD_TEST_AMPLITUDE = 0.5    # 振幅（弧度），约 28.6°
HEAD_TEST_PERIOD = 2 * np.pi  # 周期（秒），约 6.28s
HEAD_TEST_DEFAULT_CYCLES = 2  # 默认运动周期数

# ============================================================================
# V4 手部周期运动测试参数
# 参考：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/example/src/walker_s2/
#       low_level/pub_hand_v4_command.cpp
#
# V4 手 = 单手 7 关节（含 thumb_pip，区别于 V3 手的 6 关节）
# 原 SDK demo：500Hz 发布，position = sin(time_cnt + i * 0.2) * 0.6
#               每个关节相位差 0.2 rad，mode=5（手部控制器自定义模式）
#
# 注意：
#   - 手部走独立通路：JointCommand 消息 + /mc/{left,right}_hand/command 话题
#   - 不需要 switch_controller config_mc_walker_s2_v1_sps（手部控制器始终监听）
#   - 与身体关节完全独立，不在 YAML config 中
# ============================================================================

V4_HAND_LEFT_JOINTS = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_thumb_pip",      # V4 独有，V3 没有此关节
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

V4_HAND_RIGHT_JOINTS = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_thumb_pip",     # V4 独有
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

V4_HAND_TEST_AMPLITUDE = 0.6        # 振幅（rad），与 SDK demo 一致
V4_HAND_TEST_PERIOD = 2 * np.pi     # 周期（s），与 SDK demo 一致（time_cnt += 0.002 @500Hz）
V4_HAND_TEST_PHASE_DIFF = 0.2       # 关节间相位差（rad），与 SDK demo 一致
V4_HAND_TEST_DEFAULT_CYCLES = 2     # 默认循环数
V4_HAND_TEST_HZ = 200               # 手部测试发布频率
V4_HAND_LEFT_TOPIC = "/mc/left_hand/command"
V4_HAND_RIGHT_TOPIC = "/mc/right_hand/command"
V4_HAND_LEFT_STATE_TOPIC = "/mc/left_hand/joint_states"
V4_HAND_RIGHT_STATE_TOPIC = "/mc/right_hand/joint_states"

GRIP_LEFT_CMD_TOPIC = "/ecat/left_grip/cmd"
GRIP_RIGHT_CMD_TOPIC = "/ecat/right_grip/cmd"
GRIP_LEFT_STATE_TOPIC = "/ecat/left_grip/state"
GRIP_RIGHT_STATE_TOPIC = "/ecat/right_grip/state"
GRIP_POSITION_LIMIT = (0.0, 0.05)     # m
GRIP_FORCE_LIMIT = (41.0, 100.0)      # N
GRIP_VELOCITY_LIMIT = (0.0, 0.01)     # m/s
GRIP_ACCELERATION_LIMIT = (0.0, 3.0)  # m/s^2，复用 GripCmd.cur 字段

LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_wrist_pitch_joint", "L_wrist_roll_joint",
]
RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint", "R_wrist_roll_joint",
]

# 手部关节查找表：side → (joint_names_list, publisher_topic)
V4_HAND_JOINT_MAP = {
    "left": V4_HAND_LEFT_JOINTS,
    "right": V4_HAND_RIGHT_JOINTS,
}

# 手部预设姿态（用于 --hand-open / --hand-close）
V4_HAND_OPEN_POSE = {name: 0.0 for name in V4_HAND_JOINT_LIMITS}
V4_HAND_CLOSE_POSE = {name: hi for name, (_, hi) in V4_HAND_JOINT_LIMITS.items()}

# ============================================================================
# 预备姿态（双臂抬起预备抓取的站立位姿）
# ============================================================================

READY_POSE = {
    "L_elbow_roll_joint":       -1.700,
    "L_elbow_yaw_joint":        2.8800,
    # "L_elbow_yaw_joint":        1.5000,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.700,
    "R_elbow_yaw_joint":        -2.8800,
    # "R_elbow_yaw_joint":        -1.5000,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

# ---- 分段初始化中间姿态（BT pick_ready_pose_sequence，三段 5/5/3s，与仿真侧对齐）----
# 数值逐关节照抄仿真 utils/constants.py，保证 sim2real init 路径一致。
# 段 1 肩部外展 + 旋转：shoulder_roll=-0.65、shoulder_yaw=∓2.0，elbow_roll 展开到 0
READY_STAGE_1_POSE = {
    "L_shoulder_pitch_joint": 0.0000,
    "L_shoulder_roll_joint": -0.6500,
    "L_shoulder_yaw_joint": -2.0000,
    "L_elbow_roll_joint": 0.0000,
    "L_elbow_yaw_joint": 1.5000,
    "L_wrist_pitch_joint": 0.0000,
    "L_wrist_roll_joint": 0.0000,
    "R_shoulder_pitch_joint": 0.0000,
    "R_shoulder_roll_joint": -0.6500,
    "R_shoulder_yaw_joint": 2.0000,
    "R_elbow_roll_joint": 0.0000,
    "R_elbow_yaw_joint": -1.5000,
    "R_wrist_pitch_joint": 0.0000,
    "R_wrist_roll_joint": 0.0000,
}

# 段 2 肩部俯仰 + 折叠肘部 + 肘部旋转：shoulder_pitch=∓0.7、elbow_roll=-1.7
READY_STAGE_2_POSE = {
    "L_shoulder_pitch_joint": -0.7000,
    "L_shoulder_roll_joint": -0.6500,
    "L_shoulder_yaw_joint": -2.0000,
    "L_elbow_roll_joint": -1.7000,
    "L_elbow_yaw_joint": 1.5000,
    "L_wrist_pitch_joint": 0.0000,
    "L_wrist_roll_joint": 0.0000,
    "R_shoulder_pitch_joint": 0.7000,
    "R_shoulder_roll_joint": -0.6500,
    "R_shoulder_yaw_joint": 2.0000,
    "R_elbow_roll_joint": -1.7000,
    "R_elbow_yaw_joint": -1.5000,
    "R_wrist_pitch_joint": 0.0000,
    "R_wrist_roll_joint": 0.0000,
}
# 段 3 最终姿态 = READY_POSE（含 head/waist 归位）

# ---- home 归零流程（BT return_zero_sequence，三段 5/5/5s，与仿真侧对齐）----
# home 与 init 共用中间姿态（逆序）：段 1 收肩俯仰 = READY_STAGE_2_POSE，
# 段 2 展开肘部 = READY_STAGE_1_POSE，段 3 全部归零 = HOME_POSE。
HOME_POSE = {name: 0.0 for name in BODY_JOINT_NAMES}
HOME_STAGE_1_POSE = READY_STAGE_2_POSE
HOME_STAGE_2_POSE = READY_STAGE_1_POSE

# ---- 旧 4 段流程中间姿态（--legacy-init 回退用，抬臂过肩路径）----
LEGACY_READY_STAGE_1_PITCH_ROLL_POSE = {
    "L_shoulder_yaw_joint": -1.5600,
    "R_shoulder_yaw_joint": 1.5600,
    "L_elbow_yaw_joint": 1.5000,
    "R_elbow_yaw_joint": -1.5000,
}

LEGACY_READY_STAGE_1_ELBOW_YAW_POSE = {
    "L_shoulder_pitch_joint":   -2.000,
    "R_shoulder_pitch_joint":   2.000,
    "L_wrist_pitch_joint": 0.8000,
    "R_wrist_pitch_joint": -0.8000,
    "L_elbow_roll_joint":        -2.5000,
    "R_elbow_roll_joint":        -2.5000,
}

LEGACY_READY_STAGE_2_POSE = {
    "L_shoulder_pitch_joint": READY_POSE["L_shoulder_pitch_joint"],
    "R_shoulder_pitch_joint": READY_POSE["R_shoulder_pitch_joint"],
}
