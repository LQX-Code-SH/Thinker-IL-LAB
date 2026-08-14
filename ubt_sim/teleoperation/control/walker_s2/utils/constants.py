"""Walker S2 teleoperation constants.

This module is independent from source/ubt_sim because ROS2 control scripts run
with system Python while Isaac Sim runs with its own Python environment.
"""

import numpy as np

DEFAULT_COMMAND_TOPIC = "/mc/sdk/robot_command"
DEFAULT_STATE_TOPIC = "/mc/sdk/robot_state"
DEFAULT_LEFT_HAND_COMMAND_TOPIC = "/mc/left_hand/command"
DEFAULT_RIGHT_HAND_COMMAND_TOPIC = "/mc/right_hand/command"
DEFAULT_LEFT_HAND_STATE_TOPIC = "/mc/left_hand/joint_states"
DEFAULT_RIGHT_HAND_STATE_TOPIC = "/mc/right_hand/joint_states"
DEFAULT_LEFT_GRIP_COMMAND_TOPIC = "/ecat/left_grip/cmd"
DEFAULT_RIGHT_GRIP_COMMAND_TOPIC = "/ecat/right_grip/cmd"
DEFAULT_LEFT_GRIP_STATE_TOPIC = "/ecat/left_grip/state"
DEFAULT_RIGHT_GRIP_STATE_TOPIC = "/ecat/right_grip/state"
DEFAULT_RESET_TOPIC = "/sim/cmd_reset"
DEFAULT_FINGER_LINK_STATES_TOPIC = "/sim/finger_link_states"
DEFAULT_IMAGE_RGB_TOPIC = "/sensor/camera/stereo/color/raw"
DEFAULT_IMAGE_DEPTH_TOPIC = "/sensor/camera/stereo/depth/raw"

# 四路独立相机 topic（commit 961d319 新增，shm_msgs/Image2m，640x480 RGB）
DEFAULT_IMAGE_STEREO_LEFT_TOPIC = "/sensor/camera/stereo_left/image/raw"
DEFAULT_IMAGE_STEREO_RIGHT_TOPIC = "/sensor/camera/stereo_right/image/raw"
DEFAULT_IMAGE_WRIST_LEFT_TOPIC = "/sensor/camera/wrist_left/color/raw"
DEFAULT_IMAGE_WRIST_RIGHT_TOPIC = "/sensor/camera/wrist_right/color/raw"

CAMERA_TOPICS = {
    "stereo_left":  {"topic": DEFAULT_IMAGE_STEREO_LEFT_TOPIC,  "msg_type": "Image6m", "isolate": True},
    "stereo_right": {"topic": DEFAULT_IMAGE_STEREO_RIGHT_TOPIC, "msg_type": "Image6m", "isolate": True},
    "wrist_left":   {"topic": DEFAULT_IMAGE_WRIST_LEFT_TOPIC,   "msg_type": "Image1m", "isolate": False},
    "wrist_right":  {"topic": DEFAULT_IMAGE_WRIST_RIGHT_TOPIC,  "msg_type": "Image1m", "isolate": False},
}

DEFAULT_CONTROL_HZ = 200
DEFAULT_MAX_JOINT_SPEED = 6.28  # rad/s，安全速度上限
DEFAULT_LOCK_JOINTS = ["head_pitch_joint", "head_yaw_joint", "waist_yaw_joint"]

# 夹爪参数（GRIP_*，原 walker_s2_controller 独有，合并为单一来源）
GRIP_OPENING_MIN_M = 0.0
GRIP_OPENING_MAX_M = 0.05
GRIP_DEFAULT_VEL = 0.05
GRIP_DEFAULT_FORCE = 20.0

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
V4_HAND_LEFT_TOPIC = DEFAULT_LEFT_HAND_COMMAND_TOPIC
V4_HAND_RIGHT_TOPIC = DEFAULT_RIGHT_HAND_COMMAND_TOPIC
V4_HAND_LEFT_STATE_TOPIC = DEFAULT_LEFT_HAND_STATE_TOPIC
V4_HAND_RIGHT_STATE_TOPIC = DEFAULT_RIGHT_HAND_STATE_TOPIC

# 手部关节查找表：side → (joint_names_list, publisher_topic)
V4_HAND_JOINT_MAP = {
    "left": V4_HAND_LEFT_JOINTS,
    "right": V4_HAND_RIGHT_JOINTS,
}

# 手部预设姿态（用于 --hand-open / --hand-close）
V4_HAND_OPEN_POSE = {name: 0.0 for name in V4_HAND_JOINT_LIMITS}
V4_HAND_CLOSE_POSE = {name: hi for name, (_, hi) in V4_HAND_JOINT_LIMITS.items()}
HOME_POSE = {name: 0.0 for name in BODY_JOINT_NAMES}
LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_wrist_pitch_joint", "L_wrist_roll_joint",
]
RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint", "R_wrist_roll_joint",
]

# ============================================================================
# 预备姿态（双臂抬起预备抓取的站立位姿）
# 注：以下为 walker_s2_controller.move_to_ready_pose 实际执行的活值。
# ============================================================================

READY_POSE = {
    "L_elbow_roll_joint":       -1.7000,
    "L_elbow_yaw_joint":        2.8790,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.7000,
    "R_elbow_yaw_joint":        -2.8790,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

# 搬箱预备姿态：elbow_yaw = ±1.5（前臂指向前方），便于双臂靠近桌面箱子。
# 其余关节与 READY_POSE 一致。
CARRY_READY_POSE = {
    "L_elbow_roll_joint":       -1.7000,
    "L_elbow_yaw_joint":        1.5000,
    "L_shoulder_pitch_joint":   0.0000,
    "L_shoulder_roll_joint":    -0.1500,
    "L_shoulder_yaw_joint":     -1.5600,
    "L_wrist_pitch_joint":      0.0000,
    "L_wrist_roll_joint":       0.0000,
    "R_elbow_roll_joint":       -1.7000,
    "R_elbow_yaw_joint":        -1.5000,
    "R_shoulder_pitch_joint":   0.0000,
    "R_shoulder_roll_joint":    -0.1500,
    "R_shoulder_yaw_joint":     1.5600,
    "R_wrist_pitch_joint":      0.0000,
    "R_wrist_roll_joint":       0.0000,
    "head_pitch_joint":         -0.6500,
    "head_yaw_joint":           0.0000,
    "waist_yaw_joint":          0.0000,
}

# ---- 分段初始化中间姿态（BT pick_ready_pose_sequence，三段 5/5/3s）----
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

# home 流程（BT return_zero_sequence，三段 5/5/5s）与 init 流程共用中间姿态（逆序）：
#   段 1 收肩俯仰 = READY_STAGE_2_POSE
#   段 2 展开肘部 = READY_STAGE_1_POSE
#   段 3 全部归零 = HOME_POSE（双臂 + 头部 + 腰部全 0）
HOME_STAGE_1_POSE = READY_STAGE_2_POSE
HOME_STAGE_2_POSE = READY_STAGE_1_POSE


LEFT_HAND_JOINTS = V4_HAND_LEFT_JOINTS
RIGHT_HAND_JOINTS = V4_HAND_RIGHT_JOINTS
HAND_OPEN_POSE = V4_HAND_OPEN_POSE
HAND_CLOSE_POSE = V4_HAND_CLOSE_POSE
