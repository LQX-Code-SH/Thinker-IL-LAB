#!/usr/bin/env python3
"""
Walker S2 机器人直接控制脚本

从 executor_node_sdk.py 提取，移除 VLA 推理依赖（不订阅 Gr00tMotionChunk），
保留核心的安全检查、线性插值、关节锁定、200Hz 发布逻辑，
提供 Python API 直接控制真机。

控制方法：方法 2（SDK 控制器，RobotCommand/JointCmd，MODE_POSITION=2）
话题：/mc/sdk/robot_command（发布），/mc/sdk/robot_state（订阅）

【运行前置条件】

1. 必须先在运控容器中启动运控并切换到 SDK 控制器：

    docker exec -it walker-motion.manipulation_robot_app-1 bash
    source /opt/walker/setup.bash
    rosa run t800_mc_server start_mc_client
    rosa run rosa_controllers switch_controller config_mc_walker_s2_v1_sps

2. 确保机器人处于安全位置（先用遥控器移到安全位置再启动控制器）

3. 在控制容器中执行此脚本前需要 source 环境：

    source /home/ubt/additional/scripts/setup.sh

【使用示例】

    # 命令行：
    python3 robot_control.py --print-state            # 仅打印当前关节位置
    python3 robot_control.py --demo                   # 运行安全演示（小幅运动）

    # Python API：
    from robot_control import RobotController
    import rclpy, threading
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    controller = RobotController(
        lock_joints=['head_pitch_joint', 'head_yaw_joint', 'waist_yaw_joint'],
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    threading.Thread(target=executor.spin, daemon=True).start()

    controller.wait_for_state(timeout=5.0)
    pos = controller.get_current_position()
    target = pos.copy()
    target[controller.joint_index('R_elbow_yaw_joint')] += 0.1
    controller.move_to_position(target, duration_sec=2.0)

    rclpy.shutdown()
"""

import argparse
import json
import os
import sys
import threading
import time
from collections import deque
from contextlib import contextmanager
from typing import Optional

import yaml

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from ecat_task_msgs.msg import GripCmd, GripStatus
    from mc_state_msgs.msg import RobotState
    from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
except ImportError as exc:
    raise ImportError(
        "Walker S2 ROS2 SDK messages not found. Source ROS2 and the vendored Walker SDK messages first, e.g.\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash\n"
        "Then run this control script with /usr/bin/python3, not Isaac Sim Python."
    ) from exc

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


# ============================================================================
# 常量（单一来源：utils.constants；此处 import 后 re-export，
# 消费方可 from utils.controller import BODY_JOINT_NAMES / READY_POSE / ...）
# ============================================================================
from .constants import *  # noqa: F401,F403

SIXFORCE_LINKS = {
    "left": "L_sixforce_link",
    "right": "R_sixforce_link",
}
FINGER_PREFIX = {
    "left": "L_finger",
    "right": "R_finger",
}

ROBOT_WORLD_POS = (0.7, -0.2, 0.9)
ROBOT_WORLD_ROT_WXYZ = (0.7071068, 0.0, 0.0, 0.7071068)
DEFAULT_TARGET_WORLD_POS = (1.00213, 0.50822, 1.13042)
IK_KWARGS = {
    "max_iter": 200,
    "pos_tol": 1e-2,
    "rot_tol": 5e-2,
    "rot_weight": 0.2,
    "rot_axis_weights": (0.2, 0.2, 1.0),
    "null_weight": 0.1,
    "unlock_waist": False,
    "task_type": "default",
    "num_random_seeds": 0,
    "use_hierarchical": False,
}



# ============================================================================
# 模块级工具函数
# ============================================================================
def _fmt(values, precision=5):
    if values is None:
        return "None"
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"


def _link_pose_text(name, pose):
    if pose is None:
        return f"{name}: None"
    return f"{name}: pos={_fmt(pose.get('pos'))}, rot(wxyz)={_fmt(pose.get('rot'))}"


def _mean_xyz(poses):
    valid = [pose.get("pos") for pose in poses if pose and pose.get("pos") is not None]
    if not valid:
        return None
    n = float(len(valid))
    return [sum(float(pos[i]) for pos in valid) / n for i in range(3)]


def _quat_wxyz_to_matrix(q):
    w, x, y, z = [float(v) for v in q]
    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if norm <= 0.0:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _world_xyz_to_base(xyz):
    rot = _quat_wxyz_to_matrix(ROBOT_WORLD_ROT_WXYZ)
    delta = [float(xyz[i]) - ROBOT_WORLD_POS[i] for i in range(3)]
    return [sum(rot[row][col] * delta[row] for row in range(3)) for col in range(3)]


def _rpy_to_rotation_matrix(roll, pitch, yaw):
    """RPY (intrinsic XYZ) → 3×3 旋转矩阵。

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，与 URDF / Pinocchio 约定一致。
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # Rx(roll)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    # Ry(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    # Rz(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def _rotation_matrix_to_rpy(R):
    """3×3 旋转矩阵 → RPY (intrinsic XYZ)，与 URDF / Pinocchio 约定一致。"""
    # pitch = atan2(-R[2,0], sqrt(R[0,0]^2 + R[1,0]^2))
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))

    if np.abs(np.cos(pitch)) > 1e-10:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock: pitch ≈ ±90°
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    return float(roll), float(pitch), float(yaw)


def _convert_ee_delta_local_to_base(current_xyzrpy, delta_xyz, delta_rpy):
    """将末端局部坐标系下的 delta 转换到机器人 base frame。

    Args:
        current_xyzrpy: 当前 EE 在 base frame 的 [x, y, z, roll, pitch, yaw]
        delta_xyz: 局部坐标系平移增量 (dx, dy, dz) — x=夹爪下方, y=夹爪左方, z=夹爪前方
        delta_rpy: 局部坐标系旋转增量 (droll, dpitch, dyaw)

    Returns:
        (delta_xyz_base, target_xyzrpy_base) — base frame 下的增量与绝对目标
    """
    x, y, z, roll, pitch, yaw = [float(v) for v in current_xyzrpy]
    dx, dy, dz = [float(v) for v in delta_xyz]
    droll, dpitch, dyaw = [float(v) for v in delta_rpy]

    # 当前 EE 在 base frame 的姿态矩阵
    R_ee = _rpy_to_rotation_matrix(roll, pitch, yaw)

    # 局部平移 → base frame 平移
    delta_xyz_base = R_ee @ np.array([dx, dy, dz])

    # 局部旋转 → 合成旋转 → 提取 RPY
    R_delta = _rpy_to_rotation_matrix(droll, dpitch, dyaw)
    R_new = R_ee @ R_delta
    new_roll, new_pitch, new_yaw = _rotation_matrix_to_rpy(R_new)

    target_xyzrpy = np.array([
        x + delta_xyz_base[0],
        y + delta_xyz_base[1],
        z + delta_xyz_base[2],
        new_roll, new_pitch, new_yaw,
    ])

    return delta_xyz_base, target_xyzrpy


def _world_xyzrpy_to_base(world_xyzrpy):
    """world frame 6D 位姿 → base frame 6D 位姿。

    平移：复用 _world_xyz_to_base。
    旋转：world_RPY → 旋转矩阵 → 消去机器人基座旋转 → base_RPY。
    """
    base_xyz = _world_xyz_to_base(world_xyzrpy[:3])
    R_world = _rpy_to_rotation_matrix(
        float(world_xyzrpy[3]), float(world_xyzrpy[4]), float(world_xyzrpy[5]))
    R_base_to_world = _quat_wxyz_to_matrix(ROBOT_WORLD_ROT_WXYZ)
    # R_target_in_base = R_base_to_world^T @ R_world
    R_target_in_base = np.array(R_base_to_world).T @ R_world
    base_roll, base_pitch, base_yaw = _rotation_matrix_to_rpy(R_target_in_base)
    return np.array([*base_xyz, base_roll, base_pitch, base_yaw])



# ============================================================================
# WalkerS2Controller — 主控制器
# ============================================================================


class WalkerS2Controller(Node):
    """Walker S2 SDK 控制器节点

    职责：
        1. 订阅 /mc/sdk/robot_state 维护最新关节位置
        2. 提供 move_to_position / execute_trajectory 等 API
        3. 200Hz 定时器发布 RobotCommand 到 /mc/sdk/robot_command
        4. 安全检查：最大关节速度
        5. 关节锁定：发布时跳过指定关节
        6. 关节限位：超限时自动裁剪到限位边界
    """


    # ========================================================================
    # [2.1] 初始化与配置
    # ========================================================================
    def __init__(
        self,
        node_name: str = "walker_s2_controller",
        command_topic: Optional[str] = None,
        state_topic: Optional[str] = None,
        config_path: Optional[str] = None,
        control_hz: float = DEFAULT_CONTROL_HZ,
        lock_joints=None,
        max_joint_speed: float = DEFAULT_MAX_JOINT_SPEED,
        enable_safety_check: bool = True,
        enable_limit_check: bool = True,
        subscribe_images: bool = True,
        enable_ik: bool = False,
        ik_urdf_path: Optional[str] = None,
        ik_auto_initialize: bool = True,
    ):
        super().__init__(node_name)

        self._config = self._load_config(config_path)
        command_topic = command_topic or self._get_topic("sub", "command", DEFAULT_COMMAND_TOPIC)
        state_topic = state_topic or self._get_topic("pub", "state", DEFAULT_STATE_TOPIC)
        left_hand_topic = self._get_topic("sub", "left_hand_command", DEFAULT_LEFT_HAND_COMMAND_TOPIC)
        right_hand_topic = self._get_topic("sub", "right_hand_command", DEFAULT_RIGHT_HAND_COMMAND_TOPIC)
        left_hand_state_topic = self._get_topic("pub", "left_hand_state", DEFAULT_LEFT_HAND_STATE_TOPIC)
        right_hand_state_topic = self._get_topic("pub", "right_hand_state", DEFAULT_RIGHT_HAND_STATE_TOPIC)
        left_grip_topic = self._get_topic("sub", "left_grip_command", DEFAULT_LEFT_GRIP_COMMAND_TOPIC)
        right_grip_topic = self._get_topic("sub", "right_grip_command", DEFAULT_RIGHT_GRIP_COMMAND_TOPIC)
        left_grip_state_topic = self._get_topic("pub", "left_grip_state", DEFAULT_LEFT_GRIP_STATE_TOPIC)
        right_grip_state_topic = self._get_topic("pub", "right_grip_state", DEFAULT_RIGHT_GRIP_STATE_TOPIC)
        reset_topic = self._get_topic("sub", "reset", DEFAULT_RESET_TOPIC)
        finger_link_states_topic = self._get_topic("pub", "finger_link_states", DEFAULT_FINGER_LINK_STATES_TOPIC)

        # 关节配置（硬编码，原来自 YAML 文件）
        self.all_joints = list(BODY_JOINT_NAMES)
        self.n_joints = len(self.all_joints)

        # 控制参数
        self.control_hz = control_hz
        self.timer_period = 1.0 / control_hz
        self.max_joint_speed = max_joint_speed
        self.enable_safety_check = enable_safety_check
        self.enable_limit_check = enable_limit_check

        # 锁定关节
        self.lock_joints = set(DEFAULT_LOCK_JOINTS if lock_joints is None else lock_joints)

        # 状态缓冲
        self.robot_states_buffer = deque(maxlen=1)
        self.robot_states_buffer_lock = threading.Lock()

        # 缺失关节告警节流（避免 RobotState 中缺关节时 200Hz 日志洪流，与 hand_limit_warned 同理）
        self._missing_joint_warned: set = set()

        # 轨迹状态
        self.trajectory_lock = threading.Lock()
        self.current_trajectory = np.empty((0, self.n_joints), dtype=float)
        self.current_index = 0
        self.is_publishing = False
        self.safety_violation = False


        # QoS
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 订阅/发布
        self.state_sub = self.create_subscription(
            RobotState, state_topic, self._state_callback, qos_sub,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.command_pub = self.create_publisher(
            RobotCommand, command_topic, qos_pub
        )

        # 手部发布者（V4 手专用，走 JointCommand 通路，与身体控制独立）
        self.left_hand_pub = self.create_publisher(
            JointCommand, left_hand_topic, qos_pub
        )
        self.right_hand_pub = self.create_publisher(
            JointCommand, right_hand_topic, qos_pub
        )
        self._hand_pubs = {"left": self.left_hand_pub, "right": self.right_hand_pub}

        # 二指夹爪发布者（ECAT GripCmd 通路，仿真中控制 PGC 两指夹爪）
        self.left_grip_pub = self.create_publisher(
            GripCmd, left_grip_topic, qos_pub
        )
        self.right_grip_pub = self.create_publisher(
            GripCmd, right_grip_topic, qos_pub
        )
        self._grip_pubs = {"left": self.left_grip_pub, "right": self.right_grip_pub}

        # 手部状态订阅（/mc/{left,right}_hand/joint_states → sensor_msgs/JointState）
        self._hand_states = {}       # {"left": np.array(7), "right": np.array(7)}
        self._hand_state_lock = threading.Lock()
        self._hand_state_received = {
            "left": threading.Event(),
            "right": threading.Event(),
        }
        self._grip_states = {}       # {"left": {pos, vel, cur, ...}, "right": {...}}
        self._grip_state_lock = threading.Lock()
        self._grip_state_received = {
            "left": threading.Event(),
            "right": threading.Event(),
        }
        self._finger_link_states = None
        self._finger_link_states_lock = threading.Lock()
        self._finger_link_states_received = threading.Event()

        self.ik_solver = None
        self._ik_initialized = False
        self._ik_lock = threading.Lock()
        self._ik_urdf_path = ik_urdf_path

        self.left_hand_state_sub = self.create_subscription(
            JointState, left_hand_state_topic,
            lambda msg: self._hand_state_callback("left", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.right_hand_state_sub = self.create_subscription(
            JointState, right_hand_state_topic,
            lambda msg: self._hand_state_callback("right", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.left_grip_state_sub = self.create_subscription(
            GripStatus, left_grip_state_topic,
            lambda msg: self._grip_state_callback("left", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.right_grip_state_sub = self.create_subscription(
            GripStatus, right_grip_state_topic,
            lambda msg: self._grip_state_callback("right", msg),
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.finger_link_states_sub = self.create_subscription(
            String, finger_link_states_topic, self._finger_link_state_callback,
            qos_sub, callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.reset_pub = self.create_publisher(Bool, reset_topic, 1)

        # 200Hz 控制定时器
        self.control_timer = self.create_timer(
            self.timer_period, self._control_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        if enable_ik and ik_auto_initialize:
            if not self.initialize_ik():
                self.get_logger().warning("Walker S2 IK auto-initialization failed; joint-space control remains available")

        self.get_logger().info(
            f"WalkerS2Controller initialized: {self.n_joints} joints, "
            f"{control_hz}Hz, locked={sorted(self.lock_joints)}, "
            f"limit_check={self.enable_limit_check}"
        )
    @staticmethod
    def _load_config(config_path: Optional[str] = None) -> dict:
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                os.pardir, os.pardir, os.pardir, "bridges", "walker_s2_bridge_config.yaml"
            )
        try:
            with open(config_path) as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}
    def _get_topic(self, section: str, key: str, default: str) -> str:
        # section/key names follow walker_s2_bridge_config.yaml from the bridge perspective.
        try:
            return self._config["topics"][section][key]["topic"]
        except (KeyError, TypeError):
            return default

    # ========================================================================
    # [2.2] 状态查询 API
    # ========================================================================
    def wait_for_state(self, timeout=5.0):
        """阻塞等待第一个机器人状态消息。

        Args:
            timeout: 超时时间（秒）
        Returns:
            bool: True=收到状态，False=超时
        """
        start = time.time()
        while time.time() - start < timeout:
            with self.robot_states_buffer_lock:
                if len(self.robot_states_buffer) > 0:
                    self.get_logger().info(
                        f"Robot state received (took {time.time() - start:.2f}s)"
                    )
                    return True
            time.sleep(0.05)
        self.get_logger().error(f"Timeout waiting for robot state ({timeout}s)")
        return False

    def get_current_position(self):
        """获取最新关节位置（numpy 数组，n_joints 维），None 表示无数据"""
        with self.robot_states_buffer_lock:
            if len(self.robot_states_buffer) > 0:
                return self.robot_states_buffer[-1].copy()
        return None
    def joint_index(self, joint_name):
        """获取关节名对应的索引"""
        if joint_name not in self.all_joints:
            raise ValueError(f"Unknown joint: {joint_name}")
        return self.all_joints.index(joint_name)

    @property
    def joint_names(self):
        """所有关节名列表（只读）"""
        return list(self.all_joints)
    def wait_until_position(self, target_position, timeout=5.0, tolerance=0.05, ignored_joints=None):
        """等待实际关节位置收敛到目标附近。

        execute_trajectory(wait=True) 只表示轨迹点发布完毕；仿真/真机实际关节
        还需要继续收敛。此方法基于 RobotState 检查实际位置误差。
        """
        target = np.array(target_position, dtype=float)
        if target.shape != (self.n_joints,):
            self.get_logger().error(f"Target shape {target.shape} != ({self.n_joints},)")
            return False, []

        ignored = set(ignored_joints or [])
        check_indices = [i for i, name in enumerate(self.all_joints) if name not in ignored]
        deadline = time.time() + timeout
        last_pos = None

        while time.time() < deadline:
            pos = self.get_current_position()
            if pos is not None:
                last_pos = pos
                err = np.abs(pos - target)
                if check_indices and float(np.max(err[check_indices])) <= tolerance:
                    return True, []
            time.sleep(0.05)

        if last_pos is None:
            return False, [(name, None, float(target[i]), None) for i, name in enumerate(self.all_joints)]

        err = np.abs(last_pos - target)
        misses = [
            (self.all_joints[i], float(last_pos[i]), float(target[i]), float(err[i]))
            for i in check_indices
            if err[i] > tolerance
        ]
        misses.sort(key=lambda item: item[3], reverse=True)
        return False, misses

    # ========================================================================
    # [2.3] 关节空间运动控制
    # ========================================================================
    def move_to_position(self, target_position, duration_sec=3.0, wait=True):
        """平滑移动到目标位置（quintic 插值，起止速度/加速度均为 0）。

        使用 quintic 多项式 s(τ)=10τ³−15τ⁴+6τ⁵ 生成轨迹，保证起始和终止时刻
        速度=0、加速度=0，无 jerk 阶跃。始终发布全部未锁定关节。

        Args:
            target_position: 目标关节位置，长度 n_joints 的列表或 numpy 数组
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功（已开始/完成），False=失败
        """
        target = np.array(target_position, dtype=float)
        if target.shape != (self.n_joints,):
            self.get_logger().error(
                f"Target shape {target.shape} != ({self.n_joints},)"
            )
            return False

        # 限位裁剪
        if self.enable_limit_check:
            target, violations = self._clamp_position(target)
            if violations:
                for name, val, lo, hi in violations:
                    self.get_logger().warning(
                        f"CLAMPED {name}: {val:.4f} → [{lo}, {hi}]"
                    )

        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position available")
            return False

        # 起点+终点 → 逐关节 quintic 插值
        # s(τ) = 10τ³ − 15τ⁴ + 6τ⁵，保证起止 s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
        n_pts = max(2, int(duration_sec * self.control_hz))
        tau = np.linspace(0.0, 1.0, n_pts)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        trajectory = np.column_stack([
            current[j] + s * (target[j] - current[j])
            for j in range(self.n_joints)
        ])

        return self.execute_trajectory(trajectory, wait=wait)
    def execute_trajectory(self, trajectory, wait=True):
        """执行预定义轨迹。始终发布全部未锁定关节。

        Args:
            trajectory: numpy 数组 (N, n_joints)，每行一个时间步的关节位置
                        点间距按 1/control_hz 秒（200Hz → 5ms/点）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败（维度错误/安全违规）
        """
        trajectory = np.array(trajectory, dtype=float)
        if trajectory.ndim != 2 or trajectory.shape[1] != self.n_joints:
            self.get_logger().error(
                f"Trajectory shape {trajectory.shape} != (N, {self.n_joints})"
            )
            return False

        # NaN/inf 守卫：非有限值会绕过 _clamp_with_limits（NaN 比较恒 False）
        # 与安全速度检查，直达机器人/PhysX 致炸机。此处早拒。
        if not np.isfinite(trajectory).all():
            self.get_logger().error(
                "Trajectory contains non-finite values (NaN/inf); refusing to execute."
            )
            return False

        # 限位裁剪
        if self.enable_limit_check:
            all_violations = []
            for i in range(len(trajectory)):
                trajectory[i], viols = self._clamp_position(trajectory[i])
                all_violations.extend(viols)
            if all_violations:
                seen = set()
                for name, val, lo, hi in all_violations:
                    if name not in seen:
                        seen.add(name)
                        self.get_logger().warning(
                            f"CLAMPED {name} in trajectory to [{lo}, {hi}]"
                        )

        # 安全检查：最大关节速度（检查全部未锁定关节）
        if self.enable_safety_check and len(trajectory) >= 2:
            max_speeds = np.max(
                np.abs(np.diff(trajectory, axis=0)) / self.timer_period, axis=0
            )
            unsafe = []
            for i, name in enumerate(self.all_joints):
                if name in self.lock_joints:
                    continue
                if max_speeds[i] > self.max_joint_speed:
                    unsafe.append((name, max_speeds[i]))
            if unsafe:
                self.get_logger().error(
                    f"SAFETY VIOLATION: {len(unsafe)} joints exceed "
                    f"{self.max_joint_speed} rad/s"
                )
                for name, speed in unsafe:
                    self.get_logger().error(f"  {name}: {speed:.3f} rad/s")
                self.safety_violation = True
                return False

        # 写入轨迹
        with self.trajectory_lock:
            self.current_trajectory = trajectory.copy()
            self.current_index = 0
            self.is_publishing = True
            self.safety_violation = False

        self.get_logger().info(
            f"Executing trajectory: {len(trajectory)} points, "
            f"~{len(trajectory) / self.control_hz:.2f}s"
        )

        # 阻塞等待（带超时保护：轨迹时长 ×3 + 5s 兜底，防止 _control_callback
        # 停止后永久卡死调用线程）
        if wait:
            trajectory_duration = len(trajectory) / self.control_hz
            deadline = time.time() + max(trajectory_duration * 3.0, 5.0)
            while time.time() < deadline:
                with self.trajectory_lock:
                    if not self.is_publishing:
                        break
                time.sleep(0.01)
            else:
                self.get_logger().error(
                    f"Trajectory wait timed out ({max(trajectory_duration * 3.0, 5.0):.1f}s); "
                    f"forcing stop"
                )
                with self.trajectory_lock:
                    self.is_publishing = False
                    self.current_index = self.current_trajectory.shape[0]

        return True
    def stop(self):
        """立即停止发布指令（机器人保持在最后一个发送的位置）"""
        with self.trajectory_lock:
            self.is_publishing = False
            self.current_index = self.current_trajectory.shape[0]
        self.get_logger().info("Stop requested")
    def set_lock_joints(self, joint_names):
        """动态设置锁定关节列表"""
        self.lock_joints = set(joint_names or [])
        self.get_logger().info(f"Lock joints updated: {sorted(self.lock_joints)}")
    @property
    def is_busy(self):
        """是否正在执行轨迹"""
        with self.trajectory_lock:
            return self.is_publishing

    @contextmanager
    def _temporary_unlock(self, joint_names, wait=True):
        """上下文管理器：临时解锁指定关节，退出时自动恢复锁定状态。

        wait=False 时不恢复锁定：运动仍在发布（_control_callback 每点检查 lock_joints，
        跳过锁定关节），若此时 re-lock 会使轨迹中途丢弃这些关节、运动静默失效。
        调用方需在运动落定后自行 re-lock。
        """
        to_unlock = [j for j in joint_names if j in self.lock_joints]
        if not to_unlock:
            yield
            return
        original = self.lock_joints.copy()
        self.set_lock_joints(list(self.lock_joints - set(to_unlock)))
        self.get_logger().info(f"Temporarily unlocking joints: {sorted(to_unlock)}")
        try:
            yield
        finally:
            if wait:
                self.set_lock_joints(list(original))
            else:
                self.get_logger().info(
                    f"wait=False: joints {sorted(to_unlock)} left unlocked "
                    f"(trajectory still publishing); re-lock manually once settled."
                )

    @staticmethod
    def _clamp_with_limits(values, names, limits_dict, key_fn=None):
        """通用关节限位裁剪（身体和手部共用）。

        Args:
            values: 位置值列表
            names: 关节名列表
            limits_dict: {joint_key: (lower, upper)}
            key_fn: 将 names[i] 转为 limits_dict 的键（None=直接使用）
        Returns:
            (clamped_values, violations) — violations 是 [(name, val, lo, hi), ...]
        """
        key_fn = key_fn or (lambda x: x)
        clamped = list(values)
        violations = []
        for i, name in enumerate(names):
            key = key_fn(name)
            if key not in limits_dict:
                continue
            lo, hi = limits_dict[key]
            val = clamped[i]
            if val < lo:
                clamped[i] = lo
                violations.append((name, val, lo, hi))
            elif val > hi:
                clamped[i] = hi
                violations.append((name, val, lo, hi))
        return clamped, violations

    def _clamp_position(self, position):
        """裁剪身体关节位置到限位范围（委托 _clamp_with_limits）。"""
        lst, violations = self._clamp_with_limits(
            position.copy().tolist(), self.all_joints, BODY_JOINT_LIMITS)
        return np.array(lst), violations

    def _clamp_hand_position(self, joint_names, positions):
        """裁剪手部关节位置到限位范围（委托 _clamp_with_limits）。"""
        key_fn = lambda name: name.removeprefix("left_").removeprefix("right_")
        return self._clamp_with_limits(positions, joint_names, V4_HAND_JOINT_LIMITS, key_fn)

    def move_to_pose(self, pose_dict, duration_sec=1.5, wait=True,
                     unlock_required_joints=True,
                     settle_check=True, settle_timeout=None,
                     settle_tolerance=0.05):
        """按"关节名→角度"字典移动机器人。未指定的关节保持当前位置。

        相比 move_to_position（传整个 17 维向量），这个 API 更方便：
        只关心你要改的几个关节，其余自动从当前位置读取。

        Args:
            pose_dict: dict，键=关节名，值=目标弧度
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
            unlock_required_joints: 若目标关节在 lock_joints 中，是否临时解锁
                                    wait=True 时执行完自动恢复锁定；
                                    wait=False 时不恢复（无法感知完成时刻）
        Returns:
            bool: True=成功，False=失败
        """
        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position available")
            return False


        # 校验关节名 + 检测需要解锁的关节
        target = current.copy()
        joints_needing_unlock = []
        for joint_name, angle in pose_dict.items():
            if joint_name not in self.all_joints:
                self.get_logger().error(f"Unknown joint: {joint_name}")
                return False
            idx = self.all_joints.index(joint_name)
            target[idx] = float(angle)
            if joint_name in self.lock_joints:
                joints_needing_unlock.append(joint_name)

        target_joint_names = list(pose_dict.keys())

        if joints_needing_unlock and unlock_required_joints:
            with self._temporary_unlock(joints_needing_unlock, wait=wait):
                result = self.move_to_position(target, duration_sec=duration_sec, wait=wait)
        elif joints_needing_unlock and not unlock_required_joints:
            self.get_logger().warning(
                f"Joints {joints_needing_unlock} are locked; their target "
                f"values will be silently dropped"
            )
            result = self.move_to_position(target, duration_sec=duration_sec, wait=wait)
        else:
            result = self.move_to_position(target, duration_sec=duration_sec, wait=wait)

        settled = True
        if result and wait and settle_check:
            check_timeout = settle_timeout or max(8.0, float(duration_sec) * 3.0)
            # settle 只检查本次目标关节，其余关节不参与收敛判定
            ignored = set(self.all_joints) - set(target_joint_names)
            self.get_logger().info(
                f"Waiting for joints to converge (timeout={check_timeout:.1f}s, "
                f"tolerance={settle_tolerance:.3f} rad)..."
            )
            arrived, misses = self.wait_until_position(
                target,
                timeout=check_timeout,
                tolerance=settle_tolerance,
                ignored_joints=ignored,
            )
            if not arrived and misses:
                self.get_logger().warning(
                    f"Settle: {len(misses)} joint(s) not converged within "
                    f"{check_timeout:.1f}s (tolerance={settle_tolerance:.3f} rad): "
                    f"{misses[:5]}"
                )
            settled = arrived

        return bool(result and settled)
    def move_to_ready_pose(self, duration_sec=15.0, wait=True, staged=True):
        """分段移动到预备姿态（先 shoulder pitch / elbow roll，再 elbow yaw，最后 READY_POSE）。

        适用于实验开始前的初始化——将机器人从任意位置安全地移到统一的起始位姿。
        会临时解锁所有被锁定的关节（head/waist 等），以便完整执行预备姿态。

        Args:
            duration_sec: 分段运动总时长（秒），默认 10.0s
            wait: 保留 API 兼容；初始化分段为保证安全顺序始终阻塞执行
        Returns:
            bool: True=成功，False=失败
        """
        if not staged:
            return self.move_to_pose(
                READY_POSE,
                duration_sec=duration_sec,
                wait=wait,
                unlock_required_joints=True,
            )

        if not wait:
            self.get_logger().warning(
                "move_to_ready_pose(wait=False) requested, but staged init runs synchronously for safety"
            )

        duration_sec = float(duration_sec)
        # 4 段按 20%/35%/25%/20% 分配，stage 1b（2.6 rad × 6 关节）至少需要 ~5s
        if duration_sec < 15.0:
            self.get_logger().warning(
                f"Ready pose duration {duration_sec:.2f}s too short for staged motion "
                f"(stage 1b needs ~5s); using 15.00s"
            )
            duration_sec = 15.0

        # 各 stage 时长按最大位移分配（damping=40 下约 3.5s/rad 收敛）：
        #   1a: 1.56 rad × 4 关节 -> 20%
        #   1b: 2.6  rad × 6 关节 -> 35%（最大位移 + 最多关节，瓶颈 stage）
        #   2:  2.0  rad × 2 关节 -> 25%
        #   3:  0.9  rad × ~6 关节 -> 20%
        pitch_roll_duration = duration_sec * 0.20
        elbow_yaw_duration = duration_sec * 0.35
        other_duration = duration_sec * 0.25
        reset_duration = duration_sec * 0.20

        stages = [
            ("1a/3 肩 pitch + elbow roll", READY_STAGE_1_PITCH_ROLL_POSE, pitch_roll_duration),
            ("1b/3 elbow yaw", READY_STAGE_1_ELBOW_YAW_POSE, elbow_yaw_duration),
            ("2/3 肩 pitch 回到预备姿态", READY_STAGE_2_POSE, other_duration),
            ("3/3 执行完整 READY_POSE", READY_POSE, reset_duration),
        ]

        for label, pose, stage_duration in stages:
            self.get_logger().info(
                f"Ready pose stage {label}: {stage_duration:.2f}s"
            )
            # 每个 stage 都做 settle 检查，确保关节实际到位后再进入下一 stage。
            # 否则中间 stage 未收敛的误差会累积到最终 stage，叠加多关节同时运动
            # 的耦合动力学，导致 PD 控制器跟踪失败。
            if not self.move_to_pose(
                pose,
                duration_sec=stage_duration,
                wait=True,
                unlock_required_joints=True,
                settle_check=True,
                settle_tolerance=0.05,
            ):
                self.get_logger().error(f"Ready pose stage failed: {label}")
                return False

        return True
    def ready_position_vector(self):
        """返回 READY_POSE 对应的 17 维目标向量。"""
        return np.array([READY_POSE[name] for name in self.all_joints], dtype=float)
    def home(self, duration_sec=3.0, wait=True, unlock_required_joints=False):
        """移动到仿真 home 位姿（17 个 SDK body joints 全 0）。"""
        return self.move_to_pose(
            HOME_POSE,
            duration_sec=duration_sec,
            wait=wait,
            unlock_required_joints=unlock_required_joints,
        )
    def move_arm_joints(self, side, joints, duration_sec=1.5, wait=True):
        """按 7 维关节角移动单侧手臂；不做 Cartesian IK。"""
        if side not in ("left", "right"):
            self.get_logger().error(f"Invalid arm side: {side}")
            return False
        joint_names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        if len(joints) != len(joint_names):
            self.get_logger().error(f"{side} arm expects {len(joint_names)} joints, got {len(joints)}")
            return False
        return self.move_to_pose(
            dict(zip(joint_names, [float(v) for v in joints])),
            duration_sec=duration_sec,
            wait=wait,
            unlock_required_joints=True,
        )

    def move_left_arm_joints(self, joints, duration_sec=1.5, wait=True):
        return self.move_arm_joints("left", joints, duration_sec=duration_sec, wait=wait)

    def move_right_arm_joints(self, joints, duration_sec=1.5, wait=True):
        return self.move_arm_joints("right", joints, duration_sec=duration_sec, wait=wait)

    move_left_arm = move_left_arm_joints
    move_right_arm = move_right_arm_joints
    def get_joint_position(self, joint_name):
        """获取指定身体关节的当前位置（rad）。

        Args:
            joint_name: 关节名
        Returns:
            float: 当前位置，None 表示无数据或关节名无效
        """
        try:
            idx = self.joint_index(joint_name)
        except ValueError:
            self.get_logger().error(f"Unknown joint: {joint_name}")
            return None
        pos = self.get_current_position()
        if pos is None:
            return None
        return float(pos[idx])

    def move_joint(self, joint_name, target_rad, duration_sec=2.0, wait=True):
        """控制单个身体关节移动到目标角度，其他关节保持当前位置。

        等价于 move_to_pose({joint_name: target_rad}, ...)，解锁所需关节。
        """
        return self.move_to_pose(
            {joint_name: target_rad}, duration_sec=duration_sec,
            wait=wait, unlock_required_joints=True,
        )

    def shift_joint(self, joint_name, delta_rad, duration_sec=2.0, wait=True):
        """控制单个身体关节相对当前位置偏移。

        Args:
            joint_name: 关节名
            delta_rad: 偏移量（rad），正=正向，负=负向
        """
        current = self.get_joint_position(joint_name)
        if current is None:
            self.get_logger().error(f"Cannot shift {joint_name}: no current position")
            return False
        target = current + delta_rad

        if joint_name in BODY_JOINT_LIMITS:
            lo, hi = BODY_JOINT_LIMITS[joint_name]
            if target < lo or target > hi:
                clamped = max(lo, min(hi, target))
                self.get_logger().warning(
                    f"{joint_name}: target {target:.4f} rad exceeds limit "
                    f"[{lo}, {hi}], will be clamped to {clamped:.4f}"
                )

        self.get_logger().info(
            f"Shift {joint_name}: {current:.4f} -> {target:.4f} rad "
            f"(delta={delta_rad:+.4f} rad, {np.degrees(delta_rad):+.2f} deg)"
        )
        return self.move_joint(joint_name, target, duration_sec=duration_sec, wait=wait)

    # ========================================================================
    # [2.4] 内部回调（200Hz 状态订阅 + 控制发布）
    # ========================================================================
    def _state_callback(self, msg: RobotState):
        """从 RobotState 提取 n_joints 维位置向量（按 config 中的关节顺序）"""
        joint_states = msg.joint_states
        name_to_idx = {name: idx for idx, name in enumerate(joint_states.name)}

        positions = np.zeros(self.n_joints, dtype=float)
        missing = []
        for i, joint_name in enumerate(self.all_joints):
            if joint_name not in name_to_idx:
                missing.append(joint_name)
                continue
            positions[i] = joint_states.position[name_to_idx[joint_name]]

        # 有关节缺失时做节流告警，避免 200Hz 日志洪流（与 hand_limit_warned 同理）
        if missing:
            new_missing = set(missing) - self._missing_joint_warned
            if new_missing:
                self.get_logger().error(
                    f"Joints missing in RobotState: {sorted(new_missing)}"
                )
                self._missing_joint_warned.update(new_missing)
            return

        # 之前缺失的关节重新出现时清除告警标记
        if self._missing_joint_warned:
            self._missing_joint_warned.clear()
            self.get_logger().info("All joints restored in RobotState")

        with self.robot_states_buffer_lock:
            self.robot_states_buffer.append(positions)


    def _control_callback(self):
        """200Hz 定时回调：取轨迹点 -> 构造 RobotCommand -> 发布。

        仅在轨迹播放时发布；空闲时不发布，由仿真侧 action_process 的
        _hold_joint_targets 持续保持最后指令（含重力补偿）。
        """
        if self.safety_violation:
            return

        with self.trajectory_lock:
            if not self.is_publishing:
                return
            if self.current_index >= self.current_trajectory.shape[0]:
                self.is_publishing = False
                self.get_logger().info("Trajectory execution completed")
                return
            point = self.current_trajectory[self.current_index, :]
            self.current_index += 1

        cmd = RobotCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ""

        for idx, name in enumerate(self.all_joints):
            if name in self.lock_joints:
                continue
            jc = JointCmd()
            jc.name = name
            jc.control_mode = JointCmd.MODE_POSITION
            jc.position = float(point[idx])
            cmd.joint_cmd.append(jc)

        if cmd.joint_cmd:
            self.command_pub.publish(cmd)

    # ========================================================================
    # [2.5] V4 灵巧手控制（JointCommand / mode=5）
    # ========================================================================
    def hand_joint_names(self, side):
        """获取指定手的关节名列表。

        Args:
            side: "left" 或 "right"
        Returns:
            list[str]: 关节名列表
        Raises:
            ValueError: side 不是 "left" 或 "right"
        """
        if side not in V4_HAND_JOINT_MAP:
            raise ValueError(f"Invalid side '{side}', expected 'left' or 'right'")
        return list(V4_HAND_JOINT_MAP[side])

    def hand_joint_index(self, side, joint_name):
        """获取手部关节名在指定手中的索引。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名（如 "left_thumb_swing"）或短名（如 "thumb_swing"）
        Returns:
            int: 索引
        Raises:
            ValueError: 关节名无效
        """
        names = V4_HAND_JOINT_MAP[side]
        # 支持全名和短名两种写法
        full_name = f"{side}_{joint_name}" if not joint_name.startswith(side + "_") else joint_name
        if full_name in names:
            return names.index(full_name)
        # 也试试直接匹配
        if joint_name in names:
            return names.index(joint_name)
        raise ValueError(f"Unknown hand joint: {joint_name} (side={side})")

    def wait_for_hand_state(self, side=None, timeout=5.0):
        """阻塞等待手部状态消息。

        Args:
            side: "left"、"right" 或 None（等待双手）
            timeout: 超时时间（秒）
        Returns:
            bool: True=收到状态，False=超时
        """
        sides = ["left", "right"] if side is None else [side]
        deadline = time.time() + timeout
        for s in sides:
            remaining = max(0.0, deadline - time.time())
            if not self._hand_state_received[s].wait(timeout=remaining):
                self.get_logger().warning(f"Timeout waiting for {s} hand state ({remaining:.1f}s)")
                return False
        return True

    def get_hand_position(self, side):
        """获取指定手的手指关节当前位置。

        Args:
            side: "left" 或 "right"
        Returns:
            numpy.ndarray: 7 维位置数组，None 表示无数据
        """
        with self._hand_state_lock:
            if side in self._hand_states:
                return self._hand_states[side].copy()
        return None

    def get_hand_joint_position(self, side, joint_name):
        """获取指定手的单个手指关节当前位置。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名或短名
        Returns:
            float: 当前位置（rad），None 表示无数据或关节名无效
        """
        try:
            idx = self.hand_joint_index(side, joint_name)
        except ValueError:
            self.get_logger().error(f"Unknown hand joint: {joint_name} (side={side})")
            return None
        pos = self.get_hand_position(side)
        if pos is None:
            return None
        return float(pos[idx])

    def move_hand(self, side, pose_dict, duration_sec=2.0, wait=True):
        """按"关节名→角度"字典移动手指关节（线性插值 + 200Hz 发布）。

        未指定的关节保持当前位置。走 JointCommand 通路（mode=5），
        与身体 RobotCommand 通路完全独立。

        Args:
            side: "left" 或 "right"
            pose_dict: dict，键=关节名（全名或短名），值=目标弧度
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        if side not in V4_HAND_JOINT_MAP:
            self.get_logger().error(f"Invalid side '{side}'")
            return False

        joint_names = V4_HAND_JOINT_MAP[side]
        publisher = self._hand_pubs[side]

        # 获取当前位置
        current = self.get_hand_position(side)
        if current is None:
            self.get_logger().warning(
                f"No hand state for {side} hand, assuming zero position"
            )
            current = np.zeros(len(joint_names))

        # 构建目标位置
        target = current.copy()
        for name_or_short, angle in pose_dict.items():
            try:
                idx = self.hand_joint_index(side, name_or_short)
            except ValueError:
                self.get_logger().error(f"Unknown hand joint: {name_or_short}")
                return False
            target[idx] = float(angle)

        # 限位裁剪
        if self.enable_limit_check:
            target_list, violations = self._clamp_hand_position(joint_names, target.tolist())
            target = np.array(target_list)
            for name, val, lo, hi in violations:
                self.get_logger().warning(f"CLAMPED hand {name}: {val:.4f} → [{lo}, {hi}]")

        # 线性插值生成轨迹
        n_pts = max(2, int(duration_sec * V4_HAND_TEST_HZ))
        trajectory = np.column_stack([
            np.linspace(current[j], target[j], n_pts)
            for j in range(len(joint_names))
        ])

        if wait:
            self._execute_hand_trajectory(publisher, joint_names, trajectory)
        else:
            # 非阻塞：在后台线程执行
            t = threading.Thread(
                target=self._execute_hand_trajectory,
                args=(publisher, joint_names, trajectory),
                daemon=True,
            )
            t.start()

        return True

    def shift_hand(self, side, joint_name, delta_rad, duration_sec=2.0, wait=True):
        """控制手指关节相对当前位置偏移。

        Args:
            side: "left" 或 "right"
            joint_name: 关节全名或短名
            delta_rad: 偏移量（rad）
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功，False=失败
        """
        current = self.get_hand_joint_position(side, joint_name)
        if current is None:
            self.get_logger().error(
                f"Cannot shift {joint_name}: no current position"
            )
            return False
        target = current + delta_rad

        # 限位提示
        short = joint_name.removeprefix("left_").removeprefix("right_")
        if short in V4_HAND_JOINT_LIMITS:
            lo, hi = V4_HAND_JOINT_LIMITS[short]
            if target < lo or target > hi:
                clamped = max(lo, min(hi, target))
                self.get_logger().warning(
                    f"{joint_name}: target {target:.4f} rad exceeds limit "
                    f"[{lo}, {hi}], will be clamped to {clamped:.4f}"
                )

        self.get_logger().info(
            f"Shift hand {joint_name}: {current:.4f} → {target:.4f} rad "
            f"(Δ={delta_rad:+.4f} rad, {np.degrees(delta_rad):+.2f}°)"
        )

        return self.move_hand(side, {joint_name: target}, duration_sec=duration_sec, wait=wait)
    def send_hand_position(self, side, positions):
        """单次发送手指关节目标位置。

        Args:
            side: "left" 或 "right"
            positions: 位置列表或数组，长度 = 7
        """
        if side not in V4_HAND_JOINT_MAP:
            self.get_logger().error(f"Invalid side '{side}'")
            return False
        joint_names = V4_HAND_JOINT_MAP[side]
        publisher = self._hand_pubs[side]
        pos_list = [float(p) for p in positions]
        if self.enable_limit_check:
            pos_list, _ = self._clamp_hand_position(joint_names, pos_list)
        self._publish_hand_cmd(publisher, joint_names, pos_list)
        return True

    def _execute_hand_trajectory(self, publisher, joint_names, trajectory):
        """按轨迹逐点发布手部 JointCommand（阻塞执行）。

        使用 wall clock 对齐（而非累加 sleep），避免 sleep 不准确导致的累积时序漂移。

        Args:
            publisher: ROS2 publisher（JointCommand）
            joint_names: 关节名列表
            trajectory: numpy 数组 (N, n_hand_joints)
        """
        period = 1.0 / V4_HAND_TEST_HZ
        n_pts = trajectory.shape[0]
        start_time = time.time()

        for _ in range(n_pts):
            elapsed = time.time() - start_time
            if elapsed >= n_pts * period:
                break

            # 按 wall clock 时间索引轨迹点，避免 sleep 不准确导致漂移
            idx = min(int(elapsed / period), n_pts - 1)
            positions = trajectory[idx, :].tolist()
            if self.enable_limit_check:
                positions, _ = self._clamp_hand_position(joint_names, positions)
            self._publish_hand_cmd(publisher, joint_names, positions)

            # 频率控制：等到下一个时间片
            next_t = (idx + 1) * period
            sleep_t = next_t - (time.time() - start_time)
            if sleep_t > 0:
                time.sleep(sleep_t)

        self.get_logger().info("Hand trajectory execution completed")
    def _hand_state_callback(self, side, msg: JointState):
        """手部关节状态回调，缓存最新位置。"""
        joint_names = V4_HAND_JOINT_MAP[side]
        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        positions = np.zeros(len(joint_names), dtype=float)

        for i, name in enumerate(joint_names):
            if name in name_to_idx:
                positions[i] = msg.position[name_to_idx[name]]

        with self._hand_state_lock:
            self._hand_states[side] = positions
        self._hand_state_received[side].set()
    def open_hand(self, side, duration_sec=1.0, wait=True):
        return self.move_hand(side, V4_HAND_OPEN_POSE, duration_sec=duration_sec, wait=wait)

    def close_hand(self, side, duration_sec=1.0, wait=True):
        return self.move_hand(side, V4_HAND_CLOSE_POSE, duration_sec=duration_sec, wait=wait)
    def hand_periodic_motion(
        self,
        amplitude=V4_HAND_TEST_AMPLITUDE,
        period_sec=V4_HAND_TEST_PERIOD,
        cycles=V4_HAND_TEST_DEFAULT_CYCLES,
        phase_diff=V4_HAND_TEST_PHASE_DIFF,
        left_hand=True,
        right_hand=True,
        publish_hz=V4_HAND_TEST_HZ,
        return_to_zero=True,
    ):
        """V4 手部周期 sin 运动测试。

        position[i] = sin(2π * t / period + i * phase_diff) * amplitude
        7 个手指关节按 phase_diff 依次错相，产生"波浪"张合效果。

        走独立通路：JointCommand + /mc/{left,right}_hand/command，不干扰身体控制。
        """
        if not (left_hand or right_hand):
            self.get_logger().error("Must enable at least one of left/right hand")
            return False

        total_duration = period_sec * cycles
        if return_to_zero:
            total_duration += period_sec / 4

        self.get_logger().info(
            f"V4 hand periodic motion: amp={amplitude:.3f} rad, period={period_sec:.3f}s, "
            f"cycles={cycles}, phase_diff={phase_diff:.3f}, total={total_duration:.2f}s"
        )

        omega = 2 * np.pi / period_sec
        period_time = 1.0 / publish_hz
        start_time = time.time()
        hand_limit_warned = False

        try:
            for k in range(int(total_duration * publish_hz)):
                t = time.time() - start_time
                if t >= total_duration:
                    break

                full_cycles_duration = period_sec * cycles
                if return_to_zero and t > full_cycles_duration:
                    ramp_t = t - full_cycles_duration
                    ramp_factor = max(0.0, 1.0 - ramp_t / (period_sec / 4))
                else:
                    ramp_factor = 1.0

                base_phase = omega * t
                positions = [
                    amplitude * ramp_factor * np.sin(base_phase + i * phase_diff)
                    for i in range(len(V4_HAND_LEFT_JOINTS))
                ]

                if self.enable_limit_check:
                    positions, violations = self._clamp_hand_position(
                        V4_HAND_LEFT_JOINTS, positions
                    )
                    if violations and not hand_limit_warned:
                        for name, val, lo, hi in violations:
                            self.get_logger().warning(
                                f"CLAMPED hand {name}: {val:.4f} -> [{lo}, {hi}]"
                            )
                        hand_limit_warned = True

                if left_hand:
                    self._publish_hand_cmd(self.left_hand_pub, V4_HAND_LEFT_JOINTS, positions)
                if right_hand:
                    self._publish_hand_cmd(self.right_hand_pub, V4_HAND_RIGHT_JOINTS, positions)

                elapsed = time.time() - start_time
                next_t = (k + 1) * period_time
                sleep_t = next_t - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            self.get_logger().warning("Hand motion interrupted, sending zero command")
            self._send_hand_zero(left_hand, right_hand)
            return False

        return True
    def _publish_hand_cmd(self, publisher, joint_names, positions):
        """构造并发布手部 JointCommand（手部走独立通路，与身体不同）"""
        cmd = JointCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ""
        cmd.names = list(joint_names)
        cmd.position = [float(p) for p in positions]
        # 注意：JointCommand 使用并行数组 + mode[]，mode=5 是手部控制器自定义值
        # 不是 JointCommand.POSITION_MODE=1，也不是 JointCmd.MODE_POSITION=2
        cmd.mode = [5] * len(joint_names)  # mode=5: 手部控制器自定义模式
        # 其他字段（velocity/torque/acceleration/kp/kd）使用空数组即可（demo 也未设置）
        publisher.publish(cmd)
    def _send_hand_zero(self, left_hand, right_hand):
        """向双手发送零位指令（确保手指完全放松）。"""
        zeros = [0.0] * len(V4_HAND_LEFT_JOINTS)
        if left_hand:
            self._publish_hand_cmd(self.left_hand_pub, V4_HAND_LEFT_JOINTS, zeros)
        if right_hand:
            self._publish_hand_cmd(self.right_hand_pub, V4_HAND_RIGHT_JOINTS, zeros)

    # ========================================================================
    # [2.6] 二指夹爪控制（ECAT GripCmd）
    # ========================================================================
    def wait_for_grip_state(self, side=None, timeout=5.0):
        """阻塞等待二指夹爪状态消息。"""
        sides = ["left", "right"] if side is None else [side]
        deadline = time.time() + timeout
        for s in sides:
            if s not in self._grip_state_received:
                raise ValueError(f"Invalid side '{s}', expected 'left' or 'right'")
            remaining = max(0.0, deadline - time.time())
            if not self._grip_state_received[s].wait(timeout=remaining):
                self.get_logger().warning(f"Timeout waiting for {s} grip state ({remaining:.1f}s)")
                return False
        return True

    def get_grip_state(self, side):
        """获取二指夹爪状态字典，None 表示无数据。"""
        with self._grip_state_lock:
            if side in self._grip_states:
                return dict(self._grip_states[side])
        return None

    def get_grip_position(self, side):
        """获取二指夹爪开口（m），None 表示无数据。"""
        state = self.get_grip_state(side)
        if state is None:
            return None
        return float(state["pos"])

    def send_grip_command(
        self,
        side,
        pos,
        vel=GRIP_DEFAULT_VEL,
        force=GRIP_DEFAULT_FORCE,
        cur=0.0,
        mode=0,
        stop=0,
        reset=0,
        homing=0,
    ):
        """单次发送二指夹爪 ECAT GripCmd。

        Args:
            side: "left" 或 "right"
            pos: 目标开口，单位 m；仿真夹爪范围 [0.0, 0.05]
            vel: 目标速度，单位 m/s
            force: 目标夹持力，单位 N
            cur: 目标电流/占位字段
            mode/stop/reset/homing: ECAT GripCmd 控制字段
        """
        if side not in self._grip_pubs:
            self.get_logger().error(f"Invalid grip side '{side}'")
            return False

        target_pos = float(pos)
        if self.enable_limit_check:
            clamped = max(GRIP_OPENING_MIN_M, min(GRIP_OPENING_MAX_M, target_pos))
            if clamped != target_pos:
                self.get_logger().warning(
                    f"CLAMPED {side} grip pos: {target_pos:.4f} → {clamped:.4f} m"
                )
            target_pos = clamped

        msg = GripCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.init = 1
        msg.mode = int(mode)
        msg.stop = int(stop)
        msg.reset = int(reset)
        msg.homing = int(homing)
        msg.pos = target_pos
        msg.vel = float(vel)
        msg.force = float(force)
        msg.cur = float(cur)
        self._grip_pubs[side].publish(msg)
        return True

    def open_grip(self, side, wait=False, timeout=2.0):
        """张开二指夹爪。"""
        ok = self.send_grip_command(side, GRIP_OPENING_MAX_M)
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def close_grip(self, side, wait=False, timeout=2.0):
        """闭合二指夹爪。"""
        ok = self.send_grip_command(side, GRIP_OPENING_MIN_M)
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def move_grip(self, side, pos, wait=False, timeout=2.0):
        """移动二指夹爪到指定开口（m）。"""
        ok = self.send_grip_command(side, pos)
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def _grip_state_callback(self, side, msg: GripStatus):
        """二指夹爪状态回调，缓存最新 GripStatus。"""
        state = {
            "init_state": int(msg.init_state),
            "grip_state": int(msg.grip_state),
            "error_code": int(msg.error_code),
            "homed": int(msg.homed),
            "pos": float(msg.pos),
            "vel": float(msg.vel),
            "cur": float(msg.cur),
        }
        with self._grip_state_lock:
            self._grip_states[side] = state
        self._grip_state_received[side].set()


    # ========================================================================
    # [2.7] IK 子系统（Pinocchio 双求解，懒加载）
    # ========================================================================
    @staticmethod
    def _default_ik_urdf_path():
        return os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                os.pardir, os.pardir, os.pardir, os.pardir,
                "assets", "robots", "walker_s2", "s2.urdf",
            )
        )
    @staticmethod
    def _xyzrpy_dict(xyzrpy):
        values = [float(v) for v in xyzrpy]
        return dict(zip(["x", "y", "z", "roll", "pitch", "yaw"], values))
    def _current_arm_values(self, joint_names):
        current = self.get_current_position()
        if current is not None:
            pos_map = dict(zip(self.all_joints, current))
            return [float(pos_map.get(name, READY_POSE.get(name, 0.0))) for name in joint_names]
        return [float(READY_POSE.get(name, 0.0)) for name in joint_names]
    def initialize_ik(self, urdf_path=None, left_neutral=None, right_neutral=None, save_current_as_initial=True):
        """初始化 Walker S2 双臂 IK。

        IK 目标格式为 [x, y, z, roll, pitch, yaw]，单位 m/rad，坐标系为
        Walker S2 URDF 机器人基座坐标系，不是 /sim/finger_link_states 的 world 坐标系。
        """
        resolved_urdf = (
            urdf_path
            or self._ik_urdf_path
            or os.environ.get("WALKER_S2_IK_URDF")
            or self._default_ik_urdf_path()
        )
        resolved_urdf = os.path.abspath(resolved_urdf)
        if not os.path.exists(resolved_urdf):
            self.get_logger().error(f"Walker S2 IK URDF not found: {resolved_urdf}")
            return False

        try:
            from .ik import WalkerS2IK
            solver = WalkerS2IK(
                resolved_urdf,
                joint_limits=BODY_JOINT_LIMITS,
                joint_limit_margin=0.0,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize Walker S2 IK: {exc}")
            return False

        with self._ik_lock:
            self.ik_solver = solver
            self._ik_urdf_path = resolved_urdf
            self._ik_initialized = True
            self._sync_ik_from_current_state_locked()
            left = left_neutral if left_neutral is not None else self._current_arm_values(LEFT_ARM_JOINTS)
            right = right_neutral if right_neutral is not None else self._current_arm_values(RIGHT_ARM_JOINTS)
            self.ik_solver.set_neutral_config(left, right)
            if save_current_as_initial:
                self.ik_solver.save_initial_q()
        self.get_logger().info(
            f"Walker S2 IK initialized with URDF: {resolved_urdf}; "
            f"joint_limits_constrained={solver._joint_limit_override_count}"
        )
        return True
    def reset_ik(self, save_current_as_initial=True):
        if self.ik_solver is None:
            return self.initialize_ik(save_current_as_initial=save_current_as_initial)
        with self._ik_lock:
            self.ik_solver.reset_runtime_state()
            self._sync_ik_from_current_state_locked()
            if save_current_as_initial:
                self.ik_solver.save_initial_q()
        return True
    def _sync_ik_from_current_state_locked(self):
        if self.ik_solver is None:
            return False
        current = self.get_current_position()
        if current is None:
            return False
        self.ik_solver.sync_joint_positions(self.all_joints, current.tolist())
        return True

    def _sync_ik_from_current_state(self):
        with self._ik_lock:
            return self._sync_ik_from_current_state_locked()

    def _ensure_ik_initialized(self):
        if self.ik_solver is not None and self._ik_initialized:
            return True
        return self.initialize_ik()

    def get_ee_pose(self, side, as_dict=False):
        """获取当前单臂末端 [x,y,z,roll,pitch,yaw]（URDF base frame）。"""
        if side not in ("left", "right"):
            raise ValueError(f"Invalid arm side: {side}")
        if not self._ensure_ik_initialized():
            return None
        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            pose = self.ik_solver.get_ee_pose(side)
        return self._xyzrpy_dict(pose) if as_dict else pose

    def get_ee_poses(self, as_dict=False):
        """获取左右臂末端 [x,y,z,roll,pitch,yaw]（URDF base frame）。"""
        if not self._ensure_ik_initialized():
            return None
        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            poses = self.ik_solver.get_both_ee_poses()
        if as_dict:
            return {side: self._xyzrpy_dict(pose) for side, pose in poses.items()}
        return poses
    def solve_arm_ik(self, side, target_xyzrpy, sync_state=True, task_type="default", use_hierarchical=False, **ik_kwargs):
        """只求解单臂 IK，不下发控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，返回的关节目标会包含 waist_yaw_joint。
        传 task_type 指定语义种子（如 "pick_table"），失败时回退。
        传 use_hierarchical=True 启用层级 IK（torso→shoulder→full arm）。
        """
        if side not in ("left", "right"):
            raise ValueError(f"Invalid arm side: {side}")
        if not self._ensure_ik_initialized():
            return None, False, {"error": "ik_not_initialized"}
        with self._ik_lock:
            if sync_state:
                self._sync_ik_from_current_state_locked()
            if side == "left":
                result = self.ik_solver.solve_dual_arm(
                    left_target_xyzrpy=target_xyzrpy,
                    task_type=task_type, use_hierarchical=use_hierarchical, **ik_kwargs,
                )
                joints = result.get("left_joint_positions")
                ok = bool(result.get("left_success", False))
                names = result.get("left_joint_names", LEFT_ARM_JOINTS)
            else:
                result = self.ik_solver.solve_dual_arm(
                    right_target_xyzrpy=target_xyzrpy,
                    task_type=task_type, use_hierarchical=use_hierarchical, **ik_kwargs,
                )
                joints = result.get("right_joint_positions")
                ok = bool(result.get("right_success", False))
                names = result.get("right_joint_names", RIGHT_ARM_JOINTS)
        diagnostics = result.get("diagnostics", {})
        return dict(zip(names, [float(v) for v in joints])) if joints is not None else None, ok, diagnostics

    def solve_dual_arm_ik(self, left_target_xyzrpy=None, right_target_xyzrpy=None, sync_state=True, task_type="default", use_hierarchical=False, **ik_kwargs):
        """只求解双臂 IK，不下发控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，仅支持单臂目标，返回的关节列表会包含 waist_yaw_joint。
        """
        if left_target_xyzrpy is None and right_target_xyzrpy is None:
            return {}
        if not self._ensure_ik_initialized():
            return {"error": "ik_not_initialized"}
        with self._ik_lock:
            if sync_state:
                self._sync_ik_from_current_state_locked()
            result = self.ik_solver.solve_dual_arm(
                left_target_xyzrpy=left_target_xyzrpy,
                right_target_xyzrpy=right_target_xyzrpy,
                task_type=task_type, use_hierarchical=use_hierarchical,
                **ik_kwargs,
            )
        return result

    def move_arm_ik(self, side, target_xyzrpy, duration_sec=1.5, wait=True, require_success=True, task_type="default", use_hierarchical=False, **ik_kwargs):
        """单臂 Cartesian IK 控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，会在下发 waist_yaw_joint 目标时临时解锁腰部。
        """
        joint_targets, ok, diagnostics = self.solve_arm_ik(
            side, target_xyzrpy, task_type=task_type, use_hierarchical=use_hierarchical, **ik_kwargs,
        )
        if joint_targets is None:
            return False
        if require_success and not ok:
            self.get_logger().warning(f"{side} arm IK did not converge: {diagnostics}")
            return False
        return self.move_to_pose(joint_targets, duration_sec=duration_sec, wait=wait, unlock_required_joints=True)

    def move_arm_ee_delta(
        self,
        side,
        delta_xyz=(0.0, 0.0, 0.0),
        delta_rpy=(0.0, 0.0, 0.0),
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """控制单侧末端相对当前位置位移。

        delta_xyz / delta_rpy 均定义在 Walker S2 URDF base frame 下，单位 m/rad。
        例如 delta_xyz=(0, 0, 0.02) 表示末端在 base frame 的 z 方向移动 2cm。
        """
        current = self.get_ee_pose(side)
        if current is None:
            return False
        delta = np.concatenate([
            np.asarray(delta_xyz, dtype=float),
            np.asarray(delta_rpy, dtype=float),
        ])
        if delta.shape != (6,):
            self.get_logger().error(f"EE delta must have 3 xyz + 3 rpy values, got shape {delta.shape}")
            return False
        target = np.asarray(current, dtype=float) + delta
        self.get_logger().info(
            f"Move {side} EE delta xyz={delta[:3].tolist()} rpy={delta[3:].tolist()} "
            f"target={target.tolist()}"
        )
        return self.move_arm_ik(
            side,
            target,
            duration_sec=duration_sec,
            wait=wait,
            require_success=require_success,
            task_type=task_type,
            use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    def move_left_ee_delta(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta("left", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_right_ee_delta(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta("right", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_arm_ee_delta_local(
        self,
        side,
        delta_xyz=(0.0, 0.0, 0.0),
        delta_rpy=(0.0, 0.0, 0.0),
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """控制单侧末端相对当前末端局部坐标系位移。

        delta_xyz / delta_rpy 定义在末端局部坐标系下（x=夹爪下方, y=夹爪左方, z=夹爪前方），单位 m/rad。
        内部自动将局部 delta 转换到 base frame 后调用 move_arm_ik。

        例如 delta_xyz=(0, 0, 0.05) 表示沿夹爪前进方向移动 5cm，
        无论当前末端朝向如何。
        """
        current = self.get_ee_pose(side)
        if current is None:
            return False
        _, target_xyzrpy = _convert_ee_delta_local_to_base(current, delta_xyz, delta_rpy)
        self.get_logger().info(
            f"Move {side} EE local delta xyz={list(delta_xyz)} rpy={list(delta_rpy)} "
            f"→ base target={[round(float(v), 4) for v in target_xyzrpy]}"
        )
        return self.move_arm_ik(
            side, target_xyzrpy,
            duration_sec=duration_sec, wait=wait, require_success=require_success,
            task_type=task_type, use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    def move_left_ee_delta_local(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta_local("left", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_right_ee_delta_local(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta_local("right", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_arm_ik_world(self, side, target_world_xyzrpy, duration_sec=1.5, wait=True, require_success=True, task_type="default", use_hierarchical=False, **ik_kwargs):
        """单臂 Cartesian IK 控制，目标为 world frame 的 [x,y,z,r,p,y]。

        内部自动将 world frame 目标转换到 base frame 后调用 move_arm_ik。
        """
        target_base = _world_xyzrpy_to_base(target_world_xyzrpy)
        self.get_logger().info(
            f"Move {side} EE to world {[round(float(v), 4) for v in target_world_xyzrpy]} "
            f"→ base {[round(float(v), 4) for v in target_base]}"
        )
        return self.move_arm_ik(
            side, target_base,
            duration_sec=duration_sec, wait=wait, require_success=require_success,
            task_type=task_type, use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    def move_arm_ee_delta_world(
        self,
        side,
        delta_xyz=(0.0, 0.0, 0.0),
        delta_rpy=(0.0, 0.0, 0.0),
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """控制单侧末端在 world frame 下相对位移。

        delta_xyz / delta_rpy 定义在 world frame 下，单位 m/rad。
        内部通过 _world_xyzrpy_to_base 转换到 base frame 后调用 move_arm_ik。
        """
        current = self.get_ee_pose(side)
        if current is None:
            return False
        # 当前 EE 在 world frame 的位姿：base_xyzrpy → world_xyzrpy
        R_base_to_world = _quat_wxyz_to_matrix(ROBOT_WORLD_ROT_WXYZ)
        R_base = _rpy_to_rotation_matrix(
            float(current[3]), float(current[4]), float(current[5]))
        R_world = np.array(R_base_to_world) @ R_base
        world_roll, world_pitch, world_yaw = _rotation_matrix_to_rpy(R_world)
        # 位置：world = R_base_to_world @ base_xyz + robot_world_pos
        R_b2w_arr = np.array(R_base_to_world)
        world_xyz = (R_b2w_arr @ np.array([float(current[0]), float(current[1]), float(current[2])])
                     + np.array(ROBOT_WORLD_POS)).tolist()
        world_target = np.array([
            world_xyz[0] + float(delta_xyz[0]),
            world_xyz[1] + float(delta_xyz[1]),
            world_xyz[2] + float(delta_xyz[2]),
            world_roll + float(delta_rpy[0]),
            world_pitch + float(delta_rpy[1]),
            world_yaw + float(delta_rpy[2]),
        ])
        target_base = _world_xyzrpy_to_base(world_target)
        self.get_logger().info(
            f"Move {side} EE world delta xyz={list(delta_xyz)} rpy={list(delta_rpy)} "
            f"→ base target={[round(float(v), 4) for v in target_base]}"
        )
        return self.move_arm_ik(
            side, target_base,
            duration_sec=duration_sec, wait=wait, require_success=require_success,
            task_type=task_type, use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    def move_left_ee_delta_world(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta_world("left", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_right_ee_delta_world(self, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, 0.0, 0.0), **kwargs):
        return self.move_arm_ee_delta_world("right", delta_xyz=delta_xyz, delta_rpy=delta_rpy, **kwargs)

    def move_dual_arm_ik(
        self,
        left_target_xyzrpy=None,
        right_target_xyzrpy=None,
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """双臂 Cartesian IK 控制。目标为 URDF base frame 的 [x,y,z,r,p,y]。

        传 unlock_waist=True 时，仅支持单臂目标。
        """
        result = self.solve_dual_arm_ik(
            left_target_xyzrpy, right_target_xyzrpy,
            task_type=task_type, use_hierarchical=use_hierarchical, **ik_kwargs,
        )
        if not result or "error" in result:
            return False

        pose_dict = {}
        waist_target = None
        if left_target_xyzrpy is not None:
            ok = bool(result.get("left_success", False))
            if require_success and not ok:
                self.get_logger().warning(f"left arm IK did not converge: {result.get('diagnostics', {})}")
                return False
            left_targets = dict(zip(result["left_joint_names"], [float(v) for v in result["left_joint_positions"]]))
            waist_target = left_targets.get("waist_yaw_joint")
            pose_dict.update(left_targets)
        if right_target_xyzrpy is not None:
            ok = bool(result.get("right_success", False))
            if require_success and not ok:
                self.get_logger().warning(f"right arm IK did not converge: {result.get('diagnostics', {})}")
                return False
            right_targets = dict(zip(result["right_joint_names"], [float(v) for v in result["right_joint_positions"]]))
            right_waist_target = right_targets.get("waist_yaw_joint")
            if waist_target is not None and right_waist_target is not None and abs(waist_target - right_waist_target) > 1e-6:
                self.get_logger().error("Conflicting waist_yaw_joint targets from left/right IK results")
                return False
            pose_dict.update(right_targets)

        return self.move_to_pose(pose_dict, duration_sec=duration_sec, wait=wait, unlock_required_joints=True)

    def move_dual_ee_delta(
        self,
        left_delta_xyz=None,
        right_delta_xyz=None,
        left_delta_rpy=None,
        right_delta_rpy=None,
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """控制双臂末端相对当前位置位移。

        delta 均定义在 Walker S2 URDF base frame 下，单位 m/rad。传 None 表示该侧不动。
        """
        poses = self.get_ee_poses()
        if poses is None:
            return False

        left_target = None
        right_target = None
        if left_delta_xyz is not None or left_delta_rpy is not None:
            left_target = np.asarray(poses["left"], dtype=float).copy()
            left_target += np.concatenate([
                np.asarray(left_delta_xyz if left_delta_xyz is not None else (0.0, 0.0, 0.0), dtype=float),
                np.asarray(left_delta_rpy if left_delta_rpy is not None else (0.0, 0.0, 0.0), dtype=float),
            ])
        if right_delta_xyz is not None or right_delta_rpy is not None:
            right_target = np.asarray(poses["right"], dtype=float).copy()
            right_target += np.concatenate([
                np.asarray(right_delta_xyz if right_delta_xyz is not None else (0.0, 0.0, 0.0), dtype=float),
                np.asarray(right_delta_rpy if right_delta_rpy is not None else (0.0, 0.0, 0.0), dtype=float),
            ])
        if left_target is None and right_target is None:
            self.get_logger().warning("move_dual_ee_delta called with no left/right delta")
            return False

        return self.move_dual_arm_ik(
            left_target_xyzrpy=left_target,
            right_target_xyzrpy=right_target,
            duration_sec=duration_sec,
            wait=wait,
            require_success=require_success,
            task_type=task_type,
            use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    def move_dual_ee_delta_local(
        self,
        left_delta_xyz=None,
        right_delta_xyz=None,
        left_delta_rpy=None,
        right_delta_rpy=None,
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """控制双臂末端相对各自局部坐标系位移。

        delta 均定义在末端局部坐标系下（x=夹爪下方, y=夹爪左方, z=夹爪前方），单位 m/rad。
        传 None 表示该侧不动。内部自动转换到 base frame 后调用 move_dual_arm_ik。
        """
        poses = self.get_ee_poses()
        if poses is None:
            return False

        left_target = None
        right_target = None
        if left_delta_xyz is not None or left_delta_rpy is not None:
            lxyz = left_delta_xyz if left_delta_xyz is not None else (0.0, 0.0, 0.0)
            lrpy = left_delta_rpy if left_delta_rpy is not None else (0.0, 0.0, 0.0)
            _, left_target = _convert_ee_delta_local_to_base(poses["left"], lxyz, lrpy)
        if right_delta_xyz is not None or right_delta_rpy is not None:
            rxyz = right_delta_xyz if right_delta_xyz is not None else (0.0, 0.0, 0.0)
            rrpy = right_delta_rpy if right_delta_rpy is not None else (0.0, 0.0, 0.0)
            _, right_target = _convert_ee_delta_local_to_base(poses["right"], rxyz, rrpy)
        if left_target is None and right_target is None:
            self.get_logger().warning("move_dual_ee_delta_local called with no left/right delta")
            return False

        return self.move_dual_arm_ik(
            left_target_xyzrpy=left_target,
            right_target_xyzrpy=right_target,
            duration_sec=duration_sec,
            wait=wait,
            require_success=require_success,
            task_type=task_type,
            use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    def move_dual_arm_ik_world(
        self,
        left_target_world_xyzrpy=None,
        right_target_world_xyzrpy=None,
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """双臂 Cartesian IK 控制，目标为 world frame 的 [x,y,z,r,p,y]。

        内部自动将 world frame 目标转换到 base frame 后调用 move_dual_arm_ik。
        """
        left_base = None
        right_base = None
        if left_target_world_xyzrpy is not None:
            left_base = _world_xyzrpy_to_base(left_target_world_xyzrpy)
        if right_target_world_xyzrpy is not None:
            right_base = _world_xyzrpy_to_base(right_target_world_xyzrpy)
        return self.move_dual_arm_ik(
            left_target_xyzrpy=left_base,
            right_target_xyzrpy=right_base,
            duration_sec=duration_sec,
            wait=wait,
            require_success=require_success,
            task_type=task_type,
            use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    def move_dual_ee_delta_world(
        self,
        left_delta_xyz=None,
        right_delta_xyz=None,
        left_delta_rpy=None,
        right_delta_rpy=None,
        duration_sec=1.5,
        wait=True,
        require_success=True,
        task_type="default",
        use_hierarchical=False,
        **ik_kwargs,
    ):
        """控制双臂末端在 world frame 下相对位移。

        delta 均定义在 world frame 下，单位 m/rad。传 None 表示该侧不动。
        内部自动转换到 base frame 后调用 move_dual_arm_ik。
        """
        poses = self.get_ee_poses()
        if poses is None:
            return False

        R_base_to_world = _quat_wxyz_to_matrix(ROBOT_WORLD_ROT_WXYZ)
        R_b2w_arr = np.array(R_base_to_world)

        def _current_ee_world(pose):
            x, y, z, roll, pitch, yaw = [float(v) for v in pose]
            R_base = _rpy_to_rotation_matrix(roll, pitch, yaw)
            R_world = R_b2w_arr @ R_base
            wr, wp, wy = _rotation_matrix_to_rpy(R_world)
            # 位置：world = R_base_to_world @ base_xyz + robot_world_pos
            wxyz = R_b2w_arr @ np.array([x, y, z]) + np.array(ROBOT_WORLD_POS)
            return np.array([wxyz[0], wxyz[1], wxyz[2], wr, wp, wy])

        left_target = None
        right_target = None
        if left_delta_xyz is not None or left_delta_rpy is not None:
            lxyz = left_delta_xyz if left_delta_xyz is not None else (0.0, 0.0, 0.0)
            lrpy = left_delta_rpy if left_delta_rpy is not None else (0.0, 0.0, 0.0)
            left_world = _current_ee_world(poses["left"])
            left_world_target = left_world + np.concatenate([np.asarray(lxyz), np.asarray(lrpy)])
            left_target = _world_xyzrpy_to_base(left_world_target)
        if right_delta_xyz is not None or right_delta_rpy is not None:
            rxyz = right_delta_xyz if right_delta_xyz is not None else (0.0, 0.0, 0.0)
            rrpy = right_delta_rpy if right_delta_rpy is not None else (0.0, 0.0, 0.0)
            right_world = _current_ee_world(poses["right"])
            right_world_target = right_world + np.concatenate([np.asarray(rxyz), np.asarray(rrpy)])
            right_target = _world_xyzrpy_to_base(right_world_target)
        if left_target is None and right_target is None:
            self.get_logger().warning("move_dual_ee_delta_world called with no left/right delta")
            return False

        return self.move_dual_arm_ik(
            left_target_xyzrpy=left_target,
            right_target_xyzrpy=right_target,
            duration_sec=duration_sec,
            wait=wait,
            require_success=require_success,
            task_type=task_type,
            use_hierarchical=use_hierarchical,
            **ik_kwargs,
        )

    control_dual_arm_ik = move_dual_arm_ik

    # ========================================================================
    # [2.8] 仿真交互
    # ========================================================================
    def reset_sim(self):
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        self.get_logger().info("Sent simulation reset command")
    def _finger_link_state_callback(self, msg: String):
        """缓存仿真发布的 finger_link world pose JSON。"""
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid finger_link_states JSON: {exc}")
            return
        with self._finger_link_states_lock:
            self._finger_link_states = state
        self._finger_link_states_received.set()

    def wait_for_finger_link_states(self, timeout=5.0):
        """等待 /sim/finger_link_states；该状态为 world frame，仅用于验证/debug。"""
        ok = self._finger_link_states_received.wait(timeout=timeout)
        if not ok:
            self.get_logger().warning(f"Timeout waiting for finger_link_states ({timeout:.1f}s)")
        return ok

    def get_finger_link_states(self):
        """获取最新 finger_link world pose JSON，None 表示无数据。"""
        with self._finger_link_states_lock:
            if self._finger_link_states is not None:
                return json.loads(json.dumps(self._finger_link_states))
        return None

    def get_finger_link_pose(self, link_name: str):
        """获取指定 finger link 的 world pose 字典，None 表示无数据或 link 不存在。"""
        states = self.get_finger_link_states()
        if not states:
            return None
        return (states.get("links") or {}).get(link_name)

    # ========================================================================
    # [2.9] 调试 / 诊断
    # ========================================================================
    def run_reset(self):
        """安全复位：回 home 并张开双手。"""
        if not self.wait_for_state(timeout=5.0):
            self.get_logger().warning("No Walker S2 state received; only sending sim reset command")
            self.reset_sim()
            return False
        self.home(duration_sec=2.0, wait=True)
        self.open_hand("left", duration_sec=1.0, wait=True)
        self.open_hand("right", duration_sec=1.0, wait=True)
        return True
    def print_endpoint_poses(self, side="right", timeout=5.0):
        """打印当前 IK TCP 与仿真夹爪 link 位姿。

        依赖 wait_for_finger_link_states（仿真 /sim/finger_link_states 话题）。
        """
        if side not in SIXFORCE_LINKS:
            raise ValueError(f"Invalid side '{side}', expected left or right")

        ok = self.wait_for_state(timeout=timeout)
        if not ok:
            return False
        self.wait_for_grip_state(side, timeout=2.0)
        self.wait_for_finger_link_states(timeout=timeout)

        sixforce_link = SIXFORCE_LINKS[side]
        ee_pose_base = self.get_ee_pose(side)
        print(f"\n=== IK/FK TCP pose (URDF base frame) ===")
        print(f"{sixforce_link} / TCP: xyzrpy={_fmt(ee_pose_base)}")

        states = self.get_finger_link_states() or {}
        links = states.get("links") or {}
        print("\n=== Sim link poses (world frame) ===")
        print(_link_pose_text(sixforce_link, links.get(sixforce_link)))

        prefix = FINGER_PREFIX[side]
        finger_items = sorted(
            (name, pose)
            for name, pose in links.items()
            if name.startswith(prefix) and "link" in name.lower()
        )
        if not finger_items:
            print(f"No {side} finger link poses found in /sim/finger_link_states")
        else:
            print(f"\n--- {side} finger / gripper links ---")
            for name, pose in finger_items:
                print(_link_pose_text(name, pose))
            gripper_center = _mean_xyz([pose for _, pose in finger_items])
            print(f"\nObserved {side} gripper center: pos={_fmt(gripper_center)} (mean of finger link positions)")

        grip_state = self.get_grip_state(side)
        if grip_state is not None:
            print("\n=== ECAT two-finger grip state ===")
            print(grip_state)
        return True

    def move_ee_to_world_pos(self, side="right", world_pos=None, duration_sec=2.0):
        """通过 IK 将末端移动到 world frame 目标位置（保持当前姿态）。

        支持左右臂，内部通过 _world_xyz_to_base 转换位置。
        """
        if side not in ("left", "right"):
            raise ValueError(f"move_ee_to_world_pos: invalid side '{side}', expected 'left' or 'right'")
        world_pos = DEFAULT_TARGET_WORLD_POS if world_pos is None else world_pos
        current = self.get_ee_pose(side)
        if current is None:
            self.get_logger().error("No current EE pose available")
            return False
        target_base_xyz = _world_xyz_to_base(world_pos)
        target_xyzrpy = [float(v) for v in target_base_xyz] + [float(v) for v in current[3:]]
        self.get_logger().info(
            f"Move IK TCP ({SIXFORCE_LINKS[side]}) to world pos={_fmt(world_pos)} "
            f"=> base target xyzrpy={_fmt(target_xyzrpy)}"
        )
        return self.move_arm_ik(
            side, target_xyzrpy, duration_sec=duration_sec, wait=True,
            require_success=True, **IK_KWARGS,
        )
    def monitor_joints(self, joint_names, hz=10, duration_sec=None):
        """持续监控指定身体关节的位置变化。

        Args:
            joint_names: 要监控的关节名列表
            hz: 刷新频率（Hz）
            duration_sec: 监控时长（秒），None 表示持续到 Ctrl+C
        """
        interval = 1.0 / hz
        start_time = time.time()

        header = f"{'time':>6s}"
        for name in joint_names:
            short = name.replace("_joint", "").replace("_", " ")
            header += f"  {short:>12s}"
        print(header)
        print("-" * len(header))

        try:
            while True:
                if duration_sec is not None:
                    if (time.time() - start_time) >= duration_sec:
                        break

                pos = self.get_current_position()
                if pos is not None:
                    elapsed = time.time() - start_time
                    line = f"{elapsed:6.1f}"
                    for name in joint_names:
                        try:
                            idx = self.joint_index(name)
                            line += f"  {pos[idx]:>+12.4f}"
                        except ValueError:
                            line += f"  {'N/A':>12s}"
                    print(line, flush=True)
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        print("\nMonitored joints ended")

    def monitor_hand_joints(self, joint_names, hz=10, duration_sec=None):
        """持续监控手指关节的位置变化。

        Args:
            joint_names: 手指关节全名列表（如 ["left_thumb_swing", "left_index_mcp"]）
            hz: 刷新频率（Hz）
            duration_sec: 监控时长（秒），None 表示持续到 Ctrl+C
        """
        # 按手别分组
        by_side = {}
        for name in joint_names:
            if name.startswith("left_"):
                by_side.setdefault("left", []).append(name)
            elif name.startswith("right_"):
                by_side.setdefault("right", []).append(name)
            else:
                print(f"WARNING cannot infer hand side: {name}, skipped")

        interval = 1.0 / hz
        start_time = time.time()

        header = f"{'time':>6s}"
        for name in joint_names:
            short = name.removeprefix("left_").removeprefix("right_")
            header += f"  {short:>12s}"
        print(header)
        print("-" * len(header))

        try:
            while True:
                if duration_sec is not None:
                    if (time.time() - start_time) >= duration_sec:
                        break

                elapsed = time.time() - start_time
                line = f"{elapsed:6.1f}"

                for name in joint_names:
                    side = "left" if name.startswith("left_") else "right" if name.startswith("right_") else None
                    if side is None:
                        line += f"  {'N/A':>12s}"
                        continue
                    pos = self.get_hand_position(side)
                    if pos is not None:
                        try:
                            idx = V4_HAND_JOINT_MAP[side].index(name)
                            line += f"  {pos[idx]:>+12.4f}"
                        except ValueError:
                            line += f"  {'N/A':>12s}"
                    else:
                        line += f"  {'N/A':>12s}"

                print(line, flush=True)
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        print("\nMonitored hand joints ended")

# ============================================================================
# [2.10] 向后兼容别名
# ============================================================================
RobotController = WalkerS2Controller

open_two_finger_grip = WalkerS2Controller.open_grip
close_two_finger_grip = WalkerS2Controller.close_grip
move_two_finger_grip = WalkerS2Controller.move_grip


# ============================================================================
# [Block 3] 命令行入口
# ============================================================================
def _print_joint_states(controller, joint_names=None):
    """打印指定身体关节的当前状态（CLI helper）。"""
    names = joint_names or controller.all_joints
    pos = controller.get_current_position()
    if pos is None:
        print("No current position available")
        return

    print(f"\n{'Joint':<32s} {'Pos(rad)':>10s} {'Pos(deg)':>9s} {'Limit':>18s} {'Status':>8s}")
    print("-" * 82)
    for name in names:
        try:
            idx = controller.joint_index(name)
        except ValueError:
            print(f"  {name:<32s} UNKNOWN")
            continue
        val = pos[idx]
        locked = "LOCKED" if name in controller.lock_joints else ""
        if name in BODY_JOINT_LIMITS:
            lo, hi = BODY_JOINT_LIMITS[name]
            range_str = f"[{lo:.2f}, {hi:.2f}]"
            status = "OK" if lo <= val <= hi else ("BELOW" if val < lo else "ABOVE")
        else:
            range_str, status = "N/A", ""
        print(f"  {name:<32s} {val:>+10.4f} {np.degrees(val):>+9.2f} "
              f"{range_str:>18s} {status:>8s} {locked}")


def _print_hand_states(controller, sides=None):
    """打印手指关节当前状态（CLI helper）。"""
    sides = sides or ["left", "right"]
    print(f"\n{'Joint':<24s} {'Pos(rad)':>10s} {'Pos(deg)':>9s} {'Limit':>18s} {'Status':>8s}")
    print("-" * 75)
    for side in sides:
        joint_names = V4_HAND_JOINT_MAP[side]
        pos = controller.get_hand_position(side)
        for name in joint_names:
            short = name.removeprefix("left_").removeprefix("right_")
            if pos is not None:
                idx = joint_names.index(name)
                val = pos[idx]
            else:
                val = None
            if short in V4_HAND_JOINT_LIMITS:
                lo, hi = V4_HAND_JOINT_LIMITS[short]
                range_str = f"[{lo:.2f}, {hi:.2f}]"
                status = "NO DATA" if val is None else ("OK" if lo <= val <= hi else "LIMIT")
            else:
                range_str, status = "N/A", ""
            val_str = f"{val:>+10.4f}" if val is not None else f"{'N/A':>10s}"
            deg_str = f"{np.degrees(val):>+9.2f}" if val is not None else f"{'N/A':>9s}"
            print(f"  {name:<24s} {val_str} {deg_str} {range_str:>18s} {status:>8s}")

def cmd_print_state(controller):
    """打印当前关节状态"""
    pos = controller.get_current_position()
    if pos is None:
        print("No current position available")
        return
    print(f"\n当前关节位置 ({controller.n_joints} 维):")
    for i, name in enumerate(controller.all_joints):
        locked = " [LOCKED]" if name in controller.lock_joints else ""
        limit_flag = ""
        if name in BODY_JOINT_LIMITS:
            lo, hi = BODY_JOINT_LIMITS[name]
            if pos[i] < lo:
                limit_flag = f"  ⚠️ BELOW LIMIT ({lo:.4f})"
            elif pos[i] > hi:
                limit_flag = f"  ⚠️ EXCEEDS LIMIT ({hi:.4f})"
            else:
                limit_flag = f"  [{lo:.2f}, {hi:.2f}]"
        print(f"  [{i:2d}] {name:30s} = {pos[i]:+.4f} rad{locked}{limit_flag}")


def cmd_print_grip_state(controller, sides=None):
    """打印二指夹爪状态。"""
    sides = sides or ["left", "right"]
    controller.wait_for_grip_state(timeout=1.0)
    print("\n二指夹爪状态:")
    print(f"{'side':<8s} {'pos(m)':>8s} {'vel':>8s} {'cur':>8s} {'init':>6s} {'state':>6s} {'homed':>6s} {'err':>6s}")
    print("-" * 68)
    for side in sides:
        state = controller.get_grip_state(side)
        if state is None:
            print(f"{side:<8s} {'N/A':>8s}")
            continue
        print(
            f"{side:<8s} {state['pos']:>8.4f} {state['vel']:>8.4f} {state['cur']:>8.4f} "
            f"{state['init_state']:>6d} {state['grip_state']:>6d} "
            f"{state['homed']:>6d} {state['error_code']:>6d}"
        )


def cmd_print_ee(controller, sides=None):
    """打印末端笛卡尔位姿(URDF base frame,xyz + rpy)。

    依赖 IK solver;构造控制器时需传 enable_ik=True(或运行时 initialize_ik())。
    """
    sides = sides or ["left", "right"]
    print("\n末端笛卡尔位姿 (URDF base frame):")
    print(
        f"{'side':<6s} {'x(m)':>10s} {'y(m)':>10s} {'z(m)':>10s} "
        f"{'roll(rad)':>11s} {'pitch(rad)':>11s} {'yaw(rad)':>11s} "
        f"{'roll(°)':>9s} {'pitch(°)':>9s} {'yaw(°)':>9s}"
    )
    print("-" * 106)
    any_ok = False
    for side in sides:
        pose = controller.get_ee_pose(side, as_dict=True)
        if pose is None:
            print(f"{side:<6s}  N/A (IK 未初始化或未收到机器人状态)")
            continue
        any_ok = True
        print(
            f"{side:<6s} "
            f"{pose['x']:>+10.4f} {pose['y']:>+10.4f} {pose['z']:>+10.4f} "
            f"{pose['roll']:>+11.4f} {pose['pitch']:>+11.4f} {pose['yaw']:>+11.4f} "
            f"{np.degrees(pose['roll']):>+9.1f} {np.degrees(pose['pitch']):>+9.1f} {np.degrees(pose['yaw']):>+9.1f}"
        )
    if not any_ok:
        print(
            "✗ 末端位姿获取失败。请确认:1) 已收到机器人状态  "
            "2) URDF 存在(默认 ubt_sim/assets/robots/walker_s2/s2.urdf,或设 WALKER_S2_IK_URDF)"
        )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Walker S2 机器人直接控制脚本（SDK 控制器，模式B）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
注意事项：
  - 运行前必须先 switch_controller config_mc_walker_s2_v1_sps
  - 启动前用遥控器将机器人移到安全位置
  - 默认锁定 head_pitch/head_yaw/waist_yaw（不会发送这些关节的指令）
""",
    )
    parser.add_argument(
        "--no-lock", action="store_true",
        help="不锁定任何关节（默认锁定 head/waist）",
    )
    parser.add_argument(
        "--no-safety", action="store_true",
        help="禁用安全速度检查",
    )
    parser.add_argument(
        "--no-limits", action="store_true",
        help="禁用关节限位裁剪",
    )
    parser.add_argument(
        "--print-state", action="store_true",
        help="仅打印当前关节状态后退出",
    )
    parser.add_argument(
        "--print-ee", action="store_true",
        help="打印末端笛卡尔位姿(URDF base frame,xyz + rpy,需启用 IK)",
    )
    parser.add_argument(
        "--ee-side", choices=["left", "right", "both"], default="both",
        help="末端位姿打印侧,默认 both",
    )
    parser.add_argument(
        "--init", action="store_true",
        help="分段移动到预备姿态（先 shoulder pitch / elbow roll，再 elbow yaw，最后 READY_POSE）",
    )
    parser.add_argument(
        "--init-duration", type=float, default=15.0,
        help="预备姿态运动时长（秒），默认 15.0（4 段按 20%%/35%%/25%%/20%% 分配）",
    )
    parser.add_argument(
        "--init-settle-timeout", type=float, default=10.0,
        help="轨迹发布完后等待实际关节收敛的超时时间（秒），默认 10.0",
    )
    parser.add_argument(
        "--init-tolerance", type=float, default=0.08,
        help="判定预备姿态到位的最大关节误差（rad），默认 0.08",
    )
    parser.add_argument(
        "--move-joint", action="store_true",
        help="移动单个身体关节到指定角度，用于诊断关节控制",
    )
    parser.add_argument(
        "--joint", choices=BODY_JOINT_NAMES, default=None,
        help="--move-joint 使用的关节名",
    )
    parser.add_argument(
        "--pos", type=float, default=None,
        help="--move-joint 使用的目标角度（rad）",
    )
    parser.add_argument(
        "--duration", type=float, default=3.0,
        help="--move-joint 运动时长（秒），默认 3.0",
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.03,
        help="--move-joint 到位判定阈值（rad），默认 0.03",
    )
    parser.add_argument(
        "--settle-timeout", type=float, default=3.0,
        help="--move-joint 额外等待实际关节收敛的超时时间（秒），默认 3.0",
    )
    parser.add_argument(
        "--grip-side", choices=["left", "right", "both"], default="both",
        help="二指夹爪操作侧，默认 both",
    )
    parser.add_argument(
        "--grip-state", action="store_true",
        help="打印二指夹爪状态（/ecat/{side}_grip/state）",
    )
    parser.add_argument(
        "--grip-open", action="store_true",
        help=f"张开二指夹爪到 {GRIP_OPENING_MAX_M:.3f} m",
    )
    parser.add_argument(
        "--grip-close", action="store_true",
        help="闭合二指夹爪到 0.000 m",
    )
    parser.add_argument(
        "--grip-pos", type=float, default=None,
        help=f"移动二指夹爪到指定开口（m，范围 {GRIP_OPENING_MIN_M:.3f}~{GRIP_OPENING_MAX_M:.3f}）",
    )
    parser.add_argument(
        "--grip-vel", type=float, default=GRIP_DEFAULT_VEL,
        help=f"二指夹爪命令速度（m/s），默认 {GRIP_DEFAULT_VEL}",
    )
    parser.add_argument(
        "--grip-force", type=float, default=GRIP_DEFAULT_FORCE,
        help=f"二指夹爪命令夹持力（N），默认 {GRIP_DEFAULT_FORCE}",
    )
    parser.add_argument(
        "--interactive", action="store_true",
        help="保持节点运行，等待外部 Python 调用（适合 IPython/REPL）",
    )
    parser.add_argument(
        "--move-ee-delta", action="store_true",
        help="沿 EE 局部坐标系移动末端（需配合 --delta-x/y/z 和 --ee-side）",
    )
    parser.add_argument(
        "--delta-x", type=float, default=0.0,
        help="EE 局部 X 轴位移量（m），x=夹爪下方",
    )
    parser.add_argument(
        "--delta-y", type=float, default=0.0,
        help="EE 局部 Y 轴位移量（m），y=夹爪左方",
    )
    parser.add_argument(
        "--delta-z", type=float, default=0.0,
        help="EE 局部 Z 轴位移量（m），z=夹爪前方",
    )
    cli_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)

    lock_joints = None if cli_args.no_lock else DEFAULT_LOCK_JOINTS
    controller = RobotController(
        lock_joints=lock_joints,
        enable_safety_check=not cli_args.no_safety,
        enable_limit_check=not cli_args.no_limits,
        enable_ik=(cli_args.print_ee or cli_args.move_ee_delta),
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not controller.wait_for_state(timeout=5.0):
            print("[FATAL] 未收到机器人状态，请检查：")
            print("  1. 运控是否启动 (rosa run t800_mc_server start_mc_client)")
            print("  2. SDK 控制器是否切换 (switch_controller config_mc_walker_s2_v1_sps)")
            print("  3. DDS 中间件是否为 CycloneDDS")
            return

        if cli_args.print_state:
            cmd_print_state(controller)
        elif cli_args.print_ee:
            sides = ["left", "right"] if cli_args.ee_side == "both" else [cli_args.ee_side]
            cmd_print_ee(controller, sides)
        elif cli_args.move_joint:
            if cli_args.joint is None or cli_args.pos is None:
                print("[ERROR] --move-joint requires --joint and --pos")
                return
            cmd_print_state(controller)
            print(
                f"\n=== 单关节测试: {cli_args.joint} -> {cli_args.pos:+.4f} rad "
                f"({cli_args.duration:.1f}s) ==="
            )
            ok = controller.move_to_pose(
                {cli_args.joint: cli_args.pos},
                duration_sec=cli_args.duration,
                wait=True,
                unlock_required_joints=True,
            )
            current = controller.get_current_position()
            if current is not None:
                idx = controller.joint_index(cli_args.joint)
                actual = float(current[idx])
                err = abs(actual - float(cli_args.pos))
                print(f"{cli_args.joint}: actual={actual:+.4f}, target={cli_args.pos:+.4f}, err={err:.4f}")
            if ok:
                print("✓ 单关节命令完成并收敛")
            else:
                print("✗ 单关节命令未收敛")
            cmd_print_state(controller)
        elif cli_args.init:
            cmd_print_state(controller)
            print(f"\n=== 分段移动到预备姿态（{cli_args.init_duration:.1f}s）===")
            print("步骤：1a) shoulder pitch + elbow roll  1b) elbow yaw  2) shoulder pitch 回预备姿态  3) READY_POSE")
            input("按回车开始（Ctrl+C 取消）...")
            if controller.move_to_ready_pose(duration_sec=cli_args.init_duration):
                print("✓ 预备姿态轨迹发布完成，等待实际关节收敛...")
                reached, misses = controller.wait_until_position(
                    controller.ready_position_vector(),
                    timeout=cli_args.init_settle_timeout,
                    tolerance=cli_args.init_tolerance,
                )
                if reached:
                    print(f"✓ 预备姿态已到位（误差 ≤ {cli_args.init_tolerance:.3f} rad）")
                else:
                    print(
                        f"⚠️ 预备姿态未完全到位（超时 {cli_args.init_settle_timeout:.1f}s，"
                        f"阈值 {cli_args.init_tolerance:.3f} rad）"
                    )
                    for name, actual, target, err in misses[:8]:
                        if actual is None:
                            print(f"  {name}: no state, target={target:+.4f}")
                        else:
                            print(f"  {name}: actual={actual:+.4f}, target={target:+.4f}, err={err:.4f}")
                cmd_print_state(controller)
            else:
                print("✗ 预备姿态失败")
        elif cli_args.grip_state or cli_args.grip_open or cli_args.grip_close or cli_args.grip_pos is not None:
            sides = ["left", "right"] if cli_args.grip_side == "both" else [cli_args.grip_side]
            if cli_args.grip_state and not (cli_args.grip_open or cli_args.grip_close or cli_args.grip_pos is not None):
                cmd_print_grip_state(controller, sides)
            else:
                if cli_args.grip_open:
                    target = GRIP_OPENING_MAX_M
                elif cli_args.grip_close:
                    target = GRIP_OPENING_MIN_M
                else:
                    target = cli_args.grip_pos
                print(f"\n=== 二指夹爪控制: sides={sides}, pos={target:.4f} m ===")
                for side in sides:
                    if controller.send_grip_command(
                        side,
                        target,
                        vel=cli_args.grip_vel,
                        force=cli_args.grip_force,
                    ):
                        print(f"✓ sent {side} grip command: pos={target:.4f} m")
                    else:
                        print(f"✗ failed to send {side} grip command")
                time.sleep(0.5)
                cmd_print_grip_state(controller, sides)
        elif cli_args.move_ee_delta:
            side = cli_args.ee_side if cli_args.ee_side != "both" else "right"
            delta = (cli_args.delta_x, cli_args.delta_y, cli_args.delta_z)
            if all(abs(v) < 1e-9 for v in delta):
                print("[ERROR] --move-ee-delta 需要至少一个非零位移: --delta-x / --delta-y / --delta-z")
                return
            print(f"\n=== EE 局部坐标系位移: {side} 臂 ===")
            print(f"  局部 delta (m): x={cli_args.delta_x:+.4f}, y={cli_args.delta_y:+.4f}, z={cli_args.delta_z:+.4f}")
            ee_before = controller.get_ee_pose(side)
            if ee_before is not None:
                print(f"  当前位姿 (base frame): xyz={[round(v,4) for v in ee_before[:3]]}  "
                      f"rpy={[round(v,4) for v in ee_before[3:]]}")
            else:
                print("  [WARN] 无法读取当前 EE 位姿")
            ok = controller.move_arm_ee_delta_local(
                side, delta_xyz=delta, duration_sec=cli_args.duration, wait=True,
            )
            if ok:
                print("✓ EE 位移完成")
                ee_after = controller.get_ee_pose(side)
                if ee_after is not None:
                    print(f"  目标位姿 (base frame): xyz={[round(v,4) for v in ee_after[:3]]}  "
                          f"rpy={[round(v,4) for v in ee_after[3:]]}")
                    if ee_before is not None:
                        d = [float(ee_after[i]) - float(ee_before[i]) for i in range(3)]
                        print(f"  实际位移 (base frame): dx={d[0]:+.4f}, dy={d[1]:+.4f}, dz={d[2]:+.4f}")
            else:
                print("✗ EE 位移失败")
        elif cli_args.interactive:
            cmd_print_state(controller)
            print("\n节点运行中，按 Ctrl+C 退出。")
            spin_thread.join()
        else:
            cmd_print_state(controller)
            print("\n用法: --print-state | --print-ee | --init | --move-joint --joint JOINT --pos POS | --move-ee-delta | --grip-state | --grip-open | --grip-close | --grip-pos POS | --interactive")

    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted, shutting down")

    finally:
        controller.stop()
        time.sleep(0.1)
        try:
            executor.shutdown(timeout_sec=1.0)
        except TypeError:
            executor.shutdown()
        spin_thread.join(timeout=2.0)
        if spin_thread.is_alive():
            controller.get_logger().warning("Executor spin thread did not exit before shutdown")
        try:
            executor.remove_node(controller)
        except Exception:
            pass
        controller.destroy_node()
        rclpy.shutdown()

def main_reset(args=None):
    """walker_s2_controller.py reset -- 回 home + 张开双手。"""
    rclpy.init()
    node = WalkerS2Controller(node_name="walker_s2_reset_node")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        time.sleep(0.5)
        node.run_reset()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

def main_endpoint(args=None):
    """walker_s2_controller.py endpoint -- 末端/TCP 位姿调试。"""
    parser = argparse.ArgumentParser(description="Print Walker S2 arm endpoint and gripper link poses")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--move-target-world", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        default=DEFAULT_TARGET_WORLD_POS,
                        help="world-frame target position for the IK TCP before printing again")
    parser.add_argument("--duration", type=float, default=2.0, help="movement duration in seconds")
    parser.add_argument("--no-move", action="store_true", help="only print current poses")
    args = parser.parse_args(args)

    rclpy.init()
    node = WalkerS2Controller(node_name="walker_s2_endpoint_pose_test_node",
                               enable_ik=True, subscribe_images=False)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        time.sleep(0.5)
        ok = node.print_endpoint_poses(side=args.side, timeout=args.timeout)
        if not ok:
            node.get_logger().error("Failed to read Walker S2 endpoint poses")
            sys.exit(1)
        if not args.no_move:
            if not node.move_ee_to_world_pos(side=args.side, world_pos=args.move_target_world,
                                              duration_sec=args.duration):
                node.get_logger().error("Failed to move EE to requested world position")
                sys.exit(1)
            time.sleep(0.5)
            print("\n=== After moving to requested world position ===")
            if not node.print_endpoint_poses(side=args.side, timeout=args.timeout):
                node.get_logger().error("Failed to read Walker S2 endpoint poses after move")
                sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

_ALL_HAND_JOINT_NAMES = set(V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS)


def _is_hand_joint(name):
    return name in _ALL_HAND_JOINT_NAMES


def _infer_hand_side(name):
    if name.startswith("left_"):
        return "left"
    if name.startswith("right_"):
        return "right"
    return None


def _parse_move_arg(move_list):
    result = {}
    for item in move_list:
        if "=" not in item:
            print(f"ERROR bad format: '{item}', expected JointName=angle (e.g. R_elbow_yaw_joint=0.5)")
            sys.exit(1)
        name, val_str = item.split("=", 1)
        try:
            result[name.strip()] = float(val_str.strip())
        except ValueError:
            print(f"ERROR invalid number: '{val_str}'")
            sys.exit(1)
    return result


def _resolve_hand_sides(hand_arg):
    if hand_arg == "both":
        return ["left", "right"]
    if hand_arg in ("left", "right"):
        return [hand_arg]
    print(f"ERROR invalid --hand value: '{hand_arg}', expected left/right/both")
    sys.exit(1)


def _resolve_hand_pose_arg(pose_list):
    if len(pose_list) != 7:
        print(f"ERROR --hand-pose needs 7 values (V4 hand = 7 joints), got {len(pose_list)}")
        sys.exit(1)
    try:
        return [float(v) for v in pose_list]
    except ValueError:
        print("ERROR --hand-pose values must be floats")
        sys.exit(1)


def _build_hand_pose_dict_from_full(pose_values, side):
    return {name: val for name, val in zip(V4_HAND_JOINT_MAP[side], pose_values)}


def main_joint(args=None):
    """walker_s2_controller.py joint -- 关节/手部调试。"""
    all_known_joints = list(BODY_JOINT_NAMES) + list(V4_HAND_LEFT_JOINTS) + list(V4_HAND_RIGHT_JOINTS)

    parser = argparse.ArgumentParser(
        description="Walker S2 关节测试 -- 身体关节与手指关节的状态查询与位置控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--joints", nargs="+", default=None, metavar="JOINT",
                        help="指定关节名（空格分隔），用于 --print / --monitor")
    parser.add_argument("--hand", default=None, choices=["left", "right", "both"],
                        help="指定操作哪只手")
    parser.add_argument("--print", action="store_true", help="打印指定关节的当前状态")
    parser.add_argument("--move", nargs="+", default=None, metavar="JOINT=ANGLE",
                        help="移动身体关节到目标角度（rad）")
    parser.add_argument("--shift", nargs="+", default=None, metavar="JOINT=DELTA",
                        help="身体关节相对当前位置偏移（rad）")
    parser.add_argument("--monitor", action="store_true", help="持续监控指定关节的位置变化")
    parser.add_argument("--interactive", action="store_true", help="交互模式：启动后保持运行，可在 REPL 中调用 API")
    parser.add_argument("--hand-move", nargs="+", default=None, metavar="JOINT=ANGLE",
                        help="移动手指关节到目标角度（rad）")
    parser.add_argument("--hand-shift", nargs="+", default=None, metavar="JOINT=DELTA",
                        help="手指关节相对偏移（rad）")
    parser.add_argument("--hand-pose", nargs=7, default=None, metavar="ANGLE",
                        help="设置整手姿态（7 个角度值 rad）")
    parser.add_argument("--hand-open", action="store_true", help="手指张开（所有关节归零）")
    parser.add_argument("--hand-close", action="store_true", help="手指握拳（所有关节到限位上限）")
    parser.add_argument("--hand-wave", action="store_true", help="手部周期波形运动")
    parser.add_argument("--duration", type=float, default=2.0, help="运动持续时间（秒）")
    parser.add_argument("--monitor-hz", type=float, default=10.0, help="监控刷新频率（Hz）")
    parser.add_argument("--monitor-time", type=float, default=None, help="监控时长（秒）")
    parser.add_argument("--no-lock", action="store_true", help="不锁定任何身体关节")
    parser.add_argument("--no-safety", action="store_true", help="禁用安全速度检查")
    parser.add_argument("--no-limits", action="store_true", help="禁用关节限位裁剪")

    cli_args, ros_args = parser.parse_known_args(args)

    has_body = any([cli_args.print, cli_args.move, cli_args.shift, cli_args.monitor, cli_args.interactive])
    has_hand = any([cli_args.hand_move, cli_args.hand_shift, cli_args.hand_pose,
                    cli_args.hand_open, cli_args.hand_close, cli_args.hand_wave])
    if not has_body and not has_hand:
        cli_args.print = True

    if has_hand and cli_args.hand is None:
        cli_args.hand = "both"

    hand_sides = _resolve_hand_sides(cli_args.hand) if (has_hand or cli_args.hand) else []

    specified_joints = cli_args.joints or []
    body_joint_names = []
    hand_joint_names = []
    if specified_joints:
        for name in specified_joints:
            if name in BODY_JOINT_NAMES:
                body_joint_names.append(name)
            elif _is_hand_joint(name):
                hand_joint_names.append(name)
            else:
                print(f"ERROR unknown joint: '{name}'")
                sys.exit(1)
    else:
        body_joint_names = list(BODY_JOINT_NAMES)

    rclpy.init(args=ros_args)

    lock_joints = None if cli_args.no_lock else DEFAULT_LOCK_JOINTS
    controller = WalkerS2Controller(
        lock_joints=lock_joints,
        enable_safety_check=not cli_args.no_safety,
        enable_limit_check=not cli_args.no_limits,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not controller.wait_for_state(timeout=5.0):
            print("[FATAL] No robot state received. Verify:\n"
                  "  1. motion control started\n"
                  "  2. SDK controller switched (switch_controller config_mc_walker_s2_v1_sps)\n"
                  "  3. DDS middleware is CycloneDDS")
            return

        # ---- body joint operations ----
        if cli_args.print:
            if body_joint_names:
                _print_joint_states(controller, body_joint_names)
            if hand_joint_names:
                sides = sorted(set(_infer_hand_side(n) for n in hand_joint_names if _infer_hand_side(n)))
                _print_hand_states(controller, sides)
            elif hand_sides and not specified_joints:
                _print_hand_states(controller, hand_sides)

        elif cli_args.move:
            pose_dict = _parse_move_arg(cli_args.move)
            for name in pose_dict:
                if name not in BODY_JOINT_NAMES:
                    print(f"ERROR unknown body joint: '{name}'"); sys.exit(1)

            print("\n=== Move body joints ===")
            for name, angle in pose_dict.items():
                lo_hi = ""
                if name in BODY_JOINT_LIMITS:
                    lo, hi = BODY_JOINT_LIMITS[name]
                    lo_hi = f" (limit [{lo:.2f}, {hi:.2f}])"
                print(f"  {name} -> {angle:+.4f} rad ({np.degrees(angle):+.2f} deg){lo_hi}")
            input("\nPress Enter to move (Ctrl+C cancel)...")
            ok = controller.move_to_pose(pose_dict, duration_sec=cli_args.duration,
                                         wait=True, unlock_required_joints=True)
            if ok:
                print("Done. Current positions:")
                _print_joint_states(controller, list(pose_dict.keys()))
            else:
                print("Move failed")

        elif cli_args.shift:
            shift_dict = _parse_move_arg(cli_args.shift)
            for name in shift_dict:
                if name not in BODY_JOINT_NAMES:
                    print(f"ERROR unknown body joint: '{name}'"); sys.exit(1)

            print("\n=== Shift body joints ===")
            for name, delta in shift_dict.items():
                cur = controller.get_joint_position(name)
                if cur is not None:
                    print(f"  {name}: {cur:+.4f} -> {cur + delta:+.4f} rad (delta={delta:+.4f})")
                else:
                    print(f"  {name}: cannot read current position")
            input("\nPress Enter to move (Ctrl+C cancel)...")
            all_ok = True
            for name, delta in shift_dict.items():
                if not controller.shift_joint(name, delta, duration_sec=cli_args.duration, wait=True):
                    print(f"  {name} shift failed")
                    all_ok = False
            if all_ok:
                print("Shift done. Current positions:")
                _print_joint_states(controller, list(shift_dict.keys()))

        elif cli_args.monitor:
            print(f"\n=== Monitoring joints ({cli_args.monitor_hz}Hz) ===")
            if cli_args.monitor_time:
                print(f"  Duration: {cli_args.monitor_time:.1f}s")
            else:
                print("  Ctrl+C to stop")
            if body_joint_names:
                controller.monitor_joints(body_joint_names, hz=cli_args.monitor_hz,
                                          duration_sec=cli_args.monitor_time)
            elif hand_joint_names:
                controller.monitor_hand_joints(hand_joint_names, hz=cli_args.monitor_hz,
                                               duration_sec=cli_args.monitor_time)

        elif cli_args.interactive:
            _print_joint_states(controller, body_joint_names or None)
            if hand_sides:
                _print_hand_states(controller, hand_sides)
            print("\nNode running. Available API on 'controller':\n"
                  "  get_joint_position('R_elbow_yaw_joint')\n"
                  "  move_joint('R_elbow_yaw_joint', 0.5, duration_sec=2.0)\n"
                  "  shift_joint('R_elbow_yaw_joint', +0.1)\n"
                  "  monitor_joints(['head_pitch_joint'], hz=10)\n"
                  "  move_hand('left', {'thumb_swing': 0.5})\n"
                  "  shift_hand('right', 'index_mcp', +0.2)\n"
                  "  monitor_hand_joints(['left_thumb_swing', 'left_index_mcp'])\n"
                  "Ctrl+C to exit.")
            spin_thread.join()

        # ---- hand operations ----
        elif cli_args.hand_move:
            pose_dict = _parse_move_arg(cli_args.hand_move)
            print("\n=== Move hand joints ===")
            for side in hand_sides:
                print(f"\n  [{side} hand]")
                for name_or_short, angle in pose_dict.items():
                    short = name_or_short
                    if not name_or_short.startswith(side + "_"):
                        full = f"{side}_{name_or_short}"
                    else:
                        full = name_or_short
                        short = name_or_short.removeprefix(side + "_")
                    lo_hi = ""
                    if short in V4_HAND_JOINT_LIMITS:
                        lo, hi = V4_HAND_JOINT_LIMITS[short]
                        lo_hi = f" (limit [{lo:.2f}, {hi:.2f}])"
                    print(f"    {full} -> {angle:+.4f} rad ({np.degrees(angle):+.2f} deg){lo_hi}")
            input("\nPress Enter to move (Ctrl+C cancel)...")
            for side in hand_sides:
                ok = controller.move_hand(side, pose_dict, duration_sec=cli_args.duration, wait=True)
                print(f"  {'OK' if ok else 'FAIL'} {side} hand")

        elif cli_args.hand_shift:
            shift_dict = _parse_move_arg(cli_args.hand_shift)
            print("\n=== Shift hand joints ===")
            for side in hand_sides:
                print(f"\n  [{side} hand]")
                for name_or_short, delta in shift_dict.items():
                    cur = controller.get_hand_joint_position(side, name_or_short)
                    if cur is not None:
                        print(f"    {name_or_short}: {cur:+.4f} -> {cur + delta:+.4f} rad")
                    else:
                        print(f"    {name_or_short}: cannot read current position")
            input("\nPress Enter to move (Ctrl+C cancel)...")
            for side in hand_sides:
                for name_or_short, delta in shift_dict.items():
                    controller.shift_hand(side, name_or_short, delta,
                                          duration_sec=cli_args.duration, wait=True)
            print("Shift done")

        elif cli_args.hand_pose:
            angles = _resolve_hand_pose_arg(cli_args.hand_pose)
            print("\n=== Set hand pose ===")
            for side in hand_sides:
                joint_names = V4_HAND_JOINT_MAP[side]
                print(f"\n  [{side} hand]")
                for name, angle in zip(joint_names, angles):
                    print(f"    {name} -> {angle:+.4f} rad")
            input("\nPress Enter to move (Ctrl+C cancel)...")
            for side in hand_sides:
                pose_dict = _build_hand_pose_dict_from_full(angles, side)
                controller.move_hand(side, pose_dict, duration_sec=cli_args.duration, wait=True)
            print("Hand pose set")

        elif cli_args.hand_open:
            print("\n=== Open hands ===")
            for side in hand_sides:
                print(f"  {side} hand: all joints -> 0.0 rad")
            input("\nPress Enter to move (Ctrl+C cancel)...")
            for side in hand_sides:
                controller.move_hand(side, V4_HAND_OPEN_POSE, duration_sec=cli_args.duration, wait=True)
            print("Hands opened")

        elif cli_args.hand_close:
            print("\n=== Close hands ===")
            for side in hand_sides:
                print(f"  {side} hand")
                for name in V4_HAND_JOINT_MAP[side]:
                    short = name.removeprefix("left_").removeprefix("right_")
                    if short in V4_HAND_JOINT_LIMITS:
                        _, hi = V4_HAND_JOINT_LIMITS[short]
                        print(f"    {name} -> {hi:+.4f} rad")
            input("\nPress Enter to move (Ctrl+C cancel)...")
            for side in hand_sides:
                controller.move_hand(side, V4_HAND_CLOSE_POSE, duration_sec=cli_args.duration, wait=True)
            print("Hands closed")

        elif cli_args.hand_wave:
            print("\n=== Hand wave motion ===")
            for side in hand_sides:
                print(f"  {side} hand")
            print("  Ctrl+C to stop")
            controller.hand_periodic_motion(left_hand="left" in hand_sides,
                                            right_hand="right" in hand_sides)

    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted, shutting down")
    finally:
        controller.stop()
        time.sleep(0.1)
        executor.remove_node(controller)
        controller.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
