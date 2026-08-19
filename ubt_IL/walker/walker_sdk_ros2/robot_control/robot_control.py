#!/usr/bin/env python3
"""
Walker S2 机器人直接控制脚本

从 executor_node_sdk.py 提取，移除 VLA 推理依赖（不订阅 Gr00tMotionChunk），
保留核心的安全检查、线性插值、关节锁定、500Hz 发布逻辑，
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
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    controller = RobotController(
        lock_joints=['head_pitch_joint', 'head_yaw_joint', 'waist_yaw_joint'],
    )
    executor = SingleThreadedExecutor()
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
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ecat_task_msgs.msg import GripCmd, GripStatus
from mc_state_msgs.msg import RobotState
from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
from sensor_msgs.msg import JointState

try:
    from .constants import *
except ImportError:
    from constants import *

# ============================================================================
# 辅助函数
# ============================================================================

_ALL_HAND_JOINT_NAMES = set(V4_HAND_LEFT_JOINTS + V4_HAND_RIGHT_JOINTS)


def _is_hand_joint(name):
    """判断关节名是否为手指关节（全名）。"""
    return name in _ALL_HAND_JOINT_NAMES


def _infer_hand_side(name):
    """从手指关节全名推断手别，返回 "left" / "right" / None。"""
    if name.startswith("left_"):
        return "left"
    if name.startswith("right_"):
        return "right"
    return None


# ============================================================================
# 主控制器
# ============================================================================


class RobotController(Node):
    """Walker S2 SDK 控制器节点

    职责：
        1. 订阅 /mc/sdk/robot_state 维护最新关节位置
        2. 提供 move_to_position / execute_trajectory 等 API
        3. 500Hz 定时器发布 RobotCommand 到 /mc/sdk/robot_command
        4. 安全检查：最大关节速度
        5. 关节锁定：发布时跳过指定关节
        6. 关节限位：超限时自动裁剪到限位边界
    """

    def __init__(
        self,
        node_name="robot_control_node",
        command_topic=None,
        state_topic=None,
        config_path=None,
        control_hz=DEFAULT_CONTROL_HZ,
        lock_joints=None,
        max_joint_speed=DEFAULT_MAX_JOINT_SPEED,
        enable_safety_check=True,
        enable_limit_check=True,
        use_pvt=False,
        pvt_kp=None,
        pvt_kd=None,
        pvt_effort=None,
        control_mode="position",
        velocity_timeout=0.5,
        enable_hand_control=True,
    ):
        super().__init__(node_name)

        self._control_mode = str(control_mode).lower()
        self._enable_hand_control = bool(enable_hand_control)
        # backward compat: use_pvt=True => control_mode="pvt"
        if use_pvt and self._control_mode == "position":
            self._control_mode = "pvt"

        self._config = self._load_config(config_path)
        command_topic = command_topic or self._get_topic("sub", "command", DEFAULT_COMMAND_TOPIC)
        state_topic = state_topic or self._get_topic("pub", "state", DEFAULT_STATE_TOPIC)
        left_hand_topic = self._get_topic("sub", "left_hand_command", V4_HAND_LEFT_TOPIC)
        right_hand_topic = self._get_topic("sub", "right_hand_command", V4_HAND_RIGHT_TOPIC)
        left_hand_state_topic = self._get_topic("pub", "left_hand_state", V4_HAND_LEFT_STATE_TOPIC)
        right_hand_state_topic = self._get_topic("pub", "right_hand_state", V4_HAND_RIGHT_STATE_TOPIC)
        left_grip_topic = self._get_topic("sub", "left_grip_command", GRIP_LEFT_CMD_TOPIC)
        right_grip_topic = self._get_topic("sub", "right_grip_command", GRIP_RIGHT_CMD_TOPIC)
        left_grip_state_topic = self._get_topic("pub", "left_grip_state", GRIP_LEFT_STATE_TOPIC)
        right_grip_state_topic = self._get_topic("pub", "right_grip_state", GRIP_RIGHT_STATE_TOPIC)

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
        self.lock_joints = set(lock_joints or [])

        # PVT 力位混合模式（mode=7）配置
        self.use_pvt = (self._control_mode == "pvt")
        self._pvt_default_kp = _PVT_DEFAULT_KP
        self._pvt_default_kd = _PVT_DEFAULT_KD
        self._pvt_kp = self._normalize_gain_map(pvt_kp, _PVT_DEFAULT_KP)
        self._pvt_kd = self._normalize_gain_map(pvt_kd, _PVT_DEFAULT_KD)
        self._pvt_effort = dict(pvt_effort or {})
        if self.use_pvt:
            self.get_logger().warning(
                "⚠️ PVT (mode=7) enabled with UNVERIFIED conservative Kp/Kd defaults "
                f"(Kp={self._pvt_default_kp}, Kd={self._pvt_default_kd}). "
                "MUST tune on real hardware. Start from a safe pose with small motion. "
                "Too-soft Kp → arm droop; too-stiff Kp → oscillation."
            )

        # 状态缓冲
        self.robot_states_buffer = deque(maxlen=1)
        self.robot_states_buffer_lock = threading.Lock()

        # 轨迹状态
        self.trajectory_lock = threading.Lock()
        self.current_trajectory = np.empty((0, self.n_joints), dtype=float)
        self.current_velocity_trajectory = None  # PVT 速度前馈轨迹 (N, n_joints) 或 None
        self.current_index = 0
        self.is_publishing = False
        self.safety_violation = False
        self.current_publish_joints = None
        self.publish_changed_epsilon = 1e-6

        # ---- 速度模式状态 ----
        self._velocity_cmd = np.zeros(self.n_joints, dtype=float)
        self._last_velocity_time = 0.0
        self._velocity_timeout = float(velocity_timeout)
        self._velocity_timeout_override = None
        self._velocity_timed_out = False
        self._velocity_trajectory = None
        self._position_trajectory = None
        self._velocity_traj_index = 0
        self._velocity_kp = 50.0
        self._velocity_prev_moving = np.zeros(self.n_joints, dtype=bool)
        self._active_velocity_joints = set()

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

        # ==== 手/夹爪控制（可选，Bridge 集成时关闭避免话题冲突）====
        if self._enable_hand_control:
            # 手部发布者（V4 手专用，走 JointCommand 通路，与身体控制独立）
            self.left_hand_pub = self.create_publisher(
                JointCommand, left_hand_topic, qos_pub
            )
            self.right_hand_pub = self.create_publisher(
                JointCommand, right_hand_topic, qos_pub
            )
            self._hand_pubs = {"left": self.left_hand_pub, "right": self.right_hand_pub}

            # 手部状态订阅
            self._hand_states = {}
            self._hand_state_lock = threading.Lock()
            self._hand_state_received = {
                "left": threading.Event(),
                "right": threading.Event(),
            }
            self.left_hand_state_sub = self.create_subscription(
                JointState, left_hand_state_topic,
                lambda msg: self._hand_state_callback("left", msg),
                10, callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.right_hand_state_sub = self.create_subscription(
                JointState, right_hand_state_topic,
                lambda msg: self._hand_state_callback("right", msg),
                10, callback_group=MutuallyExclusiveCallbackGroup(),
            )

            # 夹爪发布者/状态订阅（大寰 PGC-140-50 / 电缸）
            self.left_grip_pub = self.create_publisher(
                GripCmd, left_grip_topic, qos_pub
            )
            self.right_grip_pub = self.create_publisher(
                GripCmd, right_grip_topic, qos_pub
            )
            self._grip_pubs = {"left": self.left_grip_pub, "right": self.right_grip_pub}
            self._grip_states = {}
            self._grip_state_lock = threading.Lock()
            self._grip_state_received = {
                "left": threading.Event(),
                "right": threading.Event(),
            }
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
        else:
            # Bridge 集成模式：手/夹爪由 Bridge 独立管理
            self.left_hand_pub = None
            self.right_hand_pub = None
            self._hand_pubs = {}
            self._hand_states = {}
            self._hand_state_lock = threading.Lock()
            self._hand_state_received = {"left": threading.Event(), "right": threading.Event()}
            self.left_grip_pub = None
            self.right_grip_pub = None
            self._grip_pubs = {}
            self._grip_states = {}
            self._grip_state_lock = threading.Lock()
            self._grip_state_received = {"left": threading.Event(), "right": threading.Event()}

        # 500Hz 控制定时器（默认互斥回调组 + 单线程 executor，对齐 pub_arm_command：
        # MultiThreadedExecutor+Reentrant 在 500Hz 下因 GIL 争用导致定时不均 → 运动关节抖）
        self.control_timer = self.create_timer(
            self.timer_period, self._control_callback,
        )

        self.get_logger().info(
            f"RobotController initialized: {self.n_joints} joints, "
            f"{control_hz}Hz, locked={sorted(self.lock_joints)}, "
            f"limit_check={self.enable_limit_check}, mode={self._control_mode}, "
            f"pvt={self.use_pvt}"
        )
        if self._control_mode == "velocity":
            self.get_logger().info(
                f"Velocity mode timeout={self._velocity_timeout}s, Kp={self._velocity_kp}"
            )

    @property
    def control_mode(self):
        """当前控制模式: "position" | "velocity" | "pvt"."""
        return self._control_mode

    @staticmethod
    def _load_config(config_path=None):
        """读取本容器内的可选 topic 配置；无配置时保持真机默认 topic。"""
        if not config_path:
            return {}
        try:
            import yaml
        except ImportError:
            # PyYAML 只在显式使用 config_path 时需要，避免给真机默认运行增加依赖。
            return {}
        try:
            with open(config_path) as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}

    def _get_topic(self, section, key, default):
        """按 bridge 配置格式取 topic；section/key 是从 bridge 视角命名。"""
        try:
            return self._config["topics"][section][key]["topic"]
        except (KeyError, TypeError):
            return default

    def _normalize_gain_map(self, gain, default):
        """把标量 / dict / None 归一化为 {joint_name: value}（覆盖所有身体关节）。

        Args:
            gain: None（用 default）/ 标量（所有关节同值）/ {joint_name: value}
            default: gain 为 None 时各关节的默认值
        Returns:
            dict: 每个身体关节 → 增益值
        """
        if gain is None:
            return {name: default for name in self.all_joints}
        if isinstance(gain, dict):
            result = {name: default for name in self.all_joints}
            for name, val in gain.items():
                if name in result:
                    result[name] = float(val)
                else:
                    self.get_logger().warning(
                        f"Unknown joint '{name}' in PVT gain map, ignored"
                    )
            return result
        # 标量
        return {name: float(gain) for name in self.all_joints}

    # ========================================================================
    # 公开 API
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

    def set_joint_velocities(self, velocities, joint_names=None, timeout=None):
        """Set joint target velocities (rad/s). Velocity mode only.

        Args:
            velocities: array matching joint_names length, or {name: rad/s} dict
            joint_names: list of joint names to control, None=all joints
            timeout: per-call safety timeout (seconds)
        Returns:
            bool: True on success
        """
        if self._control_mode != "velocity":
            self.get_logger().error(
                f"set_joint_velocities requires velocity mode, current={self._control_mode}"
            )
            return False

        if isinstance(velocities, dict):
            joint_names = list(velocities.keys())
            vel_vals = np.array([float(v) for v in velocities.values()], dtype=float)
        else:
            vel_vals = np.array(velocities, dtype=float).ravel()

        if joint_names is None:
            joint_names = list(self.all_joints)

        if len(vel_vals) != len(joint_names):
            self.get_logger().error(
                f"velocities length {len(vel_vals)} != joint_names length {len(joint_names)}"
            )
            return False

        unknown = [n for n in joint_names if n not in self.all_joints]
        if unknown:
            self.get_logger().error(f"Unknown joint names: {unknown}")
            return False

        vel_arr = np.zeros(self.n_joints, dtype=float)
        for name, val in zip(joint_names, vel_vals):
            idx = self.all_joints.index(name)
            if name in self.lock_joints:
                vel_arr[idx] = 0.0
            else:
                vel_arr[idx] = float(val)

        np.clip(vel_arr, -self.max_joint_speed, self.max_joint_speed, out=vel_arr)

        with self.trajectory_lock:
            self._velocity_cmd = vel_arr.copy()
            self._last_velocity_time = time.time()
            self._velocity_timed_out = False
            self._velocity_timeout_override = float(timeout) if timeout is not None else None
            self._active_velocity_joints = {
                name for name in joint_names
                if name not in self.lock_joints
            }

        return True

    def joint_names(self):
        """所有关节名列表（只读）"""
        return list(self.all_joints)

    def wait_until_position(self, target, timeout=10.0, tolerance=0.05):
        """等待实际关节位置收敛到目标附近（settle 检查）。

        move_to_position(wait=True) 只表示轨迹点发布完毕；真机关节还需要继续
        收敛。此方法基于 /mc/sdk/robot_state 检查实际位置误差，与仿真侧
        WalkerS2Controller.wait_until_position 行为对齐。

        Args:
            target: {joint_name: angle} 字典（只检查出现的关节），
                    或长度 n_joints 的完整目标向量
            timeout: 超时时间（秒）
            tolerance: 判定到位的误差阈值（rad）
        Returns:
            (bool, misses): 到位返回 (True, [])；超时返回 (False,
            [(joint_name, current, target, err), ...])，按误差降序
        """
        if isinstance(target, dict):
            check = {name: float(angle) for name, angle in target.items()}
        else:
            vec = np.array(target, dtype=float)
            if vec.shape != (self.n_joints,):
                self.get_logger().error(
                    f"Target shape {vec.shape} != ({self.n_joints},)"
                )
                return False, []
            check = {name: float(vec[i]) for i, name in enumerate(self.all_joints)}

        idx_map = {name: self.all_joints.index(name) for name in check}
        deadline = time.time() + timeout
        last_pos = None

        while time.time() < deadline:
            pos = self.get_current_position()
            if pos is not None:
                last_pos = pos
                misses = [
                    (name, float(pos[idx_map[name]]), tgt,
                     abs(float(pos[idx_map[name]]) - tgt))
                    for name, tgt in check.items()
                    if abs(float(pos[idx_map[name]]) - tgt) > tolerance
                ]
                if not misses:
                    return True, []
            time.sleep(0.05)

        if last_pos is None:
            return False, [(name, None, tgt, None) for name, tgt in check.items()]

        misses = [
            (name, float(last_pos[idx_map[name]]), tgt,
             abs(float(last_pos[idx_map[name]]) - tgt))
            for name, tgt in check.items()
        ]
        misses.sort(key=lambda item: item[3], reverse=True)
        return False, misses

    # ---- 手部关节 API ----

    # ========================================================================
    # V4 手部控制
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
            return
        joint_names = V4_HAND_JOINT_MAP[side]
        publisher = self._hand_pubs[side]
        pos_list = [float(p) for p in positions]
        if self.enable_limit_check:
            pos_list, _ = self._clamp_hand_position(joint_names, pos_list)
        self._publish_hand_cmd(publisher, joint_names, pos_list)

    # ---- 夹爪 API ----

    # ========================================================================
    # 夹爪控制
    # ========================================================================

    def wait_for_grip_state(self, side=None, timeout=5.0):
        """阻塞等待夹爪状态消息。

        Args:
            side: "left"、"right" 或 None（等待双侧）
            timeout: 超时时间（秒）
        Returns:
            bool: True=收到状态，False=超时
        """
        sides = ["left", "right"] if side is None else [side]
        deadline = time.time() + timeout
        for s in sides:
            if s not in self._grip_state_received:
                self.get_logger().error(f"Invalid grip side '{s}'")
                return False
            remaining = max(0.0, deadline - time.time())
            if not self._grip_state_received[s].wait(timeout=remaining):
                self.get_logger().warning(f"Timeout waiting for {s} grip state ({remaining:.1f}s)")
                return False
        return True

    def get_grip_state(self, side):
        """获取指定侧夹爪最新状态，None 表示无数据。"""
        if side not in ("left", "right"):
            self.get_logger().error(f"Invalid grip side '{side}'")
            return None
        with self._grip_state_lock:
            return self._grip_states.get(side)

    def send_grip_command(
        self,
        side,
        pos,
        force=41.0,
        vel=0.005,
        acc=0.0,
        mode=0,
        init=1,
        stop=0,
        reset=0,
        homing=0,
        repeat_sec=0.5,
        repeat_hz=20.0,
    ):
        """发送夹爪控制命令。

        Args:
            side: "left" 或 "right"
            pos: 目标位置，范围 [0, 0.05] m
            force: 目标力，范围 [41, 100] N
            vel: 目标速度，范围 [0, 0.01] m/s
            acc: 目标加速度，范围 [0, 3] m/s^2（写入 GripCmd.cur 字段）
            mode: 0=位置/速度/力控制，10=推压模式
            init/stop/reset/homing: GripCmd 控制标志
            repeat_sec: 连续发布时长（秒），默认 0.5；0 表示只发布一次
            repeat_hz: 连续发布频率（Hz）
        Returns:
            bool: True=已发布，False=参数错误
        """
        if side not in self._grip_pubs:
            self.get_logger().error(f"Invalid grip side '{side}', expected 'left' or 'right'")
            return False

        pos = self._clamp_scalar("grip pos", float(pos), GRIP_POSITION_LIMIT)
        force = self._clamp_scalar("grip force", float(force), GRIP_FORCE_LIMIT)
        vel = self._clamp_scalar("grip vel", float(vel), GRIP_VELOCITY_LIMIT)
        acc = self._clamp_scalar("grip acc", float(acc), GRIP_ACCELERATION_LIMIT)

        msg = GripCmd()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.init = int(init)
        msg.mode = int(mode)
        msg.stop = int(stop)
        msg.reset = int(reset)
        msg.homing = int(homing)
        msg.pos = pos
        msg.vel = vel
        msg.force = force
        msg.cur = acc

        if repeat_sec and repeat_sec > 0:
            interval = 1.0 / repeat_hz
            n_pub = max(1, int(repeat_sec * repeat_hz))
            for _ in range(n_pub):
                self._grip_pubs[side].publish(msg)
                time.sleep(interval)
        else:
            self._grip_pubs[side].publish(msg)

        self.get_logger().info(
            f"Grip command {side}: pos={pos:.4f}m force={force:.1f}N "
            f"vel={vel:.4f}m/s acc={acc:.2f}m/s^2 mode={mode} "
            f"init={init} stop={stop} reset={reset} homing={homing}"
        )
        return True

    def home_grip(self, side):
        """发送夹爪回零命令。"""
        return self.send_grip_command(
            side, pos=0.0, force=41.0, vel=0.005, acc=0.0, homing=1, repeat_sec=1.0
        )

    def stop_grip(self, side):
        """发送夹爪停止命令。"""
        state = self.get_grip_state(side)
        pos = state.pos if state is not None else 0.0
        return self.send_grip_command(
            side, pos=pos, force=41.0, vel=0.0, acc=0.0, stop=1, repeat_sec=0.2
        )

    def open_grip(self, side, wait=False, timeout=2.0):
        """张开二指夹爪（真机安全参数）。"""
        ok = self.send_grip_command(
            side,
            pos=GRIP_POSITION_LIMIT[1],
            force=GRIP_FORCE_LIMIT[0],
            vel=0.005,
            acc=0.0,
        )
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def close_grip(self, side, wait=False, timeout=2.0):
        """闭合二指夹爪（真机安全参数）。"""
        ok = self.send_grip_command(
            side,
            pos=GRIP_POSITION_LIMIT[0],
            force=GRIP_FORCE_LIMIT[0],
            vel=0.005,
            acc=0.0,
        )
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    def move_grip(self, side, pos, wait=False, timeout=2.0):
        """移动二指夹爪到指定开口（m）。"""
        ok = self.send_grip_command(
            side,
            pos=pos,
            force=GRIP_FORCE_LIMIT[0],
            vel=0.005,
            acc=0.0,
        )
        if ok and wait:
            self.wait_for_grip_state(side, timeout=timeout)
        return ok

    open_two_finger_grip = open_grip
    close_two_finger_grip = close_grip
    move_two_finger_grip = move_grip

    def _grip_state_callback(self, side, msg: GripStatus):
        """夹爪状态回调，缓存最新状态。"""
        with self._grip_state_lock:
            self._grip_states[side] = msg
        self._grip_state_received[side].set()

    def _clamp_scalar(self, name, value, limits):
        """裁剪标量到限位范围。"""
        if not self.enable_limit_check:
            return value
        lo, hi = limits
        if value < lo:
            self.get_logger().warning(f"CLAMPED {name}: {value:.4f} → {lo:.4f}")
            return lo
        if value > hi:
            self.get_logger().warning(f"CLAMPED {name}: {value:.4f} → {hi:.4f}")
            return hi
        return value

    def _execute_hand_trajectory(self, publisher, joint_names, trajectory):
        """按轨迹逐点发布手部 JointCommand（阻塞执行）。

        Args:
            publisher: ROS2 publisher（JointCommand）
            joint_names: 关节名列表
            trajectory: numpy 数组 (N, n_hand_joints)
        """
        period = 1.0 / V4_HAND_TEST_HZ
        n_pts = trajectory.shape[0]
        start_time = time.time()

        for k in range(n_pts):
            t = time.time() - start_time
            if t >= n_pts * period:
                break

            positions = trajectory[k, :].tolist()
            if self.enable_limit_check:
                positions, _ = self._clamp_hand_position(joint_names, positions)
            self._publish_hand_cmd(publisher, joint_names, positions)

            # 频率控制
            elapsed = time.time() - start_time
            next_t = (k + 1) * period
            sleep_t = next_t - elapsed
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

    # ========================================================================
    # 关节空间运动
    # ========================================================================

    def move_to_position(self, target_position, duration_sec=3.0, wait=True,
                         publish_changed_only=False, profile=PROFILE_QUINTIC):
        """平滑移动到目标位置（从当前位置插值）。

        Args:
            target_position: 目标关节位置，长度 n_joints 的列表或 numpy 数组
            duration_sec: 运动持续时间（秒）
            wait: 是否阻塞等待完成
            publish_changed_only: True 时仅发布本次轨迹中实际变化的关节
            profile: 插值 profile
                - 'quintic'：s(τ)=10τ³−15τ⁴+6τ⁵，起止速度/加速度均为 0，无 jerk 阶跃
                  （默认，对齐 pub_arm_command 的平滑运动，消除端点抖动）
                - 'linear'：线性插值（起止速度阶跃，有 jerk；仅向后兼容/需要匀速时显式指定）
                  PVT 模式下自动强制为 quintic（线性 profile 的速度前馈是阶跃，抵消 PVT 平滑效果）
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

        # PVT 模式强制 quintic（线性 profile 的速度前馈是阶跃 → jerk）
        effective_profile = profile
        if self.use_pvt and profile != PROFILE_QUINTIC:
            self.get_logger().warning(
                f"PVT mode active but profile='{profile}'; forcing '{PROFILE_QUINTIC}' "
                f"to avoid velocity-feedforward step (jerk)"
            )
            effective_profile = PROFILE_QUINTIC

        # 起点+终点 → 逐关节插值，s(τ) 为归一化进度 [0,1]
        n_pts = max(2, int(duration_sec * self.control_hz))
        tau = np.linspace(0.0, 1.0, n_pts)
        if effective_profile == PROFILE_QUINTIC:
            s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        elif effective_profile == PROFILE_LINEAR:
            s = tau
        else:
            self.get_logger().error(
                f"Unknown profile '{effective_profile}', use '{PROFILE_LINEAR}' or '{PROFILE_QUINTIC}'"
            )
            return False

        trajectory = np.column_stack([
            current[j] + s * (target[j] - current[j])
            for j in range(self.n_joints)
        ])

        # 速度模式：位置轨迹 -> 速度前馈，_velocity_tick 加实时位置修正
        if self._control_mode == "velocity":
            vel_traj = np.diff(trajectory, axis=0) / (1.0 / self.control_hz)
            np.clip(vel_traj, -self.max_joint_speed, self.max_joint_speed, out=vel_traj)
            for i, name in enumerate(self.all_joints):
                if name in self.lock_joints:
                    vel_traj[:, i] = 0.0
            pos_traj = trajectory[1:, :]
            changed = np.any(np.abs(np.diff(trajectory, axis=0)) > 1e-6, axis=0)
            with self.trajectory_lock:
                self._velocity_trajectory = vel_traj
                self._position_trajectory = pos_traj
                self._velocity_traj_index = 0
                self._velocity_cmd = vel_traj[0, :].copy()
                self._last_velocity_time = time.time()
                self._velocity_timed_out = False
                self._velocity_timeout_override = duration_sec + 1.0
                self._active_velocity_joints = {
                    name for i, name in enumerate(self.all_joints)
                    if changed[i] and name not in self.lock_joints
                }
            if wait:
                est = len(vel_traj) / self.control_hz
                time.sleep(est + 0.05)
            return True

        return self.execute_trajectory(
            trajectory,
            wait=wait,
            publish_changed_only=publish_changed_only,
        )

    def execute_trajectory(self, trajectory, wait=True, publish_changed_only=False):
        """执行预定义轨迹。

        Args:
            trajectory: numpy 数组 (N, n_joints)，每行一个时间步的关节位置
                        点间距按 1/control_hz 秒（500Hz → 2ms/点）
            wait: 是否阻塞等待完成
            publish_changed_only: True 时仅发布轨迹中实际变化的关节
        Returns:
            bool: True=成功，False=失败（维度错误/安全违规）
        """
        trajectory = np.array(trajectory, dtype=float)
        if trajectory.ndim != 2 or trajectory.shape[1] != self.n_joints:
            self.get_logger().error(
                f"Trajectory shape {trajectory.shape} != (N, {self.n_joints})"
            )
            return False

        publish_joints = None
        if publish_changed_only:
            publish_joints = self._infer_changed_joints(trajectory)

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

        # 安全检查：最大关节速度
        if self.enable_safety_check and len(trajectory) >= 2:
            max_speeds = np.max(
                np.abs(np.diff(trajectory, axis=0)) / self.timer_period, axis=0
            )
            unsafe = []
            for i, name in enumerate(self.all_joints):
                if name in self.lock_joints:
                    continue
                if publish_joints is not None and name not in publish_joints:
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

        # PVT 速度前馈：对位置轨迹做数值微分（quintic 下起止≈0，安全）
        velocity_trajectory = None
        if self.use_pvt:
            velocity_trajectory = np.gradient(trajectory, axis=0) / self.timer_period
            # 限幅到 max_joint_speed（防异常前馈）
            if self.enable_safety_check:
                np.clip(
                    velocity_trajectory,
                    -self.max_joint_speed,
                    self.max_joint_speed,
                    out=velocity_trajectory,
                )

        # 写入轨迹
        with self.trajectory_lock:
            self.current_trajectory = trajectory.copy()
            self.current_velocity_trajectory = (
                velocity_trajectory.copy() if velocity_trajectory is not None else None
            )
            self.current_publish_joints = publish_joints
            self.current_index = 0
            self.is_publishing = True
            self.safety_violation = False

        if publish_joints is None:
            publish_desc = "all unlocked joints"
        else:
            publish_desc = ", ".join(sorted(publish_joints)) or "none"
        self.get_logger().info(
            f"Executing trajectory: {len(trajectory)} points, "
            f"~{len(trajectory) / self.control_hz:.2f}s, publish={publish_desc}"
        )

        # 阻塞等待
        if wait:
            while True:
                with self.trajectory_lock:
                    if not self.is_publishing:
                        break
                time.sleep(0.01)

        return True

    def _infer_changed_joints(self, trajectory, epsilon=None):
        """根据轨迹列是否变化推断本次实际需要发布的关节。"""
        eps = self.publish_changed_epsilon if epsilon is None else float(epsilon)
        changed = set()
        for i, name in enumerate(self.all_joints):
            col = trajectory[:, i]
            if float(np.max(np.abs(col - col[0]))) > eps:
                changed.add(name)
        return changed

    def stop(self):
        """Stop current trajectory / velocity control.

        Velocity mode: sets zero velocity, active joints brake once via MODE_POSITION.
        Position mode: stops trajectory immediately.
        """
        if self._control_mode == "velocity":
            with self.trajectory_lock:
                self._velocity_cmd = np.zeros(self.n_joints, dtype=float)
                self._last_velocity_time = time.time()
                self._velocity_timed_out = False
            self.get_logger().info("Stop requested (velocity mode -> zero velocity)")
            return

        with self.trajectory_lock:
            self.is_publishing = False
            self.current_index = self.current_trajectory.shape[0]
            self.current_publish_joints = None
            self.current_velocity_trajectory = None
        self.get_logger().info("Stop requested (position mode)")

    def set_lock_joints(self, joint_names):
        """动态设置锁定关节列表"""
        self.lock_joints = set(joint_names or [])
        self.get_logger().info(f"Lock joints updated: {sorted(self.lock_joints)}")

    @property
    def is_busy(self):
        """是否正在执行轨迹"""
        with self.trajectory_lock:
            return self.is_publishing

    def _clamp_position(self, position):
        """裁剪关节位置到限位范围。

        Args:
            position: numpy 数组 (n_joints,)
        Returns:
            (clamped, violations) 元组：
                clamped: 裁剪后的数组
                violations: [(joint_name, requested, lower, upper), ...] 被裁剪的关节列表
        """
        clamped = position.copy()
        violations = []
        for i, name in enumerate(self.all_joints):
            if name not in BODY_JOINT_LIMITS:
                continue
            lo, hi = BODY_JOINT_LIMITS[name]
            val = clamped[i]
            if val < lo:
                clamped[i] = lo
                violations.append((name, val, lo, hi))
            elif val > hi:
                clamped[i] = hi
                violations.append((name, val, lo, hi))
        return clamped, violations

    def _clamp_hand_position(self, joint_names, positions):
        """裁剪手部关节位置到限位范围。

        Args:
            joint_names: 关节名列表（如 ["left_thumb_swing", ...]）
            positions: 对应位置列表
        Returns:
            (clamped_positions, violations) 元组
        """
        clamped = list(positions)
        violations = []
        for i, name in enumerate(joint_names):
            short = name.removeprefix("left_").removeprefix("right_")
            if short not in V4_HAND_JOINT_LIMITS:
                continue
            lo, hi = V4_HAND_JOINT_LIMITS[short]
            val = clamped[i]
            if val < lo:
                clamped[i] = lo
                violations.append((name, val, lo, hi))
            elif val > hi:
                clamped[i] = hi
                violations.append((name, val, lo, hi))
        return clamped, violations

    def move_arm_joints(self, side, joints, speed=0.5, wait=True):
        """Move single-side arm joints. Uses move_joints internally."""
        if side not in ("left", "right"):
            self.get_logger().error(f"Invalid arm side: {side}")
            return False
        joint_names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        if len(joints) != len(joint_names):
            self.get_logger().error(f"{side} arm expects {len(joint_names)} joints, got {len(joints)}")
            return False
        return self.move_joints(
            dict(zip(joint_names, [float(v) for v in joints])),
            speed=speed, wait=wait,
        )

    def move_left_arm_joints(self, joints, speed=0.5, wait=True):
        return self.move_arm_joints("left", joints, speed=speed, wait=wait)

    def move_right_arm_joints(self, joints, speed=0.5, wait=True):
        return self.move_arm_joints("right", joints, speed=speed, wait=wait)

    move_left_arm = move_left_arm_joints
    move_right_arm = move_right_arm_joints

    def move_to_ready_pose(self, duration_sec=None, wait=True, staged=True):
        """3 段初始化（BT pick_ready_pose_sequence，与仿真侧对齐）：
        肩部外展+旋转(5s) -> 肩部俯仰+折叠肘部+肘部旋转(5s) ->
        最终姿态+头部/腰部归位(3s)。

        每段发布完成后做 settle 检查（tolerance=0.05 rad），关节实际到位后
        再进入下一段，避免中间段误差累积导致跟踪失败。

        Args:
            duration_sec: 分段运动总时长（秒），默认 13.0，按 5/5/3s 比例缩放；
                不足 13.0s 强制按 13.0s 执行（stage 1 大角度外展至少需要 ~5s）
            wait: 保留 API 兼容；分段为保证安全顺序始终阻塞执行
            staged: True=3 段对齐仿真流程；"legacy"=旧 4 段抬臂过肩流程；False=直达
        Returns:
            bool: True=成功，False=失败
        """
        if not staged:
            return self.move_to_pose_dict(
                READY_POSE, duration_sec=duration_sec or 3.0, wait=wait)

        if staged == "legacy":
            return self._move_to_ready_pose_legacy(duration_sec, wait)

        if not wait:
            self.get_logger().warning("Staged init requires wait=True, forcing sync")
            wait = True

        duration_sec = float(duration_sec if duration_sec is not None else 13.0)
        # BT 固定三段 5/5/3s；stage 1（shoulder_yaw ∓2.0 rad + shoulder_roll 0.65 rad
        # 大角度外展）至少需要 ~5s，总时长不足 13.0s 强制按 13.0s 执行
        if duration_sec < 13.0:
            self.get_logger().warning(
                f"Ready pose duration {duration_sec:.2f}s too short for staged motion "
                f"(stage 1 needs ~5s); using 13.00s"
            )
            duration_sec = 13.0

        stages = [
            ("1/3 肩部外展 + 旋转", READY_STAGE_1_POSE, 5.0 / 13.0),
            ("2/3 肩部俯仰 + 折叠肘部 + 肘部旋转", READY_STAGE_2_POSE, 5.0 / 13.0),
            ("3/3 最终姿态 + 头部/腰部归位", READY_POSE, 3.0 / 13.0),
        ]

        for label, pose, ratio in stages:
            stage_duration = duration_sec * ratio
            self.get_logger().info(f"Ready pose stage {label}: {stage_duration:.2f}s")
            if not self.move_to_pose_dict(pose, duration_sec=stage_duration, wait=True):
                self.get_logger().error(f"Ready pose stage failed: {label}")
                return False
            # settle 检查：确保关节实际到位后再进入下一段（与仿真侧一致，
            # 未收敛视为本段失败，避免误差耦合累积）
            arrived, misses = self.wait_until_position(
                pose, timeout=10.0, tolerance=0.05)
            if not arrived:
                self.get_logger().error(
                    f"Ready pose stage '{label}' settle failed: "
                    f"{len(misses)} joint(s) not converged within 10.0s "
                    f"(tolerance=0.050 rad): {misses[:5]}"
                )
                return False

        return True

    def _move_to_ready_pose_legacy(self, duration_sec=None, wait=True):
        """旧 4 段 init 流程（抬臂过肩路径，--legacy-init 回退用）。

        Stage 1a: shoulder pitch + elbow roll（抬臂过肩 + 深折叠肘）
        Stage 1b: elbow yaw
        Stage 2:  shoulder pitch return
        Stage 3:  full READY_POSE
        """
        if duration_sec is None:
            duration_sec = 20.0

        if not wait:
            self.get_logger().warning("Staged init requires wait=True, forcing sync")
            wait = True

        duration_sec = float(duration_sec)
        if duration_sec < 1.5:
            duration_sec = 1.5

        stages = [
            ("1a/4 shoulder pitch + elbow roll",
             LEGACY_READY_STAGE_1_PITCH_ROLL_POSE, duration_sec * 0.35),
            ("1b/4 elbow yaw",
             LEGACY_READY_STAGE_1_ELBOW_YAW_POSE, duration_sec * 0.35),
            ("2/4 shoulder pitch return",
             LEGACY_READY_STAGE_2_POSE, duration_sec * 0.2),
            ("3/4 full READY_POSE",
             READY_POSE, duration_sec * 0.1),
        ]

        for label, pose, stage_duration in stages:
            self.get_logger().info(f"Ready pose {label}: {stage_duration:.1f}s")
            if not self.move_to_pose_dict(pose, duration_sec=stage_duration, wait=True):
                self.get_logger().error(f"Ready pose stage failed: {label}")
                return False
        return True

    def move_to_home(self, duration_sec=15.0, wait=True, staged=True):
        """3 段归零（BT return_zero_sequence，与仿真侧对齐）：
        收肩俯仰(5s) -> 展开肘部(5s) -> 全部归零(5s)。

        从任意位姿（通常为 READY_POSE 附近）安全地回到 home 全零位
        （双臂 + 头部 + 腰部）。与 move_to_ready_pose 共用中间姿态（逆序）。
        每段发布完成后做 settle 检查（tolerance=0.05 rad）。

        Args:
            duration_sec: 分段运动总时长（秒），默认 15.0，按 5/5/5s 比例缩放；
                不足 15.0s 强制按 15.0s 执行（段 2 大角度展开至少需要 ~5s）
            wait: 保留 API 兼容；分段为保证安全顺序始终阻塞执行
            staged: True=3 段对齐仿真流程；"legacy"=旧 4 段顺序回零；False=直达全零
        Returns:
            bool: True=成功，False=失败
        """
        if not staged:
            saved_lock = self.lock_joints.copy()
            self.set_lock_joints({"waist_yaw_joint"})
            try:
                return self.move_to_pose_dict(
                    HOME_POSE, duration_sec=duration_sec, wait=wait)
            finally:
                self.set_lock_joints(list(saved_lock))

        if staged == "legacy":
            return self._move_to_home_legacy(duration_sec, wait)

        if not wait:
            self.get_logger().warning("Staged home requires wait=True, forcing sync")
            wait = True

        duration_sec = float(duration_sec if duration_sec is not None else 15.0)
        # BT 固定三段 5/5/5s；段 2（elbow_roll 1.7 rad 展开 + shoulder_pitch 0.7 rad
        # 回正）至少需要 ~5s，总时长不足 15.0s 强制按 15.0s 执行
        if duration_sec < 15.0:
            self.get_logger().warning(
                f"Home duration {duration_sec:.2f}s too short for staged motion "
                f"(stage 2 needs ~5s); using 15.00s"
            )
            duration_sec = 15.0

        stages = [
            ("1/3 收肩俯仰", HOME_STAGE_1_POSE, 1.0 / 3.0),
            ("2/3 展开肘部", HOME_STAGE_2_POSE, 1.0 / 3.0),
            ("3/3 全部归零（双臂 + 头部 + 腰部）", HOME_POSE, 1.0 / 3.0),
        ]

        for label, pose, ratio in stages:
            stage_duration = duration_sec * ratio
            self.get_logger().info(f"Home stage {label}: {stage_duration:.2f}s")
            if not self.move_to_pose_dict(pose, duration_sec=stage_duration, wait=True):
                self.get_logger().error(f"Home stage failed: {label}")
                return False
            # settle 检查：确保关节实际到位后再进入下一段（与 init/仿真侧一致）
            arrived, misses = self.wait_until_position(
                pose, timeout=10.0, tolerance=0.05)
            if not arrived:
                self.get_logger().error(
                    f"Home stage '{label}' settle failed: "
                    f"{len(misses)} joint(s) not converged within 10.0s "
                    f"(tolerance=0.050 rad): {misses[:5]}"
                )
                return False

        return True

    def _move_to_home_legacy(self, duration_sec=15.0, wait=True):
        """旧 4 段回零流程（--legacy-home 回退用）。

        Stage 1: head                 (independent, safe first)
        Stage 2: wrists + elbow_yaw   (clear end-effectors)
        Stage 3: elbow_roll           (fold elbows)
        Stage 4: shoulders            (lower arms)

        Locked joints are temporarily unlocked (waist stays locked).
        """
        saved_lock = self.lock_joints.copy()

        try:
            # Unlock head so we can zero it
            self.set_lock_joints({"waist_yaw_joint"})

            stages = [
                ("1/4 head", {
                    "head_pitch_joint": 0.0, "head_yaw_joint": 0.0,
                }, duration_sec * 0.15),
                ("2/4 wrists + elbow_yaw", {
                    "L_wrist_pitch_joint": 0.0, "L_wrist_roll_joint": 0.0,
                    "R_wrist_pitch_joint": 0.0, "R_wrist_roll_joint": 0.0,
                    "L_elbow_yaw_joint": 0.0, "R_elbow_yaw_joint": 0.0,
                }, duration_sec * 0.3),
                ("3/4 elbow_roll", {
                    "L_elbow_roll_joint": 0.0, "R_elbow_roll_joint": 0.0,
                }, duration_sec * 0.3),
                ("4/4 shoulders", {
                    "L_shoulder_pitch_joint": 0.0, "L_shoulder_roll_joint": 0.0,
                    "L_shoulder_yaw_joint": 0.0,
                    "R_shoulder_pitch_joint": 0.0, "R_shoulder_roll_joint": 0.0,
                    "R_shoulder_yaw_joint": 0.0,
                }, duration_sec * 0.25),
            ]

            for label, pose, stage_dur in stages:
                self.get_logger().info(f"Home {label}: {stage_dur:.1f}s")
                if not self.move_to_pose_dict(pose, duration_sec=stage_dur, wait=True):
                    self.get_logger().error(f"Home stage failed: {label}")
                    return False
            return True
        finally:
            self.set_lock_joints(list(saved_lock))

    def move_to_pose_dict(self, pose_dict, duration_sec=1.5, wait=True,
                          unlock_required_joints=True):
        """Move joints by {name: angle} dict. Unspecified joints hold position.

        Thin wrapper around move_joints that auto-handles lock-joint unlock.
        """
        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position")
            return False

        # compute speed so each joint finishes in duration_sec
        max_dist = 0.0
        for name, angle in pose_dict.items():
            if name not in self.all_joints:
                self.get_logger().error(f"Unknown joint: {name}")
                return False
            idx = self.all_joints.index(name)
            max_dist = max(max_dist, abs(angle - current[idx]))

        speed = max(0.01, max_dist / max(duration_sec, 0.1))

        # temp unlock
        need_unlock = [n for n in pose_dict if n in self.lock_joints]
        saved_lock = None
        if need_unlock and unlock_required_joints:
            saved_lock = self.lock_joints.copy()
            self.set_lock_joints(list(self.lock_joints - set(need_unlock)))

        try:
            return self.move_joints(pose_dict, speed=speed, wait=wait)
        finally:
            if saved_lock is not None:
                self.set_lock_joints(list(saved_lock))

    # ========================================================================
    # 单关节粒度的便捷方法与诊断工具
    # ========================================================================

    # -- 身体单关节 --

    def get_joint_position(self, joint_name):
        """获取单个身体关节的当前位置（rad），None 表示无数据。"""
        try:
            idx = self.joint_index(joint_name)
        except ValueError:
            self.get_logger().error(f"Unknown joint: {joint_name}")
            return None
        pos = self.get_current_position()
        return float(pos[idx]) if pos is not None else None

    def get_joints_positions(self, joint_names):
        """批量获取身体关节的当前位置，返回 {name: position} dict。"""
        pos = self.get_current_position()
        result = {}
        for name in joint_names:
            try:
                idx = self.joint_index(name)
                result[name] = float(pos[idx]) if pos is not None else None
            except ValueError:
                result[name] = None
        return result

    # ---- 单关节/多关节统一核心 ----

    def _move_joints_impl(self, items, wait):
        """Core: build combined target, compute max duration, call move_to_position.

        Args:
            items: list of (joint_name, target_angle, speed)
            wait: block until completion
        Returns: bool
        """
        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position")
            return False

        target = current.copy()
        durations = []

        for name, angle, spd in items:
            if name not in self.all_joints:
                self.get_logger().error(f"Unknown joint: {name}")
                return False
            idx = self.all_joints.index(name)
            dist = abs(angle - current[idx])
            target[idx] = float(angle)
            durations.append(max(0.1, dist / max(abs(spd), 1e-6)))

        duration = max(durations)

        names_str = ", ".join(
            f"{n}: {current[self.all_joints.index(n)]:.3f}->{a:.3f}"
            for n, a, _ in items
        )
        self.get_logger().info(
            f"move_joints: {names_str} | duration={duration:.2f}s"
        )
        return self.move_to_position(target, duration_sec=duration, wait=wait,
                                     publish_changed_only=True)

    def _apply_pvt_gains(self, joint_names, kp, kd):
        """Temporarily override PVT gains for given joints. Returns (saved_kp, saved_kd)."""
        saved_kp, saved_kd = {}, {}
        if kp is not None:
            if isinstance(kp, dict):
                for n, v in kp.items():
                    saved_kp[n] = self._pvt_kp.get(n)
                    self._pvt_kp[n] = float(v)
            else:
                for n in joint_names:
                    saved_kp[n] = self._pvt_kp.get(n)
                    self._pvt_kp[n] = float(kp)
        if kd is not None:
            if isinstance(kd, dict):
                for n, v in kd.items():
                    saved_kd[n] = self._pvt_kd.get(n)
                    self._pvt_kd[n] = float(v)
            else:
                for n in joint_names:
                    saved_kd[n] = self._pvt_kd.get(n)
                    self._pvt_kd[n] = float(kd)
        return saved_kp, saved_kd

    def _restore_pvt_gains(self, saved_kp, saved_kd):
        """Restore PVT gains after _apply_pvt_gains."""
        for n, v in saved_kp.items():
            if v is None: self._pvt_kp.pop(n, None)
            else: self._pvt_kp[n] = v
        for n, v in saved_kd.items():
            if v is None: self._pvt_kd.pop(n, None)
            else: self._pvt_kd[n] = v

    def move_joint(self, joint_name, target_angle, speed, kp=None, kd=None, wait=True):
        """Single-joint position+speed control. Works with all modes.

        Args:
            joint_name: joint name (e.g. "R_elbow_yaw_joint")
            target_angle: target angle (rad)
            speed: max speed (rad/s)
            kp: PVT position gain override (PVT mode only, ignored otherwise)
            kd: PVT velocity gain override (PVT mode only, ignored otherwise)
            wait: block until completion
        Returns: bool
        """
        saved_kp, saved_kd = self._apply_pvt_gains([joint_name], kp, kd)
        try:
            return self._move_joints_impl([(joint_name, target_angle, speed)], wait=wait)
        finally:
            self._restore_pvt_gains(saved_kp, saved_kd)

    def shift_joint(self, joint_name, delta_rad, speed=1.0, wait=True):
        """Shift single joint by relative delta."""
        current = self.get_joint_position(joint_name)
        if current is None:
            self.get_logger().error(f"Cannot shift {joint_name}: no current position")
            return False
        return self.move_joint(joint_name, current + delta_rad, speed=speed, wait=wait)

    def move_joints(self, targets, speed=None, kp=None, kd=None, wait=True):
        """Multi-joint position+speed control. Works with all modes.

        All joints start/stop together. Duration = slowest joint.

        Args:
            targets: {name: angle} or {name: (angle, speed)}
            speed: uniform speed (rad/s)
            kp: PVT Kp override - scalar or {name: value} dict
            kd: PVT Kd override - scalar or {name: value} dict
            wait: block until completion
        Returns: bool
        """
        if isinstance(targets, (list, tuple)):
            items = [list(item) for item in targets]
        elif isinstance(targets, dict):
            items = []
            for name, val in targets.items():
                if isinstance(val, (list, tuple)):
                    items.append([name, float(val[0]),
                                  abs(float(val[1])) if len(val) > 1 and val[1] is not None else None])
                else:
                    items.append([name, float(val), None])
            for item in items:
                if item[2] is None:
                    if speed is not None:
                        item[2] = abs(float(speed))
                    else:
                        self.get_logger().error(f"No speed for {item[0]}")
                        return False
        else:
            self.get_logger().error("targets must be dict or list")
            return False

        joint_names = [item[0] for item in items]
        saved_kp, saved_kd = self._apply_pvt_gains(joint_names, kp, kd)
        try:
            return self._move_joints_impl(
                [(name, angle, spd) for name, angle, spd in items],
                wait=wait,
            )
        finally:
            self._restore_pvt_gains(saved_kp, saved_kd)

    def print_joint_states(self, joint_names=None):
        """格式化打印身体关节的当前状态（含限位和锁定信息）。"""
        names = joint_names or self.all_joints
        pos = self.get_current_position()
        if pos is None:
            print("No current position available")
            return
        print(f"\n{'关节名':<32s} {'位置(rad)':>10s} {'位置(°)':>9s} {'限位范围':>18s} {'状态':>8s}")
        print("-" * 82)
        for name in names:
            try:
                idx = self.joint_index(name)
            except ValueError:
                print(f"  {name:<32s} UNKNOWN")
                continue
            val = pos[idx]
            deg = np.degrees(val)
            locked = "LOCKED" if name in self.lock_joints else ""
            if name in BODY_JOINT_LIMITS:
                lo, hi = BODY_JOINT_LIMITS[name]
                range_str = f"[{lo:.2f}, {hi:.2f}]"
                status = "⚠️BELOW" if val < lo else ("⚠️ABOVE" if val > hi else "OK")
            else:
                range_str = "N/A"
                status = ""
            print(f"  {name:<32s} {val:>+10.4f} {deg:>+9.2f} {range_str:>18s} {status:>8s} {locked}")

    def monitor_joints(self, joint_names, hz=10, duration_sec=None):
        """持续监控指定身体关节的位置变化（终端实时打印）。"""
        interval = 1.0 / hz
        start_time = time.time()
        header = f"{'time':>6s}" + "".join(
            f"  {n.replace('_joint','').replace('_',' '):>12s}" for n in joint_names
        )
        print(header + "\n" + "-" * len(header))
        try:
            while True:
                if duration_sec is not None and (time.time() - start_time) >= duration_sec:
                    break
                pos = self.get_current_position()
                if pos is not None:
                    elapsed = time.time() - start_time
                    line = f"{elapsed:6.1f}"
                    for name in joint_names:
                        try:
                            line += f"  {pos[self.joint_index(name)]:>+12.4f}"
                        except ValueError:
                            line += f"  {'N/A':>12s}"
                    print(line, flush=True)
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        print("\n监控结束")

    # -- 手部单关节 --

    def print_hand_states(self, sides=None):
        """格式化打印手指关节的当前状态（含限位信息）。"""
        sides = sides or ["left", "right"]
        print(f"\n{'关节名':<24s} {'位置(rad)':>10s} {'位置(°)':>9s} {'限位范围':>18s} {'状态':>8s}")
        print("-" * 75)
        for side in sides:
            joint_names = V4_HAND_JOINT_MAP[side]
            pos = self.get_hand_position(side)
            for name in joint_names:
                short = name.removeprefix("left_").removeprefix("right_")
                if pos is not None:
                    idx = joint_names.index(name)
                    val, deg = pos[idx], np.degrees(pos[idx])
                else:
                    val = deg = None
                if short in V4_HAND_JOINT_LIMITS:
                    lo, hi = V4_HAND_JOINT_LIMITS[short]
                    range_str = f"[{lo:.2f}, {hi:.2f}]"
                    status = ("⚠️BELOW" if val < lo else ("⚠️ABOVE" if val > hi else "OK")) if val is not None else "NO DATA"
                else:
                    range_str, status = "N/A", ""
                val_str = f"{val:>+10.4f}" if val is not None else f"{'N/A':>10s}"
                deg_str = f"{deg:>+9.2f}" if deg is not None else f"{'N/A':>9s}"
                print(f"  {name:<24s} {val_str} {deg_str} {range_str:>18s} {status:>8s}")

    def move_hand_joint(self, side, joint_name, target_rad, duration_sec=2.0, wait=True):
        """移动单个手指关节到目标角度。等价于 ``move_hand(side, {joint_name: target_rad}, ...)``。"""
        return self.move_hand(side, {joint_name: target_rad}, duration_sec=duration_sec, wait=wait)

    def shift_hand_joint(self, side, joint_name, delta_rad, duration_sec=2.0, wait=True):
        """手指关节相对当前位置偏移。等价于 ``shift_hand(side, joint_name, delta_rad, ...)``。"""
        return self.shift_hand(side, joint_name, delta_rad, duration_sec=duration_sec, wait=wait)

    def monitor_hand_joints(self, joint_names, hz=10, duration_sec=None):
        """持续监控手指关节的位置变化（终端实时打印）。

        joint_names 必须是手指关节全名（如 ``left_thumb_swing``）。
        """
        by_side = {}
        for name in joint_names:
            side = _infer_hand_side(name)
            if side is None:
                print(f"⚠️ 无法判断手别: {name}，跳过")
                continue
            by_side.setdefault(side, []).append(name)

        interval = 1.0 / hz
        start_time = time.time()
        header = f"{'time':>6s}" + "".join(
            f"  {n.removeprefix('left_').removeprefix('right_'):>12s}" for n in joint_names
        )
        print(header + "\n" + "-" * len(header))
        try:
            while True:
                if duration_sec is not None and (time.time() - start_time) >= duration_sec:
                    break
                elapsed = time.time() - start_time
                line = f"{elapsed:6.1f}"
                for name in joint_names:
                    side = _infer_hand_side(name)
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
        print("\n监控结束")

    # -- 夹爪诊断 --

    def print_grip_states(self, sides=None):
        """格式化打印夹爪当前状态。"""
        sides = sides or ["left", "right"]
        print(f"\n{'夹爪':<8s} {'init':>6s} {'state':>6s} {'error':>6s} {'homed':>6s} {'位置(m)':>10s} {'速度(m/s)':>10s} {'电流(A)':>10s}")
        print("-" * 78)
        for side in sides:
            state = self.get_grip_state(side)
            if state is None:
                print(f"  {side:<6s} {'NO DATA':>68s}")
                continue
            print(f"  {side:<6s} {state.init_state:>6d} {state.grip_state:>6d} "
                  f"{state.error_code:>6d} {state.homed:>6d} "
                  f"{state.pos:>+10.4f} {state.vel:>+10.4f} {state.cur:>+10.4f}")

    def monitor_grips(self, sides=None, hz=10, duration_sec=None):
        """持续监控夹爪状态（终端实时打印）。"""
        sides = sides or ["left", "right"]
        interval = 1.0 / hz
        start_time = time.time()
        header = f"{'time':>6s}" + "".join(f"  {s}_pos {s}_vel {s}_state" for s in sides)
        print(header + "\n" + "-" * len(header))
        try:
            while True:
                if duration_sec is not None and (time.time() - start_time) >= duration_sec:
                    break
                elapsed = time.time() - start_time
                line = f"{elapsed:6.1f}"
                for side in sides:
                    state = self.get_grip_state(side)
                    if state is None:
                        line += f"  {'N/A':>8s} {'N/A':>8s} {'N/A':>8s}"
                    else:
                        line += f"  {state.pos:>+8.4f} {state.vel:>+8.4f} {state.grip_state:>8d}"
                print(line, flush=True)
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        print("\n监控结束")

    def open_hand(self, side, duration_sec=1.0, wait=True):
        return self.move_hand(side, V4_HAND_OPEN_POSE, duration_sec=duration_sec, wait=wait)

    def close_hand(self, side, duration_sec=1.0, wait=True):
        return self.move_hand(side, V4_HAND_CLOSE_POSE, duration_sec=duration_sec, wait=wait)

    # ========================================================================
    # 周期运动测试
    # ========================================================================

    def head_periodic_motion(
        self,
        amplitude=HEAD_TEST_AMPLITUDE,
        period_sec=HEAD_TEST_PERIOD,
        cycles=HEAD_TEST_DEFAULT_CYCLES,
        move_yaw=True,
        move_pitch=True,
        return_to_zero=True,
        wait=True,
    ):
        """头部周期 sin 运动测试（参考 SDK demo pub_head_command.cpp）。

        生成轨迹：position = sin(2π * t / period) * amplitude，
        以本控制器频率（500Hz）采样并通过 execute_trajectory 发布。

        ⚠️ 副作用：
            - 自动解锁 head_pitch_joint / head_yaw_joint
            - 其他关节（含 waist）保持当前位置不变
            - 完成后 head 关节保持解锁状态，如需重新锁定调用 set_lock_joints()

        Args:
            amplitude: 振幅（弧度），默认 0.5（与 SDK demo 一致，约 28.6°）
            period_sec: 单个周期时长（秒），默认 2π≈6.28（与 SDK demo 一致）
            cycles: 运动周期数，默认 2 个完整周期
            move_yaw: 是否运动 head_yaw_joint
            move_pitch: 是否运动 head_pitch_joint
            return_to_zero: 完成最后一个周期后是否额外用 1/4 周期回到 0
                            （防止突然停在非零位置抖动）
            wait: 是否阻塞等待完成
        Returns:
            bool: True=成功
        """
        if not (move_yaw or move_pitch):
            self.get_logger().error("Must move at least one of yaw/pitch")
            return False

        current = self.get_current_position()
        if current is None:
            self.get_logger().error("No current position available")
            return False

        # 校验头部关节存在
        head_joints = []
        if move_pitch:
            if "head_pitch_joint" not in self.all_joints:
                self.get_logger().error("head_pitch_joint not in config")
                return False
            head_joints.append("head_pitch_joint")
        if move_yaw:
            if "head_yaw_joint" not in self.all_joints:
                self.get_logger().error("head_yaw_joint not in config")
                return False
            head_joints.append("head_yaw_joint")

        # 自动解锁头部关节
        locked_head = [j for j in head_joints if j in self.lock_joints]
        if locked_head:
            self.get_logger().warning(
                f"Unlocking head joints for periodic motion: {locked_head}"
            )
            self.set_lock_joints(list(self.lock_joints - set(locked_head)))

        # 生成 sin 轨迹
        total_duration = period_sec * cycles
        if return_to_zero:
            total_duration += period_sec / 4

        n_pts = max(2, int(total_duration * self.control_hz))
        t = np.linspace(0.0, total_duration, n_pts)

        # 初始化轨迹为"当前位置保持不变"，再覆盖头部维度
        trajectory = np.tile(current, (n_pts, 1))

        # 计算 sin 波形
        omega = 2 * np.pi / period_sec
        sin_wave = amplitude * np.sin(omega * t)

        if return_to_zero:
            # 超过 cycles 整周期后线性衰减到 0
            full_cycles_duration = period_sec * cycles
            ramp_mask = t > full_cycles_duration
            if ramp_mask.any():
                ramp_t = t[ramp_mask] - full_cycles_duration
                ramp_factor = 1.0 - ramp_t / (period_sec / 4)
                ramp_factor = np.clip(ramp_factor, 0.0, 1.0)
                sin_wave[ramp_mask] = sin_wave[ramp_mask] * ramp_factor

        for joint_name in head_joints:
            idx = self.all_joints.index(joint_name)
            trajectory[:, idx] = sin_wave

        self.get_logger().info(
            f"Head periodic motion: amplitude={amplitude:.3f} rad, "
            f"period={period_sec:.3f}s, cycles={cycles}, "
            f"total={total_duration:.2f}s, points={n_pts}, "
            f"joints={head_joints}"
        )

        return self.execute_trajectory(trajectory, wait=wait)

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
        """V4 手部周期 sin 运动测试（参考 SDK demo pub_hand_v4_command.cpp）。

        与身体控制走独立通路：
            - 消息：JointCommand（不是 RobotCommand）
            - 话题：/mc/{left,right}_hand/command
            - 模式：mode=5（手部控制器自定义，非 JointCommand 标准枚举）
            - 控制器：手部控制器始终监听，**不需要 switch_controller**

        与 SDK demo 完全一致的运动模式：
            position[i] = sin(2π * t / period + i * phase_diff) * amplitude
            7 个手指关节按 phase_diff（默认 0.2 rad）依次错相，
            产生类似"波浪"的依次张合效果。

        V4 手 vs V3 手：V4 有 7 关节（含 thumb_pip），V3 只有 6 关节。
        本方法只适用于 V4 手。

        ⚠️ 与身体控制方法的差异：
            - 阻塞执行（无 wait 参数，本方法本身就是串行循环）
            - 直接以 publish_hz 循环发布，不进入轨迹队列
            - 完成后**不影响**身体控制（身体可继续 execute_trajectory）

        Args:
            amplitude: 振幅（rad），默认 0.6（与 SDK demo 一致）
            period_sec: 单周期时长（s），默认 2π
            cycles: 循环数，默认 2
            phase_diff: 相邻关节间相位差（rad），默认 0.2
            left_hand: 是否运动左手
            right_hand: 是否运动右手
            publish_hz: 发布频率（Hz），默认 200
            return_to_zero: 完成后是否平滑回到 0 位（避免手指停在张开状态）
        Returns:
            bool: True=完成
        """
        if not (left_hand or right_hand):
            self.get_logger().error("Must enable at least one of left/right hand")
            return False

        parts = []
        if left_hand:
            parts.append("left")
        if right_hand:
            parts.append("right")

        total_duration = period_sec * cycles
        if return_to_zero:
            total_duration += period_sec / 4

        n_pts = max(2, int(total_duration * publish_hz))
        full_cycles_duration = period_sec * cycles

        self.get_logger().info(
            f"V4 hand periodic motion: amplitude={amplitude:.3f} rad, "
            f"period={period_sec:.3f}s, cycles={cycles}, phase_diff={phase_diff:.3f}, "
            f"total={total_duration:.2f}s, hands={parts}, "
            f"publish_hz={publish_hz}, return_to_zero={return_to_zero}"
        )

        omega = 2 * np.pi / period_sec
        period_time = 1.0 / publish_hz
        start_time = time.time()
        hand_limit_warned = False

        try:
            for k in range(n_pts):
                # 实时计算时刻 t（与起始时间对齐，不依赖采样均匀性）
                t = time.time() - start_time
                if t >= total_duration:
                    break

                # 计算 ramp（最后 1/4 周期衰减到 0）
                if return_to_zero and t > full_cycles_duration:
                    ramp_t = t - full_cycles_duration
                    ramp_factor = max(0.0, 1.0 - ramp_t / (period_sec / 4))
                else:
                    ramp_factor = 1.0

                # 为每个手指计算 sin 值（每个关节相位差 i * phase_diff）
                base_phase = omega * t
                positions = [
                    amplitude * ramp_factor * np.sin(base_phase + i * phase_diff)
                    for i in range(len(V4_HAND_LEFT_JOINTS))   # 7 个关节
                ]

                # 限位裁剪
                if self.enable_limit_check:
                    positions, hand_violations = self._clamp_hand_position(
                        V4_HAND_LEFT_JOINTS, positions
                    )
                    # 只在首次超限时打 warning（避免 200Hz 洪水日志）
                    if hand_violations and not hand_limit_warned:
                        for name, val, lo, hi in hand_violations:
                            self.get_logger().warning(
                                f"CLAMPED hand {name}: {val:.4f} → [{lo}, {hi}]"
                            )
                        hand_limit_warned = True

                if left_hand:
                    self._publish_hand_cmd(
                        self.left_hand_pub, V4_HAND_LEFT_JOINTS, positions
                    )
                if right_hand:
                    self._publish_hand_cmd(
                        self.right_hand_pub, V4_HAND_RIGHT_JOINTS, positions
                    )

                # 频率控制：固定时间步 sleep（粗略）
                elapsed = time.time() - start_time
                next_t = (k + 1) * period_time
                sleep_t = next_t - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            self.get_logger().warning("Hand motion interrupted, sending zero command")
            self._send_hand_zero(left_hand, right_hand)
            return False

        # 兜底：完成后再发一次 0 位（保证手指完全放松）
        if return_to_zero:
            self._send_hand_zero(left_hand, right_hand)

        self.get_logger().info("V4 hand periodic motion completed")
        return True

    def _send_hand_zero(self, left_hand, right_hand):
        """向双手发送零位指令（确保手指完全放松）"""
        zeros = [0.0] * len(V4_HAND_LEFT_JOINTS)
        if left_hand:
            self._publish_hand_cmd(self.left_hand_pub, V4_HAND_LEFT_JOINTS, zeros)
        if right_hand:
            self._publish_hand_cmd(self.right_hand_pub, V4_HAND_RIGHT_JOINTS, zeros)

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

    # ========================================================================
    # 内部回调
    # ========================================================================

    def _state_callback(self, msg: RobotState):
        """从 RobotState 提取 n_joints 维位置向量（按 config 中的关节顺序）"""
        joint_states = msg.joint_states
        name_to_idx = {name: idx for idx, name in enumerate(joint_states.name)}

        positions = np.zeros(self.n_joints, dtype=float)
        for i, joint_name in enumerate(self.all_joints):
            if joint_name not in name_to_idx:
                self.get_logger().error(
                    f"Joint '{joint_name}' not in RobotState"
                )
                return
            positions[i] = joint_states.position[name_to_idx[joint_name]]

        with self.robot_states_buffer_lock:
            self.robot_states_buffer.append(positions)

    def _control_callback(self):
        """500Hz dispatch: route to velocity or position tick based on control mode."""
        if self._control_mode == "velocity":
            self._velocity_tick()
        else:
            self._position_tick()

    def _velocity_tick(self):
        """Velocity mode: control only joints in _active_velocity_joints.

        - |vel| > 0  -> MODE_VELOCITY
        - |vel| = 0, was moving -> one-shot MODE_POSITION hold (brake)
        - |vel| = 0, was idle    -> skip
        - not in active set      -> skip
        """
        if self._last_velocity_time == 0.0:
            return

        current = self.get_current_position()

        with self.trajectory_lock:
            if self._velocity_trajectory is not None and self._velocity_traj_index < len(self._velocity_trajectory):
                idx = self._velocity_traj_index
                vel_ff = self._velocity_trajectory[idx, :].copy()
                if self._position_trajectory is not None and current is not None:
                    pos_target = self._position_trajectory[idx, :]
                    pos_err = pos_target - current
                    vel = vel_ff + self._velocity_kp * pos_err
                else:
                    vel = vel_ff
                np.clip(vel, -self.max_joint_speed, self.max_joint_speed, out=vel)
                for i, name in enumerate(self.all_joints):
                    if name in self.lock_joints:
                        vel[i] = 0.0
                self._velocity_traj_index += 1
                self._last_velocity_time = time.time()
                self._velocity_timed_out = False
            else:
                self._velocity_trajectory = None
                self._position_trajectory = None
                self._velocity_traj_index = 0
                timeout = self._velocity_timeout_override if self._velocity_timeout_override is not None else self._velocity_timeout
                elapsed = time.time() - self._last_velocity_time
                if elapsed > timeout:
                    if not self._velocity_timed_out:
                        self.get_logger().warn(
                            f"Velocity safety timeout ({elapsed:.3f}s > {timeout:.3f}s), zeroing velocities"
                        )
                        self._velocity_timed_out = True
                    vel = np.zeros(self.n_joints, dtype=float)
                    self._active_velocity_joints.clear()
                else:
                    vel = self._velocity_cmd.copy()
                    self._velocity_timed_out = False

        cmd = RobotCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ""

        active = self._active_velocity_joints
        moving_now = np.abs(vel) >= 1e-9
        prev_moving = self._velocity_prev_moving

        for idx, name in enumerate(self.all_joints):
            if name in self.lock_joints:
                continue
            if name not in active:
                continue

            if moving_now[idx]:
                jc = JointCmd()
                jc.name = name
                jc.control_mode = JointCmd.MODE_VELOCITY
                jc.velocity = float(vel[idx])
                cmd.joint_cmd.append(jc)
            elif prev_moving[idx]:
                jc = JointCmd()
                jc.name = name
                jc.control_mode = JointCmd.MODE_POSITION
                jc.position = float(current[idx]) if current is not None else 0.0
                cmd.joint_cmd.append(jc)

        self._velocity_prev_moving = moving_now

        if cmd.joint_cmd:
            self.command_pub.publish(cmd)

    def _position_tick(self):
        """500Hz timer: take trajectory point -> build RobotCommand -> publish."""
        if self.safety_violation:
            return

        point = None
        vel_point = None
        publish_joints = None
        is_active = False

        with self.trajectory_lock:
            if self.is_publishing:
                if self.current_index >= self.current_trajectory.shape[0]:
                    self.is_publishing = False
                    self.current_publish_joints = None
                    self.current_velocity_trajectory = None
                    self.get_logger().info("Trajectory execution completed")
                else:
                    point = self.current_trajectory[self.current_index, :]
                    if self.current_velocity_trajectory is not None:
                        vel_point = self.current_velocity_trajectory[self.current_index, :]
                    publish_joints = self.current_publish_joints
                    if publish_joints is not None:
                        publish_joints = set(publish_joints)
                    self.current_index += 1
                    is_active = True

        if point is None:
            return

        cmd = RobotCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ""

        for idx, name in enumerate(self.all_joints):
            if name in self.lock_joints:
                continue
            if is_active and publish_joints is not None and name not in publish_joints:
                continue
            jc = JointCmd()
            jc.name = name
            if self.use_pvt:
                jc.control_mode = JointCmd.CUSTOM_MODE_1
                jc.position = float(point[idx])
                jc.velocity = float(vel_point[idx]) if (is_active and vel_point is not None) else 0.0
                jc.effort = float(self._pvt_effort.get(name, 0.0))
                jc.v1 = float(self._pvt_kp.get(name, self._pvt_default_kp))
                jc.v2 = float(self._pvt_kd.get(name, self._pvt_default_kd))
            else:
                jc.control_mode = JointCmd.MODE_POSITION
                jc.position = float(point[idx])
            cmd.joint_cmd.append(jc)

        if cmd.joint_cmd:
            self.command_pub.publish(cmd)


WalkerS2Controller = RobotController

# ============================================================================
# 命令行入口
# ============================================================================


# ============================================================================
# CLI 辅助解析函数
# ============================================================================


def parse_move_arg(move_list):
    """解析 --move / --shift / --hand-move / --hand-shift 参数，格式：JointName=angle"""
    result = {}
    for item in move_list:
        if "=" not in item:
            print(f"✗ 格式错误: '{item}'，应为 JointName=angle（如 R_elbow_yaw_joint=0.5）")
            sys.exit(1)
        name, val_str = item.split("=", 1)
        name = name.strip()
        try:
            val = float(val_str.strip())
        except ValueError:
            print(f"✗ 数值错误: '{val_str}' 不是有效浮点数")
            sys.exit(1)
        result[name] = val
    return result


def resolve_hand_sides(hand_arg):
    """将 --hand 参数值转为手别列表。"""
    if hand_arg == "both":
        return ["left", "right"]
    if hand_arg in ("left", "right"):
        return [hand_arg]
    print(f"✗ 无效的 --hand 值: '{hand_arg}'，应为 left/right/both")
    sys.exit(1)


def resolve_grip_sides(grip_arg):
    """将 --grip 参数值转为夹爪侧列表。"""
    if grip_arg == "both":
        return ["left", "right"]
    if grip_arg in ("left", "right"):
        return [grip_arg]
    print(f"✗ 无效的 --grip 值: '{grip_arg}'，应为 left/right/both")
    sys.exit(1)


def resolve_hand_pose_arg(pose_list):
    """将 --hand-pose 参数（7个浮点数）转为角度列表。"""
    if len(pose_list) != 7:
        print(f"✗ --hand-pose 需要 7 个角度值（V4 手 = 7 关节），当前 {len(pose_list)} 个")
        sys.exit(1)
    try:
        return [float(v) for v in pose_list]
    except ValueError:
        print("✗ --hand-pose 的值必须为浮点数")
        sys.exit(1)


def build_hand_pose_dict_from_full(pose_values, side):
    """从 7 个角度值和手别构建 {joint_name: angle} dict。"""
    joint_names = V4_HAND_JOINT_MAP[side]
    return dict(zip(joint_names, pose_values))


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


def cmd_demo(controller):
    """Position mode demo: R_elbow_yaw +-0.5 rad roundtrip using move_joint API."""
    _single_joint_demo(controller, "R_elbow_yaw_joint", delta=0.5, speed=0.5)


def cmd_velocity_demo(controller):
    """Velocity mode demo: R_elbow_yaw +-0.5 rad roundtrip using move_joint API."""
    _single_joint_demo(controller, "R_elbow_yaw_joint", delta=0.5, speed=0.5)


def cmd_pvt_demo(controller):
    """PVT mode demo: R_elbow_yaw +-0.5 rad roundtrip using move_joint API."""
    _single_joint_demo(controller, "R_elbow_yaw_joint", delta=0.5, speed=0.5)


def _single_joint_demo(controller, joint_name, delta, speed):
    """Shared single-joint roundtrip demo logic."""
    if joint_name not in controller.all_joints:
        print(f"Joint '{joint_name}' not found, abort demo")
        return

    start_pos = controller.get_current_position()
    if start_pos is None:
        print("No current position, abort")
        return

    try:
        target_angle = start_pos[controller.all_joints.index(joint_name)] + delta
        print(f"Step 1: {joint_name} +{delta:.1f} rad, speed={speed} rad/s ...")
        controller.move_joint(joint_name, target_angle, speed=speed, wait=True)

        time.sleep(0.5)

        start_angle = start_pos[controller.all_joints.index(joint_name)]
        print(f"Step 2: return to start, speed={speed} rad/s ...")
        controller.move_joint(joint_name, start_angle, speed=speed, wait=True)

    finally:
        controller.stop()
        time.sleep(0.2)

    print("\nDemo complete.")


def cmd_head_test(controller, amplitude, period_sec, cycles, yaw_only, pitch_only):
    """头部周期 sin 运动测试（参考 SDK demo pub_head_command.cpp）"""
    move_yaw = not pitch_only
    move_pitch = not yaw_only
    if not move_yaw and not move_pitch:
        print("✗ --yaw-only 和 --pitch-only 不能同时指定")
        return

    parts = []
    if move_pitch:
        parts.append("head_pitch")
    if move_yaw:
        parts.append("head_yaw")

    print("\n=== 头部周期 sin 运动测试 ===")
    print(f"参考脚本：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/.../pub_head_command.cpp")
    print(f"运动关节：{', '.join(parts)}")
    print(f"振幅：     ±{amplitude:.3f} rad (≈ ±{np.degrees(amplitude):.1f}°)")
    print(f"周期：     {period_sec:.3f} s")
    print(f"循环次数： {cycles}")
    print(f"总时长：   {period_sec * cycles + period_sec / 4:.2f} s (含 1/4 周期归零)")
    print()
    print("⚠️ 注意：会临时解锁 head 关节")
    print("⚠️ 注意：其他关节（含 waist、双臂）保持当前位置不变")

    input("\n按回车开始测试（Ctrl+C 取消）...")

    if not controller.head_periodic_motion(
        amplitude=amplitude,
        period_sec=period_sec,
        cycles=cycles,
        move_yaw=move_yaw,
        move_pitch=move_pitch,
        return_to_zero=True,
        wait=True,
    ):
        print("✗ 测试失败")
        return
    print("✓ 头部测试完成（已回到 0 位）")


def cmd_hand_test(controller, amplitude, period_sec, cycles, phase_diff,
                  left_only, right_only):
    """V4 手部周期 sin 运动测试（参考 SDK demo pub_hand_v4_command.cpp）"""
    left_hand = not right_only
    right_hand = not left_only
    if not (left_hand or right_hand):
        print("✗ --left-only 和 --right-only 不能同时指定")
        return

    parts = []
    if left_hand:
        parts.append("左手")
    if right_hand:
        parts.append("右手")

    print("\n=== V4 手部周期 sin 运动测试 ===")
    print(f"参考脚本：walker_sdk_ros2-ubt_ros2_demo_walkerS2_v0.1.8/.../pub_hand_v4_command.cpp")
    print(f"运动手部：{', '.join(parts)}")
    print(f"关节数：   7 个/手（thumb_swing/mcp/pip + index/middle/ring/little_mcp）")
    print(f"振幅：     ±{amplitude:.3f} rad (≈ ±{np.degrees(amplitude):.1f}°)")
    print(f"周期：     {period_sec:.3f} s")
    print(f"相位差：   {phase_diff:.3f} rad（相邻手指相位差，产生波浪效果）")
    print(f"循环次数： {cycles}")
    print(f"总时长：   {period_sec * cycles + period_sec / 4:.2f} s (含归零段)")
    print()
    print("⚠️ 注意：手部走独立通路（JointCommand → /mc/{side}_hand/command）")
    print("⚠️ 注意：不需要 switch_controller（手部控制器始终监听）")
    print("⚠️ 注意：仅对 V4 手（7 关节，含 thumb_pip）有效，V3 手会因 thumb_pip 未知而报错")
    print("⚠️ 注意：身体（双臂/头/腰）保持当前位置不变")

    input("\n按回车开始测试（Ctrl+C 取消）...")

    if not controller.hand_periodic_motion(
        amplitude=amplitude,
        period_sec=period_sec,
        cycles=cycles,
        phase_diff=phase_diff,
        left_hand=left_hand,
        right_hand=right_hand,
        return_to_zero=True,
    ):
        print("✗ 测试失败或被中断")
        return
    print("✓ V4 手部测试完成（已回到 0 位）")


def _has_joint_cli_action(cli_args):
    """Check if any joint/grip CLI action flag is set."""
    return any([
        cli_args.print,
        cli_args.move,
        cli_args.shift,
        cli_args.monitor,
        cli_args.hand_move,
        cli_args.hand_shift,
        cli_args.hand_pose,
        cli_args.hand_open,
        cli_args.hand_close,
        cli_args.hand_wave,
        cli_args.grip_print,
        cli_args.grip_move is not None,
        cli_args.grip_home,
        cli_args.grip_stop,
        cli_args.grip_monitor,
    ])


def _run_joint_cli_actions(controller, cli_args):
    """Execute joint / hand / gripper CLI actions (ported from joint_test.py)."""
    # -- resolve joint name lists --
    specified = cli_args.joints or []
    body_joint_names = []
    hand_joint_names = []
    if specified:
        for name in specified:
            if name in BODY_JOINT_NAMES:
                body_joint_names.append(name)
            elif _is_hand_joint(name):
                hand_joint_names.append(name)
            else:
                print(f"✗ 未知关节名: '{name}'")
                sys.exit(1)
    else:
        body_joint_names = list(BODY_JOINT_NAMES)

    # -- resolve hand / grip sides --
    has_hand_action = any([cli_args.hand_move, cli_args.hand_shift,
                           cli_args.hand_pose, cli_args.hand_open,
                           cli_args.hand_close, cli_args.hand_wave])
    hand_sides = resolve_hand_sides(cli_args.hand or "both") if (
        has_hand_action or cli_args.hand
    ) else []

    has_grip_action = any([cli_args.grip_print, cli_args.grip_move is not None,
                           cli_args.grip_home, cli_args.grip_stop,
                           cli_args.grip_monitor])
    grip_sides = resolve_grip_sides(cli_args.grip or "both") if (
        has_grip_action or cli_args.grip
    ) else []

    # ── Gripper actions ──────────────────────────────────────────────────

    if cli_args.grip_print:
        controller.wait_for_grip_state(timeout=2.0)
        controller.print_grip_states(grip_sides)

    elif cli_args.grip_move is not None:
        print("\n=== 移动夹爪 ===")
        for side in grip_sides:
            print(f"  {side} grip → pos={cli_args.grip_move:.4f}m "
                  f"force={cli_args.grip_force:.1f}N vel={cli_args.grip_vel:.4f}m/s "
                  f"acc={cli_args.grip_acc:.2f}m/s^2 mode={cli_args.grip_mode}")
        input("\n按回车发送夹爪命令（Ctrl+C 取消）...")
        for side in grip_sides:
            controller.send_grip_command(
                side, pos=cli_args.grip_move, force=cli_args.grip_force,
                vel=cli_args.grip_vel, acc=cli_args.grip_acc,
                mode=cli_args.grip_mode, repeat_sec=cli_args.grip_repeat,
            )
        print("✓ 夹爪命令已发送")

    elif cli_args.grip_home:
        print("\n=== 夹爪回零 ===")
        for side in grip_sides:
            print(f"  {side} grip homing=1")
        input("\n按回车发送回零命令（Ctrl+C 取消）...")
        for side in grip_sides:
            controller.home_grip(side)
        print("✓ 回零命令已发送")

    elif cli_args.grip_stop:
        print("\n=== 停止夹爪 ===")
        for side in grip_sides:
            print(f"  {side} grip stop=1")
        input("\n按回车发送停止命令（Ctrl+C 取消）...")
        for side in grip_sides:
            controller.stop_grip(side)
        print("✓ 停止命令已发送")

    elif cli_args.grip_monitor:
        print(f"\n=== 监控夹爪 ({cli_args.monitor_hz}Hz) ===")
        controller.monitor_grips(grip_sides, hz=cli_args.monitor_hz,
                                 duration_sec=cli_args.monitor_time)

    # ── Print ────────────────────────────────────────────────────────────

    elif cli_args.print:
        if body_joint_names:
            controller.print_joint_states(body_joint_names)
        if hand_joint_names:
            sides_for_print = set()
            for name in hand_joint_names:
                side = _infer_hand_side(name)
                if side:
                    sides_for_print.add(side)
            controller.print_hand_states(sorted(sides_for_print))
        elif hand_sides and not specified:
            controller.print_hand_states(hand_sides)

    # ── Body joint actions ───────────────────────────────────────────────

    elif cli_args.move:
        pose_dict = parse_move_arg(cli_args.move)
        for name in pose_dict:
            if name not in BODY_JOINT_NAMES:
                print(f"✗ 未知身体关节名: '{name}'")
                sys.exit(1)
        print("\n=== 移动身体关节 ===")
        for name, angle in pose_dict.items():
            lo_hi = ""
            if name in BODY_JOINT_LIMITS:
                lo, hi = BODY_JOINT_LIMITS[name]
                lo_hi = f" (限位 [{lo:.2f}, {hi:.2f}])"
            print(f"  {name} → {angle:+.4f} rad ({np.degrees(angle):+.2f}°){lo_hi}")
        print(f"速度: {cli_args.speed} rad/s")
        input("\n按回车开始移动（Ctrl+C 取消）...")
        ok = controller.move_joints(pose_dict, speed=cli_args.speed, wait=True)
        if ok:
            print("✓ 移动完成")
            controller.print_joint_states(list(pose_dict.keys()))
        else:
            print("✗ 移动失败")

    elif cli_args.shift:
        shift_dict = parse_move_arg(cli_args.shift)
        for name in shift_dict:
            if name not in BODY_JOINT_NAMES:
                print(f"✗ 未知身体关节名: '{name}'")
                sys.exit(1)
        print("\n=== 身体关节偏移 ===")
        for name, delta in shift_dict.items():
            current = controller.get_joint_position(name)
            if current is not None:
                print(f"  {name}: {current:+.4f} → {current+delta:+.4f} rad "
                      f"(Δ={delta:+.4f} rad, {np.degrees(delta):+.2f}°)")
            else:
                print(f"  {name}: 无法读取当前位置")
        print(f"速度: {cli_args.speed} rad/s")
        input("\n按回车开始移动（Ctrl+C 取消）...")
        for name, delta in shift_dict.items():
            controller.shift_joint(name, delta, speed=cli_args.speed, wait=True)
        print("✓ 偏移完成")
        controller.print_joint_states(list(shift_dict.keys()))

    elif cli_args.monitor:
        print(f"\n=== 监控关节 ({cli_args.monitor_hz}Hz) ===")
        if body_joint_names:
            controller.monitor_joints(body_joint_names, hz=cli_args.monitor_hz,
                                      duration_sec=cli_args.monitor_time)
        elif hand_joint_names:
            controller.monitor_hand_joints(hand_joint_names, hz=cli_args.monitor_hz,
                                           duration_sec=cli_args.monitor_time)

    # ── Hand actions ─────────────────────────────────────────────────────

    elif cli_args.hand_move:
        pose_dict = parse_move_arg(cli_args.hand_move)
        print("\n=== 移动手指关节 ===")
        for side in hand_sides:
            print(f"\n  [{side} 手]")
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
                    lo_hi = f" (限位 [{lo:.2f}, {hi:.2f}])"
                print(f"    {full} → {angle:+.4f} rad ({np.degrees(angle):+.2f}°){lo_hi}")
        input("\n按回车开始移动（Ctrl+C 取消）...")
        for side in hand_sides:
            controller.move_hand(side, pose_dict, duration_sec=cli_args.duration, wait=True)
            print(f"✓ {side} 手移动完成")

    elif cli_args.hand_shift:
        shift_dict = parse_move_arg(cli_args.hand_shift)
        print("\n=== 手指关节偏移 ===")
        for side in hand_sides:
            print(f"\n  [{side} 手]")
            for name_or_short, delta in shift_dict.items():
                current = controller.get_hand_joint_position(side, name_or_short)
                if current is not None:
                    print(f"    {name_or_short}: {current:+.4f} → {current+delta:+.4f} rad "
                          f"(Δ={delta:+.4f} rad, {np.degrees(delta):+.2f}°)")
                else:
                    print(f"    {name_or_short}: 无法读取当前位置")
        input("\n按回车开始移动（Ctrl+C 取消）...")
        for side in hand_sides:
            for name_or_short, delta in shift_dict.items():
                controller.shift_hand(side, name_or_short, delta,
                                      duration_sec=cli_args.duration, wait=True)
        print("✓ 偏移完成")

    elif cli_args.hand_pose:
        angles = resolve_hand_pose_arg(cli_args.hand_pose)
        print("\n=== 设置整手姿态 ===")
        for side in hand_sides:
            pose_dict = build_hand_pose_dict_from_full(angles, side)
            print(f"\n  [{side} 手]")
            for name, angle in pose_dict.items():
                short = name.removeprefix("left_").removeprefix("right_")
                lo_hi = ""
                if short in V4_HAND_JOINT_LIMITS:
                    lo, hi = V4_HAND_JOINT_LIMITS[short]
                    lo_hi = f" (限位 [{lo:.2f}, {hi:.2f}])"
                print(f"    {name} → {angle:+.4f} rad ({np.degrees(angle):+.2f}°){lo_hi}")
        input("\n按回车开始移动（Ctrl+C 取消）...")
        for side in hand_sides:
            controller.move_hand(side, build_hand_pose_dict_from_full(angles, side),
                                 duration_sec=cli_args.duration, wait=True)
            print(f"✓ {side} 手姿态设置完成")

    elif cli_args.hand_open:
        print("\n=== 手指张开 ===")
        for side in hand_sides:
            print(f"  {side} 手: 所有关节 → 0.0 rad")
        input("\n按回车开始移动（Ctrl+C 取消）...")
        for side in hand_sides:
            controller.move_hand(side, V4_HAND_OPEN_POSE,
                                 duration_sec=cli_args.duration, wait=True)
            print(f"✓ {side} 手张开完成")

    elif cli_args.hand_close:
        print("\n=== 手指握拳 ===")
        for side in hand_sides:
            joint_names = V4_HAND_JOINT_MAP[side]
            print(f"  {side} 手:")
            for name in joint_names:
                short = name.removeprefix("left_").removeprefix("right_")
                if short in V4_HAND_JOINT_LIMITS:
                    _, hi = V4_HAND_JOINT_LIMITS[short]
                    print(f"    {name} → {hi:+.4f} rad ({np.degrees(hi):+.2f}°)")
        input("\n按回车开始移动（Ctrl+C 取消）...")
        for side in hand_sides:
            controller.move_hand(side, V4_HAND_CLOSE_POSE,
                                 duration_sec=cli_args.duration, wait=True)
            print(f"✓ {side} 手握拳完成")

    elif cli_args.hand_wave:
        print("\n=== 手部周期波形运动 ===")
        for side in hand_sides:
            print(f"  {side} 手")
        print("  按 Ctrl+C 停止")
        controller.hand_periodic_motion(
            left_hand="left" in hand_sides,
            right_hand="right" in hand_sides,
        )


def main(args=None):
    parser = argparse.ArgumentParser(
        description=(
            "Walker S2 机器人直接控制脚本（SDK 控制器 / JointCmd MODE_POSITION=2）\n\n"
            "示例：\n"
            "  %(prog)s --print-state                     # 查看当前关节状态\n"
            "  %(prog)s --move R_elbow_yaw_joint=0.5       # 移动单个关节\n"
            "  %(prog)s --shift R_shoulder_pitch_joint=+0.1  # 关节相对偏移\n"
            "  %(prog)s --joints R_elbow_yaw_joint --monitor  # 监控关节实时位置\n"
            "  %(prog)s --hand left --hand-open            # 左手张开\n"
            "  %(prog)s --grip both --grip-move 0.02       # 双侧夹爪移动到 0.02m\n"
            "  %(prog)s --init                             # 分3步移动到预备姿态（对齐仿真，默认13s）\n"
            "  %(prog)s --home                             # 手臂关节回零（分3步安全顺序）\n"
            "  %(prog)s --init --direct-init               # 直达预备姿态（不分步）\n"
            "  %(prog)s --init --legacy-init               # 旧4段抬臂过肩流程（回退用）\n"
            "  %(prog)s --home --legacy-home               # 旧4段顺序回零流程（回退用）\n"
            "  %(prog)s --demo                             # 安全演示（位置模式）\n"

            "  %(prog)s --demo --mode velocity             # 速度模式演示\n"
            "  %(prog)s --demo --mode pvt                   # PVT 模式演示\n"
            "  %(prog)s --interactive                      # 保持运行，供 Python API 调用"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "前置条件：\n"
            "  1. 运控已启动：rosa run t800_mc_server start_mc_client\n"
            "  2. SDK 控制器已切换：switch_controller config_mc_walker_s2_v1_sps\n"
            "  3. 机器人已在安全位置（遥控器移到位后再切换控制器）\n"
            "  4. 默认锁定 head_pitch / head_yaw / waist_yaw（不发送指令，保持原位）"
        ),
    )

    # ── 全局控制参数 ──
    ctrl = parser.add_argument_group("全局控制参数")
    ctrl.add_argument("--no-lock", action="store_true",
                      help="不锁定任何关节（默认锁定 head/waist）")
    ctrl.add_argument("--no-safety", action="store_true",
                      help="禁用安全速度检查")
    ctrl.add_argument("--no-limits", action="store_true",
                      help="禁用关节限位裁剪")
    ctrl.add_argument("--hz", type=int, default=DEFAULT_CONTROL_HZ,
                      help=f"控制发布频率（Hz），默认 {DEFAULT_CONTROL_HZ}")
    ctrl.add_argument("--mode", choices=["position", "velocity", "pvt"],
                      default="position",
                      help="控制模式: position | velocity | pvt (默认 position)")
    ctrl.add_argument("--velocity-timeout", type=float, default=0.5, metavar="SEC",
                      help="速度模式安全超时（秒），默认 0.5")
    ctrl.add_argument("--pvt", action="store_true",
                      help="PVT 模式 (等价于 --mode pvt，向后兼容)")
    ctrl.add_argument("--pvt-kp", type=float, default=None, metavar="K",
                      help=f"PVT 位置增益 Kp（标量），默认 {_PVT_DEFAULT_KP}")
    ctrl.add_argument("--pvt-kd", type=float, default=None, metavar="K",
                      help=f"PVT 速度增益 Kd（标量），默认 {_PVT_DEFAULT_KD}")

    # ── 高级动作 ──
    actions = parser.add_argument_group("高级动作（互斥，每次只选一个）")
    actions.add_argument("--print-state", action="store_true",
                         help="打印全部身体关节状态后退出")
    actions.add_argument("--init", action="store_true",
                         help="移动到预备姿态（分3步对齐仿真流程，每段带 settle 检查，默认13s）")
    actions.add_argument("--init-duration", type=float, default=None, metavar="SEC",
                         help="预备姿态总时长（秒），默认13（不足13强制13）")
    actions.add_argument("--init-settle-timeout", type=float, default=10.0, metavar="SEC",
                         help="init 完成后最终收敛检查超时（秒），默认10")
    actions.add_argument("--init-tolerance", type=float, default=0.08, metavar="RAD",
                         help="最终收敛检查误差阈值（rad），默认0.08")
    actions.add_argument("--direct-init", action="store_true",
                         help="直达预备姿态（不分步，3s），覆盖默认分步行为")
    actions.add_argument("--legacy-init", action="store_true",
                         help="旧4段抬臂过肩流程（真机回退用）")
    actions.add_argument("--home", action="store_true",
                         help="所有关节回零（分3步对齐仿真流程，每段带 settle 检查，默认15s）")
    actions.add_argument("--home-duration", type=float, default=None, metavar="SEC",
                         help="回零总时长（秒），默认15（不足15强制15）")
    actions.add_argument("--legacy-home", action="store_true",
                         help="旧4段顺序回零流程（head/腕肘/肘/肩，回退用）")
    actions.add_argument("--demo", action="store_true",
                         help="安全演示：右臂 elbow_yaw +-0.5 rad 往返运动")
    actions.add_argument("--head-test", action="store_true",
                         help="头部周期 sin 运动测试")
    actions.add_argument("--hand-test", action="store_true",
                         help="V4 手部周期 sin 运动测试（波浪效果）")
    actions.add_argument("--interactive", action="store_true",
                         help="保持节点运行，供外部 Python API 调用")

    # ── 身体关节操作 ──
    body = parser.add_argument_group("身体关节操作")
    body.add_argument("--joints", nargs="+", default=None, metavar="NAME",
                      help="指定关节名（空格分隔），默认全部身体关节")
    body.add_argument("--print", action="store_true",
                      help="打印指定关节的当前状态")
    body.add_argument("--move", nargs="+", default=None, metavar="NAME=ANGLE",
                      help="移动关节到目标角度（rad），如 R_elbow_yaw_joint=0.5")
    body.add_argument("--shift", nargs="+", default=None, metavar="NAME=DELTA",
                      help="关节相对偏移（rad），如 R_shoulder_pitch_joint=+0.1")
    body.add_argument("--monitor", action="store_true",
                      help="持续监控关节位置（Ctrl+C 停止）")

    # ── 手部操作 ──
    hand = parser.add_argument_group("V4 手部操作")
    hand.add_argument("--hand", default=None, choices=["left", "right", "both"], metavar="SIDE",
                      help="指定手别：left / right / both")
    hand.add_argument("--hand-move", nargs="+", default=None, metavar="NAME=ANGLE",
                      help="移动手指关节（rad），短名如 thumb_swing=0.5")
    hand.add_argument("--hand-shift", nargs="+", default=None, metavar="NAME=DELTA",
                      help="手指关节偏移（rad），格式同 --hand-move")
    hand.add_argument("--hand-pose", nargs=7, default=None, metavar="ANGLE",
                      help="整手姿态（7 个角度 rad，按 V4 关节顺序）")
    hand.add_argument("--hand-open", action="store_true",
                      help="手指张开（全关节归零）")
    hand.add_argument("--hand-close", action="store_true",
                      help="手指握拳（全关节到限位上限）")
    hand.add_argument("--hand-wave", action="store_true",
                      help="手部波浪运动（hand_periodic_motion）")

    # ── 夹爪操作 ──
    grip = parser.add_argument_group("夹爪操作（大寰 PGC / 电缸）")
    grip.add_argument("--grip", default=None, choices=["left", "right", "both"], metavar="SIDE",
                      help="指定夹爪侧：left / right / both")
    grip.add_argument("--grip-print", action="store_true",
                      help="打印夹爪当前状态")
    grip.add_argument("--grip-move", type=float, default=None, metavar="POS",
                      help="夹爪目标位置 [0, 0.05] m")
    grip.add_argument("--grip-force", type=float, default=41.0, metavar="N",
                      help="夹爪目标力 [41, 100] N，默认 41")
    grip.add_argument("--grip-vel", type=float, default=0.005, metavar="M/S",
                      help="夹爪目标速度 [0, 0.01] m/s，默认 0.005")
    grip.add_argument("--grip-acc", type=float, default=0.0, metavar="M/S2",
                      help="夹爪加速度 [0, 3] m/s²，默认 0")
    grip.add_argument("--grip-mode", type=int, default=0, metavar="M",
                      help="夹爪模式：0=位/力/速控制，10=推压")
    grip.add_argument("--grip-repeat", type=float, default=0.5, metavar="SEC",
                      help="命令连续发布时长（秒），默认 0.5；0=只发一次")
    grip.add_argument("--grip-home", action="store_true",
                      help="夹爪回零（homing=1）")
    grip.add_argument("--grip-stop", action="store_true",
                      help="夹爪停止（stop=1）")
    grip.add_argument("--grip-monitor", action="store_true",
                      help="持续监控夹爪状态")

    # ── 头部/手部测试参数 ──
    test = parser.add_argument_group("测试参数（配合 --head-test / --hand-test）")
    test.add_argument("--yaw-only", action="store_true",
                      help="头部测试仅运动 yaw")
    test.add_argument("--pitch-only", action="store_true",
                      help="头部测试仅运动 pitch")
    test.add_argument("--head-amplitude", type=float, default=HEAD_TEST_AMPLITUDE, metavar="RAD",
                      help=f"头部测试振幅，默认 {HEAD_TEST_AMPLITUDE}")
    test.add_argument("--head-period", type=float, default=HEAD_TEST_PERIOD, metavar="SEC",
                      help=f"头部测试周期，默认 {HEAD_TEST_PERIOD:.3f}")
    test.add_argument("--head-cycles", type=int, default=HEAD_TEST_DEFAULT_CYCLES, metavar="N",
                      help=f"头部测试循环次数，默认 {HEAD_TEST_DEFAULT_CYCLES}")
    test.add_argument("--left-only", action="store_true",
                      help="手部测试仅运动左手")
    test.add_argument("--right-only", action="store_true",
                      help="手部测试仅运动右手")
    test.add_argument("--hand-amplitude", type=float, default=V4_HAND_TEST_AMPLITUDE, metavar="RAD",
                      help=f"手部测试振幅，默认 {V4_HAND_TEST_AMPLITUDE}")
    test.add_argument("--hand-period", type=float, default=V4_HAND_TEST_PERIOD, metavar="SEC",
                      help=f"手部测试周期，默认 {V4_HAND_TEST_PERIOD:.3f}")
    test.add_argument("--hand-cycles", type=int, default=V4_HAND_TEST_DEFAULT_CYCLES, metavar="N",
                      help=f"手部测试循环次数，默认 {V4_HAND_TEST_DEFAULT_CYCLES}")
    test.add_argument("--hand-phase-diff", type=float, default=V4_HAND_TEST_PHASE_DIFF, metavar="RAD",
                      help=f"手部相邻关节相位差，默认 {V4_HAND_TEST_PHASE_DIFF}")

    # ── 通用参数 ──
    common = parser.add_argument_group("通用参数")
    common.add_argument("--speed", type=float, default=0.5, metavar="RAD/S",
                        help="运动速度 (rad/s)，默认 0.5")
    common.add_argument("--monitor-hz", type=float, default=10.0, metavar="HZ",
                        help="监控刷新频率（Hz），默认 10")
    common.add_argument("--monitor-time", type=float, default=None, metavar="SEC",
                        help="监控时长（秒），默认持续到 Ctrl+C")

    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    lock_joints = None if cli_args.no_lock else DEFAULT_LOCK_JOINTS

    # Resolve control mode: --mode takes priority, --pvt is backward compat
    control_mode = cli_args.mode
    if cli_args.pvt:
        control_mode = "pvt"

    if control_mode in ("pvt",):
        print("=" * 64)
        kp_str = cli_args.pvt_kp if cli_args.pvt_kp is not None else _PVT_DEFAULT_KP
        kd_str = cli_args.pvt_kd if cli_args.pvt_kd is not None else _PVT_DEFAULT_KD
        print(f"PVT (mode=7) enabled: Kp={kp_str}, Kd={kd_str}")
        print("=" * 64)

    # PVT mode: head needs softer gains (low inertia -> oscillation with Kp=200)
    pvt_kp = cli_args.pvt_kp
    pvt_kd = cli_args.pvt_kd
    if control_mode == "pvt":
        if pvt_kp is None:
            pvt_kp = {"head_pitch_joint": 50, "head_yaw_joint": 50}
        if pvt_kd is None:
            pvt_kd = {"head_pitch_joint": 20, "head_yaw_joint": 20}

    controller = RobotController(
        lock_joints=lock_joints,
        enable_safety_check=not cli_args.no_safety,
        enable_limit_check=not cli_args.no_limits,
        control_hz=cli_args.hz,
        control_mode=control_mode,
        velocity_timeout=cli_args.velocity_timeout,
        use_pvt=(control_mode == "pvt"),
        pvt_kp=pvt_kp,
        pvt_kd=pvt_kd,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=False)
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
        elif cli_args.init:
            cmd_print_state(controller)
            if cli_args.direct_init:
                staged, mode, default_dur = False, "直达", 3.0
            elif cli_args.legacy_init:
                staged, mode, default_dur = "legacy", "旧4段(抬臂过肩)", 20.0
            else:
                staged, mode, default_dur = True, "分3步(对齐仿真)", 13.0
            duration = cli_args.init_duration
            print(f"\n=== 预备姿态（{mode}，{duration or default_dur:.0f}s）===")
            input("按回车开始（Ctrl+C 取消）...")
            if controller.move_to_ready_pose(
                duration_sec=duration, staged=staged,
            ):
                # 最终收敛检查（对齐仿真 CLI：READY_POSE 全关节 0.08 rad）
                if staged:
                    arrived, misses = controller.wait_until_position(
                        READY_POSE,
                        timeout=cli_args.init_settle_timeout,
                        tolerance=cli_args.init_tolerance,
                    )
                    if arrived:
                        print(f"✓ 预备姿态已到位（误差 ≤ {cli_args.init_tolerance:.3f} rad）")
                    else:
                        print(
                            f"⚠️ 预备姿态未完全到位（超时 {cli_args.init_settle_timeout:.1f}s，"
                            f"阈值 {cli_args.init_tolerance:.3f} rad）："
                            + ", ".join(
                                f"{m[0]} err={m[3]:.3f}" if m[3] is not None else f"{m[0]} 无状态"
                                for m in misses[:5]
                            )
                        )
                print("Done - ready pose reached")
                cmd_print_state(controller)
            else:
                print("Failed - ready pose")
        elif cli_args.home:
            cmd_print_state(controller)
            if cli_args.legacy_home:
                staged = "legacy"
                dur = cli_args.home_duration or 15.0
                print(f"\n=== 回零（旧4段，{dur:.0f}s）===")
                print("  1/4 head")
                print("  2/4 wrists + elbow_yaw")
                print("  3/4 elbow_roll")
                print("  4/4 shoulders")
            else:
                staged = True
                dur = cli_args.home_duration or 15.0
                print(f"\n=== 回零（分3步对齐仿真，{dur:.0f}s）===")
                print("  1/3 收肩俯仰")
                print("  2/3 展开肘部")
                print("  3/3 全部归零（双臂 + 头部 + 腰部）")
            input("按回车开始（Ctrl+C 取消）...")
            if controller.move_to_home(duration_sec=dur, staged=staged):
                # 最终收敛检查：HOME_POSE 全关节
                arrived, misses = controller.wait_until_position(
                    HOME_POSE,
                    timeout=cli_args.init_settle_timeout,
                    tolerance=cli_args.init_tolerance,
                )
                if arrived:
                    print(f"✓ 已回到零位（误差 ≤ {cli_args.init_tolerance:.3f} rad）")
                else:
                    print(
                        f"⚠️ 未完全回零（超时 {cli_args.init_settle_timeout:.1f}s，"
                        f"阈值 {cli_args.init_tolerance:.3f} rad）："
                        + ", ".join(
                            f"{m[0]} err={m[3]:.3f}" if m[3] is not None else f"{m[0]} 无状态"
                            for m in misses[:5]
                        )
                    )
                print("Done - all joints at zero")
                cmd_print_state(controller)
            else:
                print("Failed - home")
        elif cli_args.demo:
            cmd_print_state(controller)
            mode_label = {"velocity": "速度模式", "pvt": "PVT 模式"}.get(
                control_mode, "位置模式")
            print(f"\n{mode_label}演示：右臂 elbow_yaw +-0.5 rad")
            input("\n按回车开始（Ctrl+C 取消）...")
            if control_mode == "velocity":
                cmd_velocity_demo(controller)
            elif control_mode == "pvt":
                cmd_pvt_demo(controller)
            else:
                cmd_demo(controller)
        elif cli_args.head_test:
            cmd_print_state(controller)
            cmd_head_test(
                controller,
                amplitude=cli_args.head_amplitude,
                period_sec=cli_args.head_period,
                cycles=cli_args.head_cycles,
                yaw_only=cli_args.yaw_only,
                pitch_only=cli_args.pitch_only,
            )
        elif cli_args.hand_test:
            # 手部测试不读取关节状态（手部状态不在 RobotState 中），跳过 print_state
            cmd_hand_test(
                controller,
                amplitude=cli_args.hand_amplitude,
                period_sec=cli_args.hand_period,
                cycles=cli_args.hand_cycles,
                phase_diff=cli_args.hand_phase_diff,
                left_only=cli_args.left_only,
                right_only=cli_args.right_only,
            )
        elif cli_args.interactive:
            cmd_print_state(controller)
            print("\n节点运行中，按 Ctrl+C 退出。")
            spin_thread.join()

        # ── 单关节 / 手部 / 夹爪操作（移植自 joint_test.py）─────────────
        elif _has_joint_cli_action(cli_args):
            spin_thread.join(0)
            _run_joint_cli_actions(controller, cli_args)

        else:
            cmd_print_state(controller)
            print("\n用法: --print-state | --init | --home | --demo | --head-test | --hand-test | --interactive\n"
                  "      --joints J1 J2 --print | --move J=0.5 | --shift J=+0.1 | --monitor\n"
                  "      --hand left --print | --hand-move thumb=0.5 | --hand-open | --hand-wave\n"
                  "      --grip both --grip-print | --grip-move 0.02 | --grip-home | --grip-monitor")

    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted, shutting down")

    finally:
        controller.stop()
        time.sleep(0.1)
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=3.0)
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
