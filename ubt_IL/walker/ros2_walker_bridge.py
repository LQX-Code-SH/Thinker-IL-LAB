#!/usr/bin/env python3
"""ROS2 Deploy Bridge for LeRobot + Walker S2 robot.

Bridges between LeRobot (Python 3.12, ZMQ) and Walker S2 hardware via ROS2 DDS.
Supports both 7-DOF V4 hands and 1-DOF PGC grippers from normalized config JSON.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import signal
import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np
import zmq

# RobotController 集成：bridge 不做插值，所有 500Hz 控制由 RobotController 的 ROS2 Timer 驱动
import sys as _sys, os as _os
_robot_control_path = _os.path.join(
    _os.path.dirname(_os.path.abspath(__file__)), "walker_sdk_ros2"
)
if _robot_control_path not in _sys.path:
    _sys.path.insert(0, _robot_control_path)

logger = logging.getLogger("ros2_walker_bridge")


_V4_HAND_JOINT_LIMITS = {
    "thumb_swing":  (0.0, 2.11),
    "thumb_mcp":    (0.0, 1.85),
    "thumb_pip":    (0.0, 1.09),
    "index_mcp":    (0.0, 1.71),
    "middle_mcp":   (0.0, 1.71),
    "ring_mcp":     (0.0, 1.71),
    "little_mcp":   (0.0, 1.71),
}


def _clamp(value: float, limits: tuple[float, float] | list[float]) -> float:
    lo, hi = limits
    return max(float(lo), min(float(hi), float(value)))


def v4_clip_position(position: list, joint_names: list) -> list:
    """V4 hand clip: clamp each joint to its limit."""
    result = []
    for pos, name in zip(position, joint_names):
        short = name.removeprefix("left_").removeprefix("right_")
        if short in _V4_HAND_JOINT_LIMITS:
            pos = _clamp(pos, _V4_HAND_JOINT_LIMITS[short])
        result.append(pos)
    return result


_DEFAULT_CFG = {
    "robot_model": "walker_s2_v4_hand_31d",
    "zmq_cmd_port": 5561,
    "zmq_status_port": 5562,
    "zmq_image_port": 5563,
    "camera_topics": {},
    "ros_namespace": "",
    "cmd_namespace": "",
    "body_groups": {
        "left_arm": [
            "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_shoulder_pitch_joint",
            "L_shoulder_roll_joint", "L_shoulder_yaw_joint", "L_wrist_pitch_joint",
            "L_wrist_roll_joint",
        ],
        "right_arm": [
            "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_shoulder_pitch_joint",
            "R_shoulder_roll_joint", "R_shoulder_yaw_joint", "R_wrist_pitch_joint",
            "R_wrist_roll_joint",
        ],
        "head": ["head_pitch_joint", "head_yaw_joint"],
        "waist": ["waist_yaw_joint"],
    },
    "body_joint_names": [
        "L_elbow_roll_joint", "L_elbow_yaw_joint", "L_shoulder_pitch_joint",
        "L_shoulder_roll_joint", "L_shoulder_yaw_joint", "L_wrist_pitch_joint",
        "L_wrist_roll_joint",
        "R_elbow_roll_joint", "R_elbow_yaw_joint", "R_shoulder_pitch_joint",
        "R_shoulder_roll_joint", "R_shoulder_yaw_joint", "R_wrist_pitch_joint",
        "R_wrist_roll_joint",
        "head_pitch_joint", "head_yaw_joint", "waist_yaw_joint",
    ],
    "left_hand_joint_names": [
        "left_thumb_swing", "left_thumb_mcp", "left_thumb_pip",
        "left_index_mcp", "left_middle_mcp", "left_ring_mcp", "left_little_mcp",
    ],
    "right_hand_joint_names": [
        "right_thumb_swing", "right_thumb_mcp", "right_thumb_pip",
        "right_index_mcp", "right_middle_mcp", "right_ring_mcp", "right_little_mcp",
    ],
    "body_joint_limits": {},
    "hand_joint_limits": _V4_HAND_JOINT_LIMITS,
    "hand_type": "v4",
    "end_effector_type": "v4_hand_7dof",
    "hand_open_position": [0.0] * 7,
    "left_hand_open_position": [0.0] * 7,
    "right_hand_open_position": [0.0] * 7,
    "gripper_position_limits": [0.0, 0.05],
    "gripper_force_limits": [41.0, 100.0],
    "gripper_velocity_limits": [0.0, 0.01],
    "gripper_acceleration_limits": [0.0, 3.0],
    "gripper_force": 41.0,
    "gripper_velocity": 0.005,
    "gripper_acceleration": 0.0,
    "gripper_mode": 0,
    "control_fps": 15.0,
    "max_safe_velocity": 1.0,
    # ---- body 控制模式（集成 RobotController）----
    "body_control_mode": "pvt",       # "velocity" | "pvt" | "position"
    "body_velocity_timeout": 0.3,    # 速度模式超时（秒）
    "body_pvt_kp": None,             # PVT Kp 覆盖（None=默认：手臂200,头部50）
    "body_pvt_kd": None,             # PVT Kd 覆盖（None=默认：手臂50,头部20）
    "lock_joints": ["head_pitch_joint", "head_yaw_joint", "waist_yaw_joint"],
    "home_position": [
        -1.56, 2.88, 0.0, -0.15, -1.56, 0.0, 0.0,
        -1.56, -2.88, 0.0, -0.15, 1.56, 0.0, 0.0,
        -0.65, 0.0, 0.0,
    ],
    "topic_body_cmd": "/mc/sdk/robot_command",
    "topic_left_hand_cmd": "/mc/left_hand/command",
    "topic_right_hand_cmd": "/mc/right_hand/command",
    "topic_body_state": "/mc/sdk/robot_state",
    "topic_left_hand_state": "/mc/left_hand/joint_states",
    "topic_right_hand_state": "/mc/right_hand/joint_states",
}


class ZMQInternalBridge:
    """ZMQ sockets for communication with LeRobot process."""

    def __init__(self, cmd_port: int, status_port: int, image_port: int):
        self.context = zmq.Context()

        self.cmd_socket = self.context.socket(zmq.SUB)
        self.cmd_socket.bind(f"tcp://*:{cmd_port}")
        self.cmd_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.cmd_socket.setsockopt(zmq.RCVHWM, 1)

        self.status_socket = self.context.socket(zmq.PUB)
        self.status_socket.bind(f"tcp://*:{status_port}")
        self.status_socket.setsockopt(zmq.SNDHWM, 1)

        self.image_socket = self.context.socket(zmq.PUB)
        self.image_socket.bind(f"tcp://*:{image_port}")
        self.image_socket.setsockopt(zmq.SNDHWM, 1)

        logger.info("ZMQ internal bridge: cmd=%d, status=%d, image=%d", cmd_port, status_port, image_port)

    def recv_action(self, timeout_ms: int = 100) -> dict | None:
        try:
            return self.cmd_socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None

    def send_status(self, status: dict) -> None:
        try:
            self.status_socket.send_json(status, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("Status send dropped: ZMQ send buffer full (SNDHWM=1)")

    def send_image(self, image_data: dict) -> None:
        try:
            self.image_socket.send_string(json.dumps(image_data), flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("Image send dropped: ZMQ send buffer full")

    def close(self) -> None:
        self.cmd_socket.close()
        self.status_socket.close()
        self.image_socket.close()
        self.context.term()


class WalkerRealRobotBridge:
    """ROS2 DDS ↔ Walker S2 hardware."""

    def __init__(self, zmq_bridge: ZMQInternalBridge, cfg: dict):
        self.zmq_bridge = zmq_bridge
        self._cfg = cfg

        self._body_groups = cfg.get("body_groups") or self._legacy_body_groups(cfg["body_joint_names"])
        self._body_joint_names = [name for group in ("left_arm", "right_arm", "head", "waist") for name in self._body_groups[group]]
        self._left_hand_joint_names = cfg["left_hand_joint_names"]
        self._right_hand_joint_names = cfg["right_hand_joint_names"]
        self._body_joint_limits = cfg.get("body_joint_limits", {})
        self._hand_type = cfg.get("hand_type", "v4")
        self._end_effector_type = cfg.get("end_effector_type", "v4_hand_7dof")
        self._lock_joints = set(cfg.get("lock_joints", []))
        self._n_body = len(self._body_joint_names)
        self._n_left_hand = len(self._left_hand_joint_names)
        self._n_right_hand = len(self._right_hand_joint_names)
        self._gripper_position_limits = cfg.get("gripper_position_limits", [0.0, 0.05])
        self._gripper_force_limits = cfg.get("gripper_force_limits", [41.0, 100.0])
        self._gripper_velocity_limits = cfg.get("gripper_velocity_limits", [0.0, 0.01])
        self._gripper_acceleration_limits = cfg.get("gripper_acceleration_limits", [0.0, 3.0])
        self._gripper_force = float(cfg.get("gripper_force", 41.0))
        self._gripper_velocity = float(cfg.get("gripper_velocity", 0.005))
        self._gripper_acceleration = float(cfg.get("gripper_acceleration", 0.0))
        self._gripper_mode = int(cfg.get("gripper_mode", 0))
        self._control_fps = float(cfg.get("control_fps", 15.0))  # 保留用于日志，不再用于插值
        self._max_safe_velocity = float(cfg.get("max_safe_velocity", 1.0))
        self._body_control_mode = cfg.get("body_control_mode", "position")
        self._body_velocity_timeout = float(cfg.get("body_velocity_timeout", 0.3))

        ros_namespace = cfg.get("ros_namespace", "").rstrip("/")
        cmd_namespace = cfg.get("cmd_namespace", "").rstrip("/") if cfg.get("cmd_namespace") else ""

        import rclpy
        from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        try:
            from mc_state_msgs.msg import RobotState
            from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
            self._mc_msgs_available = True
        except ImportError:
            RobotState = JointState
            RobotCommand = JointState
            JointCmd = None
            JointCommand = JointState
            self._mc_msgs_available = False
            logger.warning("mc_task_msgs not available, using JointState fallback")

        self._GripCmd = None
        self._GripStatus = None
        if self._end_effector_type == "pgc_gripper_1dof":
            try:
                from ecat_task_msgs.msg import GripCmd, GripStatus
            except ImportError as exc:
                raise RuntimeError("pgc_gripper_1dof requires ecat_task_msgs/GripCmd and GripStatus") from exc
            self._GripCmd = GripCmd
            self._GripStatus = GripStatus

        self._RobotState = RobotState
        self._RobotCommand = RobotCommand
        self._JointCmd = JointCmd
        self._JointCommand = JointCommand
        self._JointState = JointState

        if not rclpy.ok():
            rclpy.init()

        # ================================================================
        # RobotController：处理所有 body 控制（state sub、500Hz Timer、cmd pub）
        # ================================================================
        from robot_control import RobotController

        self._robot_ctrl = RobotController(
            node_name="walker_body_controller",
            control_hz=500,
            control_mode=self._body_control_mode,
            velocity_timeout=self._body_velocity_timeout,
            lock_joints=list(self._lock_joints),
            pvt_kp=cfg.get("body_pvt_kp"),
            pvt_kd=cfg.get("body_pvt_kd"),
            enable_safety_check=True,
            enable_hand_control=False,   # 手/夹爪由 Bridge Node 管理
        )
        self._body_executor = SingleThreadedExecutor()
        self._body_executor.add_node(self._robot_ctrl)
        self._body_thread = threading.Thread(
            target=self._body_executor.spin, daemon=True, name="body_executor"
        )
        self._body_thread.start()

        if not self._robot_ctrl.wait_for_state(timeout=10.0):
            raise RuntimeError("RobotController: timeout waiting for RobotState")

        # ================================================================
        # Bridge Node：CameraRelay + 手/夹爪
        # ================================================================
        self._node = Node("ros2_walker_bridge")

        # hand/gripper 位置缓存（Bridge 自己维护，RobotController 不管手/夹爪）
        self._left_hand_pos = [0.0] * self._n_left_hand
        self._right_hand_pos = [0.0] * self._n_right_hand
        self._hand_state_lock = threading.Lock()

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 手/夹爪 publisher（保留在 Bridge Node）
        self._left_hand_pub = None
        self._right_hand_pub = None
        self._left_grip_pub = None
        self._right_grip_pub = None
        if self._end_effector_type == "pgc_gripper_1dof":
            left_topic = f"{cmd_namespace}{cfg['topic_left_hand_cmd']}" if cmd_namespace else cfg["topic_left_hand_cmd"]
            right_topic = f"{cmd_namespace}{cfg['topic_right_hand_cmd']}" if cmd_namespace else cfg["topic_right_hand_cmd"]
            self._left_grip_pub = self._node.create_publisher(self._GripCmd, left_topic, qos_cmd)
            self._right_grip_pub = self._node.create_publisher(self._GripCmd, right_topic, qos_cmd)
        else:
            left_topic = f"{cmd_namespace}{cfg['topic_left_hand_cmd']}" if cmd_namespace else cfg["topic_left_hand_cmd"]
            right_topic = f"{cmd_namespace}{cfg['topic_right_hand_cmd']}" if cmd_namespace else cfg["topic_right_hand_cmd"]
            self._left_hand_pub = self._node.create_publisher(JointCommand, left_topic, qos_cmd)
            self._right_hand_pub = self._node.create_publisher(JointCommand, right_topic, qos_cmd)

        # 手/夹爪 state subscriber（保留在 Bridge Node）
        if self._end_effector_type == "pgc_gripper_1dof":
            self._node.create_subscription(
                self._GripStatus,
                f"{ros_namespace}{cfg['topic_left_hand_state']}",
                lambda msg: self._gripper_callback("left", msg),
                qos_sensor,
            )
            self._node.create_subscription(
                self._GripStatus,
                f"{ros_namespace}{cfg['topic_right_hand_state']}",
                lambda msg: self._gripper_callback("right", msg),
                qos_sensor,
            )
        else:
            self._node.create_subscription(
                JointState, f"{ros_namespace}{cfg['topic_left_hand_state']}", self._left_hand_callback, 10
            )
            self._node.create_subscription(
                JointState, f"{ros_namespace}{cfg['topic_right_hand_state']}", self._right_hand_callback, 10
            )

        self._bridge_executor = MultiThreadedExecutor(num_threads=2)
        self._bridge_executor.add_node(self._node)
        self._bridge_thread = threading.Thread(
            target=self._bridge_executor.spin, daemon=True, name="bridge_executor"
        )
        self._bridge_thread.start()

        # ================================================================
        # ZMQ action 转发（不做插值，直接调 robot_ctrl API）
        # ================================================================
        self._last_action_time = 0.0
        self._running = True
        self._action_thread = threading.Thread(
            target=self._action_loop, daemon=True, name="action_forward"
        )
        self._action_thread.start()

        logger.info(
            "Walker bridge started model=%s end_effector=%s ns=%s cmd_ns=%s lock=%s body_joints=%d "
            "control_mode=%s",
            cfg.get("robot_model", "?"), self._end_effector_type, ros_namespace, cmd_namespace,
            sorted(self._lock_joints), self._n_body, self._body_control_mode,
        )

    @staticmethod
    def _legacy_body_groups(body_joint_names: list[str]) -> dict[str, list[str]]:
        return {
            "left_arm": list(body_joint_names[:7]),
            "right_arm": list(body_joint_names[7:14]),
            "head": list(body_joint_names[14:16]),
            "waist": list(body_joint_names[16:17]),
        }

    # ---- hand/gripper state callbacks（保留在 Bridge Node，不变）----
    def _left_hand_callback(self, msg: Any) -> None:
        self._joint_state_hand_callback("left", msg)

    def _right_hand_callback(self, msg: Any) -> None:
        self._joint_state_hand_callback("right", msg)

    def _joint_state_hand_callback(self, side: str, msg: Any) -> None:
        joint_names = self._left_hand_joint_names if side == "left" else self._right_hand_joint_names
        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        positions = [0.0] * len(joint_names)
        for i, jname in enumerate(joint_names):
            if jname in name_to_idx:
                positions[i] = msg.position[name_to_idx[jname]]
        with self._hand_state_lock:
            if side == "left":
                self._left_hand_pos[:] = positions
            else:
                self._right_hand_pos[:] = positions
        self._publish_status()

    def _gripper_callback(self, side: str, msg: Any) -> None:
        pos = float(getattr(msg, "pos", 0.0))
        with self._hand_state_lock:
            if side == "left":
                self._left_hand_pos[:] = [pos]
            else:
                self._right_hand_pos[:] = [pos]
        self._publish_status()

    # ---- ZMQ status（body 从 RobotController 读，hand/gripper 从 Bridge 缓存读）----
    def _publish_status(self) -> None:
        body_pos = self._robot_ctrl.get_current_position()
        if body_pos is None:
            return

        left_arm = [float(body_pos[self._robot_ctrl.joint_index(n)]) for n in self._body_groups["left_arm"]]
        right_arm = [float(body_pos[self._robot_ctrl.joint_index(n)]) for n in self._body_groups["right_arm"]]
        head = [float(body_pos[self._robot_ctrl.joint_index(n)]) for n in self._body_groups["head"]]
        waist = [float(body_pos[self._robot_ctrl.joint_index(n)]) for n in self._body_groups["waist"]]

        with self._hand_state_lock:
            left_hand = list(self._left_hand_pos)
            right_hand = list(self._right_hand_pos)

        status = {
            "left_arm": left_arm,
            "right_arm": right_arm,
            "head": head,
            "waist": waist,
            "left_hand": left_hand,
            "right_hand": right_hand,
            "ts": time.time(),
        }
        self.zmq_bridge.send_status(status)

    # ---- ZMQ action 转发（不做插值，直接调 RobotController API）----
    def _action_loop(self) -> None:
        while self._running:
            action = self.zmq_bridge.recv_action(timeout_ms=50)
            if action is not None:
                # body：转发给 RobotController（500Hz 插值/平滑由 controller 负责）
                self._update_body_target(action)
                # 末端执行器（手/夹爪）走独立通路
                self._publish_end_effector_command("left", action.get("left_hand", []))
                self._publish_end_effector_command("right", action.get("right_hand", []))

    def _body_action_to_dict(self, action: dict) -> dict[str, float] | None:
        """从 ZMQ action dict 提取 body 目标，返回 {joint_name: target_angle}。

        action 格式由 LeRobot 插件定义，分组顺序由 _body_groups 配置决定。
        返回 None 表示 action 中没有 body 关节数据。
        """
        result = {}
        for group in ("left_arm", "right_arm", "head", "waist"):
            values = action.get(group, [])
            joint_names = self._body_groups[group]
            for jname, val in zip(joint_names, values):
                val = float(val)
                if jname in self._body_joint_limits:
                    val = _clamp(val, self._body_joint_limits[jname])
                result[jname] = val
        return result if result else None

    def _update_body_target(self, action: dict) -> None:
        """收到新 action：转换为 {name: angle} dict，调用 RobotController API。

        dt 使用实际收包间隔（来多快发多快），RobotController 负责所有插值和时序。
        """
        goal = self._body_action_to_dict(action)
        if not goal:
            return

        now = time.time()
        dt = now - self._last_action_time if self._last_action_time > 0 else 1.0 / 15.0
        self._last_action_time = now

        mode = self._body_control_mode

        if mode == "velocity":
            current_pos = self._robot_ctrl.get_current_position()
            if current_pos is None:
                return
            vel_dict = {}
            for name, target_angle in goal.items():
                if name in self._lock_joints:
                    continue
                idx = self._robot_ctrl.joint_index(name)
                vel = (target_angle - float(current_pos[idx])) / dt
                vel = max(-self._max_safe_velocity, min(self._max_safe_velocity, vel))
                vel_dict[name] = vel
            if vel_dict:
                self._robot_ctrl.set_joint_velocities(vel_dict)

        elif mode == "pvt":
            current_pos = self._robot_ctrl.get_current_position()
            if current_pos is None:
                return
            targets = {}
            for name, target_angle in goal.items():
                if name in self._lock_joints:
                    continue
                idx = self._robot_ctrl.joint_index(name)
                dist = abs(target_angle - float(current_pos[idx]))
                speed = max(0.01, dist / dt)
                targets[name] = (target_angle, speed)
            if targets:
                self._robot_ctrl.move_joints(targets, wait=False)

        else:  # position（向后兼容）
            target_arr = np.zeros(self._robot_ctrl.n_joints, dtype=float)
            for name, angle in goal.items():
                idx = self._robot_ctrl.joint_index(name)
                target_arr[idx] = float(angle)
            current = self._robot_ctrl.get_current_position()
            if current is not None:
                for i in range(len(target_arr)):
                    if target_arr[i] == 0.0 and self._robot_ctrl.all_joints[i] not in goal:
                        target_arr[i] = current[i]
            self._robot_ctrl.move_to_position(
                target_arr, duration_sec=dt, wait=False
            )

    # ---- end effector（保留在 Bridge Node，不变）----

    def _publish_end_effector_command(self, side: str, position: list) -> None:
        if not position:
            return
        if self._end_effector_type == "pgc_gripper_1dof":
            self._publish_gripper_command(side, position)
        else:
            self._publish_hand_command(side, position)

    def _publish_hand_command(self, hand_side: str, position: list) -> None:
        """Publish JointCommand for V4 hand joints."""
        joint_names = self._left_hand_joint_names if hand_side == "left" else self._right_hand_joint_names
        position = v4_clip_position(position, joint_names)

        if self._mc_msgs_available:
            msg = self._JointCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = ""
            msg.names = list(joint_names)
            msg.position = [float(p) for p in position]
            msg.mode = [5] * len(joint_names)
        else:
            msg = self._JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = [str(i) for i in range(1, len(position) + 1)]
            msg.position = [float(p) for p in position]

        if hand_side == "left":
            self._left_hand_pub.publish(msg)
        else:
            self._right_hand_pub.publish(msg)

    def _publish_gripper_command(self, side: str, position: list) -> None:
        pos = _clamp(float(position[0]), self._gripper_position_limits)
        force = _clamp(self._gripper_force, self._gripper_force_limits)
        vel = _clamp(self._gripper_velocity, self._gripper_velocity_limits)
        acc = _clamp(self._gripper_acceleration, self._gripper_acceleration_limits)

        msg = self._GripCmd()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.init = 1
        msg.mode = self._gripper_mode
        msg.stop = 0
        msg.reset = 0
        msg.homing = 0
        msg.pos = pos
        msg.vel = vel
        msg.force = force
        msg.cur = acc

        if side == "left":
            self._left_grip_pub.publish(msg)
        else:
            self._right_grip_pub.publish(msg)

    def stop(self) -> None:
        self._running = False
        if self._action_thread is not None and self._action_thread.is_alive():
            self._action_thread.join(timeout=2.0)

        # 先停 RobotController（速度模式：零速+刹车；PVT/位置：停止轨迹）
        if self._robot_ctrl is not None:
            try:
                self._robot_ctrl.stop()
            except Exception:
                pass

        # 停 executor（先控制，后通信）
        if self._body_executor is not None:
            self._body_executor.shutdown(timeout_sec=2.0)
        if self._body_thread is not None and self._body_thread.is_alive():
            self._body_thread.join(timeout=3.0)

        if self._bridge_executor is not None:
            self._bridge_executor.shutdown(timeout_sec=2.0)
        if self._bridge_thread is not None and self._bridge_thread.is_alive():
            self._bridge_thread.join(timeout=3.0)

        if self._node is not None:
            self._node.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()


class CameraRelay:
    """Relays Walker camera images from ROS2 shm_msgs to ZMQ."""

    def __init__(self, zmq_bridge: ZMQInternalBridge, node, camera_topics: dict[str, str]):
        self._zmq_bridge = zmq_bridge
        self._node = node
        self._camera_topics = {}
        for cam_name, value in camera_topics.items():
            if isinstance(value, dict):
                self._camera_topics[cam_name] = {
                    "topic": value.get("topic"),
                    "msg_type": value.get("msg_type", "shm_msgs/Image2m"),
                }
            else:
                self._camera_topics[cam_name] = {"topic": value, "msg_type": "shm_msgs/Image2m"}
        self._running = True
        self._latest_images: dict[str, tuple] = {}
        self._image_lock = threading.Lock()

        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Dynamically resolve shm_msgs/Image* or sensor_msgs/Image type strings.
        def _resolve_msg_type(msg_type_name: str):
            if msg_type_name == "sensor_msgs/Image":
                from sensor_msgs.msg import Image
                return Image
            pkg, sep, msg_name = msg_type_name.partition("/")
            if not sep:
                return None
            try:
                import importlib
                msg_module = importlib.import_module(f"{pkg}.msg")
                return getattr(msg_module, msg_name)
            except (ImportError, AttributeError):
                return None

        for cam_name, cam_cfg in self._camera_topics.items():
            topic = cam_cfg.get("topic")
            msg_type_name = cam_cfg.get("msg_type", "shm_msgs/Image2m")
            msg_type = _resolve_msg_type(msg_type_name)
            if msg_type is None:
                logger.warning("Camera relay: unsupported/unavailable msg_type %s for %s", msg_type_name, cam_name)
                continue
            self._node.create_subscription(
                msg_type, topic,
                lambda msg, name=cam_name: self._camera_callback(name, msg),
                qos_sensor,
            )
            logger.info("Camera relay: subscribed %s (%s) → %s", cam_name, msg_type_name, topic)

        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True, name="camera_relay")
        self._pub_thread.start()

    def _camera_callback(self, cam_name: str, msg) -> None:
        try:
            height = msg.height
            width = msg.width
            step = msg.step
            encoding = self._resolve_encoding(msg)
            img_data = bytes(msg.data)

            byte_count = height * step
            if encoding == "bgr8":
                img = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, width, 3))
            elif encoding == "rgb8":
                img = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif encoding == "yuv422":
                img = self._yuv422_to_bgr(img_data[:byte_count], width, height)
            elif encoding == "mono8":
                img = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, width))
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                logger.debug("Camera relay: unsupported encoding %s for %s", encoding, cam_name)
                return

            success, jpeg_buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not success:
                return

            with self._image_lock:
                self._latest_images[cam_name] = (jpeg_buf.tobytes(), time.time())

        except Exception as e:
            logger.warning("Camera relay: callback error for %s: %s", cam_name, e)

    def _publish_loop(self) -> None:
        while self._running:
            try:
                with self._image_lock:
                    if not self._latest_images:
                        time.sleep(0.01)
                        continue
                    images_b64 = {
                        cam_name: base64.b64encode(jpeg_bytes).decode('ascii')
                        for cam_name, (jpeg_bytes, _ts) in self._latest_images.items()
                    }
                self._zmq_bridge.send_image({"images": images_b64, "ts": time.time()})
                time.sleep(0.033)
            except Exception as e:
                logger.warning("Camera relay: publish error: %s", e)
                time.sleep(0.1)

    def stop(self) -> None:
        self._running = False
        if self._pub_thread is not None and self._pub_thread.is_alive():
            self._pub_thread.join(timeout=2.0)

    @staticmethod
    def _resolve_encoding(msg) -> str:
        raw = msg.encoding
        if hasattr(raw, 'data'):
            encoding = ''.join(chr(c) for c in raw.data if c != 0)
        else:
            encoding = str(raw)
        known = ["bgr8", "rgb8", "bgra8", "rgba8", "mono8", "mono16",
                 "yuv422", "yuyv422", "uyvy422", "16UC1", "32FC1"]
        for k in known:
            if encoding.startswith(k):
                return k
        return encoding

    @staticmethod
    def _yuv422_to_bgr(yuv_data, width, height) -> np.ndarray:
        yuv = np.frombuffer(yuv_data, dtype=np.uint8).reshape((height, width // 2, 4))
        u = yuv[:, :, 0]
        y0 = yuv[:, :, 1]
        v = yuv[:, :, 2]
        y1 = yuv[:, :, 3]
        y = np.zeros((height, width), dtype=np.uint8)
        y[:, 0::2] = y0
        y[:, 1::2] = y1
        u_full = np.repeat(u, 2, axis=1)
        v_full = np.repeat(v, 2, axis=1)
        yuv_img = cv2.merge((y, u_full, v_full))
        return cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR)


def _is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def kill_existing_bridge() -> None:
    """Find and kill any already-running ros2_walker_bridge processes.

    Matches only processes whose argv[1] is ros2_walker_bridge.py, NOT the
    lerobot-rollout parent process (which carries the bridge path as the value
    of --robot.bridge_script=... in its cmdline). pgrep -f matches the whole
    cmdline and would kill the parent — use /proc scanning instead.
    """
    current_pid = os.getpid()
    parent_pid = os.getppid()

    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in (current_pid, parent_pid):
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                parts = f.read().split(b"\x00")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if len(parts) < 2:
            continue
        if parts[1].decode("utf-8", "replace").endswith("ros2_walker_bridge.py"):
            pids.append(pid)

    if not pids:
        return

    logger.info("Found existing Walker bridge processes (PIDs: %s), terminating ...", pids)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        alive = [p for p in pids if _is_alive(p)]
        if not alive:
            break
        time.sleep(0.1)
    else:
        for pid in alive:  # noqa: F821
            logger.warning("Force killing Walker bridge process %d", pid)
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(0.1)

    time.sleep(0.5)
    logger.info("Previous Walker bridge instances terminated.")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS2 Deploy Bridge for LeRobot + Walker S2")
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--zmq_cmd_port", type=int, default=None)
    parser.add_argument("--zmq_status_port", type=int, default=None)
    parser.add_argument("--zmq_image_port", type=int, default=None)
    parser.add_argument("--ros_namespace", type=str, default=None)
    parser.add_argument("--cmd_namespace", type=str, default=None)
    return parser.parse_args()


def main():
    args = _parse_args()

    cfg = dict(_DEFAULT_CFG)
    if args.config:
        try:
            cfg.update(json.loads(args.config))
        except json.JSONDecodeError as e:
            logger.error("Failed to parse --config JSON: %s", e)
            return

    if args.zmq_cmd_port is not None:
        cfg["zmq_cmd_port"] = args.zmq_cmd_port
    if args.zmq_status_port is not None:
        cfg["zmq_status_port"] = args.zmq_status_port
    if args.zmq_image_port is not None:
        cfg["zmq_image_port"] = args.zmq_image_port
    if args.ros_namespace is not None:
        cfg["ros_namespace"] = args.ros_namespace
    if args.cmd_namespace is not None:
        cfg["cmd_namespace"] = args.cmd_namespace

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    kill_existing_bridge()

    zmq_bridge = ZMQInternalBridge(cfg["zmq_cmd_port"], cfg["zmq_status_port"], cfg["zmq_image_port"])
    robot_bridge = WalkerRealRobotBridge(zmq_bridge, cfg)

    camera_topics = cfg.get("camera_topics", {})
    camera_relay = None
    if camera_topics:
        logger.info("Camera relay config: %s", camera_topics)
        camera_relay = CameraRelay(zmq_bridge, robot_bridge._node, camera_topics)
        logger.info("Camera relay started for %d cameras", len(camera_topics))
    else:
        logger.info("Camera relay disabled: no camera_topics configured")

    stop_event = threading.Event()

    def signal_handler(sig, frame):
        logger.info("Received signal %s, shutting down...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info(
        "Walker Bridge running model=%s ros_ns=%s cmd_ns=%s. Press Ctrl+C to stop.",
        cfg.get("robot_model", "?"), cfg.get("ros_namespace", ""), cfg.get("cmd_namespace", ""),
    )
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass

    logger.info("Shutting down...")
    if camera_relay is not None:
        camera_relay.stop()
    robot_bridge.stop()
    zmq_bridge.close()
    logger.info("Walker Bridge stopped.")


if __name__ == "__main__":
    main()
