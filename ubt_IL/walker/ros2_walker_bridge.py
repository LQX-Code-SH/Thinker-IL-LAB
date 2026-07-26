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


def c1_clip_position(position: list, joint_names: list, joint_limits: dict) -> list:
    """Clamp C1 logical 6-DoF hand commands using config-provided limits."""
    result = []
    for pos, name in zip(position, joint_names):
        short = name.removeprefix("left_").removeprefix("right_")
        limits = joint_limits.get(name, joint_limits.get(short))
        result.append(_clamp(pos, limits) if limits is not None else float(pos))
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
    "topic_sim_object_state": None,
    "topic_sim_object_pose_cmd": None,
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
        self._hand_joint_limits = cfg.get("hand_joint_limits", {})
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

        ros_namespace = cfg.get("ros_namespace", "").rstrip("/")
        cmd_namespace = cfg.get("cmd_namespace", "").rstrip("/") if cfg.get("cmd_namespace") else ""

        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import Point
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

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
        self._Point = Point

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("ros2_walker_bridge")

        self._body_jpos = [0.0] * self._n_body
        self._left_hand_pos = [0.0] * self._n_left_hand
        self._right_hand_pos = [0.0] * self._n_right_hand
        self._sim_object_state: dict[str, Any] = {}
        self._state_lock = threading.Lock()

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

        body_cmd_topic = f"{cmd_namespace}{cfg['topic_body_cmd']}" if cmd_namespace else cfg["topic_body_cmd"]
        self._body_cmd_pub = self._node.create_publisher(RobotCommand, body_cmd_topic, qos_cmd)

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

        self._node.create_subscription(
            RobotState, f"{ros_namespace}{cfg['topic_body_state']}", self._body_state_callback, qos_sensor
        )
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

        self._sim_object_pose_pub = None
        sim_object_state_topic = cfg.get("topic_sim_object_state")
        sim_object_pose_cmd_topic = cfg.get("topic_sim_object_pose_cmd")
        if sim_object_state_topic:
            self._node.create_subscription(
                String,
                f"{ros_namespace}{sim_object_state_topic}",
                self._sim_object_state_callback,
                qos_sensor,
            )
        if sim_object_pose_cmd_topic:
            self._sim_object_pose_pub = self._node.create_publisher(
                Point,
                f"{cmd_namespace}{sim_object_pose_cmd_topic}"
                if cmd_namespace
                else sim_object_pose_cmd_topic,
                qos_cmd,
            )

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True, name="ros2_executor")
        self._executor_thread.start()

        self._running = True
        # body 插值状态：500Hz 发布 quintic 斜坡（_start_vec → _goal_vec，窗口 _ramp_dur），
        # 平滑 15Hz action 的 0.02 rad 步进；无新 action 时 hold 在 goal。
        self._body_vec_lock = threading.Lock()
        self._body_start_vec = np.zeros(self._n_body, dtype=float)
        self._body_goal_vec = np.zeros(self._n_body, dtype=float)
        self._body_ramp_t0 = 0.0
        self._body_ramp_dur = 1.0 / 15.0  # 默认窗口，后续按实测 action 间隔自适应
        self._body_last_action_t = None
        self._body_has_target = False
        self._body_thread = None
        self._action_thread = threading.Thread(target=self._action_loop, daemon=True, name="action_forward")
        self._action_thread.start()
        self._body_thread = threading.Thread(target=self._body_publish_loop, daemon=True, name="body_forward")
        self._body_thread.start()

        logger.info(
            "Walker bridge started model=%s end_effector=%s ns=%s cmd_ns=%s lock=%s body_joints=%d",
            cfg.get("robot_model", "?"), self._end_effector_type, ros_namespace, cmd_namespace,
            sorted(self._lock_joints), self._n_body,
        )

    @staticmethod
    def _legacy_body_groups(body_joint_names: list[str]) -> dict[str, list[str]]:
        return {
            "left_arm": list(body_joint_names[:7]),
            "right_arm": list(body_joint_names[7:14]),
            "head": list(body_joint_names[14:16]),
            "waist": list(body_joint_names[16:17]),
        }

    def _body_state_callback(self, msg: Any) -> None:
        """Parse RobotState → extract body joint positions by name."""
        if self._mc_msgs_available:
            joint_states = msg.joint_states
            name_to_idx = {name: idx for idx, name in enumerate(joint_states.name)}
        else:
            name_to_idx = {name: idx for idx, name in enumerate(msg.name)}

        positions = [0.0] * self._n_body
        valid = True
        for i, jname in enumerate(self._body_joint_names):
            if jname in name_to_idx:
                if self._mc_msgs_available:
                    positions[i] = joint_states.position[name_to_idx[jname]]
                else:
                    positions[i] = msg.position[name_to_idx[jname]]
            else:
                valid = False
                break

        if valid:
            with self._state_lock:
                self._body_jpos[:] = positions
            self._publish_status()

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
        with self._state_lock:
            if side == "left":
                self._left_hand_pos[:] = positions
            else:
                self._right_hand_pos[:] = positions
        self._publish_status()

    def _gripper_callback(self, side: str, msg: Any) -> None:
        pos = float(getattr(msg, "pos", 0.0))
        with self._state_lock:
            if side == "left":
                self._left_hand_pos[:] = [pos]
            else:
                self._right_hand_pos[:] = [pos]
        self._publish_status()

    def _sim_object_state_callback(self, msg: Any) -> None:
        try:
            state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        compact = {
            key: state[key]
            for key in ("object_pos_w", "sim_step")
            if key in state
        }
        with self._state_lock:
            self._sim_object_state = compact
        self._publish_status()

    def _body_group_values(self, group: str) -> list[float]:
        start = 0
        for name in ("left_arm", "right_arm", "head", "waist"):
            count = len(self._body_groups[name])
            if name == group:
                return list(self._body_jpos[start:start + count])
            start += count
        return []

    def _publish_status(self) -> None:
        with self._state_lock:
            status = {
                "left_arm": self._body_group_values("left_arm"),
                "right_arm": self._body_group_values("right_arm"),
                "head": self._body_group_values("head"),
                "waist": self._body_group_values("waist"),
                "left_hand": list(self._left_hand_pos),
                "right_hand": list(self._right_hand_pos),
                "sim_object_state": dict(self._sim_object_state),
                "ts": time.time(),
            }
        self.zmq_bridge.send_status(status)

    def _action_loop(self) -> None:
        while self._running:
            action = self.zmq_bridge.recv_action(timeout_ms=50)
            if action is not None:
                sim_object_pose = action.get("sim_object_pose")
                if sim_object_pose is not None:
                    self._publish_sim_object_pose(sim_object_pose)
                    continue
                # body 目标交给 500Hz 插值发布线程（quintic 斜坡 + hold）
                self._update_body_target(action)
                # 末端执行器（手/夹爪）走独立通路，保持事件驱动
                self._publish_end_effector_command("left", action.get("left_hand", []))
                self._publish_end_effector_command("right", action.get("right_hand", []))

    def _publish_sim_object_pose(self, pose: Any) -> None:
        if self._sim_object_pose_pub is None:
            logger.warning("Ignoring sim_object_pose: no simulation object command topic configured")
            return
        if not isinstance(pose, (list, tuple)) or len(pose) < 3:
            logger.warning("Ignoring invalid sim_object_pose payload: %r", pose)
            return
        msg = self._Point()
        msg.x, msg.y, msg.z = (float(pose[0]), float(pose[1]), float(pose[2]))
        self._sim_object_pose_pub.publish(msg)

    def _update_body_target(self, action: dict) -> None:
        """收到新 action：从当前插值位置 retarget 到新目标，启动新 quintic 斜坡。"""
        goal = self._body_action_to_vec(action)
        if goal is None:
            return
        now = time.time()
        with self._body_vec_lock:
            if self._body_has_target:
                start = self._body_interp_at(now)  # 从当前插值位置 retarget，避免跳变
                if self._body_last_action_t is not None:
                    interval = now - self._body_last_action_t
                    ramp_dur = max(0.02, min(interval, 0.2))  # 20ms..200ms
                else:
                    ramp_dur = self._body_ramp_dur
            else:
                # 首个 action：从 goal 起步（lerobot 侧 max_relative 已限步，无大跳变）
                start = goal.copy()
                ramp_dur = self._body_ramp_dur
            self._body_start_vec = start
            self._body_goal_vec = goal
            self._body_ramp_t0 = now
            self._body_ramp_dur = ramp_dur
            self._body_last_action_t = now
            self._body_has_target = True

    def _body_action_to_vec(self, action: dict) -> np.ndarray | None:
        """从 action dict 提取 body 目标向量（按 _body_joint_names 顺序）。None=无 body 关节。"""
        vec = np.zeros(self._n_body, dtype=float)
        idx = 0
        has_any = False
        for group in ("left_arm", "right_arm", "head", "waist"):
            values = action.get(group, [])
            joint_names = self._body_groups[group]
            for _jname, val in zip(joint_names, values):
                if idx >= self._n_body:
                    break
                vec[idx] = float(val)
                idx += 1
                has_any = True
        return vec if has_any else None

    def _body_interp_at(self, t: float) -> np.ndarray:
        """时刻 t 的插值目标（quintic 斜坡，超时 hold 在 goal）。调用方须持 _body_vec_lock。"""
        if self._body_ramp_dur <= 0:
            return self._body_goal_vec.copy()
        tau = (t - self._body_ramp_t0) / self._body_ramp_dur
        if tau <= 0.0:
            return self._body_start_vec.copy()
        if tau >= 1.0:
            return self._body_goal_vec.copy()
        s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5  # quintic，起止速度/加速度=0
        return self._body_start_vec + s * (self._body_goal_vec - self._body_start_vec)

    def _body_publish_loop(self) -> None:
        """500Hz 发布 body 插值目标（quintic 斜坡 + hold），平滑 0.02 rad 步进。

        发布节拍由独立线程 sleep 控制，不依赖 executor 调度，不复现 GIL 定时器抖动。
        """
        period = 1.0 / 500.0
        next_t = time.time()
        while self._running:
            now = time.time()
            with self._body_vec_lock:
                vec = self._body_interp_at(now) if self._body_has_target else None
            if vec is not None:
                self._publish_body_vec(vec)
            next_t += period
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()  # 落后过多则重置，避免追赶风暴

    def _publish_body_vec(self, vec: np.ndarray) -> None:
        """从 body 目标向量构造并发布 RobotCommand（限位裁剪 + lock_joints 过滤）。"""
        target_joints = []
        for jname, val in zip(self._body_joint_names, vec):
            val = float(val)
            if jname in self._body_joint_limits:
                val = _clamp(val, self._body_joint_limits[jname])
            target_joints.append((jname, val))

        if not target_joints:
            return

        if self._mc_msgs_available:
            from std_msgs.msg import Header
            msg = self._RobotCommand()
            msg.header = Header()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            for jname, val in target_joints:
                if jname in self._lock_joints:
                    continue
                jc = self._JointCmd()
                jc.name = jname
                jc.control_mode = self._JointCmd.MODE_POSITION
                jc.position = val
                msg.joint_cmd.append(jc)
        else:
            msg = self._JointState()
            msg.name = [jname for jname, _ in target_joints if jname not in self._lock_joints]
            msg.position = [val for jname, val in target_joints if jname not in self._lock_joints]

        self._body_cmd_pub.publish(msg)

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
        if self._hand_type == "c1":
            position = c1_clip_position(position, joint_names, self._hand_joint_limits)
        else:
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
        if self._action_thread.is_alive():
            self._action_thread.join(timeout=2.0)
        if self._body_thread is not None and self._body_thread.is_alive():
            self._body_thread.join(timeout=2.0)
        if self._executor is not None:
            self._executor.shutdown()
        if self._executor_thread is not None and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=3.0)
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

        image_msg_types = {}
        try:
            from shm_msgs.msg import Image2m
            image_msg_types["shm_msgs/Image2m"] = Image2m
        except ImportError:
            logger.warning("shm_msgs not available; shm_msgs/Image2m camera topics are disabled")
        try:
            from sensor_msgs.msg import Image
            image_msg_types["sensor_msgs/Image"] = Image
        except ImportError:
            logger.warning("sensor_msgs/Image not available; standard Image camera topics are disabled")

        for cam_name, cam_cfg in self._camera_topics.items():
            topic = cam_cfg.get("topic")
            msg_type_name = cam_cfg.get("msg_type", "shm_msgs/Image2m")
            if msg_type_name != "shm_msgs/Image2m":
                logger.warning(
                    "Camera relay: %s uses %s; expected shm_msgs/Image2m for unified Walker sim/real pipeline",
                    cam_name, msg_type_name,
                )
            msg_type = image_msg_types.get(msg_type_name)
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
                rows = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, step))
                img = rows[:, :width * 3].reshape((height, width, 3))
            elif encoding == "rgb8":
                rows = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, step))
                img = rows[:, :width * 3].reshape((height, width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif encoding == "bgra8":
                rows = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, step))
                img = rows[:, :width * 4].reshape((height, width, 4))
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            elif encoding == "rgba8":
                rows = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, step))
                img = rows[:, :width * 4].reshape((height, width, 4))
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            elif encoding == "yuv422":
                img = self._yuv422_to_bgr(img_data, width, height)
            elif encoding == "mono8":
                rows = np.frombuffer(img_data, dtype=np.uint8)[:byte_count].reshape((height, step))
                img = rows[:, :width]
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
    """Find and kill any already-running ros2_walker_bridge processes."""
    current_pid = os.getpid()
    try:
        result = subprocess.run(["pgrep", "-f", "ros2_walker_bridge"], capture_output=True, text=True, timeout=5)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return
    if result.returncode != 0:
        return

    pids = []
    for line in result.stdout.strip().splitlines():
        try:
            pid = int(line.strip())
            if pid != current_pid:
                pids.append(pid)
        except ValueError:
            continue
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
