#!/usr/bin/env python3
"""ROS2 Deploy Bridge for LeRobot + Walker S2 robot.

Bridges between LeRobot (Python 3.12, ZMQ) and Walker S2 hardware via ROS2 DDS.
Single-node direct-publish architecture (aligned with TienKung bridge pattern).
Supports both 7-DOF V4 hands and 1-DOF PGC grippers from normalized config JSON.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import threading
import time
from typing import Any

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


_DEFAULT_CFG = {
    "robot_model": "walker_s2_v4_hand_31d",
    "zmq_cmd_port": 5561,
    "zmq_status_port": 5562,
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
}


class ZMQInternalBridge:
    """ZMQ sockets for communication with LeRobot process."""

    def __init__(self, cmd_port: int, status_port: int):
        self.context = zmq.Context()

        self.cmd_socket = self.context.socket(zmq.SUB)
        self.cmd_socket.bind(f"tcp://*:{cmd_port}")
        self.cmd_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.cmd_socket.setsockopt(zmq.RCVHWM, 1)

        self.status_socket = self.context.socket(zmq.PUB)
        self.status_socket.bind(f"tcp://*:{status_port}")
        self.status_socket.setsockopt(zmq.SNDHWM, 1)

        logger.info("ZMQ internal bridge: cmd=%d, status=%d", cmd_port, status_port)

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

    def close(self) -> None:
        self.cmd_socket.close()
        self.status_socket.close()
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

        ros_namespace = cfg.get("ros_namespace", "").rstrip("/")
        cmd_namespace = cfg.get("cmd_namespace", "").rstrip("/") if cfg.get("cmd_namespace") else ""

        import rclpy
        from rclpy.executors import MultiThreadedExecutor
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
        # 单节点：所有 body state/command + hand/gripper 在一个 Node 中
        # ================================================================
        self._node = Node("ros2_walker_bridge")

        # Body state 缓存（list[float]，与 body_joint_names 顺序对齐）
        self._body_pos = [0.0] * self._n_body
        self._body_state_lock = threading.Lock()
        self._body_state_ready = threading.Event()

        # hand/gripper 位置缓存
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

        # ---- Body state 订阅（直接替代 RobotController 的 state sub）----
        body_state_topic = f"{ros_namespace}{cfg['topic_body_state']}" if ros_namespace else cfg["topic_body_state"]
        self._node.create_subscription(
            RobotState, body_state_topic, self._body_state_callback, qos_sensor,
        )

        # ---- Body command 发布 ----
        body_cmd_topic = f"{cmd_namespace}{cfg['topic_body_cmd']}" if cmd_namespace else cfg["topic_body_cmd"]
        self._body_cmd_pub = self._node.create_publisher(RobotCommand, body_cmd_topic, qos_cmd)

        # ---- 手/夹爪 publisher ----
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

        # ---- 手/夹爪 state subscriber ----
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

        # ---- 单 executor（对齐 TienKung 模式）----
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="bridge_executor"
        )
        self._executor_thread.start()

        # 等待首次 body state（替代 RobotController.wait_for_state）
        if not self._body_state_ready.wait(timeout=10.0):
            raise RuntimeError("Timeout waiting for RobotState on %s", body_state_topic)

        # ---- ZMQ action 转发线程 ----
        self._running = True
        self._action_thread = threading.Thread(
            target=self._action_loop, daemon=True, name="action_forward"
        )
        self._action_thread.start()

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

    # ---- body state callback（直接替代 RobotController 的 state sub）----
    def _body_state_callback(self, msg: Any) -> None:
        """解析 RobotState.joint_states，按 body_joint_names 顺序缓存为 list[float]."""
        js = msg.joint_states  # sensor_msgs/JointState
        name_to_idx = {name: i for i, name in enumerate(js.name)}
        positions = [0.0] * self._n_body
        for i, name in enumerate(self._body_joint_names):
            if name in name_to_idx:
                positions[i] = float(js.position[name_to_idx[name]])
        with self._body_state_lock:
            self._body_pos[:] = positions
        self._body_state_ready.set()
        self._publish_status()

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

    # ---- ZMQ status（body 从本地缓存读，hand/gripper 从 Bridge 缓存读）----
    def _publish_status(self) -> None:
        with self._body_state_lock:
            body_pos = list(self._body_pos)
        with self._hand_state_lock:
            left_hand = list(self._left_hand_pos)
            right_hand = list(self._right_hand_pos)

        # body_groups 的值是关节名字列表，通过 body_joint_names 索引映射到 body_pos
        name_to_idx = {n: i for i, n in enumerate(self._body_joint_names)}
        status = {
            "left_arm": [body_pos[name_to_idx[n]] for n in self._body_groups["left_arm"]],
            "right_arm": [body_pos[name_to_idx[n]] for n in self._body_groups["right_arm"]],
            "head": [body_pos[name_to_idx[n]] for n in self._body_groups["head"]],
            "waist": [body_pos[name_to_idx[n]] for n in self._body_groups["waist"]],
            "left_hand": left_hand,
            "right_hand": right_hand,
            "ts": time.time(),
        }
        self.zmq_bridge.send_status(status)

    # ---- ZMQ action 转发（直接构建 RobotCommand 发布，对齐 TienKung 模式）----
    def _action_loop(self) -> None:
        while self._running:
            action = self.zmq_bridge.recv_action(timeout_ms=50)
            if action is not None:
                self._publish_body_command(action)
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

    def _publish_body_command(self, action: dict) -> None:
        """收到 ZMQ action：直接构建 RobotCommand 并发布（对齐 TienKung 模式）。

        只对 action 中实际有值的关节发布 JointCmd。不在 action 中的关节
        （含非活跃 DOF 子集的关节）不出现在 RobotCommand.joint_cmd 中，
        SDK 内部 MODE_POSITION=2 控制器会自动保持这些关节当前位置。
        """
        goal = self._body_action_to_dict(action)
        if not goal:
            return

        msg = self._RobotCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = ""

        for name, target_angle in goal.items():
            if name in self._lock_joints:
                continue

            jc = self._JointCmd()
            jc.name = name
            jc.control_mode = self._JointCmd.MODE_POSITION  # 2
            jc.position = float(target_angle)
            msg.joint_cmd.append(jc)

        if msg.joint_cmd:
            self._body_cmd_pub.publish(msg)

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
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        if self._executor_thread is not None and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()


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
    if args.ros_namespace is not None:
        cfg["ros_namespace"] = args.ros_namespace
    if args.cmd_namespace is not None:
        cfg["cmd_namespace"] = args.cmd_namespace

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    kill_existing_bridge()

    zmq_bridge = ZMQInternalBridge(cfg["zmq_cmd_port"], cfg["zmq_status_port"])
    robot_bridge = WalkerRealRobotBridge(zmq_bridge, cfg)

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
    robot_bridge.stop()
    zmq_bridge.close()
    logger.info("Walker Bridge stopped.")


if __name__ == "__main__":
    main()
