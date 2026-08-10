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

import numpy as np
import zmq

# chunk_processor 与本脚本同目录（脚本以 /ubt_IL/walker/ros2_walker_bridge.py 运行，
# sys.path[0] 即该目录），提供 chunk 融合/插值/滤波算法。
from chunk_processor import (
    BlendSchedule,
    ChunkFuser,
    ChunkInterpolator,
    smooth_window,
    transition_ramp,
)

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
    # chunk 消费者配置（act_async 引擎整块推 chunk 时启用）。
    # 桥接对整块 chunk 做: 延迟补偿 -> 融合 -> 插值加密 -> 滤波 -> 过渡 ramp -> 300Hz 下发。
    # sync 单动作路径不受影响（按消息是否含 n_points 分流）。
    "chunk_consumer": {
        "enabled": True,             # False: 收到 chunk 消息时退化为取首点单动作下发
        "blend_horizon": 10,         # 融合重叠点数（需 < 每块执行步数 exec=Δt×fps 的【下限】≈14，否则快重规划时 eff_blend>exec、新块永不上纯；实际 eff_blend=min(本值,leftover)）
        "blend_schedule": "smoothstep",  # linear | smoothstep | exp
        "interp_method": "hermite",  # hermite(C1无停顿,推荐) | linear | quintic(端点停顿)
        "smoothing_window": 1,       # 滑窗均值滤波窗口 (<=1 关闭)
        "transition_ramp_pts": 10,   # 从当前 q_cmd 缓动到 chunk 首点的 300Hz 点数 (跨 chunk 兜底)
        "latency_compensation": True,  # 按 inference_time_sec 跳过过期前缀
        "control_hz": 300,           # 加密目标频率（应与 publish_hz 一致）
        "publish_hz": 300,           # body 发布线程频率（densify 后执行频率）
        "v_max": 2.0,                # 限速 rad/s (rate limit: 每 tick 位移 <= v_max*dt)
    },
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

        # ================================================================
        # 外层 300Hz 轨迹发布 + chunk 消费者状态
        # （body_pd 单动作 quintic 路径已移除；仅保留 chunk densify 后的轨迹指针推进 +
        #  rate limit + clamp。v_max / publish_hz 从 chunk_consumer 读）
        # ================================================================
        cc_cfg = {**_DEFAULT_CFG.get("chunk_consumer", {}), **(cfg.get("chunk_consumer") or {})}
        self._chunk_consumer_enabled = bool(cc_cfg.get("enabled", True))
        self._blend_horizon = int(cc_cfg.get("blend_horizon", 10))
        self._blend_schedule = BlendSchedule(cc_cfg.get("blend_schedule", "smoothstep"))
        self._interp_method = str(cc_cfg.get("interp_method", "hermite"))
        self._smoothing_window = int(cc_cfg.get("smoothing_window", 1))
        self._transition_ramp_pts = int(cc_cfg.get("transition_ramp_pts", 10))
        self._latency_compensation = bool(cc_cfg.get("latency_compensation", True))
        self._chunk_control_hz = float(cc_cfg.get("control_hz", 300.0))
        self._v_max = float(cc_cfg.get("v_max", 2.0))
        self._publish_dt = 1.0 / float(cc_cfg.get("publish_hz", 300.0))
        self._chunk_interp = ChunkInterpolator(
            control_hz=self._chunk_control_hz, method=self._interp_method
        )
        self._chunk_fuser = ChunkFuser(
            blend_horizon=self._blend_horizon, schedule=self._blend_schedule
        )
        # 轨迹状态（_pd_lock 保护）：chunk_mode=True 时走轨迹指针推进
        self._chunk_mode = False
        self._body_traj = None               # [N, n_body] 300Hz 密轨迹
        self._body_traj_index = 0            # 当前 300Hz 指针
        self._traj_point_dt = 1.0 / 15.0     # 当前轨迹的 chunk 点间距（=1/fps）
        # hand chunk 轨迹（_pd_lock 保护）
        self._hand_traj = {"left": None, "right": None}
        self._last_hand_cp = -1              # 上次下发的 hand chunk-point 索引
        # _q_cmd 始终在 _pd_lock 内访问；首帧 state 就绪时初始化为当前位置
        self._q_cmd = None
        self._pd_lock = threading.Lock()
        self._body_publish_thread = None

        # fused 轨迹录制（RECORD_ACTIONS=1 时落盘 fused.csv，供 recorder 绘三轨迹对比图）。
        # 融合只对 body 关节；按 _body_groups 顺序预计算 (group, ji, name, col) 布局。
        self._fused_csv = None
        self._fused_layout: list[tuple[str, int, str, int]] = []
        if os.environ.get("RECORD_ACTIONS", "0") == "1":
            fused_dir = os.environ.get("RECORD_OUTPUT_DIR", "/tmp/walker_rollout")
            try:
                os.makedirs(fused_dir, exist_ok=True)
                fused_path = os.path.join(fused_dir, "fused.csv")
                self._fused_csv = open(fused_path, "w", buffering=1)  # line-buffered
                self._fused_csv.write("exec_time,chunk_id,point_idx,group,joint_idx,joint_name,fused\n")
                col = 0
                for group in ("left_arm", "right_arm", "head", "waist"):
                    for ji, jname in enumerate(self._body_groups.get(group, [])):
                        self._fused_layout.append((group, ji, jname, col))
                        col += 1
                logger.info("fused trajectory recording -> %s", fused_path)
            except OSError as e:
                logger.warning("Failed to open fused.csv: %s", e)
                self._fused_csv = None

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

        # ---- 外层 300Hz 轨迹发布线程（chunk_consumer 启用时 densify 后执行）----
        if self._chunk_consumer_enabled:
            self._body_publish_thread = threading.Thread(
                target=self._body_publish_loop, daemon=True, name="body_publish"
            )
            self._body_publish_thread.start()
            logger.info(
                "Body publish thread enabled: %dHz v_max=%.2f (chunk densify + ratelimit)",
                int(1.0 / self._publish_dt), self._v_max,
            )
        else:
            logger.info("Body publish thread disabled (event-driven fallback)")

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
        """解析 RobotState.joint_states，缓存 position/velocity（_body_state_lock）。"""
        js = msg.joint_states  # sensor_msgs/JointState
        name_to_idx = {name: i for i, name in enumerate(js.name)}
        positions = [0.0] * self._n_body
        for i, name in enumerate(self._body_joint_names):
            if name in name_to_idx:
                idx = name_to_idx[name]
                positions[i] = float(js.position[idx])
        with self._body_state_lock:
            self._body_pos[:] = positions
        # 首帧 state 就绪：初始化 _q_cmd 为当前位置（_pd_lock 保护，state_lock 已释放）
        with self._pd_lock:
            if self._q_cmd is None:
                self._q_cmd = list(positions)
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

    # ---- ZMQ action 转发（chunk 消费 / 事件驱动回滚）----
    def _action_loop(self) -> None:
        while self._running:
            action = self.zmq_bridge.recv_action(timeout_ms=50)
            if action is None:
                time.sleep(0.001)                # 避免忙循环 100% CPU
                continue
            # hold 控制消息（stop 立即停）：截断轨迹为单点当前位 + 标记耗尽，300Hz hold。
            # 须在 n_points/单动作分流之前截出（hold 消息无 n_points，否则误入单动作路径）。
            if action.get("hold"):
                self._handle_hold()
                continue
            # chunk 消息（act_async 引擎整块下发）
            if action.get("n_points"):
                if self._chunk_consumer_enabled:
                    self._handle_chunk(action)            # 融合/插值/滤波流水线
                    continue
                # chunk_consumer 关闭：退化为取每块首点作单动作下发
                action = self._chunk_first_point_as_action(action)
            # 单动作路径（sync 引擎 / chunk 退化，向后兼容）：事件驱动直接发 body
            self._publish_body_command(action)
            self._publish_end_effector_command("left", action.get("left_hand", []))
            self._publish_end_effector_command("right", action.get("right_hand", []))

    @staticmethod
    def _chunk_first_point_as_action(msg: dict) -> dict:
        """从 chunk 消息提取每块首点，组装单动作 dict（chunk_consumer 关闭时退化用）。"""
        out: dict[str, list] = {}
        for group in ("left_arm", "right_arm", "head", "waist", "left_hand", "right_hand"):
            arr = msg.get(group, [])
            if arr:
                out[group] = list(arr[0])     # 首行 = chunk 首点
        out["ts"] = msg.get("ts", time.time())
        return out

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

        # 单动作夺取控制权：清掉 300Hz chunk 轨迹 hold，避免与 chunk 发布线程双源冲突。
        # act_async rollout 期间不发单动作（只发 chunk），本分支仅关机 return-to-initial/home
        # 走单动作时触发；sync 引擎 / chunk 退化路径 chunk_mode 本就 False，此处置零为无操作。
        # 必须持 _pd_lock（_body_publish_loop 在同锁下读 _chunk_mode），避免竞态。清后
        # 300Hz 线程 if self._chunk_mode 为 False -> 本 tick 不再 publish 陈旧 hold，单动作独占。
        # _q_cmd 不清（保留当前位置），后续若来 chunk 仍可作 first-frame 插值起点。
        with self._pd_lock:
            if self._chunk_mode:
                self._chunk_mode = False
                self._body_traj = None
                self._body_traj_index = 0

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

    # ================================================================
    # chunk 消费者：整块 chunk -> 延迟补偿 -> 融合 -> 插值加密 -> 滤波 -> 300Hz 轨迹
    # ================================================================

    def _clamp_body_array(self, q: np.ndarray) -> np.ndarray:
        """按 body_joint_limits clamp 数组（就地）。"""
        for i, name in enumerate(self._body_joint_names):
            if name in self._body_joint_limits:
                lo, hi = self._body_joint_limits[name]
                q[i] = max(float(lo), min(float(hi), q[i]))
        return q

    def _chunk_groups_to_body_array(self, msg: dict) -> np.ndarray | None:
        """从 chunk 消息的 body 组数组组装 [C, n_body] body 轨迹。

        仅纳入 bridge 实际控制的组（_body_groups 中非空的组）。部分 DOF 模型
        （如 10d 右臂）不控制 left_arm/waist，对应 _body_groups 为空 -> 跳过，
        与单动作 _publish_body_command 仅发 body_joint_names 的语义一致；
        robot_sdk 自身保持非 policy 关节原位。
        """
        rows = []
        for group in ("left_arm", "right_arm", "head", "waist"):
            if not self._body_groups.get(group):
                continue  # bridge 不控制该组（部分 DOF 模型）-> 不纳入 body 数组
            arr = msg.get(group, [])
            if not arr:
                return None  # bridge 控制该组但 chunk 未提供 -> 数据缺失，丢弃
            rows.append(np.asarray(arr, dtype=float))  # [C, len(group)]
        if not rows:
            return None
        body = np.concatenate(rows, axis=1)  # [C, n_body]
        if body.shape[1] != self._n_body:
            logger.warning("chunk body dim %d != n_body %d, drop", body.shape[1], self._n_body)
            return None
        return body

    @staticmethod
    def _extract_hand_chunk(msg: dict, group: str) -> np.ndarray | None:
        arr = msg.get(group, [])
        if not arr:
            return None
        return np.asarray(arr, dtype=float)  # [C, n_hand]

    def _current_body_traj_leftover(self) -> np.ndarray | None:
        """当前密轨迹未执行后缀，采样到 chunk 点分辨率（供 ChunkFuser.blend）。

        调用方需持 _pd_lock。
        """
        traj = self._body_traj
        if traj is None or len(traj) == 0:
            return None
        idx = self._body_traj_index
        if idx >= len(traj):
            return traj[-1:]              # 已执行完：返回末帧（保持位姿）
        remaining = traj[idx:]
        pps = max(1, int(round(self._chunk_control_hz * self._traj_point_dt)))
        return remaining[::pps]

    def _handle_hold(self) -> None:
        """立即 hold 当前位姿(stop 用)：轨迹截断为单点 _q_cmd + 标记耗尽。

        300Hz 线程见 idx>=len -> hold 末帧(=当前 _q_cmd)、不推进；_chunk_mode 仍 True、
        _body_traj 非空 -> 稳定 hold，不触发单动作双源。hand 状态清空避免冻结后
        cp 回跳误发 hand[0]。下一块 chunk 因 traj_exhausted=True 走首帧平滑重启
        (插 _body_pos 实际位 + hermite)，与 start/stop 突变修复一致。
        """
        with self._pd_lock:
            if self._q_cmd is None:
                return
            self._body_traj = np.array([self._q_cmd], dtype=float)  # [1, n_body]
            self._body_traj_index = 1                               # 标记耗尽 -> hold + 下块首帧
            self._hand_traj = {"left": None, "right": None}         # 避免冻结后 hand cp 回跳
            self._last_hand_cp = -1

    def _handle_chunk(self, msg: dict) -> None:
        """消费整块 chunk：延迟补偿 -> 融合 -> 插值加密 -> 滤波 -> 过渡 ramp -> 设轨迹。

        首帧（无前驱轨迹）特殊处理：不跳过过期前缀（warmup 首块推理耗时大，skip 会
        浪费整块 chunk），改为把当前实际位置 _q_cmd 插到 chunk 最前方，由 hermite 插值
        从实际位置平滑过渡到 chunk 内容（C1 连续，无需 transition_ramp）。
        """
        n_points = int(msg.get("n_points", 0))
        body = self._chunk_groups_to_body_array(msg)
        if body is None or n_points <= 0:
            return

        # 安全 clamp（每点按 body 限位）
        for t in range(len(body)):
            body[t] = self._clamp_body_array(body[t])

        fps = float(msg.get("fps", 15.0)) or 15.0
        point_dt = 1.0 / fps
        inference_time_sec = float(msg.get("inference_time_sec", 0.0))
        C = len(body)
        consume_ts = time.time()
        obs_ts = float(msg.get("obs_time_sec", 0.0))

        # 首帧判断：无前驱轨迹，或前驱轨迹【已执行完】（stop 暂停后 hold 末帧）。
        # 首块推理 warmup 耗时大，skip 会跳过整块 chunk 大部分点 -> 执行起点远离实际位置，
        # 改为不 skip、插当前点。exhausted 判定是关键：stop(pause) 不通知桥接，_chunk_mode
        # 仍 True、_body_traj 仍指向已耗尽旧轨迹；若不判 exhausted，resume 后首块误走「后续帧」
        # 延迟补偿 skip 路径，从 hold 位突跳到 chunk[l] -> 与首帧同款突变。exhausted 即视作
        # 首帧，插当前实际位、hermite 平滑重启。常态重规划时新块到达前旧轨迹总有剩余
        # （chunk ~6.7s@100pts/15fps ≫ 1/inference_hz），exhausted 仅在 pause/停滞命中，不影响 blend。
        # 先在 _pd_lock 外快照实际位置 _body_pos（_body_state_lock 保护），避免锁嵌套。
        with self._body_state_lock:
            actual_pos = list(self._body_pos)
        with self._pd_lock:
            traj_exhausted = (
                self._body_traj is None
                or len(self._body_traj) == 0
                or self._body_traj_index >= len(self._body_traj)
            )
            is_first = traj_exhausted or not self._chunk_mode
            prev_leftover = self._current_body_traj_leftover() if not is_first else None
            if is_first:
                # 首帧/停启后首块：从【实际位置】过渡。_q_cmd 在无 publishing 间隙
                # （preheat/READY 期未 start）冻结为首次 state 值、可能陈旧；actual_pos 恒新鲜。
                q_src = actual_pos if self._body_state_ready.is_set() else self._q_cmd
                current_q = (np.array(q_src, dtype=float)
                             if q_src is not None else body[0].copy())
            else:
                current_q = (np.array(self._q_cmd, dtype=float)
                             if self._q_cmd is not None else body[0].copy())

        if is_first:
            # 首帧：不 skip，把当前实际位置插到 chunk 最前方，hermite 从实际位置平滑过渡
            skip = 0
            fused = np.vstack([current_q[np.newaxis, :], body])  # [C+1, n_body]
        else:
            # 后续帧：延迟补偿跳过过期前缀 + blend 融合
            if self._latency_compensation:
                L = (consume_ts - obs_ts) if obs_ts > 0 else inference_time_sec
                l = int(np.ceil(L / point_dt)) if L > 0 else 0
            else:
                l = 0
            l = max(0, min(l, C - 1))
            body = body[l:]
            skip = l
            with self._pd_lock:
                fused = self._chunk_fuser.blend(body, prev_leftover)

        # 录制融合后目标 -> fused.csv（recorder plot_chunks 读它画 fused 轨迹）。
        # exec_time = 桥接消费时刻 + ramp_offset + k/fps；首帧无 ramp -> ramp_offset=0。
        if self._fused_csv is not None:
            ramp_offset = ((self._transition_ramp_pts / self._chunk_control_hz)
                           if (not is_first and self._transition_ramp_pts > 0) else 0.0)
            cid = int(msg.get("chunk_id", 0))
            lines = []
            for k in range(len(fused)):
                exec_t = consume_ts + ramp_offset + k * point_dt
                fk = fused[k]
                for group, ji, jname, col in self._fused_layout:
                    lines.append(f"{exec_t:.6f},{cid},{k},{group},{ji},{jname},{float(fk[col]):.6f}")
            self._fused_csv.write("\n".join(lines) + "\n")
            self._fused_csv.flush()

        # 插值加密到 control_hz（首帧 fused[0]=current_q，hermite 从实际位置平滑出发）
        densified = self._chunk_interp.densify(fused, point_dt)

        # 可选滑窗滤波
        if self._smoothing_window > 1:
            densified = smooth_window(densified, self._smoothing_window)

        # 过渡 ramp + 设轨迹（_pd_lock 内，原子化）
        with self._pd_lock:
            if not is_first and self._q_cmd is not None and self._transition_ramp_pts > 0:
                # 后续帧：transition_ramp 从当前 q_cmd 缓动到 densified[0]（跨 chunk 兜底）
                current_q = np.array(self._q_cmd, dtype=float)
                ramp = transition_ramp(current_q, densified[0], self._transition_ramp_pts)
                if len(ramp) > 1:
                    traj = np.vstack([ramp, densified])
                else:
                    traj = densified
            else:
                # 首帧：fused 已含 current_q 前缀，hermite 已平滑，无需 ramp
                traj = densified
            self._body_traj = traj
            self._body_traj_index = 0
            self._traj_point_dt = point_dt
            self._chunk_mode = True
            self._hand_traj = {
                "left": self._extract_hand_chunk(msg, "left_hand"),
                "right": self._extract_hand_chunk(msg, "right_hand"),
            }
            self._last_hand_cp = -1

        chunk_id = int(msg.get("chunk_id", 0))
        logger.debug(
            "chunk consumed: id=%d pts=%d skip=%d fused=%d densified=%d traj=%d%s",
            chunk_id, n_points, skip, len(fused), len(densified), len(traj),
            " (first-frame insert current_q)" if is_first else "",
        )

    def _body_publish_loop(self) -> None:
        """300Hz 轨迹发布：chunk 密轨迹指针推进 -> 限速限位 -> q_cmd -> RobotCommand。

        独立线程 + time.sleep 控节拍，不依赖 ROS2 executor（避免 GIL 定时器抖动）。
        无单动作 quintic 分支；_q_cmd 未初始化时跳过（等首帧 state）。
        """
        dt = self._publish_dt
        while self._running:
            t_start = time.monotonic()
            q_cmd_snapshot = None
            hand_targets = None
            with self._pd_lock:
                if self._chunk_mode and self._q_cmd is not None:
                    q_cmd_snapshot, _ = self._pd_tick_chunk()
                    hand_targets = self._advance_hand_chunk()
            # publish body（不持锁，避免 RELIABLE QoS 阻塞持锁）
            if q_cmd_snapshot is not None:
                self._publish_body_from_cmd(q_cmd_snapshot)
            # hand chunk 下发（fps 节拍，chunk 点索引变化时下发）
            if hand_targets is not None:
                left_hand, right_hand = hand_targets
                if left_hand is not None:
                    self._publish_end_effector_command("left", left_hand)
                if right_hand is not None:
                    self._publish_end_effector_command("right", right_hand)
            # 节拍补偿
            elapsed = time.monotonic() - t_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    def _pd_tick_chunk(self) -> tuple[list | None, tuple | None]:
        """chunk 轨迹指针推进 -> 限速限位。调用方已持 _pd_lock。

        q_des 取自 300Hz 密轨迹当前指针，逐 tick 推进（末帧 hold）。
        """
        traj = self._body_traj
        if traj is None or len(traj) == 0:
            return None, None
        idx = self._body_traj_index
        if idx >= len(traj):
            idx = len(traj) - 1              # 已执行完：保持末帧
        q_des = np.array(traj[idx], dtype=float)
        # 推进指针（末帧不越界）
        if self._body_traj_index < len(traj) - 1:
            self._body_traj_index += 1
        return self._pd_apply(q_des)

    def _pd_apply(self, q_des: np.ndarray) -> tuple[list, tuple]:
        """前馈 + rate limit + clamp + 更新 _q_cmd。调用方已持 _pd_lock。

        q_des 直接作位置目标（无 PD 反馈校正），仅限速（每 tick 位移 <= v_max*dt）
        与限位。返回 (q_cmd_snapshot, (q_des, delta))。
        """
        max_delta = self._v_max * self._publish_dt
        q_cmd_prev = np.array(self._q_cmd, dtype=float)
        delta = np.clip(q_des - q_cmd_prev, -max_delta, max_delta)
        q_cmd_new = self._clamp_body_array(q_cmd_prev + delta)
        self._q_cmd = list(q_cmd_new)
        return list(q_cmd_new), (q_des, delta)

    def _advance_hand_chunk(self) -> tuple[list | None, list | None]:
        """按 body 轨迹进度推进 hand chunk 指针。调用方已持 _pd_lock。

        chunk 点索引 = body_traj_index // pps；变化时返回对应 hand 点，否则 (None, None)
        （本 tick 不重复下发 hand）。hand 与 body 同步，按 fps 节拍推进。
        """
        pps = max(1, int(round(self._chunk_control_hz * self._traj_point_dt)))
        cp = self._body_traj_index // pps
        if cp == self._last_hand_cp:
            return None, None
        self._last_hand_cp = cp

        def pick(side: str) -> list | None:
            ht = self._hand_traj.get(side)
            if ht is None or len(ht) == 0:
                return None
            idx = min(cp, len(ht) - 1)
            return [float(v) for v in ht[idx]]

        return pick("left"), pick("right")

    def _publish_body_from_cmd(self, q_cmd: list) -> None:
        """从 q_cmd（body 顺序）构建 RobotCommand 并发布（MODE_POSITION=2，跳过 lock_joints）。

        与 _publish_body_command 区别：后者从 action dict 提取（事件驱动回滚用）；
        本方法从轨迹积分输出 q_cmd 直接构建全 body 命令。
        """
        msg = self._RobotCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = ""
        for i, name in enumerate(self._body_joint_names):
            if name in self._lock_joints:
                continue
            jc = self._JointCmd()
            jc.name = name
            jc.control_mode = self._JointCmd.MODE_POSITION  # 2
            jc.position = float(q_cmd[i])
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
        if self._fused_csv is not None:
            try:
                self._fused_csv.close()
            except Exception:
                pass
            self._fused_csv = None
        # 顺序：发布线程 -> action 线程 -> executor -> node
        if self._body_publish_thread is not None and self._body_publish_thread.is_alive():
            self._body_publish_thread.join(timeout=2.0)
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
    cmdline and would kill the parent - use /proc scanning instead.
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

    # chunk_consumer 环境变量覆盖（便于 rollout.sh 透传：bridge 子进程继承环境，无需改 lerobot-rollout）
    cfg.setdefault("chunk_consumer", {})
    cc_enabled = os.environ.get("CHUNK_CONSUMER_ENABLED")
    if cc_enabled is not None:
        cfg["chunk_consumer"]["enabled"] = cc_enabled.lower() in ("1", "true", "yes", "on")
    _CHUNK_CC_ENV = {
        "BLEND_HORIZON": ("blend_horizon", int),
        "TRANSITION_RAMP_PTS": ("transition_ramp_pts", int),
        "SMOOTHING_WINDOW": ("smoothing_window", int),
        "BODY_PUBLISH_HZ": ("publish_hz", float),
        "BODY_V_MAX": ("v_max", float),
    }
    for env_key, (field, cast) in _CHUNK_CC_ENV.items():
        val = os.environ.get(env_key)
        if val is not None:
            cfg["chunk_consumer"][field] = cast(val)

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
