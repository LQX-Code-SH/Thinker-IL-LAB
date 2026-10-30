"""Walker C1 HDF5 录制节点。

订阅与真机完全一致的话题（``/mc/sdk/robot_state``、``/mc/{left,right}_hand/joint_states``、
``/mc/sdk/robot_command``、``/mc/{left,right}_hand/command`` 与头部相机），
按固定频率把一帧写入 ``control/common/hdf5_recorder.EpisodeRecorder``：

* 关节列顺序固定为 ``constants.BODY_JOINT_NAMES``（31 个身体关节，含腿），
  手部按真机 SDK 的 6 维压缩通道落盘（与真机指令严格一致）；
* action 取最近一次下发的指令，未收到指令时用当前状态填充，
  保证 action 与 state 维度一致，回放适配层无需特判；
* 深度按 ``16UC1`` 存储（C++ 图像桥已输出毫米）。

仿真与真机跑同一份代码，靠 ``ROS_DOMAIN_ID`` 区分（默认 0，与真机同域；隔离时用 146）。
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

_CONTROL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)
_COMMON_DIR = os.path.abspath(os.path.join(_CONTROL_DIR, os.pardir, "common"))
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from constants import (  # noqa: E402
    BODY_JOINT_NAMES,
    CAMERA_NAME,
    DEFAULT_CAPTURE_HZ,
    DEPTH_TOPIC,
    HDF5_REL_PATH,
    LEFT_HAND_JOINT_NAMES,
    RGB_TOPIC,
    RIGHT_HAND_JOINT_NAMES,
)
from hdf5_recorder import EpisodeRecorder  # noqa: E402

try:
    from mc_state_msgs.msg import RobotState
    from mc_task_msgs.msg import JointCommand, RobotCommand
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Walker SDK ROS2 消息包未找到，先编译并 source：\n"
        "  cd /ubt_sim/docker && bash run.sh init\n"
        "  source /opt/ubt_sim/walker_c1_msgs/install/setup.bash"
    ) from exc

# mc_task_msgs/JointCmd 的常量（只采集位置模式指令）
_MODE_POSITION = 2


def episode_path(output_dir: str | Path, epoch_seconds: int) -> Path:
    """``<output_dir>/<epoch秒>/trajectory.hdf5``；同秒连续保存加 ``-2`` 后缀。"""
    output_dir = Path(output_dir)
    stem = str(int(epoch_seconds))
    candidate = output_dir / stem
    suffix = 1
    while (candidate / HDF5_REL_PATH).exists():
        suffix += 1
        candidate = output_dir / f"{stem}-{suffix}"
    return candidate / HDF5_REL_PATH


def prepare_episode_file(path: str | Path) -> Path:
    """建好 episode 目录并放权（目录 0777，文件由 ``EpisodeRecorder.save`` 置 0666）。"""
    path = Path(path)
    os.makedirs(path.parent, mode=0o777, exist_ok=True)
    os.chmod(path.parent, 0o777)
    return path


class WalkerC1DataRecorder(Node):
    """采集 31 身体关节 + 双手 SDK 6 维 + 头部 RGB/深度。"""

    def __init__(self, capture_hz: float = DEFAULT_CAPTURE_HZ, record_depth: bool = True):
        super().__init__("walker_c1_data_recorder")
        self.capture_hz = float(capture_hz)
        self.record_depth = bool(record_depth)

        self._lock = threading.Lock()
        self._body_pos: dict[str, float] = {}
        self._body_vel: dict[str, float] = {}
        self._left_pos: dict[str, float] = {}
        self._right_pos: dict[str, float] = {}
        self._cmd_body: dict[str, float] = {}
        self._cmd_left: dict[str, float] = {}
        self._cmd_right: dict[str, float] = {}

        self._rgb: np.ndarray | None = None
        self._depth: np.ndarray | None = None

        self.recorder = EpisodeRecorder()

        self.create_subscription(RobotState, "/mc/sdk/robot_state", self._state_cb, 10)
        self.create_subscription(RobotCommand, "/mc/sdk/robot_command", self._command_cb, 10)
        self.create_subscription(
            JointCommand, "/mc/left_hand/command", lambda m: self._hand_command_cb("left", m), 10
        )
        self.create_subscription(
            JointCommand, "/mc/right_hand/command", lambda m: self._hand_command_cb("right", m), 10
        )
        self.create_subscription(
            self._joint_state_type(), "/mc/left_hand/joint_states", lambda m: self._hand_state_cb("left", m), 10
        )
        self.create_subscription(
            self._joint_state_type(), "/mc/right_hand/joint_states", lambda m: self._hand_state_cb("right", m), 10
        )

        # C++ 图像桥用 best_effort 发布；订阅端 QoS 必须一致，否则收不到帧
        qos_image = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, RGB_TOPIC, self._rgb_cb, qos_image)
        if self.record_depth:
            self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, qos_image)

        self._timer = self.create_timer(1.0 / self.capture_hz, self._capture_cb)
        self.get_logger().info(
            f"Walker C1 recorder ready @ {self.capture_hz:.1f} Hz "
            f"(domain={os.environ.get('ROS_DOMAIN_ID', '0')}, rgb={RGB_TOPIC}, depth={self.record_depth})"
        )

    @staticmethod
    def _joint_state_type():
        from sensor_msgs.msg import JointState

        return JointState

    # ------------------------------------------------------------- callbacks
    def _state_cb(self, msg) -> None:
        states = getattr(msg, "joint_states", None)
        if states is None:
            return
        names = list(states.name)
        positions = list(states.position)
        velocities = list(states.velocity)
        with self._lock:
            for name, pos in zip(names, positions):
                if name in BODY_JOINT_NAMES:
                    self._body_pos[name] = float(pos)
            for name, vel in zip(names, velocities):
                if name in BODY_JOINT_NAMES:
                    self._body_vel[name] = float(vel)

    def _command_cb(self, msg) -> None:
        with self._lock:
            for ctrl in msg.joint_cmd:
                if int(ctrl.control_mode) != _MODE_POSITION:
                    continue
                name = str(ctrl.name)
                if name in BODY_JOINT_NAMES:
                    self._cmd_body[name] = float(ctrl.position)

    def _hand_state_cb(self, side: str, msg) -> None:
        target = self._left_pos if side == "left" else self._right_pos
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                target[str(name)] = float(pos)

    def _hand_command_cb(self, side: str, msg) -> None:
        target = self._cmd_left if side == "left" else self._cmd_right
        names = list(msg.names)
        if not names:
            names = LEFT_HAND_JOINT_NAMES if side == "left" else RIGHT_HAND_JOINT_NAMES
        with self._lock:
            for name, pos in zip(names, msg.position):
                target[str(name)] = float(pos)

    def _rgb_cb(self, msg: Image) -> None:
        try:
            array = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ("rgb8", "bgr8"):
                frame = array.reshape(msg.height, msg.width, 3)
                if msg.encoding == "bgr8":
                    frame = frame[:, :, ::-1]
            else:
                self.get_logger().warn(f"不支持的 RGB 编码: {msg.encoding}")
                return
            with self._lock:
                self._rgb = np.ascontiguousarray(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"RGB 解码失败: {exc}")

    def _depth_cb(self, msg: Image) -> None:
        try:
            if msg.encoding == "16UC1":
                frame = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            elif msg.encoding == "32FC1":
                frame = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            else:
                self.get_logger().warn(f"不支持的深度编码: {msg.encoding}")
                return
            with self._lock:
                self._depth = frame.copy()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"深度解码失败: {exc}")

    # ------------------------------------------------------------- recording
    def _capture_cb(self) -> None:
        if not self.recorder.recording:
            return
        with self._lock:
            if not self._body_pos:
                return
            # 图像未到齐时跳过这一拍，否则各 dataset 长度不一致
            if self._rgb is None or (self.record_depth and self._depth is None):
                return
            body = [float(self._body_pos.get(n, 0.0)) for n in BODY_JOINT_NAMES]
            body_vel = [float(self._body_vel.get(n, 0.0)) for n in BODY_JOINT_NAMES]
            left = [float(self._left_pos.get(n, 0.0)) for n in LEFT_HAND_JOINT_NAMES]
            right = [float(self._right_pos.get(n, 0.0)) for n in RIGHT_HAND_JOINT_NAMES]

            cmd_body = [
                float(self._cmd_body.get(n, self._body_pos.get(n, 0.0))) for n in BODY_JOINT_NAMES
            ]
            cmd_left = [
                float(self._cmd_left.get(n, self._left_pos.get(n, 0.0)))
                for n in LEFT_HAND_JOINT_NAMES
            ]
            cmd_right = [
                float(self._cmd_right.get(n, self._right_pos.get(n, 0.0)))
                for n in RIGHT_HAND_JOINT_NAMES
            ]

            payload: dict[str, Any] = {
                "timestamp": time.time(),
                "joint_position": body,
                "joint_velocity": body_vel,
                "joint_effort": [0.0] * len(BODY_JOINT_NAMES),
                "hand_left_position": left,
                "hand_left_velocity": [0.0] * len(LEFT_HAND_JOINT_NAMES),
                "hand_left_effort": [0.0] * len(LEFT_HAND_JOINT_NAMES),
                "hand_right_position": right,
                "hand_right_velocity": [0.0] * len(RIGHT_HAND_JOINT_NAMES),
                "hand_right_effort": [0.0] * len(RIGHT_HAND_JOINT_NAMES),
                "action_joint_position": cmd_body,
                "action_hand_left_position": cmd_left,
                "action_hand_right_position": cmd_right,
            }
            payload[f"png:{CAMERA_NAME}"] = self._rgb.copy()
            if self.record_depth:
                payload[f"depth:{CAMERA_NAME}"] = self._depth.copy()

        self.recorder.add_frame(payload)

    # ---------------------------------------------------------------- control
    def start_episode(self) -> None:
        self.recorder.start()
        self.get_logger().info("开始录制")

    def stop_episode(self) -> None:
        self.recorder.stop()
        self.get_logger().info(f"停止录制，共 {self.recorder.num_frames} 帧")

    def save_episode(self, path: str, attrs: dict[str, Any] | None = None) -> str:
        attrs = {
            # robot_type 是 playback/robots.detect_robot_type 的首选字段
            "robot_type": "walker_c1",
            "robot": "walker_c1",
            "fps": float(self.capture_hz),
            "camera_names": json.dumps([CAMERA_NAME]),
            "created_at": self.get_clock().now().nanoseconds / 1e9,
            "body_joint_names": BODY_JOINT_NAMES,
            "left_hand_joint_names": LEFT_HAND_JOINT_NAMES,
            "right_hand_joint_names": RIGHT_HAND_JOINT_NAMES,
            "capture_hz": self.capture_hz,
            **(attrs or {}),
        }
        saved = self.recorder.save(path, attrs)
        self.get_logger().info(f"已保存: {saved}")
        return str(saved)


def list_hdf5_keys(path: str) -> list[str]:
    """调试辅助：列出 HDF5 内所有 dataset 路径。"""
    import h5py

    keys: list[str] = []
    with h5py.File(path, "r") as f:
        f.visit(lambda name: keys.append(name) if isinstance(f[name], h5py.Dataset) else None)
    return keys
