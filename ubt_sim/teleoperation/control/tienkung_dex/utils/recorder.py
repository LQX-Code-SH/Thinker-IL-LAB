"""tienkung_dex HDF5 录制节点。

订阅桥接发布的 ``JointState`` 与 ``sensor_msgs/Image``，按固定频率把一帧写入
``control/common/hdf5_recorder.EpisodeRecorder``：

* 关节列顺序固定为 ``constants.BODY_JOINTS`` + 当前变体的左右手关节清单
  （inspire 18+18 / brainco 11+11；与训练侧 state 顺序解耦，转换脚本按 name 重排）；
* action 取最近一次下发的指令（body / 双手 command 话题），未收到指令时用当前状态填充；
* 深度按 ``16UC1`` 存储（C++ 图像桥已输出毫米），空值由核心层置 65535。
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

_CONTROL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)
_COMMON_DIR = os.path.abspath(os.path.join(_CONTROL_DIR, os.pardir, "common"))
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from constants import BODY_JOINTS, DEFAULT_VARIANT, hand_joints  # noqa: E402
from hdf5_recorder import EpisodeRecorder  # noqa: E402

DEFAULT_CAPTURE_HZ = 15.0

BODY_STATE_TOPIC = "/tienkung_dex/body_state"
LEFT_HAND_STATE_TOPIC = "/tienkung_dex/left_hand_state"
RIGHT_HAND_STATE_TOPIC = "/tienkung_dex/right_hand_state"
BODY_COMMAND_TOPIC = "/tienkung_dex/body_command"
LEFT_HAND_COMMAND_TOPIC = "/tienkung_dex/left_hand_command"
RIGHT_HAND_COMMAND_TOPIC = "/tienkung_dex/right_hand_command"
IMAGE_RGB_TOPIC = "/sensor/camera/head/color/raw"
IMAGE_DEPTH_TOPIC = "/sensor/camera/head/depth/raw"


class TienkungDexDataRecorder(Node):
    """采集 body / 双手关节 + 头部 RGB/深度。"""

    def __init__(
        self,
        capture_hz: float = DEFAULT_CAPTURE_HZ,
        record_depth: bool = True,
        variant: str = DEFAULT_VARIANT,
    ):
        super().__init__("tienkung_dex_data_recorder")
        self.capture_hz = float(capture_hz)
        self.record_depth = bool(record_depth)
        self.variant = variant
        self.left_hand_joints, self.right_hand_joints = hand_joints(variant)

        self._lock = threading.Lock()
        self._body_pos: dict[str, float] = {}
        self._body_vel: dict[str, float] = {}
        self._left_pos: dict[str, float] = {}
        self._left_vel: dict[str, float] = {}
        self._right_pos: dict[str, float] = {}
        self._right_vel: dict[str, float] = {}
        self._cmd_body: dict[str, float] = {}
        self._cmd_left: dict[str, float] = {}
        self._cmd_right: dict[str, float] = {}

        self._rgb: np.ndarray | None = None
        self._depth: np.ndarray | None = None

        self.recorder = EpisodeRecorder()

        self.create_subscription(JointState, BODY_STATE_TOPIC, self._body_state_cb, 10)
        self.create_subscription(JointState, LEFT_HAND_STATE_TOPIC, self._left_state_cb, 10)
        self.create_subscription(JointState, RIGHT_HAND_STATE_TOPIC, self._right_state_cb, 10)
        self.create_subscription(JointState, BODY_COMMAND_TOPIC, lambda m: self._command_cb(m, "body"), 10)
        self.create_subscription(JointState, LEFT_HAND_COMMAND_TOPIC, lambda m: self._command_cb(m, "left"), 10)
        self.create_subscription(JointState, RIGHT_HAND_COMMAND_TOPIC, lambda m: self._command_cb(m, "right"), 10)
        self.create_subscription(Image, IMAGE_RGB_TOPIC, self._rgb_cb, 10)
        if self.record_depth:
            self.create_subscription(Image, IMAGE_DEPTH_TOPIC, self._depth_cb, 10)

        self._timer = self.create_timer(1.0 / self.capture_hz, self._capture_cb)
        self.get_logger().info(f"TienKung Dex recorder ready @ {self.capture_hz:.1f} Hz (variant={self.variant})")

    # ------------------------------------------------------------- callbacks
    def _body_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._body_pos.update(zip(msg.name, msg.position))
            self._body_vel.update(zip(msg.name, msg.velocity))

    def _left_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._left_pos.update(zip(msg.name, msg.position))
            self._left_vel.update(zip(msg.name, msg.velocity))

    def _right_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._right_pos.update(zip(msg.name, msg.position))
            self._right_vel.update(zip(msg.name, msg.velocity))

    def _command_cb(self, msg: JointState, group: str) -> None:
        target = {"body": self._cmd_body, "left": self._cmd_left, "right": self._cmd_right}[group]
        with self._lock:
            target.update(zip(msg.name, msg.position))

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
            body = [float(self._body_pos.get(n, 0.0)) for n in BODY_JOINTS]
            body_vel = [float(self._body_vel.get(n, 0.0)) for n in BODY_JOINTS]
            left = [float(self._left_pos.get(n, 0.0)) for n in self.left_hand_joints]
            left_vel = [float(self._left_vel.get(n, 0.0)) for n in self.left_hand_joints]
            right = [float(self._right_pos.get(n, 0.0)) for n in self.right_hand_joints]
            right_vel = [float(self._right_vel.get(n, 0.0)) for n in self.right_hand_joints]

            cmd_body = [float(self._cmd_body.get(n, self._body_pos.get(n, 0.0))) for n in BODY_JOINTS]
            cmd_left = [float(self._cmd_left.get(n, self._left_pos.get(n, 0.0))) for n in self.left_hand_joints]
            cmd_right = [float(self._cmd_right.get(n, self._right_pos.get(n, 0.0))) for n in self.right_hand_joints]

            payload: dict[str, Any] = {
                "timestamp": time.time(),
                "joint_position": body,
                "joint_velocity": body_vel,
                "joint_effort": [0.0] * len(BODY_JOINTS),
                "hand_left_position": left,
                "hand_left_velocity": left_vel,
                "hand_left_effort": [0.0] * len(self.left_hand_joints),
                "hand_right_position": right,
                "hand_right_velocity": right_vel,
                "hand_right_effort": [0.0] * len(self.right_hand_joints),
                "action_joint_position": cmd_body,
                "action_hand_left_position": cmd_left,
                "action_hand_right_position": cmd_right,
            }
            if self._rgb is not None:
                payload["png:head_rgb"] = self._rgb.copy()
            if self.record_depth and self._depth is not None:
                payload["depth:head_depth"] = self._depth.copy()

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
            "robot": "tienkung_dex",
            "hand_variant": self.variant,
            "body_joint_names": BODY_JOINTS,
            "left_hand_joint_names": self.left_hand_joints,
            "right_hand_joint_names": self.right_hand_joints,
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
