"""tienkung_dex HDF5 录制节点。

订阅与真机完全一致的话题（``/robot_state``、厂商灵巧手状态、头部相机），
按固定频率把一帧写入 ``control/common/hdf5_recorder.EpisodeRecorder``：

* 关节列顺序固定为 ``constants.BODY_JOINTS`` + 当前变体的左右手**驱动**关节
  （inspire 13+13 / brainco 6+6，见 ``constants.driven_hand_joints``；只录真机
  通道实际驱动的关节，mimic 的 5 个 ``*_dip`` 不落盘，与真机 13 通道一致）；
* action 取最近一次下发的指令（``/arm|head|waist|leg/cmd`` 与灵巧手指令），
  未收到指令时用当前状态填充；
* 深度按 ``16UC1`` 存储（C++ 图像桥已输出毫米），空值由核心层置 65535。

仿真与真机跑同一份代码，靠 ``ROS_DOMAIN_ID`` 区分（默认 0（与真机同域），隔离时用 146）。
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
    BODY_JOINTS,
    DEFAULT_VARIANT,
    JOINT_ID_TO_SIM,
    driven_hand_joints,
    hand_channel_joints,
    hand_code_to_rad,
)
from dex_controller import GROUP_MSG, TOPICS, load_real_msgs  # noqa: E402
from hdf5_recorder import EpisodeRecorder  # noqa: E402

try:  # 作为包导入（utils.recorder）/ 作为顶层模块导入都能用
    from .camera import image_topics
except ImportError:  # pragma: no cover
    from camera import image_topics  # type: ignore[no-redef]

DEFAULT_CAPTURE_HZ = 15.0
GROUP_TOPICS = tuple(f"/{group}/cmd" for group in ("arm", "head", "waist", "leg"))
# 头部相机在 HDF5 里的名字：camera_observations/{color_images,depth_images}/camera_head，
# 与 walker_s2 / tienkung_pro 的 <camera_name> 命名一致（话题 /ob_camera_head/*）
CAMERA_NAME = "camera_head"
HDF5_REL_PATH = "trajectory.hdf5"


def episode_path(output_dir: str | Path, epoch_seconds: int) -> Path:
    """与 tienkung_pro / walker_s2 一致的落盘位置：``<output_dir>/<epoch秒>/trajectory.hdf5``。

    同一秒内连续存两集时给目录名加 ``-2`` / ``-3`` 后缀，避免静默覆盖上一集
    （``--duration`` 设得很短、或手动连按 ``s`` 时会撞上）。
    """
    output_dir = Path(output_dir)
    stem = str(int(epoch_seconds))
    candidate = output_dir / stem
    suffix = 1
    while (candidate / HDF5_REL_PATH).exists():
        suffix += 1
        candidate = output_dir / f"{stem}-{suffix}"
    return candidate / HDF5_REL_PATH


def prepare_episode_file(path: str | Path) -> Path:
    """建好 episode 目录并放权，返回 HDF5 路径（目录 0777，文件由 ``EpisodeRecorder.save`` 置 0666）。"""
    path = Path(path)
    # mkdir 的 mode 会被 umask 削掉，必须再 chmod 一次
    os.makedirs(path.parent, mode=0o777, exist_ok=True)
    os.chmod(path.parent, 0o777)
    return path


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
        # 只录真机通道驱动的关节（inspire 13 / brainco 6），mimic 的 dip 不落盘
        self.left_hand_joints, self.right_hand_joints = driven_hand_joints(variant)
        self.real_msgs = load_real_msgs(variant)

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

        # ---- 状态：/robot_state + 厂商灵巧手状态 ----
        self.create_subscription(
            self.real_msgs["RobotState"], TOPICS["robot_state"], self._state_cb, 10
        )
        for group, topic in zip(("arm", "head", "waist", "leg"), GROUP_TOPICS):
            self.create_subscription(
                self.real_msgs[GROUP_MSG[group]],
                topic,
                lambda msg, g=group: self._command_cb(g, msg),
                10,
            )

        for side in ("left", "right"):
            topics = TOPICS["hand"][variant][side]
            self.create_subscription(
                self.real_msgs["hand_state"],
                topics["state"],
                lambda msg, s=side: self._hand_state_cb(s, msg),
                10,
            )
            self.create_subscription(
                self.real_msgs["hand_command"],
                topics["command"],
                lambda msg, s=side: self._hand_command_cb(s, msg),
                10,
            )

        # C++ 图像桥用 best_effort 发布；订阅端必须一致，否则收不到帧
        qos_image = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        rgb_topic, depth_topic = image_topics()
        self.create_subscription(Image, rgb_topic, self._rgb_cb, qos_image)
        if self.record_depth:
            self.create_subscription(Image, depth_topic, self._depth_cb, qos_image)

        self._timer = self.create_timer(1.0 / self.capture_hz, self._capture_cb)
        self.get_logger().info(
            f"TienKung Dex recorder ready @ {self.capture_hz:.1f} Hz "
            f"(variant={self.variant}, domain={os.environ.get('ROS_DOMAIN_ID', '0')}, rgb={rgb_topic})"
        )

    # ------------------------------------------------------------- callbacks
    def _state_cb(self, msg) -> None:
        with self._lock:
            for group in ("head", "waist", "arm", "leg"):
                container = getattr(msg, group, None)
                if container is None:
                    continue
                for item in getattr(container, "status", ()) or ():
                    try:
                        joint_id = int(item.name)
                    except (TypeError, ValueError):
                        continue
                    sim_name = JOINT_ID_TO_SIM.get(joint_id)
                    if sim_name is None:
                        continue
                    self._body_pos[sim_name] = float(item.pos)
                    self._body_vel[sim_name] = float(getattr(item, "speed", 0.0))

    def _command_cb(self, group: str, msg) -> None:
        with self._lock:
            for ctrl in msg.ctrl:
                try:
                    joint_id = int(ctrl.name)
                except (TypeError, ValueError):
                    continue
                sim_name = JOINT_ID_TO_SIM.get(joint_id)
                if sim_name is not None:
                    self._cmd_body[sim_name] = float(ctrl.pos)

    def _hand_codes(self, msg) -> list[int]:
        field = "positions" if self.variant == "brainco" else "joint_values"
        raw = getattr(msg, field, None)
        return [] if raw is None else list(raw)

    def _hand_state_cb(self, side: str, msg) -> None:
        target = self._left_pos if side == "left" else self._right_pos
        codes = self._hand_codes(msg)
        with self._lock:
            for channel, joints in enumerate(hand_channel_joints(self.variant, side)):
                if not joints or channel >= len(codes):
                    continue
                joint = joints[0]
                target[joint] = hand_code_to_rad(self.variant, channel, codes[channel], joint)

    def _hand_command_cb(self, side: str, msg) -> None:
        target = self._cmd_left if side == "left" else self._cmd_right
        codes = self._hand_codes(msg)
        with self._lock:
            for channel, joints in enumerate(hand_channel_joints(self.variant, side)):
                if not joints or channel >= len(codes):
                    continue
                joint = joints[0]
                target[joint] = hand_code_to_rad(self.variant, channel, codes[channel], joint)

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
            # 图像还没到齐就跳过这一拍。否则缺首帧的那几拍只写关节不写图像，
            # 各 dataset 长度不一致，depth 相对 rgb/joint 整体错位 2 帧。
            if self._rgb is None or (self.record_depth and self._depth is None):
                return
            body = [float(self._body_pos.get(n, 0.0)) for n in BODY_JOINTS]
            body_vel = [float(self._body_vel.get(n, 0.0)) for n in BODY_JOINTS]
            left = [float(self._left_pos.get(n, 0.0)) for n in self.left_hand_joints]
            right = [float(self._right_pos.get(n, 0.0)) for n in self.right_hand_joints]

            cmd_body = [float(self._cmd_body.get(n, self._body_pos.get(n, 0.0))) for n in BODY_JOINTS]
            cmd_left = [float(self._cmd_left.get(n, self._left_pos.get(n, 0.0))) for n in self.left_hand_joints]
            cmd_right = [float(self._cmd_right.get(n, self._right_pos.get(n, 0.0))) for n in self.right_hand_joints]

            payload: dict[str, Any] = {
                "timestamp": time.time(),
                "joint_position": body,
                "joint_velocity": body_vel,
                "joint_effort": [0.0] * len(BODY_JOINTS),
                "hand_left_position": left,
                # 厂商灵巧手状态只有位置，速度/力矩留 0（真机同样没有）
                "hand_left_velocity": [0.0] * len(self.left_hand_joints),
                "hand_left_effort": [0.0] * len(self.left_hand_joints),
                "hand_right_position": right,
                "hand_right_velocity": [0.0] * len(self.right_hand_joints),
                "hand_right_effort": [0.0] * len(self.right_hand_joints),
                "action_joint_position": cmd_body,
                "action_hand_left_position": cmd_left,
                "action_hand_right_position": cmd_right,
            }
            # 相机名与 walker_s2 / tienkung_pro 对齐：color_images/camera_head、
            # depth_images/camera_head（话题 /ob_camera_head/*）
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
            # robot_type 是 playback/robots.detect_robot_type 的首选字段（与 walker_s2 同名），
            # robot 是旧字段，保留给历史脚本
            "robot_type": "tienkung_dex",
            "robot": "tienkung_dex",
            "hand_variant": self.variant,
            "fps": float(self.capture_hz),
            "camera_names": json.dumps([CAMERA_NAME]),
            "created_at": self.get_clock().now().nanoseconds / 1e9,
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
