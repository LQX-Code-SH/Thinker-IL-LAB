# -*- coding: utf-8 -*-
"""Walker S2 数据集回放适配层：HDF5 schema + ROS2 发布器 + 首帧对齐。

动作格式（recorder.py 录制，回放直发原始记录值）：
- 身体 17 关节 RobotCommand(MODE_POSITION)      → /mc/sdk/robot_command
- V4 手 7+7 JointCommand(mode=5)                → /mc/{left,right}_hand/command
- 二指夹爪 2×GripCmd(开口 m)                     → /ecat/{left,right}_grip/cmd

仿真桥接只转发 MODE_POSITION 且身体限 100Hz（15Hz 直发安全）；真机运动控制器
直接订阅同名话题。ROS_DOMAIN_ID：仿真 146 / 真机 0（以容器内实际为准）。

依赖（仅控制模式、运行期懒导入）—— 运行前：
    source /opt/ros/humble/setup.bash
    source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
用 /usr/bin/python3（ROS2 Py 3.10），勿用 Isaac Sim Python。
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import threading
import time

import h5py
import numpy as np

from ..common import (
    CameraStream,
    CurveGroup,
    DATASET_ROOT,
    Episode,
    clamp_array,
    derive_fps,
    format_vector,
    read_hdf5_array,
)

# ── teleoperation 侧常量：importlib 以唯一模块名加载，避免与天工 constants 撞名 ──
_CONST_PATH = DATASET_ROOT.parent / "teleoperation" / "control" / "walker_s2" / "utils" / "constants.py"


def _load_constants():
    spec = importlib.util.spec_from_file_location("playback_walker_s2_constants", _CONST_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CONST = _load_constants()
_HAND_LIMITS_SHORT = {  # V4 手限位键为短名（去掉 left_/right_ 前缀）
    name.removeprefix("left_").removeprefix("right_"): bounds
    for name, bounds in CONST.V4_HAND_JOINT_LIMITS.items()
}


class WalkerS2Adapter:
    """Walker S2 适配器：17 身体关节 + 7+7 V4 手 + 2 夹爪 + 4 相机。"""

    robot_type = "walker_s2"
    supports_interp = True

    @staticmethod
    def add_cli_args(p) -> None:
        p.add_argument("--interp", action="store_true",
                       help="在记录帧之间线性插值到 --rate（更平滑；默认直发原始记录值）")

    @staticmethod
    def load(path: str) -> Episode:
        with h5py.File(path, "r") as f:
            attrs = {k: v for k, v in f.attrs.items()}
            names = None
            if "joint_names" in attrs:
                try:
                    names = list(json.loads(attrs["joint_names"]))
                except (TypeError, ValueError, json.JSONDecodeError):
                    names = None
            if not names:
                names = list(CONST.BODY_JOINT_NAMES)
            if names != list(CONST.BODY_JOINT_NAMES):
                print("[playback] 警告: HDF5 joint_names 与 BODY_JOINT_NAMES 顺序不一致，按 HDF5 记录顺序")

            body = read_hdf5_array(f, "action/joint_state/position")
            hand_l = read_hdf5_array(f, "action/hand_left_position")
            hand_r = read_hdf5_array(f, "action/hand_right_position")
            grip_l = read_hdf5_array(f, "action/grip_left_position")
            grip_r = read_hdf5_array(f, "action/grip_right_position")
            n = len(body)

            groups = [
                CurveGroup("action/body", names, body),
                CurveGroup("action/hand_left", list(CONST.V4_HAND_LEFT_JOINTS), hand_l),
                CurveGroup("action/hand_right", list(CONST.V4_HAND_RIGHT_JOINTS), hand_r),
                CurveGroup("action/grip_left", ["opening_m"], grip_l),
                CurveGroup("action/grip_right", ["opening_m"], grip_r),
                CurveGroup("observation/body", names,
                           read_hdf5_array(f, "observation/joint_state/position")),
                CurveGroup("observation/hand_left", list(CONST.V4_HAND_LEFT_JOINTS),
                           read_hdf5_array(f, "observation/hand_left_position")),
                CurveGroup("observation/hand_right", list(CONST.V4_HAND_RIGHT_JOINTS),
                           read_hdf5_array(f, "observation/hand_right_position")),
                CurveGroup("observation/grip_left", ["opening_m"],
                           read_hdf5_array(f, "observation/grip_left_position")),
                CurveGroup("observation/grip_right", ["opening_m"],
                           read_hdf5_array(f, "observation/grip_right_position")),
            ]

            ts = None
            try:
                ts = read_hdf5_array(f, "observation/timestamp")
            except KeyError:
                pass

            cameras = {}
            for cam in ("stereo_left", "stereo_right", "wrist_left", "wrist_right"):
                try:
                    color = f["camera_observations/color_images"][cam][()]
                except KeyError:
                    color = None
                cameras[cam] = CameraStream(color=color)
            # 可选深度（录制时未配置 depth_camera 则不存在）
            try:
                depth = f["camera_observations/depth_images"]["camera_head"][()]
                cameras["camera_head"] = CameraStream(depth=depth)
            except KeyError:
                pass

            fps = float(attrs["fps"]) if attrs.get("fps") else derive_fps(ts)
            return Episode(str(path), "walker_s2", n, fps, ts, groups, cameras, attrs)

    @staticmethod
    def clamp(episode: Episode, args) -> Episode:
        """限位裁剪（身体/手/夹爪）+ NaN/inf 守卫。返回新 Episode。"""
        groups = []
        for g in episode.curve_groups:
            data = g.data
            if not np.isfinite(np.asarray(data, dtype=float)).all():
                raise ValueError(f"{g.label} 含 NaN/inf，拒绝回放")
            count = 0
            if g.label in ("action/body", "observation/body"):
                data, count = clamp_array(data, g.channel_names, CONST.BODY_JOINT_LIMITS)
            elif "hand" in g.label:
                data, count = clamp_array(data, g.channel_names, _HAND_LIMITS_SHORT)
            elif "grip" in g.label:
                data, count = clamp_array(
                    data, g.channel_names,
                    {"opening_m": (CONST.GRIP_OPENING_MIN_M, CONST.GRIP_OPENING_MAX_M)},
                )
            if count:
                print(f"[control] {g.label}: {count} 个值超限已裁剪")
            groups.append(CurveGroup(g.label, g.channel_names, data))
        return Episode(episode.path, episode.robot_type, episode.num_frames,
                       episode.fps, episode.timestamps, groups, episode.cameras,
                       episode.attrs)

    @staticmethod
    def frame_at(episode: Episode, i: int) -> dict:
        return {
            "body": episode.group("action/body").data[i],
            "hand_l": episode.group("action/hand_left").data[i],
            "hand_r": episode.group("action/hand_right").data[i],
            "grip_l": episode.group("action/grip_left").data[i],
            "grip_r": episode.group("action/grip_right").data[i],
        }

    @staticmethod
    def describe_frame(frame: dict) -> str:
        return (f"body={format_vector(frame['body'])} "
                f"hand_l={format_vector(frame['hand_l'])} "
                f"hand_r={format_vector(frame['hand_r'])} "
                f"grip_l={frame['grip_l']:.4f} grip_r={frame['grip_r']:.4f}")

    @staticmethod
    def make_publisher(node, args):
        return WalkerS2Publisher(node)

    @staticmethod
    def align_first(episode: Episode, publisher, args) -> None:
        """把机器人平滑移动到首帧位姿（quintic 插值，复用 WalkerS2Controller）。

        注意 lock_joints=[]：17 关节（含 head/waist）全部参与对齐。
        """
        from rclpy.executors import MultiThreadedExecutor

        # 懒导入 utils.controller（模块级 import rclpy + 消息，仅容器内可用）
        _utils_parent = str(_CONST_PATH.parent.parent)  # control/walker_s2
        if _utils_parent not in sys.path:
            sys.path.insert(0, _utils_parent)
        from utils.controller import WalkerS2Controller

        first = WalkerS2Adapter.frame_at(episode, 0)["body"]
        print("[control] 对齐首帧位姿（quintic 插值，3s）...")
        controller = WalkerS2Controller(
            node_name="playback_align",
            lock_joints=[],  # 全部 17 关节（含 head/waist）参与对齐
            subscribe_images=False,
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(controller)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            if not controller.wait_for_state(timeout=5.0):
                print("[control] 警告: 5s 内未收到机器人状态，跳过对齐"
                      "（move_to_position 需要当前位姿）")
                return
            if not controller.move_to_position(first, duration_sec=3.0, wait=True):
                print("[control] 警告: 对齐失败，继续回放（限位/安全裁剪仍生效）")
        finally:
            executor.shutdown()
            controller.destroy_node()


class WalkerS2Publisher:
    """按录制格式直发：RobotCommand(17 关节 MODE_POSITION) + 双手 JointCommand(mode=5)
    + 双 GripCmd。与 utils/controller.py 的 _control_callback / _publish_hand_cmd /
    send_grip_command 逐字段镜像；仿真桥接只接受 MODE_POSITION。

    每帧 5 条消息错峰发送：旧版桥接/仿真 ZMQ HWM=1 会把 1ms 内连发的第 2-5 条
    全部丢弃（仿真按物理步 ~10ms 批量 drain），夹爪台阶指令偶发丢失。
    间隔 13ms > drain 周期；帧周期放不下时（--rate 过高）不间隔，
    由提升后的 HWM=10（桥接 + 仿真侧）兜底。
    """

    _STAGGER_S = 0.013

    def __init__(self, node):
        # 懒导入：仅控制模式需要 ROS2 环境
        from ecat_task_msgs.msg import GripCmd
        from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        self._node = node
        self._JointCmd = JointCmd
        self._RobotCommand = RobotCommand
        self._JointCommand = JointCommand
        self._GripCmd = GripCmd
        self.frame_period = 0.1  # run_control 会按实际 1/rate 覆盖

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.body_pub = node.create_publisher(
            self._RobotCommand, CONST.DEFAULT_COMMAND_TOPIC, qos_pub
        )
        self.hand_pubs = {
            "left": node.create_publisher(
                self._JointCommand, CONST.DEFAULT_LEFT_HAND_COMMAND_TOPIC, qos_pub
            ),
            "right": node.create_publisher(
                self._JointCommand, CONST.DEFAULT_RIGHT_HAND_COMMAND_TOPIC, qos_pub
            ),
        }
        self.grip_pubs = {
            "left": node.create_publisher(
                self._GripCmd, CONST.DEFAULT_LEFT_GRIP_COMMAND_TOPIC, qos_pub
            ),
            "right": node.create_publisher(
                self._GripCmd, CONST.DEFAULT_RIGHT_GRIP_COMMAND_TOPIC, qos_pub
            ),
        }

    def publish_frame(self, frame: dict) -> None:
        # 4 个间隔（body 后每发一条前 sleep 一次）共 4×_STAGGER_S，需放得进帧周期
        gap = self._STAGGER_S if self.frame_period > 4.5 * self._STAGGER_S else 0.0
        self._publish_body(frame)
        for side, short in (("left", "l"), ("right", "r")):
            if gap:
                time.sleep(gap)
            self._publish_hand(side, frame[f"hand_{short}"])
            if gap:
                time.sleep(gap)
            self._publish_grip(side, frame[f"grip_{short}"])

    def _publish_body(self, frame: dict) -> None:
        now = self._node.get_clock().now().to_msg()

        # 身体：17 关节全部发布（录制时含 head/waist，回放不做锁关节过滤）
        cmd = self._RobotCommand()
        cmd.header.stamp = now
        cmd.header.frame_id = ""
        for name, pos in zip(CONST.BODY_JOINT_NAMES, frame["body"]):
            jc = self._JointCmd()
            jc.name = name
            jc.control_mode = self._JointCmd.MODE_POSITION
            jc.position = float(pos)
            cmd.joint_cmd.append(jc)
        self.body_pub.publish(cmd)

    def _publish_hand(self, side: str, positions) -> None:
        now = self._node.get_clock().now().to_msg()

        h = self._JointCommand()
        h.header.stamp = now
        h.header.frame_id = ""
        h.names = list(CONST.V4_HAND_JOINT_MAP[side])
        h.position = [float(v) for v in positions]
        # mode=5：手部控制器自定义模式（不是 JointCommand.POSITION_MODE）
        h.mode = [5] * len(h.names)
        self.hand_pubs[side].publish(h)

    def _publish_grip(self, side: str, opening: float) -> None:
        now = self._node.get_clock().now().to_msg()

        g = self._GripCmd()
        g.header.stamp = now
        g.init = 1
        g.mode = 0
        g.stop = 0
        g.reset = 0
        g.homing = 0
        g.pos = float(opening)
        g.vel = float(CONST.GRIP_DEFAULT_VEL)
        g.force = float(CONST.GRIP_DEFAULT_FORCE)
        g.cur = 0.0
        self.grip_pubs[side].publish(g)
