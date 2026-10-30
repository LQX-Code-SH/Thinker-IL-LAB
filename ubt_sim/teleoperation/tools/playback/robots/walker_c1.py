# -*- coding: utf-8 -*-
"""Walker C1 数据集回放适配层。

数据由 ``teleoperation/control/walker_c1/utils/recorder.py`` 录制：

* ``joint_observations/*``：31 个身体关节（臂 14 + 头 2 + 腰 3 + 腿 12）
* ``hand_observations/*``：左右手各 6 个 SDK 压缩通道（与真机一致）
* ``action_observations/*``：下发目标（回放使用）
* ``camera_observations/{color_images,depth_images}/camera_head``：PNG 字节流 / uint16 毫米

回放直接向真机同款话题发布（``/mc/sdk/robot_command`` + ``/mc/{left,right}_hand/command``），
仿真与真机只差 ``ROS_DOMAIN_ID``（仿真 146 / 真机 0）。
"""

from __future__ import annotations

import h5py
import numpy as np

from ..common import CameraStream, CurveGroup, Episode, derive_fps, read_hdf5_array

# 关节名一律取自 HDF5 attrs（见 load()），因此这里**不**向 sys.path 注入
# control/walker_c1 —— 否则会遮蔽其它适配层懒加载的同名 `constants` 模块。


class WalkerC1Adapter:
    robot_type = "walker_c1"
    supports_interp = True

    @staticmethod
    def add_cli_args(p) -> None:
        p.add_argument("--no-ramp", action="store_true", help="跳过首帧对齐，直接按数据回放")

    @staticmethod
    def load(path: str) -> Episode:
        with h5py.File(path, "r") as f:
            attrs = {k: f.attrs[k] for k in f.attrs}
            body_names = [str(n) for n in attrs.get("body_joint_names", [])]
            left_names = [str(n) for n in attrs.get("left_hand_joint_names", [])]
            right_names = [str(n) for n in attrs.get("right_hand_joint_names", [])]

            body = read_hdf5_array(f, "joint_observations/joint_position")
            body_vel = read_hdf5_array(f, "joint_observations/joint_velocity")
            left = read_hdf5_array(f, "hand_observations/hand_left_position")
            right = read_hdf5_array(f, "hand_observations/hand_right_position")

            action_body = read_hdf5_array(f, "action_observations/joint_position")
            action_left = read_hdf5_array(f, "action_observations/hand_left_position")
            action_right = read_hdf5_array(f, "action_observations/hand_right_position")

            n = int(body.shape[0])
            body_names = body_names or [f"body_{i}" for i in range(body.shape[1])]
            left_names = left_names or [f"left_hand_{i}" for i in range(left.shape[1])]
            right_names = right_names or [f"right_hand_{i}" for i in range(right.shape[1])]

            groups = [
                CurveGroup("action/body", body_names, action_body),
                CurveGroup("action/hand_left", left_names, action_left),
                CurveGroup("action/hand_right", right_names, action_right),
                CurveGroup("state/body", body_names, body),
                CurveGroup("state/body_velocity", body_names, body_vel),
                CurveGroup("state/hand_left", left_names, left),
                CurveGroup("state/hand_right", right_names, right),
            ]

            timestamps = None
            try:
                timestamps = read_hdf5_array(f, "timestamp")
            except KeyError:
                pass

            cameras = {}
            # color 与 depth 分开取：本机位仿真只发 rgb，缺 depth 时不能连 color 一起丢
            color = None
            try:
                color = f["camera_observations/color_images"]["camera_head"][()]
            except KeyError:
                pass
            if color is not None:
                depth = None
                try:
                    depth = f["camera_observations/depth_images"]["camera_head"][()]
                except KeyError:
                    pass
                cameras["head"] = CameraStream(color=color, depth=depth)

            return Episode(
                str(path), "walker_c1", n, derive_fps(timestamps), timestamps, groups, cameras, attrs
            )

    @staticmethod
    def clamp(episode: Episode, args) -> Episode:
        """仿真侧已有 USD 软限位，这里只做 NaN/inf 守卫。"""
        for group in episode.curve_groups:
            if not np.isfinite(np.asarray(group.data, dtype=float)).all():
                raise ValueError(f"{group.label} 含 NaN/inf，拒绝回放")
        return episode

    @staticmethod
    def frame_at(episode: Episode, i: int) -> dict:
        body = episode.group("action/body")
        left = episode.group("action/hand_left")
        right = episode.group("action/hand_right")
        return {
            "body": body.data[i],
            "hand_l": left.data[i],
            "hand_r": right.data[i],
            "body_names": list(body.channel_names),
            "left_hand_names": list(left.channel_names),
            "right_hand_names": list(right.channel_names),
        }

    @staticmethod
    def describe_frame(frame: dict) -> str:
        return (
            f"body(31)={np.array2string(np.asarray(frame['body']), precision=3, max_line_width=120)} "
            f"hand_l={np.array2string(np.asarray(frame['hand_l']), precision=2, max_line_width=80)} "
            f"hand_r={np.array2string(np.asarray(frame['hand_r']), precision=2, max_line_width=80)}"
        )

    @staticmethod
    def make_publisher(node, args):
        return WalkerC1Publisher(node)

    @staticmethod
    def align_first(episode: Episode, publisher, args) -> None:
        """发布首帧目标，让仿真先到录制起点。"""
        if getattr(args, "no_ramp", False):
            return
        if episode.num_frames == 0:
            return
        publisher.publish_frame(WalkerC1Adapter.frame_at(episode, 0))


class WalkerC1Publisher:
    """按真机 SDK 话题发布关节目标。"""

    def __init__(self, node):
        self._node = node
        self._ready = False

    def _ensure(self) -> None:
        if self._ready:
            return
        # 懒导入：--dry-run 时不要求 ROS 环境
        from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand

        self._JointCmd = JointCmd
        self._JointCommand = JointCommand
        self._RobotCommand = RobotCommand
        self._body_pub = self._node.create_publisher(
            RobotCommand, "/mc/sdk/robot_command", 5
        )
        self._left_pub = self._node.create_publisher(
            JointCommand, "/mc/left_hand/command", 5
        )
        self._right_pub = self._node.create_publisher(
            JointCommand, "/mc/right_hand/command", 5
        )
        self._ready = True

    def publish_frame(self, frame: dict) -> None:
        self._ensure()

        stamp = self._node.get_clock().now().to_msg()
        body = np.atleast_1d(frame["body"])
        body_names = list(frame.get("body_names") or [])

        # /mc/sdk/robot_command：JointCmd[]，位置模式
        msg = self._RobotCommand()
        msg.header.stamp = stamp
        for name, pos in zip(body_names, (float(v) for v in body)):
            ctrl = self._JointCmd()
            msg.joint_cmd.append(ctrl)
            ctrl.name = str(name)
            ctrl.control_mode = 2  # JointCmd.MODE_POSITION
            ctrl.position = float(pos)
        if msg.joint_cmd:
            self._body_pub.publish(msg)

        # /mc/{left,right}_hand/command：names + position
        for side, values, names, pub in (
            ("left", frame["hand_l"], frame.get("left_hand_names"), self._left_pub),
            ("right", frame["hand_r"], frame.get("right_hand_names"), self._right_pub),
        ):
            hmsg = self._JointCommand()
            hmsg.header.stamp = stamp
            hmsg.names = [str(n) for n in (names or [])]
            hmsg.position = [float(v) for v in np.atleast_1d(values)]
            pub.publish(hmsg)
