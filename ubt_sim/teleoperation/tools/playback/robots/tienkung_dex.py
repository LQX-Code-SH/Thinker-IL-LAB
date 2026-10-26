# -*- coding: utf-8 -*-
"""TienKung Dex 数据集回放适配层。

数据由 ``teleoperation/control/tienkung_dex/utils/recorder.py`` 录制：

* ``joint_observations/*``：31 个身体关节
* ``hand_observations/*``：左右手各 18 关节
* ``action_observations/*``：下发目标（回放使用）
* ``camera_observations/{images,depth_images}/head_*``：PNG 字节流 / uint16 毫米

回放直接向 ``/tienkung_dex/{body,left_hand,right_hand}_command`` 发布 ``JointState``。
"""

from __future__ import annotations

import h5py
import numpy as np

from ..common import CameraStream, CurveGroup, Episode, derive_fps, read_hdf5_array


class TienkungDexAdapter:
    robot_type = "tienkung_dex"
    supports_interp = True

    @staticmethod
    def add_cli_args(p) -> None:
        p.add_argument("--no-ramp", action="store_true", help="跳过首帧对齐，直接按数据回放")

    @staticmethod
    def load(path: str) -> Episode:
        with h5py.File(path, "r") as f:
            attrs = {k: f.attrs[k] for k in f.attrs}
            body_names = list(attrs.get("body_joint_names", []))
            left_names = list(attrs.get("left_hand_joint_names", []))
            right_names = list(attrs.get("right_hand_joint_names", []))

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
            try:
                color = f["camera_observations/images"]["head_rgb"][()]
                depth = f["camera_observations/depth_images"]["head_depth"][()]
                cameras["head"] = CameraStream(color=color, depth=depth)
            except KeyError:
                pass

            return Episode(str(path), "tienkung_dex", n, derive_fps(timestamps), timestamps, groups, cameras, attrs)

    @staticmethod
    def clamp(episode: Episode, args) -> Episode:
        """仿真侧已有软限位，这里只做 NaN/inf 守卫。"""
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
        return TienkungDexPublisher(node)

    @staticmethod
    def align_first(episode: Episode, publisher, args) -> None:
        """发布首帧目标，让仿真先到录制起点（关节位置驱动会自行平滑）。"""
        if getattr(args, "no_ramp", False):
            return
        if episode.num_frames == 0:
            return
        publisher.publish_frame(TienkungDexAdapter.frame_at(episode, 0))


class TienkungDexPublisher:
    """向桥接的话题发布关节目标。"""

    def __init__(self, node):
        from sensor_msgs.msg import JointState

        self._JointState = JointState
        self.pubs = {
            "body": node.create_publisher(JointState, "/tienkung_dex/body_command", 10),
            "left_hand": node.create_publisher(JointState, "/tienkung_dex/left_hand_command", 10),
            "right_hand": node.create_publisher(JointState, "/tienkung_dex/right_hand_command", 10),
        }
    def publish_frame(self, frame: dict) -> None:
        for key, value_key, names_key in (
            ("body", "body", "body_names"),
            ("left_hand", "hand_l", "left_hand_names"),
            ("right_hand", "hand_r", "right_hand_names"),
        ):
            positions = np.atleast_1d(frame[value_key])
            names = list(frame.get(names_key) or [])
            msg = self._JointState()
            msg.name = names if len(names) == len(positions) else [str(i) for i in range(len(positions))]
            msg.position = [float(v) for v in positions]
            self.pubs[key].publish(msg)
