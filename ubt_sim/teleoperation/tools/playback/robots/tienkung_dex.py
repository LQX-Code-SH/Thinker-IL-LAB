# -*- coding: utf-8 -*-
"""TienKung Dex 数据集回放适配层。

数据由 ``teleoperation/control/tienkung_dex/utils/recorder.py`` 录制：

* ``joint_observations/*``：31 个身体关节
* ``hand_observations/*``：左右手各 18 关节
* ``action_observations/*``：下发目标（回放使用）
* ``camera_observations/{images,depth_images}/head_*``：PNG 字节流 / uint16 毫米

回放直接向真机同款话题发布（``/arm|head|waist|leg/cmd`` + 灵巧手指令），
仿真与真机只差 ``ROS_DOMAIN_ID``（仿真 146 / 真机 0）。
"""

from __future__ import annotations

import os
import sys

import h5py
import numpy as np

from ..common import CameraStream, CurveGroup, Episode, derive_fps, read_hdf5_array

# 复用 control/tienkung_dex 的关节表与话题定义
_CONTROL_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir, "control", "tienkung_dex")
)
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)


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
    """按真机编址（关节 ID / 手部通道码）发布关节目标。"""

    def __init__(self, node):
        self._node = node
        self._ready = False

    def _ensure(self, left_names: list[str]) -> None:
        """首帧推断手型并建发布器（录制数据的关节表在 attrs 里，构造时还拿不到）。"""
        if self._ready:
            return
        import constants as C
        from dex_controller import GROUP_MSG, TOPICS, load_real_msgs

        self.C = C
        self.variant = "brainco" if len(left_names) == len(C.hand_joints("brainco")[0]) else "inspire"
        self._msgs = load_real_msgs(self.variant)
        self._pubs = {}
        for group in ("arm", "head", "waist", "leg"):
            self._pubs[f"{group}_command"] = self._node.create_publisher(
                self._msgs[GROUP_MSG[group]], TOPICS[f"{group}_command"], 10
            )
        for side in ("left", "right"):
            self._pubs[f"{side}_hand_command"] = self._node.create_publisher(
                self._msgs["hand_command"], TOPICS["hand"][self.variant][side]["command"], 10
            )
        self._ready = True

    def publish_frame(self, frame: dict) -> None:
        body = np.atleast_1d(frame["body"])
        left = np.atleast_1d(frame["hand_l"])
        right = np.atleast_1d(frame["hand_r"])
        body_names = list(frame.get("body_names") or [])
        left_names = list(frame.get("left_hand_names") or [])
        right_names = list(frame.get("right_hand_names") or [])
        self._ensure(left_names)

        C = self.C
        stamp = self._node.get_clock().now().to_msg()
        lookup = dict(zip(body_names, (float(v) for v in body)))
        for group, joint_names in C.GROUP_JOINTS.items():
            msg = self._msgs[GROUP_MSG[group]]()
            msg.header.stamp = stamp
            msg.header.frame_id = group
            msg.mode = C.CONTROL_MODE_POSITION
            limits = C.GROUP_CMD_LIMITS[group]
            for name in joint_names:
                joint_id = C.SIM_TO_JOINT_ID.get(name)
                if joint_id is None or name not in lookup:
                    continue
                ctrl = self._msgs["MotorCtrl"]()
                ctrl.name = int(joint_id)
                ctrl.pos = lookup[name]
                ctrl.spd = float(limits["spd"])
                ctrl.cur = float(limits["cur"])
                msg.ctrl.append(ctrl)
            if msg.ctrl:
                self._pubs[f"{group}_command"].publish(msg)

        for side, names, values in (("left", left_names, left), ("right", right_names, right)):
            values_by_name = dict(zip(names, (float(v) for v in values)))
            codes: list[int] = []
            for channel, joints in enumerate(C.hand_channel_joints(self.variant, side)):
                if not joints:
                    codes.append(0)
                    continue
                joint = joints[0]
                codes.append(
                    C.hand_rad_to_code(
                        self.variant, channel, values_by_name.get(joint, 0.0), joint
                    )
                )

            msg = self._msgs["hand_command"]()
            if self.variant == "brainco":
                count = C.HAND_CHANNEL_COUNT["brainco"]
                msg.mode = 1
                msg.positions = codes[:count]
                msg.speeds = [0] * count
                msg.currents = [0] * count
                msg.pwms = [0] * count
                msg.durations = [0] * count
            else:
                msg.hand_id = 1 if side == "left" else 2
                msg.joint_values = codes
            self._pubs[f"{side}_hand_command"].publish(msg)
