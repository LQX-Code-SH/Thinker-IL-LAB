# -*- coding: utf-8 -*-
"""天工 Pro 数据集回放适配层：HDF5 schema + ROS2 发布器 + 首帧对齐。

动作格式（pick_place_save_data.py 录制，回放直发原始记录值）：
- 双臂 7+7 CmdSetMotorPosition(电机 ID 11-17/21-27) → /arm/cmd_pos
- 双手 6+6 JointState([0,1]) → /inspire_hand/ctrl/{left,right}_hand

录制/控制节奏均为 15Hz。ROS_DOMAIN_ID：仿真 146 / 真机 0（以容器内实际为准）。

依赖（仅控制模式、运行期懒导入）：bodyctrl_msgs / sensor_msgs
（source /opt/ros/humble/setup.bash 后可用；用 /usr/bin/python3）。
"""
from __future__ import annotations

import importlib.util
import os
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

# ── teleoperation 侧常量：importlib 以唯一模块名加载，避免与 walker constants 撞名 ──
_CONST_PATH = DATASET_ROOT.parent / "teleoperation" / "control" / "tienkung_pro" / "constants.py"


def _load_constants():
    spec = importlib.util.spec_from_file_location("playback_tienkung_constants", _CONST_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CONST = _load_constants()


class TienkungProAdapter:
    """天工 Pro 适配器：双臂 7+7 + 双手 6+6 + 1 相机（含深度）。

    注意：teleoperation 侧没有臂关节限位常量，clamp 用保守默认 [-2.5, 2.5] rad
    （采集数据远在范围内），可用 --arm-clamp-lo/hi 覆盖。
    """

    robot_type = "tienkung_pro"
    supports_interp = False  # 电机由 spd/cur 自行平滑，插值无意义

    @staticmethod
    def add_cli_args(p) -> None:
        p.add_argument("--spd", type=float, default=CONST.DEFAULT_MOTOR_SPEED,
                       help="电机默认速度")
        p.add_argument("--cur", type=float, default=CONST.DEFAULT_MOTOR_CURRENT,
                       help="电机默认电流")
        p.add_argument("--arm-clamp-lo", type=float, default=-2.5,
                       help="臂关节限位下界 rad（保守假设）")
        p.add_argument("--arm-clamp-hi", type=float, default=2.5,
                       help="臂关节限位上界 rad（保守假设）")

    @staticmethod
    def load(path: str) -> Episode:
        with h5py.File(path, "r") as f:
            arm_l = read_hdf5_array(f, "action/arm_left_position_align")
            arm_r = read_hdf5_array(f, "action/arm_right_position_align")
            hand_l = read_hdf5_array(f, "action/end_effector_left_position_align")
            hand_r = read_hdf5_array(f, "action/end_effector_right_position_align")
            n = len(arm_l)

            arm_l_names = [CONST.ID_TO_NAME[i] for i in CONST.ID_ARM_L]
            arm_r_names = [CONST.ID_TO_NAME[i] for i in CONST.ID_ARM_R]
            hand_names = [str(i) for i in range(1, 7)]

            groups = [
                CurveGroup("action/arm_left", arm_l_names, arm_l),
                CurveGroup("action/arm_right", arm_r_names, arm_r),
                CurveGroup("action/hand_left", hand_names, hand_l),
                CurveGroup("action/hand_right", hand_names, hand_r),
                CurveGroup("puppet/arm_left", arm_l_names,
                           read_hdf5_array(f, "puppet/arm_left_position_align")),
                CurveGroup("puppet/arm_right", arm_r_names,
                           read_hdf5_array(f, "puppet/arm_right_position_align")),
                CurveGroup("puppet/hand_left", hand_names,
                           read_hdf5_array(f, "puppet/end_effector_left_position_align")),
                CurveGroup("puppet/hand_right", hand_names,
                           read_hdf5_array(f, "puppet/end_effector_right_position_align")),
            ]

            ts = None
            try:
                ts = read_hdf5_array(f, "observations/timestamp")  # 裸数据集，无 /data 子组
            except KeyError:
                pass

            cameras = {}
            try:
                color = f["camera_observations/color_images"]["camera_head"][()]
            except KeyError:
                color = None
            try:
                depth = f["camera_observations/depth_images"]["camera_head"][()]
            except KeyError:
                depth = None
            if color is not None or depth is not None:
                cameras["camera_head"] = CameraStream(color=color, depth=depth)

            # 无任何根属性（无 fps/robot_type），fps 由时间戳推导
            return Episode(str(path), "tienkung_pro", n, derive_fps(ts),
                           ts, groups, cameras, {})

    @staticmethod
    def clamp(episode: Episode, args) -> Episode:
        """限位裁剪（手 [0,1]、臂保守限位）+ NaN/inf 守卫。返回新 Episode。"""
        groups = []
        for g in episode.curve_groups:
            data = g.data
            if not np.isfinite(np.asarray(data, dtype=float)).all():
                raise ValueError(f"{g.label} 含 NaN/inf，拒绝回放")
            count = 0
            if "hand" in g.label:
                data, count = clamp_array(data, g.channel_names,
                                          {n: (0.0, 1.0) for n in g.channel_names})
            elif "arm" in g.label:
                data, count = clamp_array(
                    data, g.channel_names,
                    {n: (args.arm_clamp_lo, args.arm_clamp_hi) for n in g.channel_names},
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
            "arm_l": episode.group("action/arm_left").data[i],
            "arm_r": episode.group("action/arm_right").data[i],
            "hand_l": episode.group("action/hand_left").data[i],
            "hand_r": episode.group("action/hand_right").data[i],
        }

    @staticmethod
    def describe_frame(frame: dict) -> str:
        return (f"arm_l={format_vector(frame['arm_l'])} "
                f"arm_r={format_vector(frame['arm_r'])} "
                f"hand_l={format_vector(frame['hand_l'])} "
                f"hand_r={format_vector(frame['hand_r'])}")

    @staticmethod
    def make_publisher(node, args):
        return TienkungProPublisher(node, spd=args.spd, cur=args.cur)

    @staticmethod
    def align_first(episode: Episode, publisher, args) -> None:
        """关节空间直线 ramp：ARM_HOME_PICK_PLACE → 首帧位姿（臂+手），15Hz。

        不用 IK move_arm：目标是"回到录制位姿"，关节空间直插即可；
        臂每步 ≤0.1 rad（上限 ~2s），手从 HAND_OPEN ramp。
        """
        f0 = TienkungProAdapter.frame_at(episode, 0)
        target_arm = np.concatenate(
            [np.atleast_1d(f0["arm_l"]), np.atleast_1d(f0["arm_r"])]
        )
        home_arm = np.asarray(CONST.ARM_HOME_PICK_PLACE, dtype=float)
        max_delta = float(np.max(np.abs(target_arm - home_arm)))
        steps = max(1, int(np.ceil(max_delta / 0.1)))
        steps = min(steps, 30)  # 上限 ~2s @15Hz
        dt = 1.0 / CONST.CONTROL_LOOP_HZ

        print(f"[control] 对齐首帧位姿（{steps} 步关节空间 ramp，~{steps * dt:.1f}s）...")
        for k in range(1, steps + 1):
            cur = home_arm + (target_arm - home_arm) * (k / steps)
            publisher.publish_arms(cur[:7], cur[7:])
            time.sleep(dt)

        hand_steps = 10
        for k in range(1, hand_steps + 1):
            alpha = k / hand_steps
            publisher.publish_hands(
                [float(o + alpha * (t - o)) for o, t in zip(CONST.HAND_OPEN, f0["hand_l"])],
                [float(o + alpha * (t - o)) for o, t in zip(CONST.HAND_OPEN, f0["hand_r"])],
            )
            time.sleep(dt)


class TienkungProPublisher:
    """按录制格式直发：CmdSetMotorPosition(14 电机) + 双手 JointState。
    与 robot_controller.py 的 make_motor_cmd / make_hand_msg / push 镜像。"""

    def __init__(self, node, spd: float = 0.2, cur: float = 5.0):
        from bodyctrl_msgs.msg import CmdSetMotorPosition
        from sensor_msgs.msg import JointState

        self._node = node
        self._spd = float(spd)
        self._cur = float(cur)
        self._CmdSetMotorPosition = CmdSetMotorPosition
        self._SetMotorPosition = None  # 延迟到 publish_arms
        self._JointState = JointState
        self.arm_pub = node.create_publisher(CmdSetMotorPosition, "/arm/cmd_pos", 10)
        self.hand_pubs = {
            "left": node.create_publisher(JointState, "/inspire_hand/ctrl/left_hand", 10),
            "right": node.create_publisher(JointState, "/inspire_hand/ctrl/right_hand", 10),
        }

    def publish_frame(self, frame: dict) -> None:
        self.publish_arms(frame["arm_l"], frame["arm_r"])
        self.publish_hands(frame["hand_l"], frame["hand_r"])

    def publish_arms(self, arm_l, arm_r) -> None:
        if self._SetMotorPosition is None:
            from bodyctrl_msgs.msg import SetMotorPosition
            self._SetMotorPosition = SetMotorPosition
        msg = self._CmdSetMotorPosition()
        for motor_id, val in zip(CONST.ID_ARM_L, arm_l):
            msg.cmds.append(self._SetMotorPosition(
                name=motor_id, pos=float(val), spd=self._spd, cur=self._cur))
        for motor_id, val in zip(CONST.ID_ARM_R, arm_r):
            msg.cmds.append(self._SetMotorPosition(
                name=motor_id, pos=float(val), spd=self._spd, cur=self._cur))
        self.arm_pub.publish(msg)

    def publish_hands(self, hand_l, hand_r) -> None:
        for side, vals in (("left", hand_l), ("right", hand_r)):
            msg = self._JointState()
            msg.name = [str(i) for i in range(1, 7)]
            msg.position = [float(v) for v in vals]
            self.hand_pubs[side].publish(msg)
