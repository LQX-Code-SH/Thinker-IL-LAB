#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 关节空间键盘遥操作。

通过 ``/tienkung_dex/*_command``（``sensor_msgs/JointState``）下发目标角，
经 ROS2-ZMQ 桥接送到仿真。首帧从 ``/tienkung_dex/*_state`` 同步当前姿态，
之后只做增量修改，避免"一上来就跳回零位"。

键位::

    1 左臂   2 右臂   3 左手   4 右手   5 头/腰   6 腿
    [ / ]   切换选中关节
    w / s   目标角 +step / -step（rad）
    o / c   选中手部张开 / 闭合
    r       回到初始姿态
    q       退出

用法::

    source /opt/ros/humble/setup.bash
    /usr/bin/python3 teleoperation/control/tienkung_dex/teleop_controller.py --step 0.02
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from constants import BODY_JOINTS, DEFAULT_VARIANT, hand_joints  # noqa: E402

try:
    from pynput import keyboard
except Exception as exc:  # noqa: BLE001
    print(f"[ERROR] 需要 pynput（{exc}），请 pip install pynput，或在有 DISPLAY 的终端运行")
    sys.exit(1)

GROUPS_BASE: dict[str, tuple[str, list[str]]] = {
    "1": ("左臂", ["shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
                   "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint"]),
    "2": ("右臂", ["shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
                   "elbow_pitch_r_joint", "elbow_yaw_r_joint", "wrist_pitch_r_joint", "wrist_roll_r_joint"]),
    "5": ("头/腰", ["head_yaw_joint", "head_pitch_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]),
    "6": ("腿", ["hip_pitch_l_joint", "hip_roll_l_joint", "hip_yaw_l_joint",
                 "knee_pitch_l_joint", "ankle_pitch_l_joint", "ankle_roll_l_joint",
                 "hip_pitch_r_joint", "hip_roll_r_joint", "hip_yaw_r_joint",
                 "knee_pitch_r_joint", "ankle_pitch_r_joint", "ankle_roll_r_joint"]),
}

DEFAULT_STEP = 0.02


def build_groups(variant: str) -> dict[str, tuple[str, list[str]]]:
    """按手部变体组装分组表（手部关节数 inspire 18 / brainco 11）。"""
    left, right = hand_joints(variant)
    groups = dict(GROUPS_BASE)
    groups["3"] = ("左手", left)
    groups["4"] = ("右手", right)
    return dict(sorted(groups.items()))


PUBLISH_HZ = 30.0


class TienkungDexTeleop(Node):
    def __init__(self, step: float = DEFAULT_STEP, variant: str = DEFAULT_VARIANT):
        super().__init__("tienkung_dex_teleop")
        self.step = float(step)
        self.variant = variant
        self.groups = build_groups(variant)
        self.group_key = "1"
        self.joint_index = 0
        grouped_joints = [name for _, joints in self.groups.values() for name in joints]
        self.targets: dict[str, float] = {name: 0.0 for name in grouped_joints + BODY_JOINTS}
        self._synced = False
        self._received_groups: set[str] = set()
        self._lock = threading.Lock()
        self._pending: list[str] = []
        self._running = True

        self.body_pub = self.create_publisher(JointState, "/tienkung_dex/body_command", 10)
        self.left_pub = self.create_publisher(JointState, "/tienkung_dex/left_hand_command", 10)
        self.right_pub = self.create_publisher(JointState, "/tienkung_dex/right_hand_command", 10)

        self.create_subscription(JointState, "/tienkung_dex/body_state", lambda m: self._state_cb(m, "body"), 10)
        self.create_subscription(JointState, "/tienkung_dex/left_hand_state", lambda m: self._state_cb(m, "left"), 10)
        self.create_subscription(JointState, "/tienkung_dex/right_hand_state", lambda m: self._state_cb(m, "right"), 10)

        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)
        self._print_selection()

    # ------------------------------------------------------------- keyboard
    def _on_press(self, key) -> None:
        try:
            char = key.char
        except AttributeError:
            char = None
        with self._lock:
            self._pending.append(char or str(key))

    def _on_release(self, key) -> None:
        if key == keyboard.Key.esc:
            self._running = False
            return False
        try:
            if key.char in ("q", "Q"):
                self._running = False
                return False
        except AttributeError:
            pass
        return True

    def _handle_pending(self) -> None:
        while self._pending:
            char = self._pending.pop(0)
            if char in self.groups:
                self.group_key = char
                self.joint_index = 0
                self._print_selection()
            elif char in ("[", "{"):
                self.joint_index = max(0, self.joint_index - 1)
                self._print_selection()
            elif char in ("]", "}"):
                names = self.groups[self.group_key][1]
                self.joint_index = min(len(names) - 1, self.joint_index + 1)
                self._print_selection()
            elif char in ("w", "W"):
                self._adjust(self.step)
            elif char in ("s", "S"):
                self._adjust(-self.step)
            elif char in ("o", "O"):
                self._set_hand(open_hand=True)
            elif char in ("c", "C"):
                self._set_hand(open_hand=False)
            elif char in ("r", "R"):
                self._reset_targets()

    def _current_joint(self) -> str:
        return self.groups[self.group_key][1][self.joint_index]

    def _print_selection(self) -> None:
        name, joints = self.groups[self.group_key]
        print(f"[INFO] 分组={name} ({self.group_key}) 关节={joints[self.joint_index]} "
              f"[{self.joint_index + 1}/{len(joints)}] 目标={self.targets.get(joints[self.joint_index], 0.0):+.3f}")

    def _adjust(self, delta: float) -> None:
        name = self._current_joint()
        self.targets[name] = self.targets.get(name, 0.0) + delta
        print(f"[INFO] {name} -> {self.targets[name]:+.3f}")

    def _set_hand(self, open_hand: bool) -> None:
        _, joints = self.groups[self.group_key]
        if self.group_key not in ("3", "4"):
            print("[WARN] 仅在手部分组（3/4）下支持 o/c")
            return
        for name in joints:
            # 张开趋近 0，闭合趋近正向限位（约 1.0~1.3 rad，具体由仿真限位截断）
            self.targets[name] = 0.0 if open_hand else 1.0
        print(f"[INFO] {'张开' if open_hand else '闭合'} {self.groups[self.group_key][0]}")

    def _reset_targets(self) -> None:
        for name in self.targets:
            self.targets[name] = 0.0
        print("[INFO] 目标已归零")

    # ---------------------------------------------------------------- state
    def _state_cb(self, msg: JointState, group: str) -> None:
        if self._synced:
            return
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if name in self.targets:
                    self.targets[name] = float(position)
            self._received_groups.add(group)
            if self._received_groups >= {"body", "left", "right"}:
                self._synced = True
                print("[INFO] 已同步仿真当前姿态作为初始目标")

    def _publish_cb(self) -> None:
        if not self._running:
            return
        self._handle_pending()

        stamp = self.get_clock().now().to_msg()
        for names, pub in (
            (BODY_JOINTS, self.body_pub),
            (self.groups["3"][1], self.left_pub),
            (self.groups["4"][1], self.right_pub),
        ):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(names)
            msg.position = [float(self.targets.get(name, 0.0)) for name in names]
            pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(description="TienKung Dex joint-space keyboard teleop")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP, help="单次按键增量（rad）")
    parser.add_argument("--variant", default=DEFAULT_VARIANT, choices=["inspire", "brainco"], help="手部变体")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TienkungDexTeleop(step=args.step, variant=args.variant)
    try:
        while rclpy.ok() and node._running:
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[INFO] teleop 退出")
    return 0


if __name__ == "__main__":
    sys.exit(main())
