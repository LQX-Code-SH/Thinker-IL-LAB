#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 关节空间键盘遥操作。

所有下发都走 :class:`dex_controller.DexController`，因此与真机/仿真无关——
仿真和真机只有 ``ROS_DOMAIN_ID`` 不同（仿真 146 / 真机 0）。

首帧从当前实测姿态初始化目标，之后只做增量修改，避免"一上来就跳回零位"。

键位::

    1 左臂   2 右臂   3 左手   4 右手   5 头/腰   6 腿
    [ / ]   切换选中关节
    w / s   目标角 +step / -step（rad）
    o / c   选中手部张开 / 闭合（预备位手型：拇指侧摆保持 90°）
    r       回到初始姿态
    q       退出

用法::

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=146          # 仿真；真机改成 0
    /usr/bin/python3 teleoperation/control/tienkung_dex/keyboard_joint_control.py --step 0.02
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import constants as C  # noqa: E402
from dex_controller import DexController, _shutdown  # noqa: E402

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
PUBLISH_HZ = 30.0


def build_groups(variant: str) -> dict[str, tuple[str, list[str]]]:
    """按手部变体组装分组表（手部关节数 inspire 18 / brainco 11）。"""
    left, right = C.hand_joints(variant)
    groups = dict(GROUPS_BASE)
    groups["3"] = ("左手", left)
    groups["4"] = ("右手", right)
    return dict(sorted(groups.items()))


class TienkungDexTeleop:
    """键盘增量遥操作：维护一份绝对目标，按 30 Hz 交给 DexController 下发。"""

    def __init__(self, controller: DexController, step: float = DEFAULT_STEP):
        self.controller = controller
        self.step = float(step)
        self.groups = build_groups(controller.variant)
        self.group_key = "1"
        self.joint_index = 0
        self.running = True
        self.targets: dict[str, float] = {}
        self._lock = threading.Lock()
        self._pending: list[str] = []

        self._sync_from_robot()

        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()
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
            self.running = False
            return False
        try:
            if key.char in ("q", "Q"):
                self.running = False
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
                self._sync_from_robot()
                print("[INFO] 目标已同步回当前姿态")

    def _current_joint(self) -> str:
        return self.groups[self.group_key][1][self.joint_index]

    def _print_selection(self) -> None:
        name, joints = self.groups[self.group_key]
        joint = joints[self.joint_index]
        print(f"[INFO] 分组={name} ({self.group_key}) 关节={joint} "
              f"[{self.joint_index + 1}/{len(joints)}] 目标={self.targets.get(joint, 0.0):+.3f}")

    def _adjust(self, delta: float) -> None:
        name = self._current_joint()
        self.targets[name] = self.targets.get(name, 0.0) + delta
        print(f"[INFO] {name} -> {self.targets[name]:+.3f}")

    def _set_hand(self, open_hand: bool) -> None:
        if self.group_key not in ("3", "4"):
            print("[WARN] 仅在手部分组（3/4）下支持 o/c")
            return
        side = "left" if self.group_key == "3" else "right"
        # 保持预备位手型（拇指侧摆 850 = 90°）：张开不是全伸直，闭合只弯其余自由度。
        base = C.ready_hand_pose(self.controller.variant)
        closed = C.hand_closed_pose(self.controller.variant, base_pose=base)
        for name in self.groups[self.group_key][1]:
            self.targets[name] = base[name] if open_hand else closed[name]
        print(f"[INFO] {'张开' if open_hand else '闭合'} {side}（保持预备位手型）")

    # ---------------------------------------------------------------- state
    def _sync_from_robot(self) -> None:
        """把当前实测姿态填进目标（未收到读数的关节保持原值）。"""
        positions = self.controller.get_current_positions()
        for name, value in positions.items():
            self.targets[name] = float(value)
        for side in ("left", "right"):
            for name, value in self.controller.get_hand_positions(side).items():
                self.targets[name] = float(value)

    # -------------------------------------------------------------- publish
    def publish_loop(self) -> None:
        period = 1.0 / PUBLISH_HZ
        while self.running and rclpy.ok():
            self._handle_pending()
            body = {name: value for name, value in self.targets.items() if name in C.BODY_JOINTS}
            if body:
                self.controller.send_body_position(body)
            for side, key in (("left", "3"), ("right", "4")):
                hand = {name: self.targets[name] for name in self.groups[key][1] if name in self.targets}
                if hand:
                    self.controller.send_hand_position(side, hand)
            time.sleep(period)

    def stop(self) -> None:
        self.running = False
        try:
            self._listener.stop()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description="TienKung Dex joint-space keyboard teleop")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP, help="单次按键增量（rad）")
    parser.add_argument("--variant", default=C.DEFAULT_VARIANT, choices=["inspire", "brainco"], help="手部变体")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    controller = DexController(
        node_name="tienkung_dex_teleop",
        variant=args.variant,
        lock_joints=[],          # 键盘遥操作由人显式选择关节，不做默认锁腿
    )
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        controller.wait_for_state(timeout=5.0)
        teleop = TienkungDexTeleop(controller, step=args.step)
        teleop.publish_loop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            teleop.stop()
        except NameError:
            pass
        _shutdown(controller, executor, spin_thread)
        print("[INFO] teleop 退出")
    return 0


if __name__ == "__main__":
    sys.exit(main())
