#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tienkung_dex 右臂抓取 parlor 苹果：回放遥操作示教的路径点。

路径真值在 ``grasp_path.json``（2026-10-27 用 ``keyboard_ee_control.py`` 示教，
``side=right``，位姿都是 pelvis 系 ``[x, y, z, roll, pitch, yaw]``）。
**只示教两个点**，后三步由规则推出来（:func:`derive_waypoints`）：

====  ============  =========================================================
点    role          动作
====  ============  =========================================================
1     pre_approach  接近位（手张开）
2     grasp         抓取位，压到苹果上；到位后 **合手**
3     lift          2 号点沿 world z 抬升 ``LIFT_HEIGHT_M``（0.20 m），姿态不变
5     place         放置位，示教；到位后 **张手**
—     ready         关节一步回预备位（``RETURN_READY_SEC`` = 5 s），抓取结束
====  ============  =========================================================

示教三个点就够：**1、2、5**，3 由 2 推出来。编号跳过 4 是历史原因（原先有一个
"放置位正上方"的过渡点，后来省掉了）；抬升高度改 :data:`LIFT_HEIGHT_M` 一个数就行。

1 号点是可选的：执行序列按文件里实际存在的点生成（:func:`build_steps`），
删掉 1 号点就自动少走一步（预备位 → 抓取位一条直线）。2 号点是必需的
（:data:`STOP_REQUIRES`），缺了直接报错、不发任何动作。

两条约定：

1. **回放的位姿是示教时记录的「期望目标变量」（``target``），不是实测读数。**
   遥操作里目标变量是唯一事实源：按键改它、发送器照它下发。回放照抄这个语义——
   ``_goto`` 解一次 IK 就把 ``target`` 下发，**不做在线补偿**，和
   ``keyboard_ee_control`` 一样是开环。执行器（隐式 PD）的稳态下垂由**仿真侧**
   的重力前馈消掉（``ubt_sim/devices/tienkung_dex/gravity_compensation.py``，
   2026-10-27 起默认开启），所以开环下发就能到位；用 ``TIENKUNG_DEX_GRAVITY_COMP=0``
   关掉它，手臂会稳在指令下方（2 号点 72 mm、预备位 133 mm），与旧版现象一致。
   （前馈吃掉 85~92%，剩下的十几毫米残差在预备位已扣进 ``constants.READY_POSE_TARGET``。）
   ``--pose-source measured`` 只给带实测读数的旧路径文件用；旧文件里缺实测值的
   点按"上一个已解析点 + 记录里的目标增量"外推。
2. **3、5 号点是纯绝对位姿，不参考苹果**——苹果这时候已经在手里，跟着手走。
   日志里这三个点也不再报"距苹果心"。

合手/张手沿用示教时的同一套手型：合手传 ``base_pose=ready_hand_pose``（拇指
侧摆留在 850 = 90°，不随 ``grip`` 缩放），张手回预备位手型——与遥操作里按空格
的动作逐条对应。用默认 ``base_pose``（侧摆回 0）合手会在合拢瞬间把拇指甩到
全外展，和示教时捏住苹果的那一下不是同一个手型。

**曾经有过、现已去掉的 PD 稳态误差补偿**：旧版 ``_goto`` 每段量一次实测 TCP，把
残差（73 mm / 17° 量级）叠加到指令后重发，最多 3 次。它的响应太慢——每一次都要
走完一条 ≥3 s 的轨迹再等手臂停稳，一次接近要摆 4 个回合、40 s；而且每段都是
"指令跳到新位置→手臂落后→停稳时再量"的过冲过程，看上去就是手臂上下晃。回放
应当和遥操作一样开环，补偿留给更合适的方案——已经落地为**仿真侧一次标定的重力
前馈**（RNEA 算 τ_g 再折算成位置偏移，见 ``devices/tienkung_dex/gravity_compensation.py``）：
2 号点的末端稳态误差从 72.1 mm 降到 5.9 mm（k=80 时），手臂刚度提到 200/150、
kd 按 ζ≈0.7 配之后是 3.4 mm——已经落在 IK 默认 5 mm 容差（``utils/ik.py`` 的
``pos_tol``，实测贡献 4.7 mm）之内，再往下就得先收紧 IK 容差。

用法::

    export ROS_DOMAIN_ID=146                      # 仿真；真机为 0
    /usr/bin/python3 pick_apple.py --dry-run       # 只解算 5 个点，不动机器人
    /usr/bin/python3 pick_apple.py                 # 回预备位 → 抓 → 放 → 回预备位
    /usr/bin/python3 pick_apple.py --skip-ready    # 已就位，直接开始
    /usr/bin/python3 pick_apple.py --stop-after 2  # 停在抓取位（未合手）
    /usr/bin/python3 pick_apple.py --stop-after grasp   # 只练到合手
    /usr/bin/python3 pick_apple.py --stop-after 5  # 到放置位但先不张手
    /usr/bin/python3 pick_apple.py --no-return-ready    # 放完停在原地
    /usr/bin/python3 pick_apple.py --save          # 顺手录一集抓取段（批量走 save_data.sh --batch）

仿真与真机跑同一份代码，只有 ``ROS_DOMAIN_ID`` 不同。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import MultiThreadedExecutor

_CONTROL_DIR = os.path.dirname(os.path.abspath(__file__))
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

import constants as C  # noqa: E402
from dex_controller import DexController, _shutdown  # noqa: E402
from utils.recorder import (  # noqa: E402
    DEFAULT_CAPTURE_HZ,
    HDF5_REL_PATH,
    TienkungDexDataRecorder,
    episode_path,
    prepare_episode_file,
)

DEFAULT_PATH_FILE = os.path.join(_CONTROL_DIR, "grasp_path.json")
# --save 的默认落点，与 save_data.py 相同：ubt_sim/dataset/tienkung_dex
DEFAULT_SAVE_DIR = Path(_CONTROL_DIR).parents[2] / "dataset" / "tienkung_dex"
DEFAULT_TASK_NAME = "pick_apple"

# 路径点编号的固定含义（见模块 docstring 的表）；JSON 里也写了同名键，这里对一遍。
GRASP_INDEX = 2
LIFT_INDEX = 3
PLACE_INDEX = 5          # 跳号：4 号点（放置位正上方）2026-10-27 已省掉

# 3 号点的推导量：抬多高。2 号点的 z 已经是苹果上方 8.8 cm（见 grasp_path.json
# 的 relative_to_apple），所以抬 0.20 m 后末端离桌面约 0.33 m，再直接去 5 号点。
LIFT_HEIGHT_M = 0.20

# 收尾回预备位：关节一步走完的时长（staged=False）。这是**期望**时长，
# ``DexController._min_duration`` 会按单关节 1.5 rad/s 的速度上限托底，
# 实际可能略长（见运行日志的"预备位阶段 预备位（X.Xs）"）。
# 5 号点 → 预备位最大关节跨度 1.056 rad（wrist_pitch_r），速度托底只要 1.55 s，
# 5 s 远在托底之上；原来是 3 s，实测 3.01 s 太快，2026-10-27 放到 5 s。
RETURN_READY_SEC = 5.0

DEFAULT_GRIP = 1.0          # 合手闭合度 0~1
DEFAULT_CLOSE_SPEED = 0.35  # rad/s
DEFAULT_SETTLE_SEC = 0.8    # 合手后保持秒数（原 1.5，2026-10-27 缩短）
DEFAULT_SPEED = 0.05        # m/s，1~2 号点（接近段）
DEFAULT_LIFT_SPEED = 0.03   # m/s，3 号点（抬升，手里有苹果）
DEFAULT_PLACE_SPEED = 0.03  # m/s，5 号点（下落段）
DEFAULT_ROT_SPEED = 0.30    # rad/s，姿态变化的速度上限；3 沿用抓取姿态，3→5 转一次腕
DEFAULT_SETTLE_TIME = 0.15  # 每段轨迹发完后的最短静置时间 s（之后还要等 TCP 不再动）
MIN_DURATION = 1.5          # s，每段最短时长
SETTLE_WINDOW = 0.15        # s，判定"不再动"的观察窗
SETTLE_THRESH = 0.0008      # m，窗口内 TCP 位移小于此值才算稳定（0.8mm/0.15s ≈ 5.3mm/s）
SETTLE_MAX = 1.0            # s，等待稳定的上限（仿真手臂下沉慢，2 s 也到不了更严的判据）

# --stop-after 名称 -> 在该步之后停下
STOP_AFTER_STEP = {
    "1": ("goto", 1),
    "2": ("goto", GRASP_INDEX),
    "grasp": ("close", None),
    "3": ("goto", LIFT_INDEX),
    "5": ("goto", PLACE_INDEX),
}
STOP_STAGES = tuple(STOP_AFTER_STEP)
# 该档位**必不可少**的、需要示教的点。3 由 2 号点推出来，所以一切都从
# 2 号点起；1 号点（接近位）可以整段不采，脚本直接跳过、少走一步。
STOP_REQUIRES = {
    "1": (1,),
    "2": (GRASP_INDEX,),
    "grasp": (GRASP_INDEX,),
    "3": (GRASP_INDEX,),
    "5": (GRASP_INDEX,),
    None: (GRASP_INDEX,),
}
STOP_TITLES = {
    None: "抓取完成（已张手并回预备位）",
    "1": "停在 1 号点（接近段）",
    "2": "停在 2 号点（抓取位，未合手）",
    "grasp": "合手完成（未抬升）",
    "3": "停在 3 号点（已抬升 20 cm）",
    "5": "停在 5 号点（放置位，未张手）",
}
POSE_SOURCES = ("target", "measured")


def build_steps(indices) -> tuple[tuple[str, int | None], ...]:
    """按路径文件里**实际存在**的点生成执行序列。

    顺序固定为：接近点（<2，升序）→ 2 → 合手 → 抬升/放置点（>2，升序）→ 张手。
    少采的接近点直接不进序列——1 号点删掉后就只剩 ``open → goto 2``，不再有
    "第一段总要经过 1" 的假设。3 号点是 :func:`derive_waypoints` 补的，
    缺 2 号点的报错由 :data:`STOP_REQUIRES` 判，这里只负责把点排成一条可执行的链。
    """
    present = {int(i) for i in indices}
    steps: list[tuple[str, int | None]] = [
        ("goto", i) for i in sorted(i for i in present if 1 <= i < GRASP_INDEX)
    ]
    if GRASP_INDEX in present:
        steps.append(("goto", GRASP_INDEX))
        steps.append(("close", None))
    after = sorted(i for i in present if i > GRASP_INDEX)
    steps.extend(("goto", i) for i in after)
    if PLACE_INDEX in present:  # 走到放置位才张手；只练到 3 号点时手保持合上
        steps.append(("open", None))
    return tuple(steps)


def planned_steps(stop_after: str | None, indices) -> tuple[tuple[str, int | None], ...]:
    """按 ``--stop-after`` 截断执行序列。"""
    steps = build_steps(indices)
    if stop_after is None:
        return steps
    marker = STOP_AFTER_STEP[stop_after]
    if marker not in steps:
        raise ValueError(f"路径里没有 {stop_after!r} 这一档需要的点")
    return steps[: steps.index(marker) + 1]


def required_indices(stop_after: str | None) -> list[int]:
    """执行到该档位**必须**采到的点（接近点可缺，见 STOP_REQUIRES）。"""
    return list(STOP_REQUIRES[stop_after])


@dataclass
class Waypoint:
    """已解析成实际回放目标的路径点。"""

    index: int
    role: str
    pose: np.ndarray     # pelvis 系 [x, y, z, roll, pitch, yaw]
    source: str          # measured / target / derived


def load_grasp_path(path: str) -> dict:
    """读并校验 ``grasp_path.json``；校验失败抛 ``ValueError``。"""
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)

    if data.get("side") not in ("left", "right"):
        raise ValueError(f"{path}: side 必须是 left/right，实际 {data.get('side')!r}")
    items = data.get("waypoints")
    if not isinstance(items, list) or not items:
        raise ValueError(f"{path}: waypoints 为空")

    seen: set[int] = set()
    for item in items:
        try:
            index = int(item["index"])
            xyzrpy = item["target"]["xyzrpy"]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"{path}: 路径点缺少 index/target.xyzrpy: {exc}") from exc
        if index in seen:
            raise ValueError(f"{path}: 路径点编号重复: {index}")
        if len(xyzrpy) != 6 or not np.isfinite(np.asarray(xyzrpy, dtype=float)).all():
            raise ValueError(f"{path}: 路径点 {index} 的 target.xyzrpy 不是 6 个有限数")
        seen.add(index)
    # 完整性不在这里判：采集中途文件可能只有 1、2 号点，能否执行由
    # required_indices(stop_after) 决定（见 main / run）。
    return data


def resolve_waypoints(data: dict, pose_source: str = "target") -> list[Waypoint]:
    """把 JSON 解析成回放目标，按编号排序，再补上推出的 3 号点。

    ``pose_source=target``（默认）：直接回放记录的期望目标变量，不做外推。
    ``pose_source=measured``：只给带实测读数的旧路径文件用——有实测值的点用实测值，
    没有的点沿用上一个已解析点 + 记录里的目标增量外推。

    **文件里只认 1、2、5 号点**：3 一律由 :func:`derive_waypoints` 重算，
    文件里若还留着这两个点的示教值会被忽略。
    """
    if pose_source not in POSE_SOURCES:
        raise ValueError(f"未知 pose_source: {pose_source}")

    resolved: list[Waypoint] = []
    prev_target: np.ndarray | None = None
    prev_pose: np.ndarray | None = None
    ignored: list[int] = []
    for item in sorted(data["waypoints"], key=lambda entry: int(entry["index"])):
        index = int(item["index"])
        if index > GRASP_INDEX and index != PLACE_INDEX:
            ignored.append(index)
            continue
        target = np.asarray(item["target"]["xyzrpy"], dtype=float)
        measured = item.get("measured")

        if pose_source == "target":
            pose, source = target.copy(), "target"
        elif measured is not None:
            pose, source = np.asarray(measured["xyzrpy"], dtype=float), "measured"
        elif prev_pose is not None and prev_target is not None:
            pose, source = prev_pose + (target - prev_target), "derived"
        else:
            pose, source = target.copy(), "target"

        resolved.append(Waypoint(index, str(item.get("role", "")), pose, source))
        prev_target, prev_pose = target, pose

    if ignored:
        print(
            f"注意: 路径文件里的 {ignored} 号点是旧格式的示教值，已被推导规则取代（忽略）",
            file=sys.stderr,
        )
    return derive_waypoints(resolved)


def derive_waypoints(taught: list[Waypoint]) -> list[Waypoint]:
    """补出 3 号点：2 号点（抓取位）沿 world z 抬升 :data:`LIFT_HEIGHT_M`，姿态不变。

    抬起来以后直接去示教的 5 号点（放置位），中间不再有"放置位正上方"的过渡点——
    3→5 是一段带姿态变化的直线，抓着苹果走弧比先摆到正上方再垂直下落更省事。
    """
    grasp = next((wp for wp in taught if wp.index == GRASP_INDEX), None)
    if grasp is None:
        return list(taught)

    lift = grasp.pose.copy()
    lift[2] += LIFT_HEIGHT_M
    return list(taught) + [Waypoint(LIFT_INDEX, "lift", lift, "derived")]


def _rotation(pose) -> np.ndarray:
    rpy = np.asarray(pose, dtype=float)[3:6]
    return pin.rpy.rpyToMatrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))


def _rotation_angle(pose_a, pose_b) -> float:
    """两个位姿之间的姿态夹角（rad）。"""
    return float(np.linalg.norm(pin.log3(_rotation(pose_a).T @ _rotation(pose_b))))


class ApplePicker:
    """按示教路径回放：1 → 2（合手）→ 3（抬升）→ 4（盘子正上方）→ 5（张手）→ 回预备位。"""

    def __init__(
        self,
        ctrl: DexController,
        waypoints: list[Waypoint],
        side: str = "right",
        grip: float = DEFAULT_GRIP,
        speed: float = DEFAULT_SPEED,
        lift_speed: float = DEFAULT_LIFT_SPEED,
        place_speed: float = DEFAULT_PLACE_SPEED,
        rot_speed: float = DEFAULT_ROT_SPEED,
        close_speed: float = DEFAULT_CLOSE_SPEED,
        settle_sec: float = DEFAULT_SETTLE_SEC,
        settle_time: float = DEFAULT_SETTLE_TIME,
        return_ready: bool = True,
    ):
        self.ctrl = ctrl
        self.side = side
        self.waypoints = sorted(waypoints, key=lambda wp: wp.index)
        self.grip = float(grip)
        self.speed = float(speed)
        self.lift_speed = float(lift_speed)
        self.place_speed = float(place_speed)
        self.rot_speed = float(rot_speed)
        self.close_speed = float(close_speed)
        self.settle_sec = float(settle_sec)
        self.settle_time = float(settle_time)
        self.return_ready = bool(return_ready)

        self.apple = np.asarray(C.APPLE_WORLD_POS) - np.asarray(C.ROBOT_WORLD_POS)
        self.plate = np.asarray(C.PLATE_WORLD_POS) - np.asarray(C.ROBOT_WORLD_POS)
        self.table_z = C.TABLETOP_WORLD_Z - C.ROBOT_WORLD_POS[2]

        # 录制钩子：张手放到盘上那一刻回调（参数 = 本集是否成功）。回预备位是
        # 空手离场，不算抓取段，所以钩子在 return_ready 之前触发（--save 用）。
        self.on_place_done: Callable[[bool], None] | None = None

    # ------------------------------------------------------------------ 报告
    def _tcp(self) -> np.ndarray | None:
        pose = self.ctrl.get_ee_pose(self.side)
        return None if pose is None else np.asarray(pose, dtype=float)

    def _target_text(self, xyz, reference_apple: bool) -> str:
        xyz = np.asarray(xyz, dtype=float)
        text = f"目标 ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})  离桌面 {xyz[2] - self.table_z:+.3f} m"
        if reference_apple:
            text += f"  距苹果心 {np.linalg.norm(xyz - self.apple):.3f} m"
        return text

    def _log_target(self, name: str, xyz, reference_apple: bool) -> None:
        self.ctrl.get_logger().info(f"[{name}] " + self._target_text(xyz, reference_apple))

    def log_path(self) -> None:
        self.ctrl.get_logger().info(
            f"路径（pelvis 系，回放目标）：苹果 {np.round(self.apple, 3).tolist()}，"
            f"盘心 {np.round(self.plate, 3).tolist()}"
        )
        for wp in self.waypoints:
            pose = wp.pose
            extra = ""
            if wp.index < GRASP_INDEX:
                extra = f"  距苹果心 {np.linalg.norm(pose[:3] - self.apple):.3f} m"
            elif wp.index == GRASP_INDEX:
                extra = f"  距苹果心 {np.linalg.norm(pose[:3] - self.apple):.3f} m（合手）"
            elif wp.index == PLACE_INDEX:
                extra = f"  距盘心 {np.linalg.norm(pose[:3] - self.plate):.3f} m"
            self.ctrl.get_logger().info(
                f"  {wp.index} [{wp.role:<11}] {wp.source:<8} "
                f"xyz=({pose[0]:+.3f}, {pose[1]:+.3f}, {pose[2]:+.3f}) "
                f"rpy=({pose[3]:+.3f}, {pose[4]:+.3f}, {pose[5]:+.3f})"
                f"  离桌面 {pose[2] - self.table_z:+.3f} m" + extra
            )

    def _waypoint(self, index: int) -> Waypoint:
        for wp in self.waypoints:
            if wp.index == index:
                return wp
        raise KeyError(f"路径里没有 {index} 号点")

    # ------------------------------------------------------------------ 执行
    def _duration(self, target, speed: float) -> float:
        """按平移速度和姿态角速度取较长的一段，避免大角度腕转被压进几秒里。"""
        cur = self._tcp()
        if cur is None:
            return MIN_DURATION
        dist = float(np.linalg.norm(np.asarray(target, dtype=float)[:3] - cur[:3]))
        angle = _rotation_angle(cur, target)
        return max(
            MIN_DURATION,
            dist / max(speed, 1e-3),
            angle / max(self.rot_speed, 1e-3),
        )

    @staticmethod
    def _wrap(err: np.ndarray) -> np.ndarray:
        out = np.array(err, dtype=float)
        out[3:] = (out[3:] + np.pi) % (2.0 * np.pi) - np.pi
        return out

    def _wait_settled(self) -> None:
        """等手臂在重力下垂稳，再走下一段。

        仿真手臂是隐式 PD，轨迹走完后还要 2 s 左右才收敛（实测肘 yaw 偏差
        0.19 → 0.11 → 0.03 rad 用了 2 s，肩 pitch 还会慢慢往下沉）。不等稳就发
        下一段，轨迹会从"还在沉降的中点"起步，段与段之间就会叠出一层层抖动。
        注意这里只等稳、不改指令——回放本身是开环的，见模块 docstring。
        """
        if self.settle_time > 0:
            time.sleep(self.settle_time)
        start = time.time()
        prev = self._tcp()
        if prev is None:
            return
        stable_since = None
        while time.time() - start < SETTLE_MAX:
            time.sleep(SETTLE_WINDOW)
            cur = self._tcp()
            if cur is None:
                return
            moving = (
                float(np.linalg.norm(cur[:3] - prev[:3])) > SETTLE_THRESH
                or float(np.linalg.norm(cur[3:] - prev[3:])) > 0.01
            )
            if moving:
                stable_since = None
            elif stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= SETTLE_WINDOW:
                return
            prev = cur

    def _send(self, name: str, cmd: np.ndarray, duration: float) -> bool:
        """解一次完整的 6 自由度 IK 并下发轨迹（开环，不判到位）。"""
        joints, ok, diag = self.ctrl.solve_arm_ik(self.side, cmd)
        if joints is None:
            self.ctrl.get_logger().error(f"[{name}] IK 无解")
            return False
        if not ok:
            pos_err = float((diag or {}).get("pos_err", float("inf")))
            rot_err = float((diag or {}).get("rot_err", float("inf")))
            if pos_err > 0.005 or rot_err > 0.05:
                self.ctrl.get_logger().error(f"[{name}] IK 未收敛，中止: {diag}")
                return False
            self.ctrl.get_logger().warning(
                f"[{name}] IK 近似收敛（{pos_err * 1000:.1f} mm / {rot_err:.3f} rad），继续"
            )
        # settle_check=False：仿真的 PD 稳态误差本来就在 0.08 rad 量级，
        # 按容差判失败没有意义——回放是开环的，落地到哪由执行器决定。
        ok = self.ctrl.move_to_pose(joints, duration_sec=duration, wait=True, settle_check=False)
        self._wait_settled()
        if not ok:
            self.ctrl.get_logger().error(f"[{name}] 轨迹下发失败")
        return ok

    def _goto(self, name: str, xyzrpy_target, speed: float, reference_apple: bool) -> bool:
        """解 IK 下发**一次**轨迹：指令就是记录的 ``target``，开环、不回头补偿。

        与 ``keyboard_ee_control`` 的语义一致——遥操作按键改的是目标变量，发送器
        照它下发，最终落在哪由执行器决定；回放也照抄这个语义，这样回放出来的
        运动才是示教时那段运动本身。仿真臂有 7~13 cm 的重力下垂，实测会低于
        目标，这里只把它记进日志，不再改指令重发（旧版的补偿环一次接近就要摆
        4 个回合、40 s，且段段过冲）。
        """
        target = np.asarray(xyzrpy_target, dtype=float)
        self._log_target(name, target[:3], reference_apple)
        cur = self._tcp()
        if cur is not None:
            angle = _rotation_angle(cur, target)
            if angle > np.radians(30.0):
                self.ctrl.get_logger().warning(
                    f"[{name}] 本段姿态变化 {np.degrees(angle):.1f}°"
                    f"（抓着苹果做大角度腕转，注意别甩脱）"
                )
        if not self._send(name, target, self._duration(target, speed)):
            return False
        cur = self._tcp()
        if cur is not None:
            err = self._wrap(target - cur)
            self.ctrl.get_logger().info(
                f"[{name}] 到位（开环，未补偿）：实测差 "
                f"{float(np.linalg.norm(err[:3])) * 1000:.1f} mm / "
                f"{np.degrees(float(np.linalg.norm(err[3:]))):.1f}°，"
                f"TCP {np.round(cur[:3], 3).tolist()}"
            )
        return True

    def _open(self) -> bool:
        """张开手，用**预备位手型**（拇指侧摆内收 850 = 90°），与遥操作空格一致。

        全伸直只出现在回 home；抓取前和放下后的手都保持"张开 + 拇指内收"，
        否则 ``_open()`` 会在合手前后把拇指甩回全外展，示教时捏住苹果的那一下
        就复现不出来。
        """
        self.ctrl.get_logger().info("[hand] 张开手（预备位手型，拇指侧摆内收）")
        pose = C.ready_hand_pose(self.ctrl.variant)
        names = self.ctrl.hand_joint_names(self.side)
        return self.ctrl.move_hand(
            self.side, {name: pose[name] for name in names if name in pose}, duration_sec=1.2, wait=True
        )

    def _close(self, grip: float) -> bool:
        """合手，拇指侧摆保持预备位 90°（``base_pose``），与遥操作空格一致。"""
        base = C.ready_hand_pose(self.ctrl.variant)
        closed = C.hand_closed_pose(self.ctrl.variant, base_pose=base)
        names = self.ctrl.hand_joint_names(self.side)
        travel = max(abs(grip * closed[name] - base.get(name, 0.0)) for name in names)
        duration = max(1.0, travel / max(self.close_speed, 1e-3))
        self.ctrl.get_logger().info(
            f"[grasp] 合手 grip={grip:.2f}（{duration:.1f}s，{self.close_speed:.2f} rad/s，"
            f"拇指侧摆保持预备位 90°）"
        )
        if not self.ctrl.close_hand(
            self.side, duration_sec=duration, wait=True, grip=grip, base_pose=base
        ):
            return False
        if self.settle_sec > 0:
            self.ctrl.get_logger().info(f"[grasp] 保持 {self.settle_sec:.1f}s 让手指压稳")
            time.sleep(self.settle_sec)
        return True

    def run(self, stop_after: str | None = None) -> bool:
        self.log_path()
        present = {wp.index for wp in self.waypoints}
        missing = sorted(set(required_indices(stop_after)) - present)
        if missing:
            self.ctrl.get_logger().error(
                f"路径缺少 {missing} 号点，无法执行到 --stop-after {stop_after}"
            )
            return False
        try:
            steps = planned_steps(stop_after, present)
        except ValueError as exc:
            self.ctrl.get_logger().error(str(exc))
            return False
        if not self._open():
            return False

        # 1、2 参考苹果（"够到苹果"）；3~5 苹果已经在手里，是纯绝对位姿。
        for kind, index in steps:
            if kind == "goto":
                wp = self._waypoint(int(index))
                if index <= GRASP_INDEX:
                    speed = self.speed
                elif index == LIFT_INDEX:
                    speed = self.lift_speed
                else:
                    speed = self.place_speed
                if not self._goto(
                    f"wp{index}", wp.pose, speed, reference_apple=index <= GRASP_INDEX
                ):
                    return False
            elif kind == "close":
                if not self._close(self.grip):
                    return False
            elif kind == "open":
                if not self._open():
                    return False

        # 先把终止位姿记进日志——回预备位之后 TCP 就变了，那串数就没法对照了。
        ok = self._finish(STOP_TITLES[stop_after])

        # 录制的收尾点：张手放到盘上的这一刻。放在 return_ready 之前，
        # 免得把"空手回预备位"那 5 s 也录进抓取段（见 --save）。
        if self.on_place_done is not None:
            self.on_place_done(bool(ok))

        # 放完直接回预备位收工；--stop-after 是"停在这儿看"，不回。
        # 走**一步到底**（staged=False，5 s）：四段式的分段轨迹是给"从站立位出发、
        # 手要贴着桌面从体侧扫过去"的场景准备的，收尾时手臂本来就在桌面上方、
        # 一路空旷，没必要再摆 23 s。hands=False——手在 _open() 时已经是预备位手型，
        # 头这一趟从来没动过，都不用再走一遍。
        if stop_after is None and self.return_ready:
            self.ctrl.get_logger().info(f"放置完成，回预备位（一步 {RETURN_READY_SEC:.1f}s）")
            if not self.ctrl.move_to_ready_pose(
                duration_sec=RETURN_READY_SEC, staged=False, hands=False
            ):
                self.ctrl.get_logger().error("回预备位失败")
                return False
        return ok

    def _finish(self, title: str) -> bool:
        cur = self._tcp()
        if cur is not None:
            self.ctrl.get_logger().info(
                f"{title}: TCP pelvis={np.round(cur[:3], 3).tolist()}，"
                f"rpy={np.round(cur[3:], 3).tolist()}"
            )
        return True

    # ------------------------------------------------------------------ 干跑
    def dry_run(self) -> bool:
        """从 ready 位姿逐步热启动解 IK，打印 5 个点的可达性；不发运动指令。

        刻意不走 ``DexController`` 的 IK：那边的种子是机器人**当前**姿态，
        和回放流程里的热启动顺序不同，测不出真实可达性。这里按路径顺序自己
        链式求解。
        """
        from utils.ik import TienkungDexIK, default_urdf_path

        solver = TienkungDexIK(
            default_urdf_path(self.ctrl.variant),
            joint_limits=C.BODY_JOINT_LIMITS,
            joint_limit_margin=0.0,
        )
        model = solver.model
        arm_idx = [model.joints[model.getJointId(n)].idx_q for n in C.ARM_JOINTS[self.side]]
        low = np.array([model.lowerPositionLimit[i] for i in arm_idx])
        high = np.array([model.upperPositionLimit[i] for i in arm_idx])

        self.ctrl.get_logger().info(
            f"苹果 pelvis={np.round(self.apple, 3).tolist()}，从预备位链式解 IK"
        )
        ready = {k: v for k, v in C.READY_POSE.items() if k in C.ARM_JOINTS[self.side]}
        solver.sync_joint_positions(list(ready), list(ready.values()))
        q = solver.q.copy()

        ok_all = True
        for wp in self.waypoints:
            solver.q = q.copy()
            _, ok = solver.solve_ik_single_arm(solver.xyzrpy_to_se3(wp.pose), self.side)
            diag = solver.last_fail_info or {}
            q = solver.q.copy()
            err = float(diag.get("pos_err", 0.0 if ok else float("inf")))
            rot = float(diag.get("rot_err", 0.0 if ok else float("inf")))
            qa = np.array([q[i] for i in arm_idx])
            margin = float(np.min(np.minimum(qa - low, high - qa)))
            ok_all &= ok
            self.ctrl.get_logger().info(
                f"  {wp.index} [{wp.role:<11}] {wp.source:<8} {'可达' if ok else '不可达'}"
                f"（{err * 1000:.1f} mm / {rot:.3f} rad，关节余量 {margin:+.3f} rad）  "
                + self._target_text(wp.pose[:3], wp.index <= GRASP_INDEX)
            )
        self.ctrl.get_logger().info("干跑结束：" + ("全部可达" if ok_all else "有目标到不了"))
        return ok_all


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="tienkung_dex 右臂抓取苹果：回放 grasp_path.json 里的示教路径",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--variant", default=None, choices=["inspire", "brainco"],
                        help="灵巧手型号（默认取路径文件里的 hand_variant）")
    parser.add_argument("--path-file", default=DEFAULT_PATH_FILE, help="示教路径文件")
    parser.add_argument("--pose-source", default="target", choices=POSE_SOURCES,
                        help="回放期望目标变量（默认）还是实测读数")
    parser.add_argument("--grip", type=float, default=DEFAULT_GRIP, help="合手闭合度 0~1")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="接近段速度 m/s")
    parser.add_argument("--lift-speed", type=float, default=DEFAULT_LIFT_SPEED, help="抬升段速度 m/s")
    parser.add_argument("--place-speed", type=float, default=DEFAULT_PLACE_SPEED, help="放置段速度 m/s")
    parser.add_argument("--rot-speed", type=float, default=DEFAULT_ROT_SPEED,
                        help="姿态变化速度上限 rad/s")
    parser.add_argument("--close-speed", type=float, default=DEFAULT_CLOSE_SPEED, help="合手速度 rad/s")
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_SEC, help="合手后保持秒数")
    parser.add_argument("--settle-time", type=float, default=DEFAULT_SETTLE_TIME,
                        help="每段轨迹发完后的静置时间 s")
    parser.add_argument("--stop-after", choices=STOP_STAGES, help="执行到该段后停止")
    parser.add_argument("--skip-ready", action="store_true", help="不先回预备位")
    parser.add_argument("--no-return-ready", action="store_true",
                        help="放完不回预备位（默认回；--stop-after 时本来就不回）")
    parser.add_argument("--ready-duration", type=float, default=None,
                        help="回预备位总时长 s；不给则按 READY_STAGE_DURATIONS 分段（每段 5 s）")
    parser.add_argument("--dry-run", action="store_true", help="只解算路径点，不发运动指令")
    parser.add_argument("--save", action="store_true",
                        help="录制抓取段到 dataset/tienkung_dex/<epoch秒>/trajectory.hdf5"
                             "（从 ready 后开始，到张手放置那一刻停）")
    parser.add_argument("--save-dir", default=None, help="录制输出根目录（默认 dataset/tienkung_dex）")
    parser.add_argument("--capture-hz", type=float, default=None, help="录制频率 Hz（默认 15）")
    parser.add_argument("--no-depth", action="store_true", help="--save 时不录深度")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    try:
        data = load_grasp_path(args.path_file)
        waypoints = resolve_waypoints(data, args.pose_source)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"路径文件不可用: {exc}", file=sys.stderr)
        return 2

    # 示教文件可能只采到一半：先判"这次要跑到的点齐不齐"，免得机器人先回预备位
    # 再中途报错。1 号点可以缺（序列自动跳过），2 号点缺了就一步都跑不了。
    present = {wp.index for wp in waypoints}
    missing = sorted(set(required_indices(args.stop_after)) - present)
    if missing:
        viable = [stage for stage in STOP_STAGES if not set(required_indices(stage)) - present]
        print(
            f"路径缺 {missing} 号点，跑不到 --stop-after {args.stop_after}；"
            f"当前文件可执行到: {viable or '无（连抓取前的接近点都没有）'}",
            file=sys.stderr,
        )
        return 2

    side = str(data["side"])
    recorded_variant = data.get("hand_variant")
    variant = args.variant or recorded_variant or C.DEFAULT_VARIANT
    if args.variant and recorded_variant and args.variant != recorded_variant:
        print(
            f"警告: 路径是 {recorded_variant} 手示教的，现在按 {args.variant} 回放，手型可能对不上",
            file=sys.stderr,
        )

    if args.save and args.stop_after is not None:
        print(
            "--save 只在完整抓取（张手放到盘上）时录制；--stop-after 是调试开关，"
            "两者不能同时给",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    ctrl = DexController(variant=variant)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(ctrl)

    recorder = None
    if args.save:
        # 录制节点和控制器放同一个 executor：单进程一集，边界由 on_place_done
        # 精确给出，不靠 sleep 掐时间（批量采集时这点最要紧）
        recorder = TienkungDexDataRecorder(
            capture_hz=args.capture_hz or DEFAULT_CAPTURE_HZ,
            record_depth=not args.no_depth,
            variant=variant,
        )
        executor.add_node(recorder)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if not ctrl.wait_for_state(timeout=10.0):
            ctrl.get_logger().error("收不到 /robot_state，检查桥接与 ROS_DOMAIN_ID")
            return 1

        picker = ApplePicker(
            ctrl,
            waypoints,
            side=side,
            grip=args.grip,
            speed=args.speed,
            lift_speed=args.lift_speed,
            place_speed=args.place_speed,
            rot_speed=args.rot_speed,
            close_speed=args.close_speed,
            settle_sec=args.settle,
            settle_time=args.settle_time,
            return_ready=not args.no_return_ready,
        )
        ctrl.get_logger().info(
            f"路径 {args.path_file}（{side} 臂，{variant} 手，回放源 {args.pose_source}）"
        )
        if args.dry_run:
            return 0 if picker.dry_run() else 2

        if not args.skip_ready:
            ctrl.get_logger().info("先回预备位")
            ready_duration = (
                None if args.ready_duration is None else max(args.ready_duration, 10.0)
            )
            if not ctrl.move_to_ready_pose(duration_sec=ready_duration):
                ctrl.get_logger().error("预备位未到位，中止")
                return 1
            time.sleep(0.5)

        if recorder is not None:
            def _save_episode(success: bool) -> None:
                """张手放置那一刻的回调：成功才落盘，失败/空缓存直接丢。"""
                if not success:
                    recorder.stop_episode()
                    recorder.get_logger().warning("抓取未完成，丢弃本集")
                    recorder.recorder.clear()
                    return
                recorder.stop_episode()
                if recorder.recorder.num_frames == 0:
                    recorder.get_logger().warning("一帧都没收到，跳过保存")
                    return
                out_dir = Path(args.save_dir) if args.save_dir else DEFAULT_SAVE_DIR
                epoch = recorder.get_clock().now().seconds_nanoseconds()[0]
                path = prepare_episode_file(episode_path(out_dir, epoch))
                recorder.save_episode(
                    str(path), attrs={"task": DEFAULT_TASK_NAME, "episode_index": 1}
                )
                recorder.recorder.clear()

            picker.on_place_done = _save_episode
            recorder.start_episode()
            ctrl.get_logger().info(
                f"开始录制抓取段（{recorder.capture_hz:g} Hz）→ "
                f"{args.save_dir or DEFAULT_SAVE_DIR}/<epoch秒>/{HDF5_REL_PATH}"
            )

        ok = picker.run(stop_after=args.stop_after)
        if recorder is not None and recorder.recorder.recording:
            # 走到了这里说明 run() 没跑完就退出了（中途报错），别把半集留下来
            recorder.stop_episode()
            recorder.recorder.clear()
            ctrl.get_logger().warning("流程未走到放置，丢弃本集")
        ctrl.get_logger().info("抓取完成" if ok else "抓取失败")
        return 0 if ok else 2
    except KeyboardInterrupt:
        ctrl.get_logger().warning("用户中断")
        return 130
    finally:
        _shutdown(ctrl, executor, spin_thread, extra_nodes=[recorder] if recorder else None)


if __name__ == "__main__":
    sys.exit(main())
