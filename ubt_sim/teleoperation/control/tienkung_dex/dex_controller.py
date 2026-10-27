#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 基础控制器（ROS 侧，系统 Python 3.10，不依赖 Isaac Sim）。

对标 ``walker_s2/utils/controller.py`` 的 ``WalkerS2Controller``：上层脚本
（数据采集、EE 遥操作、抓取任务）只依赖本类，不直接碰话题。

职责
----
1. 关节状态缓存：订阅状态话题，维护 ``{关节名: 角度}`` / ``{关节名: 速度}``；
2. 轨迹控制：quintic 插值 + 限位裁剪 + 速度安全检查，按 ``control_hz`` 下发；
3. 位姿控制：``move_to_pose`` / ``move_joint`` / ``shift_joint`` / ``home``；
4. 灵巧手：``move_hand`` / ``open_hand`` / ``close_hand`` / 周期动作；
5. 双臂 IK：``move_arm_ik`` / ``move_arm_ee_delta[_local|_world]``；
6. 任务真值：订阅仿真专有的 ``/sim/task_completed``（苹果↔盘心距离），
   供采集脚本判定这一集是否真的抓住并放进盘子（``latest_task_dist``）；
7. 话题与具身天工 3.0 真机完全一致（``ros2_bridge_msgs`` 的
   ``/arm|head|waist|leg/cmd``、``/robot_state``；灵巧手
   ``/left_hand/angle_set`` / ``set_motor_multi``），按关节 ID 编址。
   仿真与真机只有 ``ROS_DOMAIN_ID`` 不同（仿真 146 / 真机 0）——
   同一份脚本、同一套 API，不用改代码。

安全默认值
----------
* 腿部关节默认 **锁定**（``lock_joints = LEG_JOINTS``）：真机站立时腿由行走
  控制器持有，位置指令会打架。仿真里要动腿用 ``--include-legs``；
* 轨迹下发前做关节限位裁剪与最大速度检查（``--no-limits`` / ``--no-safety``
  可关，不建议）；
* 关节名可以用省略 ``_joint`` 后缀的写法（见 ``constants.normalize_joint_name``）。

用法（容器内）::

    source /opt/ros/humble/setup.bash
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=146          # 仿真；真机改成 0
    /usr/bin/python3 teleoperation/control/tienkung_dex/dex_controller.py --print-state
    /usr/bin/python3 teleoperation/control/tienkung_dex/dex_controller.py --home
    /usr/bin/python3 teleoperation/control/tienkung_dex/dex_controller.py --print-hand

统一 CLI 入口是 ``tienkung_dex_controller.py``（子命令 state/joint/endpoint/
hand/home/analyze/camera），本文件提供类与各子命令的 argparse。
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from contextlib import contextmanager, nullcontext
from typing import Any, Iterable, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import constants as C  # noqa: E402

# ---------------------------------------------------------------------------
# 话题表（= 具身天工 3.0 真机话题；仿真桥接发同一套，靠 ROS_DOMAIN_ID 分流）
# ---------------------------------------------------------------------------
TOPICS = {
    "arm_command": "/arm/cmd",
    "head_command": "/head/cmd",
    "waist_command": "/waist/cmd",
    "leg_command": "/leg/cmd",
    "robot_state": "/robot_state",
    "reset": "/sim/cmd_reset",
    # 仿真专有：苹果中心到盘心的距离（真值，米），与 tienkung_pro 同名话题。
    # 真机没有它，订阅着等不到消息（采集脚本据此跳过任务判定）。
    "task_dist": "/sim/task_completed",
    # 仿真专有：苹果位置偏移（世界系 x/y，米），--randomize-apple 用；真机没有订阅者
    "apple_offset": "/scene/apple/offset",
    "hand": {
        "brainco": {
            "left": {"command": "/left_hand/set_motor_multi", "state": "/left_hand/motor_status"},
            "right": {"command": "/right_hand/set_motor_multi", "state": "/right_hand/motor_status"},
        },
        "inspire": {
            "left": {"command": "/left_hand/angle_set", "state": "/left_hand/angle_actual"},
            "right": {"command": "/right_hand/angle_set", "state": "/right_hand/angle_actual"},
        },
    },
}

GROUP_MSG = {"arm": "ArmCtrl", "head": "HeadCtrl", "waist": "ArmCtrl", "leg": "ArmCtrl"}
HAND_CMD_MODE = 1          # 强脑 SetMotorMulti 的 SDK 默认控制模式


def load_real_msgs(variant: str) -> dict[str, Any]:
    """导入真机消息包（``ros2_bridge_msgs`` + 当前手型的厂商包）。

    数据采集等其它 ROS 节点也用它，避免各处重复写 import 与报错文案。
    """
    try:
        from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorCtrl, RobotState
    except ImportError as exc:  # pragma: no cover - 依赖运行环境
        raise RuntimeError(
            f"{exc}：需要 ros2_bridge_msgs。容器内先 bash run.sh init，"
            "或 source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash"
        ) from exc

    msgs: dict[str, Any] = {
        "ArmCtrl": ArmCtrl,
        "HeadCtrl": HeadCtrl,
        "MotorCtrl": MotorCtrl,
        "RobotState": RobotState,
    }
    if variant == "brainco":
        try:
            from brainco_hand_msgs.msg import MotorStatus, SetMotorMulti
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                f"{exc}：brainco 变体需要 brainco_hand_msgs，容器内先 bash run.sh init"
            ) from exc
        msgs.update({"hand_command": SetMotorMulti, "hand_state": MotorStatus})
    else:
        try:
            from inspire_hand_msgs.msg import GetAngleAct, SetAngle
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                f"{exc}：inspire 变体需要 inspire_hand_msgs，容器内先 bash run.sh init"
            ) from exc
        msgs.update({"hand_command": SetAngle, "hand_state": GetAngleAct})
    return msgs


# ---------------------------------------------------------------------------
# 位姿数学（与 pinocchio 的 URDF 约定一致：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)）
# ---------------------------------------------------------------------------
def _quat_wxyz_to_matrix(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if norm <= 0.0:
        return np.eye(3)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rotation_matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if abs(np.cos(pitch)) > 1e-10:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return float(roll), float(pitch), float(yaw)


def _convert_ee_delta_local_to_base(
    current_xyzrpy: Sequence[float],
    delta_xyz: Sequence[float],
    delta_rpy: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """末端局部坐标系的 delta -> base 坐标系的增量与绝对目标。"""
    x, y, z, roll, pitch, yaw = [float(v) for v in current_xyzrpy]
    R_ee = _rpy_to_rotation_matrix(roll, pitch, yaw)
    delta_xyz_base = R_ee @ np.asarray(delta_xyz, dtype=float)
    R_new = R_ee @ _rpy_to_rotation_matrix(*[float(v) for v in delta_rpy])
    new_roll, new_pitch, new_yaw = _rotation_matrix_to_rpy(R_new)
    target = np.array(
        [x + delta_xyz_base[0], y + delta_xyz_base[1], z + delta_xyz_base[2],
         new_roll, new_pitch, new_yaw]
    )
    return delta_xyz_base, target


# ---------------------------------------------------------------------------
# 控制器
# ---------------------------------------------------------------------------
class DexController(Node):
    """TienKung Dex 基础控制器（仿真是它，真机也是它）。"""

    def __init__(
        self,
        node_name: str = "dex_controller",
        variant: str = C.DEFAULT_VARIANT,
        control_hz: float = C.DEFAULT_CONTROL_HZ,
        max_joint_speed: float = C.DEFAULT_MAX_JOINT_SPEED,
        enable_safety_check: bool = True,
        enable_limit_check: bool = True,
        lock_joints: Iterable[str] | None = None,
        enable_ik: bool = False,
        ik_urdf_path: str | None = None,
        ik_auto_initialize: bool = True,
        control_mode: int = C.CONTROL_MODE_POSITION,
        impedance_kp: float = 50.0,
        impedance_kd: float = 2.0,
    ):
        super().__init__(node_name)
        if variant not in C.HAND_JOINTS_BY_VARIANT:
            raise ValueError(f"未知的手部变体: {variant}")
        self.variant = variant
        self.left_hand_joints, self.right_hand_joints = C.hand_joints(variant)

        self.control_hz = float(control_hz)
        self.timer_period = 1.0 / self.control_hz
        self.max_joint_speed = float(max_joint_speed)
        self.enable_safety_check = bool(enable_safety_check)
        self.enable_limit_check = bool(enable_limit_check)
        self.lock_joints = set(C.LEG_JOINTS if lock_joints is None else lock_joints)
        self.control_mode = int(control_mode)
        self.impedance_kp = float(impedance_kp)
        self.impedance_kd = float(impedance_kd)

        # ---- 状态缓存 ----
        self._state_lock = threading.RLock()
        self._joint_positions: dict[str, float] = {}
        self._joint_velocities: dict[str, float] = {}
        self._hand_positions: dict[str, dict[str, float]] = {"left": {}, "right": {}}
        self._state_received = threading.Event()
        self._missing_warned: set[str] = set()

        # ---- 任务真值（仿真专有）----
        # latest_task_dist 收不到消息时停在 inf，事件用于区分"没收到过"
        # （真机 / 旧桥接）与"收到了但内容无效"（仿真发 -1.0，见 pick_apple）
        self.latest_task_dist = float("inf")
        self._task_dist_received = threading.Event()

        # ---- 轨迹状态 ----
        self._traj_lock = threading.RLock()
        self._traj_names: list[str] = []
        self._traj_points = np.empty((0, 0), dtype=float)
        self._traj_index = 0
        self._is_publishing = False
        self._body_target: dict[str, float] = {}
        self._hand_traj: dict[str, dict[str, Any]] = {
            "left": {"names": [], "points": np.empty((0, 0)), "index": 0},
            "right": {"names": [], "points": np.empty((0, 0)), "index": 0},
        }

        # ---- IK ----
        self.ik_solver = None
        self._ik_initialized = False
        self._ik_lock = threading.RLock()
        self._ik_urdf_path = ik_urdf_path

        # ---- ROS I/O ----
        self._qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pubs: dict[str, Any] = {}
        self.real_msgs: dict[str, Any] = {}
        self._setup_io()

        self._control_timer = self.create_timer(self.timer_period, self._control_callback)

        if enable_ik and ik_auto_initialize and not self.initialize_ik():
            self.get_logger().warning("IK 自动初始化失败；关节空间控制仍可用")

        self.get_logger().info(
            f"DexController 就绪: variant={variant}, {self.control_hz:.0f}Hz, "
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}, "
            f"锁定关节={sorted(self.lock_joints) or '无'}"
        )

    # ==================================================================
    # 话题装配
    # ==================================================================
    def _setup_io(self) -> None:
        self.real_msgs = load_real_msgs(self.variant)
        for group in ("arm", "head", "waist", "leg"):
            self._pubs[f"{group}_command"] = self.create_publisher(
                self.real_msgs[GROUP_MSG[group]], TOPICS[f"{group}_command"], self._qos_pub
            )
        self._pubs["reset"] = self.create_publisher(Bool, TOPICS["reset"], 1)
        self._pubs["apple_offset"] = self.create_publisher(Point, TOPICS["apple_offset"], 1)
        self.create_subscription(
            self.real_msgs["RobotState"], TOPICS["robot_state"], self._real_state_cb, self._qos_sub
        )
        # 仿真专有话题：真机没有发布者，订阅着不会报错，只是永远收不到
        self.create_subscription(
            Float32, TOPICS["task_dist"], self._task_dist_cb, self._qos_sub
        )
        for side in ("left", "right"):
            topics = TOPICS["hand"][self.variant][side]
            self._pubs[f"{side}_hand_command"] = self.create_publisher(
                self.real_msgs["hand_command"], topics["command"], self._qos_pub
            )
            self.create_subscription(
                self.real_msgs["hand_state"],
                topics["state"],
                lambda msg, s=side: self._real_hand_state_cb(s, msg),
                self._qos_sub,
            )

    # ==================================================================
    # 状态回调
    # ==================================================================
    def _real_state_cb(self, msg) -> None:
        with self._state_lock:
            for group in ("head", "waist", "arm", "leg"):
                container = getattr(msg, group, None)
                if container is None:
                    continue
                for item in getattr(container, "status", ()) or ():
                    try:
                        joint_id = int(item.name)
                    except (TypeError, ValueError):
                        continue
                    sim_name = C.JOINT_ID_TO_SIM.get(joint_id)
                    if sim_name is None:
                        if joint_id not in self._missing_warned:
                            self._missing_warned.add(joint_id)
                            self.get_logger().warning(f"/robot_state 收到未知关节 ID {joint_id}，已忽略")
                        continue
                    self._joint_positions[sim_name] = float(item.pos)
                    self._joint_velocities[sim_name] = float(getattr(item, "speed", 0.0))
        self._state_received.set()

    def _task_dist_cb(self, msg) -> None:
        self.latest_task_dist = float(msg.data)
        self._task_dist_received.set()

    def wait_for_task_dist(self, timeout: float = 1.0) -> bool:
        """等第一帧 ``/sim/task_completed``；收到过返回 True，超时返回 False。

        只在需要任务判定时调用（``pick_apple.py --save``）。真机上没有这个话题，
        ``timeout`` 就是白等的秒数，所以给得短。
        """
        return self._task_dist_received.wait(timeout=max(float(timeout), 0.0))

    def _real_hand_state_cb(self, side: str, msg) -> None:
        table = C.hand_channel_joints(self.variant, side)
        # 定长数组字段在 rclpy 里是 numpy 数组，不能用 `or ()` 兜底（真值有歧义）
        raw = getattr(msg, "positions" if self.variant == "brainco" else "joint_values", None)
        codes = [] if raw is None else list(raw)
        with self._state_lock:
            for channel, joints in enumerate(table):
                if not joints or channel >= len(codes):
                    continue
                joint = joints[0]
                self._hand_positions[side][joint] = C.hand_code_to_rad(
                    self.variant, channel, codes[channel], joint
                )

    # ==================================================================
    # 下发：身体 / 手
    # ==================================================================
    def _publish_body(self, partial_targets: dict[str, float]) -> None:
        """只下发 ``partial_targets`` 里点名的关节。

        仿真端 ``_action`` 是合并语义，桥接的 ``group_command_cb`` 也只转发
        ``msg.ctrl`` 里出现的关节；没点名的关节保持上一次的目标。因此本进程
        只影响自己正在控制的关节，不会覆盖其它控制器（另一条手臂、头、腰、腿）
        的目标。

        这里刻意**不**用实测关节角补全全身：那等于把"机器人现在在哪"写成
        "要它去哪"。仿真手臂是隐式 PD、无重力补偿，实测自带静差，每补全一次
        就再掉一格；而且会顶掉别人下发的目标。
        """
        with self._state_lock:
            targets = dict(self._body_target)
            targets.update(partial_targets)
            self._body_target = targets

        for group, joint_names in C.GROUP_JOINTS.items():
            active = [
                name for name in joint_names
                if name in partial_targets and name not in self.lock_joints
            ]
            if not active:
                continue
            msg_cls = self.real_msgs[GROUP_MSG[group]]
            msg = msg_cls()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = group
            msg.mode = self.control_mode
            limits = C.GROUP_CMD_LIMITS[group]
            ctrl_cls = self.real_msgs["MotorCtrl"]
            for name in active:
                ctrl = ctrl_cls()
                ctrl.name = int(C.SIM_TO_JOINT_ID[name])
                ctrl.pos = float(partial_targets[name])
                ctrl.spd = float(limits["spd"])
                ctrl.cur = float(limits["cur"])
                if self.control_mode == C.CONTROL_MODE_IMPEDANCE:
                    ctrl.kp = self.impedance_kp
                    ctrl.kd = self.impedance_kd
                msg.ctrl.append(ctrl)
            self._pubs[f"{group}_command"].publish(msg)

    def _publish_hand(self, side: str, targets: dict[str, float]) -> None:
        """把 ``{手部关节: rad}`` 转成真机通道码发出去。

        因时每个通道一个关节，``rad -> code`` 是纯几何换算（0.1°/码，带零位
        偏置）；强脑仍按闭合度换算。未在 ``targets`` 里的通道按 0 rad（伸直）
        下发，与旧版一致。
        """
        table = C.hand_channel_joints(self.variant, side)
        codes: list[int] = []
        for channel, joints in enumerate(table):
            if not joints:
                codes.append(0)
                continue
            joint = joints[0]
            codes.append(
                C.hand_rad_to_code(self.variant, channel, targets.get(joint, 0.0), joint)
            )

        msg = self.real_msgs["hand_command"]()
        if self.variant == "brainco":
            count = C.HAND_CHANNEL_COUNT["brainco"]
            msg.mode = HAND_CMD_MODE
            msg.positions = codes[:count]
            msg.speeds = [0] * count
            msg.currents = [0] * count
            msg.pwms = [0] * count
            msg.durations = [0] * count
        else:
            msg.hand_id = 1 if side == "left" else 2
            msg.joint_values = codes
        self._pubs[f"{side}_hand_command"].publish(msg)

    # ==================================================================
    # 控制循环
    # ==================================================================
    def _control_callback(self) -> None:
        targets: dict[str, float] | None = None
        with self._traj_lock:
            if self._is_publishing:
                if self._traj_index < self._traj_points.shape[0]:
                    row = self._traj_points[self._traj_index]
                    targets = {
                        name: float(value) for name, value in zip(self._traj_names, row)
                    }
                    self._traj_index += 1
                if self._traj_index >= self._traj_points.shape[0]:
                    self._is_publishing = False
        if targets:
            self._publish_body(targets)

        for side in ("left", "right"):
            hand_targets: dict[str, float] | None = None
            with self._traj_lock:
                state = self._hand_traj[side]
                if state["index"] < state["points"].shape[0]:
                    row = state["points"][state["index"]]
                    hand_targets = {
                        name: float(value) for name, value in zip(state["names"], row)
                    }
                    state["index"] += 1
            if hand_targets:
                self._publish_hand(side, hand_targets)

    # ==================================================================
    # 状态查询
    # ==================================================================
    def wait_for_state(self, timeout: float = 5.0) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self._state_received.is_set() and self._joint_positions:
                self.get_logger().info(
                    f"收到机器人状态（{time.monotonic() - start:.2f}s，"
                    f"{len(self._joint_positions)} 个关节）"
                )
                return True
            time.sleep(0.02)
        self.get_logger().error(f"等待机器人状态超时（{timeout}s）")
        return False

    def get_current_positions(self) -> dict[str, float]:
        """``{关节名: 当前角度}``（只含已收到读数的关节）。"""
        with self._state_lock:
            return dict(self._joint_positions)

    def get_joint_position(self, joint_name: str) -> float | None:
        name = C.normalize_joint_name(joint_name)
        if name is None:
            self.get_logger().error(f"未知关节: {joint_name}")
            return None
        with self._state_lock:
            return self._joint_positions.get(name)

    def get_hand_positions(self, side: str) -> dict[str, float]:
        with self._state_lock:
            return dict(self._hand_positions[side])

    def get_hand_joint_position(self, side: str, joint_name: str) -> float | None:
        name = C.normalize_joint_name(joint_name)
        if name is None:
            return None
        with self._state_lock:
            return self._hand_positions[side].get(name)

    @property
    def joint_names(self) -> list[str]:
        return list(C.BODY_JOINTS)

    @property
    def is_busy(self) -> bool:
        with self._traj_lock:
            return self._is_publishing

    def hand_joint_names(self, side: str) -> list[str]:
        return list(self.left_hand_joints if side == "left" else self.right_hand_joints)

    # ==================================================================
    # 身体轨迹控制
    # ==================================================================
    def _clamp_position(self, names: Sequence[str], values: Sequence[float]) -> tuple[np.ndarray, list]:
        clamped = np.array(values, dtype=float)
        violations = []
        for i, name in enumerate(names):
            limits = C.BODY_JOINT_LIMITS.get(name)
            if limits is None:
                continue
            low, high = limits
            if clamped[i] < low or clamped[i] > high:
                violations.append((name, float(clamped[i]), low, high))
                clamped[i] = min(max(clamped[i], low), high)
        return clamped, violations

    def _start_trajectory(self, names: list[str], points: np.ndarray) -> None:
        with self._traj_lock:
            self._traj_names = list(names)
            self._traj_points = np.asarray(points, dtype=float)
            self._traj_index = 0
            self._is_publishing = True

    def execute_trajectory(
        self, joint_names: Sequence[str], trajectory, wait: bool = True
    ) -> bool:
        """执行 ``(N, len(joint_names))`` 轨迹，点间距为 ``1/control_hz`` 秒。"""
        names = [C.normalize_joint_name(name) or name for name in joint_names]
        trajectory = np.asarray(trajectory, dtype=float)
        if trajectory.ndim != 2 or trajectory.shape[1] != len(names):
            self.get_logger().error(
                f"轨迹形状 {trajectory.shape} != (N, {len(names)})"
            )
            return False
        if not np.isfinite(trajectory).all():
            self.get_logger().error("轨迹含 NaN/inf，拒绝执行")
            return False

        # 锁定关节不下发
        keep = [i for i, name in enumerate(names) if name not in self.lock_joints]
        if len(keep) != len(names):
            dropped = sorted({names[i] for i in range(len(names)) if i not in keep})
            self.get_logger().warning(f"锁定关节跳过: {dropped}")
            names = [names[i] for i in keep]
            trajectory = trajectory[:, keep]
            if not names:
                return False

        if self.enable_limit_check:
            seen: set[str] = set()
            for i in range(trajectory.shape[0]):
                clamped, violations = self._clamp_position(names, trajectory[i])
                trajectory[i] = clamped
                for name, value, low, high in violations:
                    if name not in seen:
                        seen.add(name)
                        self.get_logger().warning(f"限位裁剪 {name}: {value:.4f} -> [{low}, {high}]")

        if self.enable_safety_check and trajectory.shape[0] >= 2:
            speeds = np.max(np.abs(np.diff(trajectory, axis=0)) / self.timer_period, axis=0)
            unsafe = [(names[i], float(speeds[i])) for i in range(len(names))
                      if speeds[i] > self.max_joint_speed]
            if unsafe:
                for name, speed in unsafe:
                    self.get_logger().error(
                        f"安全违规: {name} 速度 {speed:.2f} rad/s > {self.max_joint_speed}"
                    )
                return False

        self._start_trajectory(names, trajectory)
        self.get_logger().info(
            f"执行轨迹: {trajectory.shape[0]} 点，约 {trajectory.shape[0] / self.control_hz:.2f}s"
        )
        if wait:
            duration = trajectory.shape[0] / self.control_hz
            deadline = time.monotonic() + max(duration * 3.0, 5.0)
            while time.monotonic() < deadline:
                if not self.is_busy:
                    break
                time.sleep(0.01)
            else:
                self.get_logger().error("等待轨迹执行超时，强制停止")
                self.stop()
        return True

    def _interpolate(
        self, start: dict[str, float], target: dict[str, float], duration_sec: float
    ) -> np.ndarray:
        names = list(target)
        n_pts = max(2, int(round(duration_sec * self.control_hz)))
        tau = np.linspace(0.0, 1.0, n_pts)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5  # 起止速度/加速度为 0
        return np.column_stack(
            [np.asarray(start[name], dtype=float) + s * (target[name] - start[name]) for name in names]
        )

    def move_to_pose(
        self,
        pose_dict: dict[str, float],
        duration_sec: float = C.DEFAULT_MOVE_DURATION,
        wait: bool = True,
        unlock_required_joints: bool = True,
        settle_check: bool = True,
        settle_tolerance: float = 0.08,
        ramp_from_commanded: bool = False,
    ) -> bool:
        """按 ``{关节名: 目标角}`` 移动；未指定的关节保持当前值。

        ``settle_tolerance`` 默认放到 0.08 rad：仿真的手臂执行器是隐式 PD，加
        重力前馈后稳态误差已经很小（预备位 7 mrad 量级，ζ≈0.7 下 10 s 墙钟内就
        停稳），但 RTF≈0.11 意味着墙钟上的收敛本来就慢，容差再收紧容易把"还在
        慢慢收敛"误报成失败。真机是驱动器闭环，实际误差远小于此；需要严格判定
        时显式传更小的值，并把等待时间一起放宽。

        ``ramp_from_commanded``：从**本进程上次下发的关节目标**起步，而不是从
        实测起步。这是 ``tienkung_pro`` 的 ``move_arm`` 的做法——那段代码的命令
        链只认 ``self.right_joints``（IK 结果），状态回调 ``_arm_status_cb`` 的
        实测值从不回写进去。仿真的隐式 PD 有 τ_g/k 的静态下垂，从实测起步等于
        每一拍都把这段下垂重走一遍：预备位上肩 pitch 差 0.164 rad，0.2 s/10 点
        的 quintic 峰值 0.208，换算 1.80 rad/s，稳定超过 1.5 的限速被判违规丢弃。
        从上次命令起步，每拍的增量只剩操作者真正给出的那一点。

        目标离上次命令太远时（本进程第一次下发、或调用方刚大步移动过），自动
        按 ``_min_duration`` 拉长本段时长——等价于 pro 的"追加步数"：宁可走慢
        也不整条丢弃。默认 ``False`` 保持既有脚本的行为不变。
        """
        current = self.get_current_positions()
        if not current:
            self.get_logger().error("没有关节状态，无法移动")
            return False

        targets: dict[str, float] = {}
        locked_involved: list[str] = []
        for raw_name, value in pose_dict.items():
            name = C.normalize_joint_name(raw_name)
            if name is None or name not in C.BODY_JOINT_LIMITS:
                self.get_logger().error(f"未知关节: {raw_name}")
                return False
            targets[name] = float(value)
            if name in self.lock_joints:
                locked_involved.append(name)

        if ramp_from_commanded:
            with self._state_lock:
                commanded = dict(self._body_target)
            start = {
                name: float(
                    commanded.get(name, current.get(name, C.HOME_POSE.get(name, 0.0)))
                )
                for name in targets
            }
            duration = max(float(duration_sec), self._min_duration(start, targets))
        else:
            start = {
                name: float(current.get(name, C.HOME_POSE.get(name, 0.0))) for name in targets
            }
            duration = float(duration_sec)
        trajectory = self._interpolate(start, targets, max(duration, self.timer_period * 2))

        context = (
            self._temporary_unlock(locked_involved, wait=wait)
            if (locked_involved and unlock_required_joints)
            else nullcontext()
        )
        with context:
            ok = self.execute_trajectory(list(targets), trajectory, wait=wait)
        if not ok:
            return False

        if wait and settle_check:
            arrived, misses = self.wait_until_position(
                targets, timeout=max(8.0, duration * 3.0), tolerance=settle_tolerance
            )
            if not arrived:
                self.get_logger().warning(
                    f"{len(misses)} 个关节未在容差 {settle_tolerance} 内收敛: {misses[:5]}"
                )
            return arrived
        return True

    def move_to_position(
        self,
        target_position: Sequence[float],
        duration_sec: float = C.DEFAULT_MOVE_DURATION,
        wait: bool = True,
    ) -> bool:
        """按 ``BODY_JOINTS`` 顺序的完整目标向量移动。"""
        if len(target_position) != len(C.BODY_JOINTS):
            self.get_logger().error(
                f"目标维度 {len(target_position)} != {len(C.BODY_JOINTS)}"
            )
            return False
        return self.move_to_pose(
            dict(zip(C.BODY_JOINTS, [float(v) for v in target_position])),
            duration_sec=duration_sec,
            wait=wait,
        )

    def send_body_position(self, targets: dict[str, float]) -> bool:
        """直接下发一帧身体关节目标（不做插值）。

        给阶跃/正弦辨识这类需要"原始指令"的场景用；常规运动请走
        ``move_to_pose``（有限速与限位保护）。
        """
        normalized: dict[str, float] = {}
        for raw_name, value in targets.items():
            name = C.normalize_joint_name(raw_name)
            if name is None:
                self.get_logger().error(f"未知关节: {raw_name}")
                return False
            normalized[name] = float(value)
        self._publish_body(normalized)
        return True

    def wait_until_position(
        self,
        target: dict[str, float],
        timeout: float = 5.0,
        tolerance: float = 0.05,
    ) -> tuple[bool, list[tuple[str, float | None, float, float]]]:
        """等待实际位置收敛到目标。返回 (是否到位, 未到位明细)。"""
        deadline = time.monotonic() + timeout
        misses: list[tuple[str, float | None, float, float]] = []
        while True:
            with self._state_lock:
                current = {name: self._joint_positions.get(name) for name in target}
            misses = [
                (name, current[name], float(goal), abs((current[name] or 0.0) - goal))
                for name, goal in target.items()
                if current[name] is None or abs(current[name] - goal) > tolerance
            ]
            if not misses or time.monotonic() >= deadline:
                break
            time.sleep(0.02)
        return (not misses), misses

    def stop(self) -> None:
        with self._traj_lock:
            self._is_publishing = False
            self._traj_index = self._traj_points.shape[0]
            for side in ("left", "right"):
                self._hand_traj[side]["index"] = self._hand_traj[side]["points"].shape[0]
        self.get_logger().info("已停止下发")

    @contextmanager
    def _temporary_unlock(self, joint_names: Iterable[str], wait: bool = True):
        names = [name for name in joint_names if name in self.lock_joints]
        if not names:
            yield
            return
        self.lock_joints -= set(names)
        try:
            yield
        finally:
            if wait:
                self.lock_joints |= set(names)
            else:
                self.get_logger().warning(
                    f"wait=False：{names} 保持解锁，运动结束后请自行 set_lock_joints 恢复"
                )

    def set_lock_joints(self, joint_names: Iterable[str] | None) -> None:
        self.lock_joints = set(joint_names or [])
        self.get_logger().info(f"锁定关节更新: {sorted(self.lock_joints) or '无'}")

    # ==================================================================
    # 单关节
    # ==================================================================
    def move_joint(
        self, joint_name: str, target_rad: float, duration_sec: float = C.DEFAULT_JOINT_DURATION,
        wait: bool = True,
    ) -> bool:
        return self.move_to_pose(
            {joint_name: float(target_rad)}, duration_sec=duration_sec, wait=wait
        )

    def shift_joint(
        self, joint_name: str, delta_rad: float, duration_sec: float = C.DEFAULT_JOINT_DURATION,
        wait: bool = True,
    ) -> bool:
        current = self.get_joint_position(joint_name)
        if current is None:
            self.get_logger().error(f"无法偏移 {joint_name}：没有当前读数")
            return False
        target = current + float(delta_rad)
        limits = C.BODY_JOINT_LIMITS.get(C.normalize_joint_name(joint_name) or "")
        if limits is not None and not (limits[0] <= target <= limits[1]):
            self.get_logger().warning(
                f"{joint_name} 目标 {target:.4f} 超出限位 {limits}，将裁剪"
            )
        self.get_logger().info(
            f"偏移 {joint_name}: {current:.4f} -> {target:.4f} rad ({delta_rad:+.4f})"
        )
        return self.move_joint(joint_name, target, duration_sec=duration_sec, wait=wait)

    def move_arm_joints(
        self, side: str, joints: Sequence[float], duration_sec: float = 1.5, wait: bool = True
    ) -> bool:
        names = C.ARM_JOINTS[side]
        if len(joints) != len(names):
            self.get_logger().error(f"{side} 臂需要 {len(names)} 个关节值，收到 {len(joints)}")
            return False
        return self.move_to_pose(
            dict(zip(names, [float(v) for v in joints])), duration_sec=duration_sec, wait=wait
        )

    # ==================================================================
    # 姿态：home / ready
    # ==================================================================
    def move_to_ready_pose(
        self,
        duration_sec: float | None = None,
        wait: bool = True,
        staged: bool = True,
        hands: bool = True,
        final_pose: dict[str, float] | None = None,
        stage_durations: Sequence[float] | None = None,
    ) -> bool:
        """移动到预备抓取位（``constants.READY_POSE`` + 预备位手型）。

        ``final_pose`` 默认为 ``READY_POSE`` 叠上 ``READY_POSE_TARGET``——后者由
        **加仿真重力前馈之前**命令 READY_POSE 的实测稳态读数（``READY_POSE_PRE_GRAV_COMP``）
        扣掉前馈残差得到，物理姿态与旧版一致（前馈只保证"指令 = 实际"，不改姿态本身）。
        真机要回到设计姿态时显式传 ``dict(C.READY_POSE)``。

        时长取值优先级：``stage_durations`` > ``duration_sec``（四段均分）>
        ``constants.READY_STAGE_DURATIONS``（默认每段 5 s）。
        每段仍会被 ``_min_duration`` 托底，不会低于速度检查允许的下限。

        正前方有桌面（本任务抬高到 0.987 m，见 constants.TABLETOP_WORLD_Z），
        一步到位时手要贴着桌面从体侧扫过去，会被顶住、肘收不到位。因此照
        tiangong2dex 的 reset 分四段：纯肩外展抬臂 → 高位折前臂 → 大臂转回
        近垂直 → 前臂落到预备位。每段都验证过手部离桌面 ≥ 0.23 m。

        ``hands=True`` 时最后再把双手摆成预备位手型（``constants.ready_hand_pose``：
        拇指侧摆内收到 850 = 90°，其余手指伸直）。手型放在手臂和头都到位之后，
        是整个预备动作的最后一步。

        节奏上算五段：5/5/5/5 s 的四段手臂轨迹（``READY_STAGE_DURATIONS``）+
        3 s 收尾（头部追平 + 手型，``READY_TAIL_DURATION``），合计 23 s。
        """
        # (标签, 目标, 容差, 是否必须到位)：中间航点只求"大致经过"。
        # 大臂外展时肩膀要扛住整条手臂的重力，隐式 PD 会稳定差 0.2 rad，
        # 按最终容差判定会误报失败——航点放宽，只有最后一个姿态严格判定。
        # 预备位自身稳态误差实测 0.17 rad（抬臂后力臂变长，比站立位的
        # 0.06 大；肩 pitch 在手臂前伸时被重力拖下去），所以最终容差取 0.20：
        # 仍能抓住顶到桌面这类 0.3 rad 级的失败。
        ready_target = dict(C.READY_POSE)
        ready_target.update(
            C.READY_POSE_TARGET if final_pose is None else final_pose
        )
        stages = (
            [
                # 满外展时手臂水平伸出，肩 roll 稳态实测差 0.34-0.48 rad（隐式 PD
                # 无重力补偿，时长越久陷得越深）。这一段在侧向空中摆动，桌面挡不住，
                # 且下垂到 1.13 rad 时手仍有 1.17 m（离桌面 0.18 m），容差放宽只为不误报
                ("1/4 肩外展抬臂过桌面", dict(C.READY_CLEARANCE_POSE), 0.55, False),
                ("2/4 高位折前臂", dict(C.READY_FOREARM_POSE), 0.30, False),
                ("3/4 大臂转回近垂直", dict(C.READY_UPPER_VERTICAL_POSE), 0.30, False),
                ("4/4 前臂落到预备位", dict(ready_target), 0.20, True),
            ]
            if staged
            else [("预备位", dict(ready_target), 0.20, True)]
        )
        durations = self._resolve_stage_durations(stages, duration_sec, stage_durations)
        self.get_logger().info(
            "移动到预备位（{}，{}）".format(
                " / ".join(f"{d:.1f}s" for d in durations),
                "分段" if staged else "一步",
            )
        )
        prev = self._stage_start(stages)
        for (label, pose, tolerance, required), want_duration in zip(stages, durations):
            stage_duration = max(float(want_duration), self._min_duration(prev, pose))
            self.get_logger().info(f"预备位阶段 {label}（{stage_duration:.1f}s）")
            if not self.move_to_pose(
                pose, duration_sec=stage_duration, wait=True, settle_tolerance=tolerance
            ):
                if required:
                    self.get_logger().error(f"预备位阶段失败: {label}")
                    return False
                self.get_logger().warning(
                    f"预备位航点未完全到位（重力下垂，或速度检查拒绝了整条轨迹）: {label}"
                )
            prev.update({name: float(value) for name, value in pose.items()})
        # 头单独再等一次：仿真主循环只有 ~11 Hz（`--perf_stats` 实测 Phys 30 ms +
        # Render 40 ms，RTF≈0.11），远超 velocity_limit_sim=1.27 rad/s 能跟上的
        # 速度，头在最后一段结束时还差 ~0.2 rad（正好落在 0.20 的段容差里，所以
        # move_to_pose 直接就返回了）。这里用 0.01 rad 的紧容差等它追上，保证返回
        # 时低头角就是 READY_POSE 的 45°。超时按 READY_TAIL_DURATION 给，不算短：
        # 头部只在最后一段才被下发，5 s 内基本已经走完，剩下的零点几秒就够。
        head_targets = {name: C.READY_POSE[name] for name in C.HEAD_JOINTS if name in C.READY_POSE}
        if hands:
            self.get_logger().info(f"预备位收尾 5/5 头部 + 手型（{C.READY_TAIL_DURATION:.1f}s）")
        elif wait and head_targets:
            # hands=False（比如 pick_apple 收尾：手已经是预备位手型）时不摆手指，
            # 但仍然要等头追上，否则返回时低头角还没到 READY_POSE 的 45°。
            self.get_logger().info(f"预备位收尾：等头部对齐（≤{C.READY_TAIL_DURATION:.1f}s）")
        # 手型与头部**并行**收尾：跟在四段手臂之后（手腕已经停住，手指不会追着
        # 转动的腕走），但它不依赖头，没必要串行。实测串行时头部要多等 ~1 s，
        # 合起来 4 s；并行就是 3 s 整。
        tail_t0 = time.monotonic()
        if hands:
            self.apply_ready_hand_pose(duration_sec=C.READY_TAIL_DURATION, wait=False)
        if wait and head_targets:
            arrived, misses = self.wait_until_position(
                head_targets, timeout=C.READY_TAIL_DURATION, tolerance=0.01
            )
            if not arrived:
                self.get_logger().warning(f"头部未在容差内到位: {misses}")
        if hands and wait:
            # move_hand 的等待上限口径：两倍轨迹时长（正常走完立即返回）
            deadline = tail_t0 + max(C.READY_TAIL_DURATION * 2.0, 3.0)
            for side in ("left", "right"):
                if not self._wait_hand_traj(side, deadline - time.monotonic()):
                    self.get_logger().warning(f"{side} 手型未在预算内走完")
        return True

    def apply_ready_hand_pose(
        self, duration_sec: float = 1.5, wait: bool = True, side: str | None = None
    ) -> bool:
        """把手摆成预备位手型（``constants.ready_hand_pose``：拇指侧摆内收 850 = 90°）。

        与 ``open_hand`` 的区别只有拇指侧摆：``open_hand`` 是全伸直，只给回 home 用。
        ``side=None`` 动双手；传 ``"left"`` / ``"right"`` 只动一侧（遥操作按空格时用）。
        """
        if side is not None and side not in ("left", "right"):
            self.get_logger().error(f"无效的 side: {side}")
            return False
        pose = C.ready_hand_pose(self.variant)
        ok = True
        sides = ("left", "right") if side is None else (side,)
        for i, s in enumerate(sides):
            names = self.hand_joint_names(s)
            # 双手并行：左右手各有独立的轨迹状态（_hand_traj[side]），先起左手
            # 的轨迹不阻塞，再用右手等，整步耗时就是 duration_sec 而不是两倍。
            last = i == len(sides) - 1
            ok = self.move_hand(
                s,
                {name: pose[name] for name in names if name in pose},
                duration_sec,
                wait and last,
            ) and ok
        return ok

    def _stage_start(self, stages) -> dict[str, float]:
        """第一段之前各关节的实际位置（缺省按站立位）。"""
        current = self.get_current_positions() or {}
        return {
            name: float(current.get(name, C.HOME_POSE.get(name, 0.0)))
            for _, pose, _, _ in stages
            for name in pose
        }

    def _resolve_stage_durations(
        self,
        stages,
        duration_sec: float | None,
        stage_durations: Sequence[float] | None,
    ) -> list[float]:
        """定出每一段的轨迹时长：``stage_durations`` > ``duration_sec`` 均分 > 常量表。"""
        if stage_durations is not None:
            durations = [float(d) for d in stage_durations]
        elif duration_sec is None:
            durations = [float(d) for d in C.READY_STAGE_DURATIONS]
        else:
            base = max(1.2, float(duration_sec) / len(stages))
            return [base] * len(stages)

        if len(durations) == len(stages):
            return durations
        if len(stages) == 1:  # 不分段：把四段的时长合起来当整段时长
            return [sum(durations)]
        fallback = max(1.2, sum(durations) / len(stages))
        self.get_logger().warning(
            f"分段时长 {durations} 与段数 {len(stages)} 不符，回退到每段 {fallback:.1f}s"
        )
        return [fallback] * len(stages)

    def _min_duration(self, start: dict[str, float], target: dict[str, float]) -> float:
        """让本段不触发速度检查所需的最短时长。

        ``_interpolate`` 用的是 quintic（峰值速度 1.875·Δq/T），而
        ``execute_trajectory`` 的检查按 ``max_joint_speed`` 卡离散差分。
        按 10s/4 段 = 2.5s 下发时，elbow 从 0 到 -2.00 正好是 1.51 rad/s，
        超 1.5 一点点就被整条丢弃——2/4、3/4 段是非强制航点，只会留一条
        "重力下垂属正常"的警告，于是绕桌面的中间姿态根本没走。
        """
        peak = max(
            (abs(float(value) - start.get(name, float(value))) for name, value in target.items()),
            default=0.0,
        )
        return 1.875 * peak / (0.85 * self.max_joint_speed)

    def home(
        self,
        duration_sec: float = 10.0,
        wait: bool = True,
        staged: bool = True,
        include_legs: bool = False,
        open_hands: bool = True,
    ) -> bool:
        """回到站立位（``constants.HOME_POSE``）；``open_hands`` 时双手**全伸直**。

        全伸直（``open_hand``，拇指侧摆也回 1750）只在回 home 时发生——预备位和
        抓取前用的是 :meth:`apply_ready_hand_pose` 的预备位手型（拇指内收 850 = 90°）。

        分段默认是 :meth:`move_to_ready_pose` 的**倒放**，四段逐段回退：
        前臂先收回高位 → 大臂外展 → 手臂伸直侧向 → 整体归位。
        这保证每一段都是正向预备动作已经走过的路径，指尖全程离桌面 ≥0.25 m。

        不要简化成"直接外展再落下"：从预备位一步外展时，肘还在 -1.10，
        指尖会从桌面正上方 0.98 m 扫过去（x≈0.50），仿真的 PD 下垂再压低
        0.1-0.3 rad 就会碰到桌面。

        腿部默认不动（真机站立时腿由行走控制器持有），``include_legs=True``
        才会下发腿部目标。
        """
        pose = dict(C.HOME_POSE) if include_legs else dict(C.HOME_UPPER_POSE)
        if staged:
            stages = [
                # 与 move_to_ready_pose 的四段严格互为逆序
                ("1/4 收回前臂到高位", dict(C.READY_UPPER_VERTICAL_POSE), 0.30, False),
                ("2/4 大臂外展离开桌面上方", dict(C.READY_FOREARM_POSE), 0.30, False),
                ("3/4 手臂伸直侧向", dict(C.READY_CLEARANCE_POSE), 0.55, False),
                ("4/4 上肢归位", pose, 0.15, True),
            ]
            base_duration = max(1.0, float(duration_sec) / len(stages))
            prev = self._stage_start(stages)
            for label, stage_pose, tolerance, required in stages:
                stage_duration = max(base_duration, self._min_duration(prev, stage_pose))
                self.get_logger().info(f"home 阶段 {label}（{stage_duration:.1f}s）")
                if not self.move_to_pose(
                    stage_pose, duration_sec=stage_duration, wait=True,
                    settle_tolerance=tolerance,
                ):
                    if required:
                        self.get_logger().error(f"home 阶段失败: {label}")
                        return False
                    self.get_logger().warning(
                        f"home 航点未完全到位（重力下垂，或速度检查拒绝了整条轨迹）: {label}"
                    )
                prev.update({name: float(value) for name, value in stage_pose.items()})
        elif not self.move_to_pose(pose, duration_sec=duration_sec, wait=wait):
            return False

        if open_hands:
            for side in ("left", "right"):
                self.open_hand(side, duration_sec=1.0, wait=wait)
        return True

    # ==================================================================
    # 灵巧手
    # ==================================================================
    def move_hand(
        self,
        side: str,
        pose_dict: dict[str, float],
        duration_sec: float = 2.0,
        wait: bool = True,
    ) -> bool:
        """按 ``{手部关节名: 目标角}`` 移动；单位 rad（legacy 侧就是原生单位）。"""
        if side not in ("left", "right"):
            self.get_logger().error(f"无效的 side: {side}")
            return False
        valid = set(self.hand_joint_names(side))
        targets: dict[str, float] = {}
        for raw_name, value in pose_dict.items():
            name = C.normalize_joint_name(raw_name)
            if name is None or name not in valid:
                self.get_logger().error(f"{side} 手没有关节 {raw_name}")
                return False
            targets[name] = float(value)
        if not targets:
            return False

        current = self.get_hand_positions(side)
        start = {name: float(current.get(name, 0.0)) for name in targets}
        points = self._interpolate(start, targets, max(float(duration_sec), self.timer_period * 2))
        with self._traj_lock:
            state = self._hand_traj[side]
            state["names"] = list(targets)
            state["points"] = points
            state["index"] = 0
        self.get_logger().info(f"{side} 手目标: {targets}")
        if wait:
            duration = points.shape[0] / self.control_hz
            self._wait_hand_traj(side, max(duration * 2.0, 3.0))
        return True

    def _wait_hand_traj(self, side: str, timeout: float) -> bool:
        """等 ``side`` 这只手把当前轨迹走完，超时返回 ``False``。

        双手可以并行（左右手各有独立的 ``_hand_traj[side]``），所以等待也要能
        分开做——预备动作收尾就是先起两只手的轨迹，再一起等它们走完。
        """
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while time.monotonic() < deadline:
            with self._traj_lock:
                state = self._hand_traj[side]
                done = state["index"] >= state["points"].shape[0]
            if done:
                return True
            time.sleep(0.01)
        return False

    def open_hand(self, side: str, duration_sec: float = 1.0, wait: bool = False) -> bool:
        names = self.hand_joint_names(side)
        return self.move_hand(side, {name: 0.0 for name in names}, duration_sec, wait)

    def close_hand(
        self,
        side: str,
        duration_sec: float = 1.0,
        wait: bool = False,
        grip: float = 1.0,
        base_pose: dict[str, float] | None = None,
    ) -> bool:
        """合手。``grip`` 为闭合度 0~1（0=全开，1=握拳），即握拳目标的缩放。

        与 tiangong2dex 的 ``close_hand(grip=...)`` 语义一致：抓放时用它做
        "预成型 / 抓取 / 释放"三档（如 0.35 / 1.0 / 0.30）。

        ``base_pose`` 是握拳时的基准姿态（默认全 0，偏航/外展轴回 0）。
        传 :func:`constants.ready_hand_pose` 就是"预备位手型只弯其他自由度"：
        拇指侧摆留在 850 = 90°，合手时不甩回全外展，且不随 ``grip`` 缩放。
        """
        grip = max(0.0, min(float(grip), 1.0))
        base = base_pose or {}
        closed = C.hand_closed_pose(self.variant, base_pose=base)
        names = self.hand_joint_names(side)
        targets: dict[str, float] = {}
        for name in names:
            if name in base and C.is_hand_abduction_joint(name):
                targets[name] = float(base[name])
            else:
                targets[name] = grip * closed[name]
        return self.move_hand(side, targets, duration_sec, wait)

    def send_hand_position(
        self, side: str, positions: dict[str, float] | Sequence[float]
    ) -> bool:
        """直接下发一帧手部目标（不做插值）。"""
        names = self.hand_joint_names(side)
        if isinstance(positions, dict):
            targets = {C.normalize_joint_name(k) or k: float(v) for k, v in positions.items()}
        else:
            if len(positions) != len(names):
                self.get_logger().error(
                    f"{side} 手需要 {len(names)} 个值，收到 {len(positions)}"
                )
                return False
            targets = dict(zip(names, [float(v) for v in positions]))
        self._publish_hand(side, targets)
        return True

    def hand_periodic_motion(
        self,
        side: str,
        joint_name: str,
        amplitude: float,
        period_sec: float = 2.0,
        cycles: int = 3,
        offset: float = 0.0,
    ) -> None:
        """单关节正弦开合（对手部/关节响应做目视检查）。"""
        name = C.normalize_joint_name(joint_name)
        if name is None:
            self.get_logger().error(f"未知关节: {joint_name}")
            return
        steps = max(2, int(period_sec * self.control_hz))
        for cycle in range(max(1, cycles)):
            for i in range(steps):
                phase = 2.0 * np.pi * i / steps
                self._publish_hand(side, {name: offset + amplitude * np.sin(phase)})
                time.sleep(self.timer_period)
            self.get_logger().info(f"{side} {name} 正弦第 {cycle + 1}/{cycles} 周期完成")

    # ==================================================================
    # IK / 末端位姿
    # ==================================================================
    def initialize_ik(
        self,
        urdf_path: str | None = None,
        left_neutral: Sequence[float] | None = None,
        right_neutral: Sequence[float] | None = None,
        save_current_as_initial: bool = True,
    ) -> bool:
        """初始化双臂 IK。目标位姿为 URDF 基座系 ``[x,y,z,roll,pitch,yaw]``。"""
        from utils.ik import TienkungDexIK, default_urdf_path

        resolved = (
            urdf_path
            or self._ik_urdf_path
            or os.environ.get("TIENKUNG_DEX_IK_URDF")
            or default_urdf_path(self.variant)
        )
        resolved = os.path.abspath(resolved)
        if not os.path.exists(resolved):
            self.get_logger().error(f"IK URDF 不存在: {resolved}")
            return False
        try:
            solver = TienkungDexIK(
                resolved, joint_limits=C.BODY_JOINT_LIMITS, joint_limit_margin=0.0
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"IK 初始化失败: {exc}")
            return False

        with self._ik_lock:
            self.ik_solver = solver
            self._ik_urdf_path = resolved
            self._ik_initialized = True
            self._sync_ik_from_current_state_locked()
            current = self.get_current_positions()
            left = list(left_neutral) if left_neutral is not None else [
                current.get(name, C.HOME_POSE[name]) for name in C.ARM_JOINTS["left"]
            ]
            right = list(right_neutral) if right_neutral is not None else [
                current.get(name, C.HOME_POSE[name]) for name in C.ARM_JOINTS["right"]
            ]
            solver.set_neutral_config(left, right)
            if save_current_as_initial:
                solver.save_initial_q()
        self.get_logger().info(
            f"IK 就绪: {resolved}（末端 {solver.ee_frames}, "
            f"限位约束 {solver._joint_limit_override_count} 个关节）"
        )
        return True

    def _sync_ik_from_current_state_locked(self) -> bool:
        if self.ik_solver is None:
            return False
        current = self.get_current_positions()
        if not current:
            return False
        self.ik_solver.sync_joint_positions(
            list(current.keys()), [current[name] for name in current]
        )
        return True

    def _ensure_ik_initialized(self) -> bool:
        if self.ik_solver is not None and self._ik_initialized:
            return True
        return self.initialize_ik()

    def get_ee_pose(self, side: str, as_dict: bool = False):
        """当前末端位姿（base frame ``[x,y,z,roll,pitch,yaw]``）。"""
        if side not in ("left", "right"):
            raise ValueError(f"无效的 arm side: {side}")
        if not self._ensure_ik_initialized():
            return None
        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            pose = self.ik_solver.get_ee_pose(side)
        return dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), pose.tolist())) if as_dict else pose

    def get_ee_poses(self, as_dict: bool = False):
        if not self._ensure_ik_initialized():
            return None
        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            poses = self.ik_solver.get_both_ee_poses()
        if as_dict:
            return {
                side: dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), pose.tolist()))
                for side, pose in poses.items()
            }
        return poses

    def fk_ee_pose(self, side: str, joint_positions: dict[str, float]):
        """按给定关节角做正运动学，返回基座系 ``[x,y,z,roll,pitch,yaw]``。

        ``get_ee_pose`` 永远用**实测**关节角；这里用调用方给的**命令**关节角，
        回答"这条命令下发后末端应该在哪"。隐式 PD 没有重力前馈时，两者稳态差
        ``τ_g/kp``（预备位 0.13 m）；2026-10-27 加了仿真侧前馈后，一般姿态只剩
        十几毫米的 URDF↔USD 力矩差异（2 号路径点实测 72 mm → 5.9 mm）。

        预备位是个例外：它的目标是刻意保留的旧版下垂姿态 ``READY_POSE_TARGET``
        （见 constants），所以传**设计**姿态 ``READY_POSE`` 进来仍会差 0.13 m，
        传 ``READY_POSE_TARGET`` 只差 15 mm。遥操作的目标变量就用
        ``READY_POSE_TARGET`` 播种（``keyboard_ee_control._seed_target_from_ready``），
        与实际姿态只差毫米级，第一次按键不会跳。

        未出现在 ``joint_positions`` 里的关节保持求解器当前的 ``q``（通常已同步
        过实测）；算完恢复，不污染 IK 热启动。
        """
        if side not in ("left", "right"):
            raise ValueError(f"无效的 arm side: {side}")
        if not self._ensure_ik_initialized():
            return None
        with self._ik_lock:
            solver = self.ik_solver
            saved = solver.q.copy()
            try:
                for raw_name, value in joint_positions.items():
                    name = C.normalize_joint_name(raw_name) or raw_name
                    if not solver.model.existJointName(name):
                        continue
                    jid = solver.model.getJointId(name)
                    solver.q[solver.model.joints[jid].idx_q] = float(value)
                return solver.get_ee_pose(side)
            finally:
                solver.q[:] = saved

    def solve_arm_ik(
        self,
        side: str,
        target_xyzrpy: Sequence[float],
        unlock_waist: bool = False,
        num_random_seeds: int = 0,
        **ik_kwargs: Any,
    ) -> tuple[dict[str, float] | None, bool, dict | None]:
        """单臂 IK，返回 ``({关节名: 目标角}, 是否收敛, 诊断)``。"""
        if not self._ensure_ik_initialized():
            return None, False, None
        from utils.ik import TienkungDexIK

        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            active_q, ok = self.ik_solver.solve_ik_single_arm(
                TienkungDexIK.xyzrpy_to_se3(target_xyzrpy),
                side,
                unlock_waist=unlock_waist,
                num_random_seeds=num_random_seeds,
                **ik_kwargs,
            )
            diagnostics = self.ik_solver.last_fail_info
        names = list(C.ARM_JOINTS[side])
        if unlock_waist:
            names = list(C.WAIST_JOINTS) + names
        return dict(zip(names, [float(v) for v in active_q])), ok, diagnostics

    def move_arm_ik(
        self,
        side: str,
        target_xyzrpy: Sequence[float],
        duration_sec: float = 1.5,
        wait: bool = True,
        require_success: bool = True,
        allow_approximate: float = 0.02,
        unlock_waist: bool = False,
        num_random_seeds: int = 0,
        ramp_from_commanded: bool = False,
        **ik_kwargs: Any,
    ) -> bool:
        """解 IK 并移动。

        ``allow_approximate``：未严格收敛但位置残差小于该值（米）时仍然执行，
        只告警。默认 2 cm——仿真里 6 mm 的残差对遥操作/抓取没有意义，直接放弃
        反而更糟；要严格收敛传 ``allow_approximate=0.0``。

        ``ramp_from_commanded``：透传给 ``move_to_pose``，从上次下发的关节目标
        起步（详见该方法说明）。遥操作的连续小步下发应打开它。
        """
        joint_targets, ok, diagnostics = self.solve_arm_ik(
            side,
            target_xyzrpy,
            unlock_waist=unlock_waist,
            num_random_seeds=num_random_seeds,
            **ik_kwargs,
        )
        if joint_targets is None:
            return False
        if not ok:
            pos_err = float((diagnostics or {}).get("pos_err", float("inf")))
            rot_err = float((diagnostics or {}).get("rot_err", float("inf")))
            if not require_success:
                self.get_logger().warning(f"{side} 臂 IK 未收敛（require_success=False，继续执行）: {diagnostics}")
            elif pos_err <= allow_approximate and rot_err <= 3.0 * allow_approximate:
                self.get_logger().warning(
                    f"{side} 臂 IK 近似收敛（位置残差 {pos_err * 1000:.1f}mm / 姿态 {rot_err:.4f}rad），继续执行"
                )
            else:
                self.get_logger().warning(f"{side} 臂 IK 未收敛，放弃: {diagnostics}")
                return False
        return self.move_to_pose(
            joint_targets,
            duration_sec=duration_sec,
            wait=wait,
            ramp_from_commanded=ramp_from_commanded,
        )

    def solve_arm_ik_position(
        self,
        side: str,
        target_xyz: Sequence[float],
        unlock_waist: bool = False,
        num_random_seeds: int = 0,
        **ik_kwargs: Any,
    ) -> tuple[dict[str, float] | None, bool, dict | None]:
        """只解位置的单臂 IK（姿态不参与），见 ``TienkungDexIK.solve_ik_position_only``。"""
        if not self._ensure_ik_initialized():
            return None, False, None
        with self._ik_lock:
            self._sync_ik_from_current_state_locked()
            active_q, ok = self.ik_solver.solve_ik_position_only(
                target_xyz,
                side,
                unlock_waist=unlock_waist,
                num_random_seeds=num_random_seeds,
                **ik_kwargs,
            )
            diagnostics = self.ik_solver.last_fail_info
        names = list(C.ARM_JOINTS[side])
        if unlock_waist:
            names = list(C.WAIST_JOINTS) + names
        return dict(zip(names, [float(v) for v in active_q])), ok, diagnostics

    def move_arm_ik_position(
        self,
        side: str,
        target_xyz: Sequence[float],
        duration_sec: float = 1.5,
        wait: bool = True,
        require_success: bool = True,
        allow_approximate: float = 0.01,
        unlock_waist: bool = False,
        num_random_seeds: int = 0,
        **ik_kwargs: Any,
    ) -> bool:
        """解「只约束位置」的 IK 并移动（手腕姿态由调用方单独下发）。

        ``allow_approximate`` 默认 1 cm（比 6 自由度的 2 cm 严）：抓取位置直接
        决定手指能不能合到物体上，残差 1 cm 以上就不该继续，宁可报失败。
        """
        joint_targets, ok, diagnostics = self.solve_arm_ik_position(
            side,
            target_xyz,
            unlock_waist=unlock_waist,
            num_random_seeds=num_random_seeds,
            **ik_kwargs,
        )
        if joint_targets is None:
            return False
        if not ok:
            pos_err = float((diagnostics or {}).get("pos_err", float("inf")))
            if pos_err <= allow_approximate:
                self.get_logger().warning(
                    f"{side} 臂位置 IK 近似收敛（残差 {pos_err * 1000:.1f}mm），继续执行"
                )
            else:
                self.get_logger().warning(f"{side} 臂位置 IK 未收敛，放弃: {diagnostics}")
                return False
        return self.move_to_pose(joint_targets, duration_sec=duration_sec, wait=wait)

    def move_arm_ee_delta(
        self,
        side: str,
        delta_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        delta_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        duration_sec: float = 1.5,
        wait: bool = True,
        require_success: bool = True,
        **ik_kwargs: Any,
    ) -> bool:
        """末端在 base frame 下的相对位移。"""
        current = self.get_ee_pose(side)
        if current is None:
            return False
        target = np.asarray(current, dtype=float) + np.concatenate(
            [np.asarray(delta_xyz, dtype=float), np.asarray(delta_rpy, dtype=float)]
        )
        return self.move_arm_ik(
            side, target, duration_sec=duration_sec, wait=wait,
            require_success=require_success, **ik_kwargs,
        )

    def move_arm_ee_delta_local(
        self,
        side: str,
        delta_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        delta_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        duration_sec: float = 1.5,
        wait: bool = True,
        require_success: bool = True,
        **ik_kwargs: Any,
    ) -> bool:
        """末端在自身坐标系下的相对位移（z=手掌前方，x/y 见末端 frame 定义）。"""
        current = self.get_ee_pose(side)
        if current is None:
            return False
        _, target = _convert_ee_delta_local_to_base(current, delta_xyz, delta_rpy)
        return self.move_arm_ik(
            side, target, duration_sec=duration_sec, wait=wait,
            require_success=require_success, **ik_kwargs,
        )

    def move_arm_ee_delta_world(
        self,
        side: str,
        delta_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        delta_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        duration_sec: float = 1.5,
        wait: bool = True,
        require_success: bool = True,
        **ik_kwargs: Any,
    ) -> bool:
        """末端在世界系下的相对位移（世界系来自 constants.ROBOT_WORLD_POS）。"""
        current = self.get_ee_pose(side)
        if current is None:
            return False
        world = self.base_xyzrpy_to_world(current)
        world_target = np.asarray(world, dtype=float) + np.concatenate(
            [np.asarray(delta_xyz, dtype=float), np.asarray(delta_rpy, dtype=float)]
        )
        target = self.world_xyzrpy_to_base(world_target)
        return self.move_arm_ik(
            side, target, duration_sec=duration_sec, wait=wait,
            require_success=require_success, **ik_kwargs,
        )

    def move_dual_arm_ik(
        self,
        left_target_xyzrpy: Sequence[float] | None = None,
        right_target_xyzrpy: Sequence[float] | None = None,
        duration_sec: float = 1.5,
        wait: bool = True,
        require_success: bool = True,
        **ik_kwargs: Any,
    ) -> bool:
        targets: dict[str, float] = {}
        success = True
        for side, target in (("left", left_target_xyzrpy), ("right", right_target_xyzrpy)):
            if target is None:
                continue
            joint_targets, ok, diagnostics = self.solve_arm_ik(side, target, **ik_kwargs)
            if joint_targets is None:
                return False
            if not ok:
                success = False
                if require_success:
                    self.get_logger().warning(f"{side} 臂 IK 未收敛: {diagnostics}")
                    return False
            targets.update(joint_targets)
        return self.move_to_pose(targets, duration_sec=duration_sec, wait=wait) and success

    def move_dual_ee_delta(
        self,
        left_delta_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        left_delta_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        right_delta_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        right_delta_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        **kwargs: Any,
    ) -> bool:
        left = self.get_ee_pose("left")
        right = self.get_ee_pose("right")
        if left is None or right is None:
            return False
        left_target = np.asarray(left, dtype=float) + np.concatenate(
            [np.asarray(left_delta_xyz, dtype=float), np.asarray(left_delta_rpy, dtype=float)]
        )
        right_target = np.asarray(right, dtype=float) + np.concatenate(
            [np.asarray(right_delta_xyz, dtype=float), np.asarray(right_delta_rpy, dtype=float)]
        )
        return self.move_dual_arm_ik(left_target, right_target, **kwargs)

    # ------------------------------------------------------------ 坐标系换算
    @staticmethod
    def world_xyzrpy_to_base(world_xyzrpy: Sequence[float]) -> np.ndarray:
        R_base_to_world = _quat_wxyz_to_matrix(C.ROBOT_WORLD_ROT_WXYZ)
        delta = np.asarray(world_xyzrpy[:3], dtype=float) - np.asarray(C.ROBOT_WORLD_POS)
        base_xyz = R_base_to_world.T @ delta
        R_world = _rpy_to_rotation_matrix(*[float(v) for v in world_xyzrpy[3:6]])
        roll, pitch, yaw = _rotation_matrix_to_rpy(R_base_to_world.T @ R_world)
        return np.array([*base_xyz, roll, pitch, yaw])

    @staticmethod
    def world_xyz_delta_to_base(delta_world: Sequence[float]) -> np.ndarray:
        """世界系**位移**（3 维）-> pelvis 系位移：只转不平移。

        位姿换算走 :meth:`world_xyzrpy_to_base`（要减机器人位置），而位移没有
        原点，只乘姿态的转置。当前 ``ROBOT_WORLD_ROT_WXYZ`` 是单位四元数，
        两个坐标系同向；保留旋转是防止以后机器人摆位改了。
        """
        R_base_to_world = _quat_wxyz_to_matrix(C.ROBOT_WORLD_ROT_WXYZ)
        return R_base_to_world.T @ np.asarray(delta_world, dtype=float)

    @staticmethod
    def base_xyzrpy_to_world(base_xyzrpy: Sequence[float]) -> np.ndarray:
        R_base_to_world = _quat_wxyz_to_matrix(C.ROBOT_WORLD_ROT_WXYZ)
        world_xyz = R_base_to_world @ np.asarray(base_xyzrpy[:3], dtype=float) + np.asarray(
            C.ROBOT_WORLD_POS
        )
        R_base = _rpy_to_rotation_matrix(*[float(v) for v in base_xyzrpy[3:6]])
        roll, pitch, yaw = _rotation_matrix_to_rpy(R_base_to_world @ R_base)
        return np.array([*world_xyz, roll, pitch, yaw])

    # ==================================================================
    # 复位 / 监视
    # ==================================================================
    def set_apple_offset(self, x: float, y: float, repeat: int = 3, interval: float = 0.15) -> None:
        """把苹果从「复位后的位姿」挪 (x, y)（世界系，米）。只对仿真有效。

        真机上没人订阅这个偏移话题，发出去没有副作用。偏移是相对量：仿真侧按
        「复位位姿 + 偏移」摆苹果，所以调用前场景应当刚复位过（批量采集每集都
        先 reset），否则基准会变成苹果当前所在的位置（比如盘子里）。

        与 :meth:`reset_sim` 一样连发几次：脚本刚起来时发现可能还没完成，单发
        会静默丢掉，苹果不动而航点照偏移走，抓空一集。
        """
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        for _ in range(max(1, repeat)):
            self._pubs["apple_offset"].publish(msg)
            time.sleep(interval)
        self.get_logger().info(f"苹果偏移已下发: x={x:+.3f} m, y={y:+.3f} m（世界系）")

    def reset_sim(self, repeat: int = 3, interval: float = 0.15) -> None:
        """复位仿真场景（真机没有对应接口，真机上调用无意义）。"""
        msg = Bool()
        msg.data = True
        for _ in range(max(1, repeat)):
            self._pubs["reset"].publish(msg)
            time.sleep(interval)
        self.get_logger().info(f"已发送复位指令到 {TOPICS['reset']}")

    def monitor_joints(
        self,
        joint_names: Sequence[str] | None = None,
        hz: float = 10.0,
        duration_sec: float | None = None,
    ) -> None:
        names = [C.normalize_joint_name(n) or n for n in (joint_names or C.BODY_JOINTS)]
        deadline = None if duration_sec is None else time.monotonic() + duration_sec
        period = 1.0 / max(hz, 1.0)
        try:
            while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
                with self._state_lock:
                    line = ", ".join(
                        f"{name}={self._joint_positions.get(name, float('nan')):+.3f}"
                        for name in names
                    )
                print(f"[state] {line}")
                time.sleep(period)
        except KeyboardInterrupt:
            pass


# 兼容别名：部分文档/脚本用 TienkungDexController 这个名字
TienkungDexController = DexController


# ============================================================================
# 打印辅助
# ============================================================================
def _fmt(values: Iterable[float] | None, precision: int = 4) -> str:
    if values is None:
        return "None"
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"


def print_joint_states(controller: DexController, joint_names: Sequence[str] | None = None) -> None:
    names = [C.normalize_joint_name(n) or n for n in (joint_names or C.BODY_JOINTS)]
    current = controller.get_current_positions()
    print("\n=== 身体关节（rad） ===")
    for name in names:
        value = current.get(name)
        print(f"  {name:<28} {'—' if value is None else f'{value:+.4f}'}")


def print_hand_states(controller: DexController, sides: Sequence[str] = ("left", "right")) -> None:
    print("\n=== 灵巧手（rad） ===")
    for side in sides:
        values = controller.get_hand_positions(side)
        print(f"  [{side}]")
        for name in controller.hand_joint_names(side):
            value = values.get(name)
            print(f"    {name:<34} {'—' if value is None else f'{value:+.4f}'}")


def print_ee_poses(controller: DexController, sides: Sequence[str] = ("left", "right")) -> None:
    print("\n=== 末端位姿（base frame, xyz m / rpy rad） ===")
    for side in sides:
        pose = controller.get_ee_pose(side)
        if pose is None:
            print(f"  [{side}] 无数据（IK 未初始化？）")
            continue
        print(f"  [{side}] xyz={_fmt(pose[:3])} rpy={_fmt(pose[3:])}")


# ============================================================================
# CLI 公共参数
# ============================================================================
def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--variant", choices=list(C.HAND_JOINTS_BY_VARIANT),
                        default=C.DEFAULT_VARIANT, help="手部变体")
    parser.add_argument("--control-hz", type=float, default=C.DEFAULT_CONTROL_HZ,
                        help="指令下发频率")
    parser.add_argument("--no-lock", action="store_true", help="不锁定任何关节（默认锁定双腿）")
    parser.add_argument("--include-legs", action="store_true",
                        help="解锁并下发腿部关节（默认锁腿）")
    parser.add_argument("--no-safety", action="store_true", help="关闭最大速度检查")
    parser.add_argument("--no-limits", action="store_true", help="关闭关节限位裁剪")
    parser.add_argument("--ik-urdf", default=None, help="IK 使用的 URDF（默认按变体从 assets 取）")


def _make_controller(args: argparse.Namespace, node_name: str, enable_ik: bool) -> DexController:
    lock = None
    if not args.no_lock:
        lock = [] if args.include_legs else C.LEG_JOINTS
    return DexController(
        node_name=node_name,
        variant=args.variant,
        control_hz=args.control_hz,
        enable_safety_check=not args.no_safety,
        enable_limit_check=not args.no_limits,
        lock_joints=lock,
        enable_ik=enable_ik,
        ik_urdf_path=args.ik_urdf,
    )


def _resolve_sides(value: str) -> list[str]:
    return ["left", "right"] if value == "both" else [value]


def _shutdown(
    controller: DexController,
    executor,
    spin_thread: threading.Thread,
    extra_nodes: list | None = None,
) -> None:
    """按 walker_s2 的顺序收尾：停下发 → 停 executor → 等 spin 线程 → 销毁节点。

    顺序错了（先 destroy_node 再等线程）会让仍在 spin 的线程访问已销毁的节点，
    DDS 层直接 ``terminate called without an active exception`` 并 core dump。

    ``extra_nodes`` 是同一个 executor 上的附加节点（如采集用的 recorder），
    与 controller 一起按同样的顺序收尾。
    """
    controller.stop()
    time.sleep(0.1)
    try:
        executor.shutdown(timeout_sec=1.0)
    except TypeError:  # 老版本 rclpy 没有 timeout_sec
        executor.shutdown()
    spin_thread.join(timeout=2.0)
    if spin_thread.is_alive():
        controller.get_logger().warning("Executor spin 线程未在超时内退出")
    for node in extra_nodes or []:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        node.destroy_node()
    try:
        executor.remove_node(controller)
    except Exception:
        pass
    controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main(args: Sequence[str] | None = None) -> None:
    """state 子命令：状态查询 / 单关节 / 预备位 / home / 手部开合。"""
    parser = argparse.ArgumentParser(description="TienKung Dex 基础控制器 - 状态与常用动作")
    _add_common_args(parser)
    parser.add_argument("--print-state", action="store_true", help="打印身体关节状态")
    parser.add_argument("--print-hand", action="store_true", help="打印双手状态")
    parser.add_argument("--print-ee", action="store_true", help="打印末端位姿（自动启用 IK）")
    parser.add_argument("--ee-side", choices=["left", "right", "both"], default="both")
    parser.add_argument(
        "--init", action="store_true",
        help="移动到预备抓取位（READY_POSE + 预备位手型：拇指侧摆内收 850 = 90°）",
    )
    parser.add_argument("--ready", action="store_true", help="同 --init")
    parser.add_argument("--no-staged", action="store_true", help="预备位不分段，一次到位")
    parser.add_argument("--home", action="store_true", help="分段回站立位 + 张开双手")
    parser.add_argument(
        "--duration", type=float, default=C.DEFAULT_MOVE_DURATION,
        help="运动时长（秒）；--home 最少 10 s，--init 不给则按每段 5 s 分段",
    )
    parser.add_argument("--move-joint", action="store_true", help="移动单个身体关节")
    parser.add_argument("--joint", default=None, help="--move-joint / --shift-joint 的关节名")
    parser.add_argument("--pos", type=float, default=None, help="--move-joint 目标角（rad）")
    parser.add_argument("--shift-joint", action="store_true", help="相对当前值偏移单个关节")
    parser.add_argument("--delta", type=float, default=None, help="--shift-joint 偏移量（rad）")
    parser.add_argument("--hand-open", action="store_true", help="张开手")
    parser.add_argument("--hand-close", action="store_true", help="握拳")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--interactive", action="store_true", help="打印状态后保持节点运行")
    cli = parser.parse_args(list(args) if args is not None else None)

    rclpy.init()
    controller = _make_controller(cli, "tienkung_dex_state", enable_ik=cli.print_ee)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        controller.wait_for_state(timeout=5.0)

        wants_dump = cli.print_state or cli.print_hand or cli.print_ee
        perform_action = any((
            cli.init, cli.ready, cli.home, cli.move_joint, cli.shift_joint,
            cli.hand_open, cli.hand_close,
        ))

        def _dump() -> None:
            if cli.print_ee:
                print_ee_poses(controller, _resolve_sides(cli.ee_side))
            if cli.print_hand:
                print_hand_states(controller, _resolve_sides(cli.side))
            if cli.print_state:
                print_joint_states(controller)

        if wants_dump:
            _dump()
        elif not perform_action:
            print_joint_states(controller)  # 什么都没指定：默认打印状态

        if cli.init or cli.ready:
            # --duration 保持默认值（C.DEFAULT_MOVE_DURATION）时视为"没指定"，
            # 走 constants.READY_STAGE_DURATIONS 的分段节奏（每段 5 s，合计 20 s）；
            # 显式给了才按均分覆盖。
            ready_duration = (
                None
                if cli.duration == C.DEFAULT_MOVE_DURATION
                else max(cli.duration, 10.0)
            )
            ok = controller.move_to_ready_pose(
                duration_sec=ready_duration, wait=True, staged=not cli.no_staged
            )
            print("✓ 已到预备位" if ok else "✗ 预备位未到位")
        if cli.home:
            # 倒放四段共 10 s，与 --init 一致
            controller.home(duration_sec=max(cli.duration, 10.0), wait=True,
                            include_legs=cli.include_legs)
        if cli.move_joint:
            if cli.joint is None or cli.pos is None:
                parser.error("--move-joint 需要 --joint 和 --pos")
            controller.move_joint(cli.joint, cli.pos, duration_sec=cli.duration)
        if cli.shift_joint:
            if cli.joint is None or cli.delta is None:
                parser.error("--shift-joint 需要 --joint 和 --delta")
            controller.shift_joint(cli.joint, cli.delta, duration_sec=cli.duration)
        if cli.hand_open or cli.hand_close:
            for side in _resolve_sides(cli.side):
                if cli.hand_open:
                    controller.open_hand(side, duration_sec=cli.duration, wait=True)
                else:
                    controller.close_hand(side, duration_sec=cli.duration, wait=True)

        if perform_action:
            if wants_dump:
                print("\n--- 动作后 ---")
                _dump()
            else:
                print_joint_states(controller)
        if cli.interactive:
            print("\n节点运行中，按 Ctrl+C 退出。")
            spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(controller, executor, spin_thread)


def main_joint(args: Sequence[str] | None = None) -> None:
    """joint 子命令：单关节调试 / 手部开合 / 连续监视。"""
    parser = argparse.ArgumentParser(description="TienKung Dex 关节调试")
    _add_common_args(parser)
    parser.add_argument("--print", action="store_true", help="打印关节状态")
    parser.add_argument("--move", action="append", default=[], metavar="JOINT=ANGLE",
                        help="移动关节到绝对角度（可重复）")
    parser.add_argument("--shift", action="append", default=[], metavar="JOINT=DELTA",
                        help="关节相对偏移（可重复）")
    parser.add_argument("--hand-open", action="store_true")
    parser.add_argument("--hand-close", action="store_true")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--monitor", action="store_true", help="持续打印关节状态")
    parser.add_argument("--joints", default="", help="监视的关节名（逗号分隔，默认全部身体关节）")
    parser.add_argument("--duration", type=float, default=C.DEFAULT_JOINT_DURATION)
    cli = parser.parse_args(list(args) if args is not None else None)

    def _parse_pairs(items: list[str]) -> dict[str, float]:
        pairs: dict[str, float] = {}
        for item in items:
            if "=" not in item:
                parser.error(f"格式应为 NAME=VALUE，收到 {item}")
            name, value = item.split("=", 1)
            pairs[name.strip()] = float(value)
        return pairs

    moves = _parse_pairs(cli.move)
    shifts = _parse_pairs(cli.shift)

    rclpy.init()
    controller = _make_controller(cli, "tienkung_dex_joint", enable_ik=False)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        controller.wait_for_state(timeout=5.0)
        if cli.print or not (moves or shifts or cli.hand_open or cli.hand_close or cli.monitor):
            print_joint_states(controller)
        if moves:
            print(f"移动: {moves}")
            controller.move_to_pose(moves, duration_sec=cli.duration)
        for name, delta in shifts.items():
            print(f"偏移: {name} {delta:+.3f}")
            controller.shift_joint(name, delta, duration_sec=cli.duration)
        if cli.hand_open or cli.hand_close:
            for side in _resolve_sides(cli.side):
                if cli.hand_open:
                    controller.open_hand(side, duration_sec=cli.duration, wait=True)
                else:
                    controller.close_hand(side, duration_sec=cli.duration, wait=True)
            print_hand_states(controller, _resolve_sides(cli.side))
        if cli.monitor:
            names = [n for n in cli.joints.split(",") if n] or None
            controller.monitor_joints(names)
            spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(controller, executor, spin_thread)


def main_endpoint(args: Sequence[str] | None = None) -> None:
    """endpoint 子命令：末端位姿读取 / Cartesian IK / EE 相对位移。"""
    parser = argparse.ArgumentParser(description="TienKung Dex 末端位姿与笛卡尔控制")
    _add_common_args(parser)
    parser.add_argument("--side", choices=["left", "right"], default="right")
    parser.add_argument("--no-move", action="store_true", help="只打印当前末端位姿，不运动")
    parser.add_argument("--xyz", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                        help="绝对目标位置（base frame, m）")
    parser.add_argument("--rpy", type=float, nargs=3, default=None, metavar=("R", "P", "Y"),
                        help="绝对目标姿态（base frame, rad）；缺省保持当前姿态")
    parser.add_argument("--delta-xyz", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=("DX", "DY", "DZ"), help="相对位移（m）")
    parser.add_argument("--delta-rpy", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=("DR", "DP", "DY"), help="相对姿态增量（rad）")
    parser.add_argument("--local", action="store_true", help="delta 使用末端自身坐标系")
    parser.add_argument("--world", action="store_true", help="delta/目标使用世界坐标系")
    parser.add_argument("--unlock-waist", action="store_true", help="允许腰部参与 IK")
    parser.add_argument("--duration", type=float, default=1.5)
    cli = parser.parse_args(list(args) if args is not None else None)

    rclpy.init()
    controller = _make_controller(cli, "tienkung_dex_endpoint", enable_ik=True)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        controller.wait_for_state(timeout=5.0)
        before = controller.get_ee_pose(cli.side)
        print(f"[{cli.side}] 当前末端 (base): {_fmt(before)}")
        if cli.no_move:
            return
        if cli.xyz is not None:
            target = list(cli.xyz) + (list(cli.rpy) if cli.rpy is not None else list(before[3:]))
            if cli.world:
                target = controller.world_xyzrpy_to_base(target)
            ok = controller.move_arm_ik(
                cli.side, target, duration_sec=cli.duration, unlock_waist=cli.unlock_waist
            )
        else:
            if cli.local:
                ok = controller.move_arm_ee_delta_local(
                    cli.side, cli.delta_xyz, cli.delta_rpy, duration_sec=cli.duration
                )
            elif cli.world:
                ok = controller.move_arm_ee_delta_world(
                    cli.side, cli.delta_xyz, cli.delta_rpy, duration_sec=cli.duration
                )
            else:
                ok = controller.move_arm_ee_delta(
                    cli.side, cli.delta_xyz, cli.delta_rpy, duration_sec=cli.duration
                )
        after = controller.get_ee_pose(cli.side)
        print(f"[{cli.side}] 目标后末端 (base): {_fmt(after)}")
        if before is not None and after is not None:
            delta = [float(after[i]) - float(before[i]) for i in range(6)]
            print(f"[{cli.side}] 实际变化: xyz={_fmt(delta[:3])} rpy={_fmt(delta[3:])}")
        print("✓ 完成" if ok else "✗ 失败")
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(controller, executor, spin_thread)


def main_hand(args: Sequence[str] | None = None) -> None:
    """hand 子命令：灵巧手开合 / 指定角度 / 归一化闭合度 / 单指正弦。"""
    parser = argparse.ArgumentParser(description="TienKung Dex 灵巧手控制")
    _add_common_args(parser)
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--open", action="store_true", help="张开（全部归零）")
    parser.add_argument("--close", action="store_true", help="握拳（闭合到限位）")
    parser.add_argument("--percent", type=float, default=None,
                        help="所有关节按闭合度百分比下发（0=张开，1=闭合）")
    parser.add_argument("--pose", action="append", default=[], metavar="JOINT=RAD",
                        help="指定关节目标角（可重复）")
    parser.add_argument("--periodic", default=None, metavar="JOINT",
                        help="对单个关节做正弦开合（配合 --amplitude/--period/--cycles）")
    parser.add_argument("--amplitude", type=float, default=None, help="正弦振幅（rad，默认取半个行程）")
    parser.add_argument("--period", type=float, default=2.0, help="正弦周期（s）")
    parser.add_argument("--cycles", type=int, default=3, help="正弦周期数")
    parser.add_argument("--duration", type=float, default=1.0, help="开合/到位时长（s）")
    parser.add_argument("--print", action="store_true", help="打印双手状态（默认无动作时也打印）")
    cli = parser.parse_args(list(args) if args is not None else None)

    rclpy.init()
    controller = _make_controller(cli, "tienkung_dex_hand", enable_ik=False)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        controller.wait_for_state(timeout=5.0)
        sides = _resolve_sides(cli.side)
        did_action = False

        if cli.percent is not None:
            percent = min(max(float(cli.percent), 0.0), 1.0)
            for side in sides:
                targets = {
                    name: percent * C.action_limit(name, cli.variant)
                    for name in controller.hand_joint_names(side)
                }
                controller.move_hand(side, targets, duration_sec=cli.duration, wait=True)
            did_action = True
        if cli.pose:
            targets = {}
            for item in cli.pose:
                if "=" not in item:
                    parser.error(f"--pose 格式应为 JOINT=RAD，收到 {item}")
                name, value = item.split("=", 1)
                targets[name.strip()] = float(value)
            for side in sides:
                controller.move_hand(side, targets, duration_sec=cli.duration, wait=True)
            did_action = True
        if cli.open:
            for side in sides:
                controller.open_hand(side, duration_sec=cli.duration, wait=True)
            did_action = True
        if cli.close:
            for side in sides:
                controller.close_hand(side, duration_sec=cli.duration, wait=True)
            did_action = True
        if cli.periodic:
            side = sides[0]
            name = C.normalize_joint_name(cli.periodic) or cli.periodic
            limit = C.action_limit(name, cli.variant)
            amplitude = cli.amplitude if cli.amplitude is not None else abs(limit) / 2.0
            controller.hand_periodic_motion(
                side, name, amplitude=amplitude, period_sec=cli.period,
                cycles=cli.cycles, offset=abs(limit) / 2.0,
            )
            did_action = True

        if did_action:
            print("\n--- 动作后 ---")
        print_hand_states(controller, sides)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(controller, executor, spin_thread)


def main_home(args: Sequence[str] | None = None) -> None:
    """home 子命令：分段回站立位 + 张开双手。"""
    parser = argparse.ArgumentParser(description="TienKung Dex 回站立位")
    _add_common_args(parser)
    parser.add_argument("--duration", type=float, default=10.0, help="总时长（秒）")
    parser.add_argument("--no-staged", action="store_true", help="不做分段，一次到位")
    parser.add_argument("--keep-hands", action="store_true", help="不改手部姿态")
    cli = parser.parse_args(list(args) if args is not None else None)

    rclpy.init()
    controller = _make_controller(cli, "tienkung_dex_home", enable_ik=False)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        controller.wait_for_state(timeout=5.0)
        ok = controller.home(
            duration_sec=cli.duration,
            staged=not cli.no_staged,
            include_legs=cli.include_legs,
            open_hands=not cli.keep_hands,
        )
        print("✓ 已回站立位" if ok else "✗ 回站立位失败")
        print_joint_states(controller)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(controller, executor, spin_thread)


if __name__ == "__main__":
    main()
