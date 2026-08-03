
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 简化零件抓取脚本（part_sorting 新场景）。

设计要点
========

1. **末端偏移运动（delta）执行**：抓取/放置的每一段运动都通过
   ``controller.move_arm_ee_delta_world`` 下发。该方法内部读取当前 EE 位姿，
   以 ``current + delta`` 作为 IK 目标并从当前关节状态 warm-start，小位移几乎
   必然可解，规避了"采样绝对目标点再求 IK 经常无解"的问题。大位移由
   ``_move_ee_to`` 自动按 ``--ee-step-m`` 细分成多段小 delta，每段前重新读取
   当前位姿，避免漂移。

2. **实时位姿 + 保持当前姿态（水平移动）**：从 ``/sim/part_states`` 读取每个零件的
   实时 world 位置（处理 ±5cm 随机化）。默认**不旋转夹爪**，仅平移到目标（水平
   移动）；如需固定姿态可用 ``--grasp-rpy-deg`` 指定 world RPY（度）。

3. **全 world frame**：所有路点在 world frame 计算，坐标转换由控制器内部完成，
   pick_part.py 不再维护 world↔base 转换矩阵（仅保留一个 ~15 行的
   ``_ee_world_pose`` 用于计算 delta）。

4. **保留能力**：放置入箱、多零件顺序、``--save`` 录制、``--step`` 分步调试、
   ``--dry-run``。

运行前置：仿真/真机端启动 ROS2 bridge；本容器内 source ROS2 + Walker SDK 环境。
"""

import argparse
import json
import os
import sys
import threading
import time
from contextlib import contextmanager
from copy import deepcopy

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# 支持直接运行和包导入两种方式
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from utils.camera import Camera  # noqa: E402
from utils.controller import (  # noqa: E402
    CAMERA_TOPICS,
    DEFAULT_IMAGE_DEPTH_TOPIC,
    READY_POSE,
    ROBOT_WORLD_POS,
    ROBOT_WORLD_ROT_WXYZ,
    WalkerS2Controller,
)

# ============================================================================
# [Block 1] 常量定义
# ============================================================================

# ---- ROS topic ----
DEFAULT_PART_STATES_TOPIC = "/sim/part_states"
DEFAULT_RANDOMIZE_PARTS_TOPIC = "/sim/cmd_randomize_parts"

# ---- 时序 / 超时 ----
DEFAULT_RESET_SCENE_SETTLE_TIME = 1.0
DEFAULT_TIMEOUT = 5.0

# ---- 机器人初始化 ----
DEFAULT_ROBOT_INIT_DURATION = 10.0
DEFAULT_ROBOT_INIT_SETTLE_TIMEOUT = 10.0
DEFAULT_ROBOT_INIT_TOLERANCE = 0.08

# ---- 零件 / 抓取 ----
DEFAULT_PART_NAME = "part_a_red"
DEFAULT_PART_SEQUENCE = ("part_a_ori", "part_a_red", "part_b_blue", "part_b_ori")
DEFAULT_SIDE = "right"

# 默认不旋转夹爪（None=保持当前姿态，仅平移/水平移动）。如需固定 world RPY，
# 用 --grasp-rpy-deg 指定（度）；脚本会打印 target/achieved 位姿辅助调试。
DEFAULT_GRASP_RPY_WORLD_DEG = None
DEFAULT_PREGRASP_HEIGHT = 0.10          # pregrasp 在零件上方 world z 偏移
DEFAULT_PREGRASP_Y_OFFSET = 0.10       # pregrasp 在零件后方 world -y 偏移
DEFAULT_LIFT_HEIGHT = 0.20             # 抓起后 world z 抬升
DEFAULT_GRASP_TARGET_OFFSET = (0.0, 0.0, -0.01)  # 抓取目标偏移（零件 world pos 的 xyz 偏移量）
DEFAULT_PREGRASP_PITCH_DOWN_DEG = 20.0  # pregrasp 后 EE 局部 y 轴俯仰向下（度）
DEFAULT_DURATION = 1.5               # 每段 delta 轨迹时长（s）
DEFAULT_GRIPPER_DURATION = 1.0          # 夹爪开合等待超时（s）

# ---- IK 默认 ----
DEFAULT_ROT_WEIGHT = 0.10
DEFAULT_UNLOCK_WAIST = False
# IK 旋转轴权重（EE 局部系 x/y/z）：x=夹爪下方(≈世界/base z, 即 yaw)释放；
# y=夹爪左方(俯仰)、z=夹爪前方(翻滚)约束。即"允许绕竖直轴旋转，减少俯仰/翻滚"。
DEFAULT_ROT_AXIS_WEIGHTS = (0.0, 1.0, 1.0)
DEFAULT_REQUIRE_IK_OK = True
DEFAULT_TASK_TYPE = "pick_table"

# ---- 放置 ----
# box 在机器人正前方 [0.75, 0.28, 0.90]；A 类 -> 箱内位置 1（y-0.06），B 类 -> 位置 2（y+0.07）。
DEFAULT_RIGHT_BOX_WORLD_POS1 = (0.75, 0.28 - 0.08, 0.90)
DEFAULT_RIGHT_BOX_WORLD_POS2 = (0.75, 0.28 + 0.08, 0.90)
DEFAULT_PLACE = True
DEFAULT_PLACE_RELEASE_HEIGHT = 0.13
DEFAULT_PLACE_LIFT_HEIGHT = 0.13


# ============================================================================
# [Block 2] PartStateMonitor - 零件状态缓存
# ============================================================================
class PartStateMonitor(Node):
    """缓存 /sim/part_states 的最新零件状态。"""

    def __init__(self, topic=DEFAULT_PART_STATES_TOPIC):
        super().__init__("walker_s2_part_state_monitor")
        self._part_states = None
        self._seq = 0
        self._lock = threading.Lock()
        self._received = threading.Event()
        self._updated = threading.Condition(self._lock)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            String,
            topic,
            self._part_state_callback,
            qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _part_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid part_states JSON: {exc}")
            return
        with self._updated:
            self._part_states = data
            self._seq += 1
            self._received.set()
            self._updated.notify_all()

    def wait_for_part_states(self, timeout=5.0):
        ok = self._received.wait(timeout=timeout)
        if not ok:
            self.get_logger().warning(f"Timeout waiting for part_states ({timeout:.1f}s)")
        return ok

    def get_part_states(self):
        with self._lock:
            return deepcopy(self._part_states)

    def get_update_seq(self):
        with self._lock:
            return self._seq

    def wait_for_new_part_states(self, last_seq, timeout=0.3):
        with self._updated:
            ok = self._updated.wait_for(lambda: self._seq > int(last_seq), timeout=timeout)
            if not ok:
                self.get_logger().warning(f"Timeout waiting for new part_states ({timeout:.2f}s)")
            return ok

    def get_part_pose(self, part_name):
        states = self.get_part_states()
        if not states:
            return None
        return (states.get("parts") or {}).get(part_name)


# ============================================================================
# [Block 3] 模块级工具函数
# ============================================================================

def _fmt(values, precision=4):
    if values is None:
        return "None"
    return "[" + ", ".join(f"{float(v):+.{precision}f}" for v in values) + "]"


# --------------------------------------------------------------------------
# [3.1] 极简旋转数学（仅用于 _ee_world_pose 计算 delta，避免改动控制器）
# --------------------------------------------------------------------------
def _quat_wxyz_to_matrix(q):
    w, x, y, z = [float(v) for v in q]
    n = (w * w + x * x + y * y + z * z) ** 0.5
    if n <= 0.0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _rpy_to_matrix(rpy):
    r, p, y = [float(v) for v in rpy]
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _matrix_to_rpy(rot):
    rot = np.asarray(rot, dtype=float)
    pitch = float(np.arctan2(-rot[2, 0], np.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2)))
    if abs(np.cos(pitch)) > 1e-9:
        roll = float(np.arctan2(rot[2, 1], rot[2, 2]))
        yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
    else:
        roll = 0.0
        yaw = float(np.arctan2(-rot[0, 1], rot[1, 1]))
    return np.array([roll, pitch, yaw], dtype=float)


def _ee_world_pose(controller, side):
    """当前 EE 在 world frame 的 [x,y,z,roll,pitch,yaw]。

    base frame -> world frame，复用控制器模块级常量 ROBOT_WORLD_POS /
    ROBOT_WORLD_ROT_WXYZ（与新场景 robot.init_state 一致）。
    """
    base = controller.get_ee_pose(side)
    if base is None:
        return None
    base = np.asarray(base, dtype=float)
    r_b2w = _quat_wxyz_to_matrix(ROBOT_WORLD_ROT_WXYZ)
    world_xyz = r_b2w @ base[:3] + np.asarray(ROBOT_WORLD_POS, dtype=float)
    world_rpy = _matrix_to_rpy(r_b2w @ _rpy_to_matrix(base[3:]))
    return np.concatenate([world_xyz, world_rpy])


# --------------------------------------------------------------------------
# [3.2] 末端偏移运动（delta）核心
# --------------------------------------------------------------------------
@contextmanager
def _suppress_logger(logger, temporary_level=None):
    """临时将 rclpy logger 级别降为 WARN，退出时恢复。

    用于抑制控制器 move_arm_* 每段大量 INFO 日志
    （Lock joints / Executing trajectory / Waiting converge），
    避免淹没 pick_part 自身的路点日志。
    """
    if temporary_level is None:
        try:
            from rclpy.logging import LoggingSeverity
            temporary_level = LoggingSeverity.WARN
        except Exception:
            temporary_level = None
    if temporary_level is None:
        yield
        return
    try:
        from rclpy.logging import LoggingSeverity
        _restore_level = LoggingSeverity.INFO
    except Exception:
        _restore_level = None
    try:
        logger.set_level(temporary_level)
    except Exception:
        _restore_level = None
    try:
        yield
    finally:
        if _restore_level is not None:
            try:
                logger.set_level(_restore_level)
            except Exception:
                pass


def _move_ee_to(controller, side, target_xyz, target_rpy=None,
                duration_per_step=DEFAULT_DURATION,
                pos_tol=0.02, rot_tol=0.03, max_iters=8, logger=None, label="", **ik_kwargs):
    """一次（或少数几次）delta 把 EE 平移到 target_xyz，不再细分。

    每次直接把完整 delta（target - current）发给 ``move_arm_ee_delta_world``，
    wait=True 阻塞等待 settle 完成后重新读取世界系位姿，检查是否收敛到 target；
    若一次 delta 后未收敛，重发补齐（通常 1–2 次即可）。

    target_rpy 为 None 时**保持当前姿态**（仅平移，不旋转夹爪）。
    """
    target_xyz = np.asarray(target_xyz, dtype=float)
    target_rpy = None if target_rpy is None else np.asarray(target_rpy, dtype=float)

    seg_ik = dict(ik_kwargs, require_success=False, max_iter=300)
    n_attempts = 0
    ok = False
    with _suppress_logger(controller.get_logger()):
        for n_attempts in range(1, int(max_iters) + 1):
            cur = _ee_world_pose(controller, side)
            if cur is None:
                break
            dxyz = target_xyz - cur[:3]
            dist = float(np.linalg.norm(dxyz))
            if target_rpy is None:
                drpy = np.zeros(3, dtype=float)  # 保持当前姿态，不旋转夹爪
            else:
                drpy = (target_rpy - cur[3:] + np.pi) % (2.0 * np.pi) - np.pi
            if dist <= pos_tol and float(np.max(np.abs(drpy))) <= rot_tol:
                ok = True
                break

            # 下发完整 delta，wait=True 保证关节解锁/轨迹/锁恢复时序正确
            seg_ok = controller.move_arm_ee_delta_world(
                side, dxyz.tolist(), drpy.tolist(),
                duration_sec=duration_per_step, wait=True, **seg_ik)
            if not seg_ok:
                # settle 失败不等同于位置没到——马上 resync 检查世界系位置
                cur2 = _ee_world_pose(controller, side)
                d2 = float(np.linalg.norm(target_xyz - cur2[:3])) if cur2 is not None else float("nan")
                if d2 <= pos_tol:
                    ok = True
                    break
                if logger is not None:
                    logger.warning(
                        f"[{label}] attempt {n_attempts}: settle failed, dist={d2:.4f}m "
                        f"(delta xyz={_fmt(dxyz)} rpy={_fmt(drpy)})")
                # 位置也不够 → 继续重试

    if logger is not None:
        if ok:
            logger.info(f"[{label}] reached in {n_attempts} attempt(s)")
        else:
            logger.error(
                f"[{label}] did NOT reach {_fmt(target_xyz)} after {n_attempts} attempt(s)")
    return ok


# --------------------------------------------------------------------------
# [3.3] 放置默认值
# --------------------------------------------------------------------------
def _default_box_pos_for_part(part_name):
    """A 类零件放入箱内位置 1，B 类零件放入箱内位置 2。"""
    name = str(part_name).lower()
    if name.startswith("part_b") or "_b_" in name:
        return DEFAULT_RIGHT_BOX_WORLD_POS2
    return DEFAULT_RIGHT_BOX_WORLD_POS1


# ============================================================================
# [Block 4] 场景管理
# ============================================================================
def initialize_robot_pose(controller, duration_sec=DEFAULT_ROBOT_INIT_DURATION,
                          settle_timeout=DEFAULT_ROBOT_INIT_SETTLE_TIMEOUT,
                          tolerance=DEFAULT_ROBOT_INIT_TOLERANCE, timeout=DEFAULT_TIMEOUT):
    """执行 controller.move_to_ready_pose 等价的分段 READY_POSE 初始化。"""
    if not controller.wait_for_state(timeout=timeout):
        return False
    controller.get_logger().info(
        f"Initialize robot pose via move_to_ready_pose(duration={float(duration_sec):.1f}s)"
    )
    if not controller.move_to_ready_pose(duration_sec=duration_sec):
        controller.get_logger().error("Robot READY_POSE initialization trajectory failed")
        return False
    reached, misses = controller.wait_until_position(
        controller.ready_position_vector(),
        timeout=settle_timeout,
        tolerance=tolerance,
    )
    if reached:
        controller.get_logger().info(f"Robot READY_POSE reached (tolerance <= {float(tolerance):.3f} rad)")
        return True
    controller.get_logger().warning(
        f"Robot READY_POSE not fully reached within {float(settle_timeout):.1f}s "
        f"(tolerance={float(tolerance):.3f} rad)"
    )
    for name, actual, target, err in (misses or [])[:8]:
        if actual is None:
            controller.get_logger().warning(f"  {name}: no state, target={target:+.4f}")
        else:
            controller.get_logger().warning(f"  {name}: actual={actual:+.4f}, target={target:+.4f}, err={err:.4f}")
    return True


def reset_scene(controller, part_monitor=None, timeout=DEFAULT_TIMEOUT,
                settle_time=DEFAULT_RESET_SCENE_SETTLE_TIME):
    """发布仿真场景 reset，并等待零件状态刷新/稳定。"""
    last_seq = (
        part_monitor.get_update_seq()
        if part_monitor is not None and hasattr(part_monitor, "get_update_seq")
        else None
    )
    controller.reset_sim()
    if last_seq is not None and float(timeout) > 0.0:
        if not part_monitor.wait_for_new_part_states(last_seq, timeout=float(timeout)):
            controller.get_logger().error("No part_states update received after scene reset")
            return False
    if float(settle_time) > 0.0:
        time.sleep(float(settle_time))
    return True


def randomize_part_positions(controller, part_monitor, part_names=DEFAULT_PART_SEQUENCE,
                             topic=DEFAULT_RANDOMIZE_PARTS_TOPIC, timeout=DEFAULT_TIMEOUT,
                             settle_time=0.5, seed=None):
    """通过 ROS2 bridge 请求仿真随机化零件位置，并等待 part_states 更新。"""
    part_names = tuple(part_names)
    if not part_names:
        controller.get_logger().error("part_names must not be empty")
        return False
    last_seq = part_monitor.get_update_seq() if hasattr(part_monitor, "get_update_seq") else None
    pub = controller.create_publisher(String, topic, 1)
    time.sleep(0.2)
    if seed is None:
        seed = time.time_ns() & 0xFFFFFFFF
    payload = {"parts": list(part_names), "seed": int(seed)}
    msg = String()
    msg.data = json.dumps(payload)
    controller.get_logger().info(f"Randomize part positions via {topic}: {msg.data}")
    pub.publish(msg)
    if last_seq is not None and float(timeout) > 0.0:
        if not part_monitor.wait_for_new_part_states(last_seq, timeout=float(timeout)):
            controller.get_logger().error("No part_states update received after randomize request")
            return False
    if float(settle_time) > 0.0:
        time.sleep(float(settle_time))
    return True


# ============================================================================
# [Block 5] 执行 / 编排
# ============================================================================
def execute_pick_place(controller, part_monitor, part_name, *, side,
                       grasp_rpy_world, place_rpy_world,
                       pregrasp_height, pregrasp_y_offset, grasp_target_offset, lift_height,
                       place_after_grasp, box_world_pos,
                       place_release_height, place_lift_height,
                       duration_per_step, gripper_duration,
                       timeout, ik_kwargs,
                       step1_rot_weight=0.5, pregrasp_pitch_down_deg=10.0,
                       grasp_retries=3,
                       dry_run=False, step=None, before_execute_callback=None):
    """对一个零件执行抓取（+放置）。所有路点 world frame，delta 执行。

    step: 指定分组阶段（1..N）只执行该阶段；None 跑全部。
    grasp_retries: 抓取失败（零件未抬起）后重试次数；step 模式不重试。
    """
    logger = controller.get_logger()

    part_pose = part_monitor.get_part_pose(part_name)
    if not part_pose or part_pose.get("pos") is None:
        states = part_monitor.get_part_states() or {}
        available = sorted((states.get("parts") or {}).keys())
        logger.error(f"Part '{part_name}' not found in part_states. Available: {available}")
        return False
    part_pos = np.asarray(part_pose["pos"], dtype=float)
    grasp_rpy = None if grasp_rpy_world is None else np.asarray(grasp_rpy_world, dtype=float)
    place_rpy = None if place_rpy_world is None else np.asarray(place_rpy_world, dtype=float)
    box = np.asarray(box_world_pos, dtype=float)
    gto = np.asarray(grasp_target_offset, dtype=float)
    z_up = np.array([0.0, 0.0, 1.0], dtype=float)
    y_back = np.array([0.0, -1.0, 0.0], dtype=float)

    # 所有路点统一放在可变容器 wp 中，lambda 闭包通过 wp[key] 访问；
    # 重试时更新 wp["<key>"] 即可，无需依赖 numpy 原地修改的隐式副作用。
    wp = {
        "pregrasp": part_pos + z_up * float(pregrasp_height) + y_back * float(pregrasp_y_offset),
        "grasp":    part_pos + gto,
        "lift":     part_pos + z_up * float(lift_height),
        "place_release": box + z_up * float(place_release_height),
        "place_lift":    box + z_up * float(place_lift_height),
    }

    rpy_tag = "keep" if grasp_rpy is None else _fmt(grasp_rpy)
    logger.info(f"Part '{part_name}' world pos: {_fmt(part_pos)}")
    logger.info(
        f"Grasp rpy={rpy_tag} pregrasp={_fmt(wp['pregrasp'])} grasp={_fmt(wp['grasp'])} lift={_fmt(wp['lift'])}"
    )
    if place_after_grasp:
        logger.info(
            f"Box={_fmt(box)} release={_fmt(wp['place_release'])} "
            f"lift={_fmt(wp['place_lift'])}"
        )

    # step 1（pregrasp 大位移）专用 IK：提高 rot_weight 以减少俯仰/翻滚运动。
    step1_ik = dict(ik_kwargs)
    if step1_rot_weight is not None:
        step1_ik["rot_weight"] = float(step1_rot_weight)

    # 放置到箱口时仅约束位置，不约束姿态（避免肘部触限位 R_elbow_yaw CLAMPED）
    place_ik = dict(ik_kwargs, rot_weight=0.0)
    # 抬升时仅约束位置（同 place，不约束姿态）
    lift_ik = dict(ik_kwargs, rot_weight=0.0)

    def move_to(label, target_xyz, target_rpy, ik_override=None):
        ik = ik_override if ik_override is not None else ik_kwargs
        rpy_str = "keep" if target_rpy is None else _fmt(target_rpy)
        logger.info(
            f"--> {label}: target world xyz={_fmt(target_xyz)} rpy={rpy_str} "
            f"rot_weight={ik.get('rot_weight')} rot_axis_weights={ik.get('rot_axis_weights')}"
        )
        if dry_run:
            return True
        return _move_ee_to(
            controller, side, target_xyz, target_rpy=target_rpy,
            duration_per_step=duration_per_step,
            logger=logger, label=label, **ik,
        )

    def grip(open_):
        if dry_run:
            return True
        if open_:
            return controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration))
        return controller.close_grip(side, wait=True, timeout=max(timeout, gripper_duration))

    def ready(duration=3.0):
        if dry_run:
            return True
        return controller.move_to_pose(
            READY_POSE, duration_sec=duration, wait=True, unlock_required_joints=True
        )

    # pregrasp 后让 EE 局部 y 轴俯仰向下（绕夹爪左方轴低头，指向零件）。
    # 只在首次执行，重试时跳过（避免俯仰角每次叠加 10°）。
    pitch_down_rad = np.deg2rad(float(pregrasp_pitch_down_deg))
    _pitch_done = False
    def pitch_down():
        nonlocal _pitch_done
        if _pitch_done or abs(pitch_down_rad) < 1e-9:
            return True
        if dry_run:
            _pitch_done = True
            return True
        ok = controller.move_arm_ee_delta_local(
            side, delta_xyz=(0.0, 0.0, 0.0), delta_rpy=(0.0, float(pitch_down_rad), 0.0),
            duration_sec=0.4, wait=True, **ik_kwargs)
        _pitch_done = True
        return ok

    # 俯仰后重读零件位姿 + 夹指位置，更新抓取/抬升高度
    def _finger_midpoint_world():
        """当前夹爪两指尖中点的 world 坐标；None 表示无数据。"""
        prefix = "R" if side == "right" else "L"
        f1 = controller.get_finger_link_pose(f"{prefix}_finger1_link")
        f2 = controller.get_finger_link_pose(f"{prefix}_finger2_link")
        if not (f1 and f2 and f1.get("pos") and f2.get("pos")):
            return None
        return (np.asarray(f1["pos"], dtype=float) + np.asarray(f2["pos"], dtype=float)) / 2.0

    def _apply_finger_correction(pp_pos):
        """读取当前指尖中点，计算校正后的 grasp/lift 路点并回写到 wp。

        思路：把 TCP 移动"指尖到零件中心"所需的 world-frame delta，使指尖中点落到零件上。
        """
        finger_mid = _finger_midpoint_world()
        tcp_cur = _ee_world_pose(controller, side)
        if finger_mid is None or tcp_cur is None:
            wp["grasp"] = pp_pos + gto
            wp["lift"] = pp_pos + z_up * float(lift_height)
            return False
        finger_desired = pp_pos + gto
        delta = finger_desired - finger_mid
        wp["grasp"] = tcp_cur[:3] + delta
        wp["lift"] = wp["grasp"] + z_up * (float(lift_height) - float(gto[2]))
        logger.info(
            f"Finger correction: part={_fmt(pp_pos)} fingers_mid={_fmt(finger_mid)} "
            f"tcp_cur={_fmt(tcp_cur[:3])} delta_to_target={_fmt(delta)} "
            f"grasp={_fmt(wp['grasp'])}")
        return True

    def resync_grasp_target():
        """俯仰后重读零件位姿和指尖位置，更新 grasp/lift 路点。"""
        if dry_run:
            return True
        time.sleep(0.3)
        pp = part_monitor.get_part_pose(part_name)
        if not pp or pp.get("pos") is None:
            logger.warning("Cannot resync part pos after pitch_down")
            return True
        pp_pos = np.asarray(pp["pos"], dtype=float)
        ok = _apply_finger_correction(pp_pos)
        if not ok:
            logger.info(f"Resync part (no finger data): pos={_fmt(pp_pos)} grasp={_fmt(wp['grasp'])}")
        return True

    # 供重试循环复用
    def resync_grasp_for_retry(pp_pos):
        """重试时用最新零件位姿 + 当前指尖位置更新 grasp/lift"""
        _apply_finger_correction(pp_pos)

    # 分组阶段
    # lambda 闭包通过 wp["<key>"] 显式访问路点，重试时更新 wp dict 即可生效。
    groups = [
        ("1: open+pregrasp+pitch_down", [
            lambda: grip(True),
            lambda: move_to("pregrasp", wp["pregrasp"], grasp_rpy, ik_override=step1_ik),
            pitch_down,
            resync_grasp_target,
        ]),
        ("2: grasp", [lambda: move_to("grasp", wp["grasp"], grasp_rpy)]),
        ("3: close+lift", [lambda: grip(False), lambda: (time.sleep(2.0), True)[-1], lambda: move_to("lift", wp["lift"], grasp_rpy, ik_override=lift_ik)]),
    ]
    if place_after_grasp:
        groups.append((
            "4: place_release",
            [lambda: move_to("place_release", wp["place_release"], place_rpy, ik_override=place_ik)],
        ))
        groups.append((
            "5: open(1s wait)+place_lift",
            [lambda: grip(True), lambda: (time.sleep(1.0), True)[-1], lambda: move_to("place_lift", wp["place_lift"], place_rpy)],
        ))
        groups.append(("6: ready", [lambda: ready(3.0)]))
    else:
        groups.append(("4: ready", [ready]))

    if dry_run:
        logger.info("Dry run: planned action groups:")
        for gname, acts in groups:
            logger.info(f"  {gname}: {len(acts)} action(s)")
        return True

    # 在第一个动作前触发一次性回调（例如启动数据录制）
    if before_execute_callback is not None:
        try:
            before_execute_callback()
        except Exception as exc:
            logger.error(f"before_execute_callback failed: {exc}")
            raise

    grasp_groups = groups[:3]
    place_groups = groups[3:]
    max_grasp_attempts = 1 if step is not None else max(1, int(grasp_retries) + 1)
    part_z_before = float(part_pose["pos"][2])

    # ---- 抓取阶段（step 1-3），含重试 ----
    grasp_ok = False
    for grasp_attempt in range(max_grasp_attempts):
        if grasp_attempt > 0:
            logger.warning(f"Grasp retry {grasp_attempt}/{max_grasp_attempts - 1}")
            time.sleep(0.3)
            part_pose = part_monitor.get_part_pose(part_name)
            if not part_pose or part_pose.get("pos") is None:
                logger.error(f"Part '{part_name}' lost on retry, cannot re-grasp")
                break
            # 更新 wp 路点 dict（lambda 闭包通过 wp[key] 读取，无需 numpy 原地修改）
            part_pos = np.asarray(part_pose["pos"], dtype=float)
            wp["pregrasp"] = part_pos + z_up * float(pregrasp_height) + y_back * float(pregrasp_y_offset)
            resync_grasp_for_retry(part_pos)  # 更新 wp["grasp"] / wp["lift"]（含指尖偏移）
            logger.info(f"Retry part pos: {_fmt(part_pos)} grasp={_fmt(wp['grasp'])}")

        if step is not None:
            if step <= 3:
                idx = int(step) - 1
                run_now = [grasp_groups[idx]]
            else:
                run_now = []
        else:
            run_now = grasp_groups

        action_failed = False
        for gname, acts in run_now:
            logger.info(f"=== {gname} ===")
            for act in acts:
                if not act():
                    logger.error(f"Action failed in {gname}")
                    action_failed = True
                    break
            if action_failed:
                break

        if action_failed:
            return False

        if step is not None and step <= 3:
            return True  # step mode: done

        # 等 /sim/part_states 更新（抬升后物理需要 1-2 帧才能反映新位置）
        last_seq = part_monitor.get_update_seq()
        part_monitor.wait_for_new_part_states(last_seq, timeout=1.0)

        # 检查零件是否被抓起（z 抬升 > 2cm）
        after_pose = part_monitor.get_part_pose(part_name)
        if after_pose and after_pose.get("pos"):
            after_z = float(after_pose["pos"][2])
            if after_z - part_z_before >= 0.02:
                grasp_ok = True
                logger.info(f"Grasp OK: part z {part_z_before:.4f} → {after_z:.4f}")
                break
            logger.warning(
                f"Grasp check: part z {part_z_before:.4f} → {after_z:.4f} "
                f"(Δ={after_z - part_z_before:.4f}m < 0.02m)")
        else:
            logger.warning("Cannot check part lift (part_states unavailable), assuming OK")
            grasp_ok = True
            break

        # 重试前先张开夹爪
        if grasp_attempt < max_grasp_attempts - 1:
            controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration))

    if not grasp_ok and (step is None or step <= 3):
        logger.error(f"Grasp failed after {max_grasp_attempts} attempt(s)")
        if not dry_run:
            controller.open_grip(side, wait=True, timeout=max(timeout, gripper_duration))
        return False

    # ---- 放置阶段（step 4-6）----
    if step is not None:
        if step > 3:
            idx = int(step) - 1
            run_place = [place_groups[idx - 3]]
        else:
            run_place = []
    else:
        run_place = place_groups

    for gname, acts in run_place:
        logger.info(f"=== {gname} ===")
        for act in acts:
            if not act():
                logger.error(f"Action failed in {gname}")
                return False

    return True


def move_ee_by_waypoints(controller, part_monitor, part_name=DEFAULT_PART_NAME, *,
                         side=DEFAULT_SIDE, grasp_rpy_world=None, place_rpy_world=None,
                         pregrasp_height=DEFAULT_PREGRASP_HEIGHT,
                         pregrasp_y_offset=DEFAULT_PREGRASP_Y_OFFSET,
                         grasp_target_offset=DEFAULT_GRASP_TARGET_OFFSET,
                         lift_height=DEFAULT_LIFT_HEIGHT,
                         place_after_grasp=DEFAULT_PLACE, box_world_pos=None,
                         place_release_height=DEFAULT_PLACE_RELEASE_HEIGHT,
                         place_lift_height=DEFAULT_PLACE_LIFT_HEIGHT,
                         duration_per_step=DEFAULT_DURATION,
                         gripper_duration=DEFAULT_GRIPPER_DURATION,
                         timeout=DEFAULT_TIMEOUT,
                         rot_weight=DEFAULT_ROT_WEIGHT, unlock_waist=DEFAULT_UNLOCK_WAIST,
                         rot_axis_weights=DEFAULT_ROT_AXIS_WEIGHTS,
                         require_ik_ok=DEFAULT_REQUIRE_IK_OK, task_type=DEFAULT_TASK_TYPE,
                         use_hierarchical=False, dry_run=False, step=None,
                         step1_rot_weight=0.5, pregrasp_pitch_down_deg=10.0,
                         grasp_retries=3,
                         before_execute_callback=None):
    """单零件抓取（+放置）：等待状态 -> 初始化 IK -> execute_pick_place。"""
    if box_world_pos is None:
        box_world_pos = _default_box_pos_for_part(part_name)
    # grasp_rpy_world / place_rpy_world 为 None 时表示保持当前姿态（仅平移）

    if not controller.wait_for_state(timeout=timeout):
        return False
    if not part_monitor.wait_for_part_states(timeout=timeout):
        return False
    # IK 已在 controller __init__(enable_ik=True) 中初始化；仅当未初始化时才重新初始化，
    # 避免重复打印 "Walker S2 IK initialized"。
    if not (getattr(controller, "_ik_initialized", False) or controller.initialize_ik()):
        return False

    ik_kwargs = dict(
        rot_weight=rot_weight,
        unlock_waist=unlock_waist,
        require_success=require_ik_ok,
        task_type=task_type,
        use_hierarchical=use_hierarchical,
        rot_axis_weights=tuple(rot_axis_weights),
    )

    return execute_pick_place(
        controller, part_monitor, part_name,
        side=side, grasp_rpy_world=grasp_rpy_world, place_rpy_world=place_rpy_world,
        pregrasp_height=pregrasp_height, pregrasp_y_offset=pregrasp_y_offset,
        grasp_target_offset=grasp_target_offset,
        lift_height=lift_height, place_after_grasp=place_after_grasp,
        box_world_pos=box_world_pos,
        place_release_height=place_release_height, place_lift_height=place_lift_height,
        duration_per_step=duration_per_step, gripper_duration=gripper_duration,
        timeout=timeout, ik_kwargs=ik_kwargs,
        step1_rot_weight=step1_rot_weight, pregrasp_pitch_down_deg=pregrasp_pitch_down_deg,
        grasp_retries=grasp_retries,
        dry_run=dry_run, step=step, before_execute_callback=before_execute_callback,
    )


def move_parts_by_waypoints(controller, part_monitor, part_names=DEFAULT_PART_SEQUENCE, *,
                            reset_scene_before=False,
                            reset_scene_settle_time=DEFAULT_RESET_SCENE_SETTLE_TIME,
                            robot_init_before=False,
                            robot_init_duration=DEFAULT_ROBOT_INIT_DURATION,
                            robot_init_settle_timeout=DEFAULT_ROBOT_INIT_SETTLE_TIMEOUT,
                            robot_init_tolerance=DEFAULT_ROBOT_INIT_TOLERANCE,
                            randomize_before=True,
                            randomize_topic=DEFAULT_RANDOMIZE_PARTS_TOPIC,
                            randomize_timeout=None, randomize_settle_time=0.5,
                            randomize_seed=None, before_execute_callback=None, **kwargs):
    """按顺序对多个零件执行同一套抓取/放置流程。"""
    part_names = tuple(part_names) if part_names else (DEFAULT_PART_NAME,)
    if not part_names:
        controller.get_logger().error("part_names must not be empty")
        return False

    timeout = kwargs.get("timeout", DEFAULT_TIMEOUT) if randomize_timeout is None else randomize_timeout
    if reset_scene_before and not reset_scene(
        controller, part_monitor, timeout=timeout, settle_time=reset_scene_settle_time
    ):
        return False
    if robot_init_before and not initialize_robot_pose(
        controller, duration_sec=robot_init_duration,
        settle_timeout=robot_init_settle_timeout, tolerance=robot_init_tolerance, timeout=timeout,
    ):
        return False
    if randomize_before and not randomize_part_positions(
        controller, part_monitor, part_names=part_names, topic=randomize_topic,
        timeout=timeout, settle_time=randomize_settle_time, seed=randomize_seed,
    ):
        return False

    for index, pn in enumerate(part_names, start=1):
        controller.get_logger().info(f"Start part {index}/{len(part_names)}: {pn}")
        ok = move_ee_by_waypoints(
            controller, part_monitor, part_name=pn,
            before_execute_callback=before_execute_callback, **kwargs,
        )
        if not ok:
            controller.get_logger().error(f"Part {index}/{len(part_names)} failed: {pn}")
            return False
        controller.get_logger().info(f"Part {index}/{len(part_names)} completed: {pn}")
    controller.get_logger().info(f"Completed {len(part_names)} part(s): {list(part_names)}")
    return True


# ============================================================================
# [Block 6] 命令行入口
# ============================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="Walker S2 零件抓取（末端偏移运动 + 实时位姿 + 固定 top-down 姿态）"
    )
    # ---- 零件选择 ----
    parser.add_argument("--part", default=DEFAULT_PART_NAME,
                        help="零件名，例如 part_a_ori / part_a_red / part_b_blue / part_b_ori")
    parser.add_argument("--all-parts", action="store_true", help="按默认顺序依次抓取四个零件")
    parser.add_argument("--parts", nargs="+", default=None, help="自定义多零件抓取顺序")

    # ---- 场景前置 ----
    parser.add_argument("--randomize-parts", action=argparse.BooleanOptionalAction, default=True,
                        help="多零件抓取前先通过 bridge 随机化零件位置")
    parser.add_argument("--randomize-parts-topic", default=DEFAULT_RANDOMIZE_PARTS_TOPIC)
    parser.add_argument("--randomize-seed", type=int, default=None, help="零件随机化 seed；不指定则自动生成")
    parser.add_argument("--randomize-settle-time", type=float, default=0.5)
    parser.add_argument("--reset-scene", action="store_true", help="抓取前发布 /sim/cmd_reset 重置仿真场景")
    parser.add_argument("--reset-scene-settle-time", type=float, default=DEFAULT_RESET_SCENE_SETTLE_TIME)
    parser.add_argument("--robot-init", action="store_true", help="抓取前移动到 READY_POSE")
    parser.add_argument("--robot-init-duration", type=float, default=DEFAULT_ROBOT_INIT_DURATION)
    parser.add_argument("--robot-init-settle-timeout", type=float, default=DEFAULT_ROBOT_INIT_SETTLE_TIMEOUT)
    parser.add_argument("--robot-init-tolerance", type=float, default=DEFAULT_ROBOT_INIT_TOLERANCE)

    # ---- 抓取几何 ----
    parser.add_argument("--side", choices=("left", "right"), default=DEFAULT_SIDE)
    parser.add_argument("--grasp-rpy-deg", type=float, nargs=3, default=DEFAULT_GRASP_RPY_WORLD_DEG,
                        metavar=("R", "P", "Y"),
                        help="固定 world RPY（度）；不指定则保持当前姿态（仅平移/水平移动，不旋转夹爪）")
    parser.add_argument("--grasp-yaw-deg", type=float, default=None,
                        help="仅覆盖 grasp RPY 的 yaw 分量（度）；需配合 --grasp-rpy-deg 使用")
    parser.add_argument("--pregrasp-height", type=float, default=DEFAULT_PREGRASP_HEIGHT,
                        help="pregrasp 在零件上方 world z 偏移 (m)")
    parser.add_argument("--pregrasp-y-offset", type=float, default=DEFAULT_PREGRASP_Y_OFFSET,
                        help="pregrasp 在零件后方 world -y 偏移 (m)")
    parser.add_argument("--grasp-target-offset", type=float, nargs=3, default=DEFAULT_GRASP_TARGET_OFFSET,
                        metavar=("X", "Y", "Z"), help="grasp 相对零件 world pos 的偏移 (m)")
    parser.add_argument("--pregrasp-pitch-down-deg", type=float, default=DEFAULT_PREGRASP_PITCH_DOWN_DEG,
                        help="pregrasp 后 EE 局部 y 轴俯仰向下（度）；0=不旋转，保持水平")
    parser.add_argument("--lift-height", type=float, default=DEFAULT_LIFT_HEIGHT, help="抓起后 world z 抬升 (m)")
    # ---- 时序 ----
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="每段 delta 轨迹时长 (s)")
    parser.add_argument("--gripper-duration", type=float, default=DEFAULT_GRIPPER_DURATION,
                        help="夹爪开合等待超时 (s)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="等待 ROS 状态 topic 超时 (s)")

    # ---- IK ----
    parser.add_argument("--rot-weight", type=float, default=DEFAULT_ROT_WEIGHT, help="IK 姿态误差权重")
    parser.add_argument("--rot-axis-weights", type=float, nargs=3, default=list(DEFAULT_ROT_AXIS_WEIGHTS),
                        metavar=("X", "Y", "Z"),
                        help="IK 旋转轴权重(EE 局部系 x/y/z)；默认 (0,1,1)=释放局部x(世界z/yaw)、约束俯仰(y)/翻滚(z)")
    parser.add_argument("--step1-rot-weight", type=float, default=0.5,
                        help="step1(pregrasp) 专用 rot_weight；提高可减少俯仰/翻滚运动，yaw 轴(权重0)不受影响保持自由")
    parser.add_argument("--unlock-waist", action=argparse.BooleanOptionalAction, default=DEFAULT_UNLOCK_WAIST,
                        help="IK 求解时允许 waist_yaw_joint 参与")
    parser.add_argument("--require-ik-ok", action=argparse.BooleanOptionalAction, default=DEFAULT_REQUIRE_IK_OK,
                        help="要求 IK 返回 success；--no-require-ik-ok 用于调试")
    parser.add_argument("--task-type", default=DEFAULT_TASK_TYPE, help="IK 语义种子标识")
    parser.add_argument("--use-hierarchical-ik", action="store_true", help="启用层级 IK")

    # ---- 放置 ----
    parser.add_argument("--place", action=argparse.BooleanOptionalAction, default=DEFAULT_PLACE,
                        help="抓取后搬运到箱子上方并松爪")
    parser.add_argument("--place-box-pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                        help="放置箱子 world 位置；不指定时 A 类->位置 1，B 类->位置 2")
    parser.add_argument("--place-rpy-deg", type=float, nargs=3, default=None, metavar=("R", "P", "Y"),
                        help="放置 world RPY（度）；不指定则沿用 grasp RPY")
    parser.add_argument("--place-release-height", type=float, default=DEFAULT_PLACE_RELEASE_HEIGHT)
    parser.add_argument("--place-lift-height", type=float, default=DEFAULT_PLACE_LIFT_HEIGHT)

    # ---- 调试 / 录制 ----
    parser.add_argument("--grasp-retries", type=int, default=3,
                        help="抓取失败（零件未抬起）后重试次数；step 模式不重试")
    parser.add_argument("--step", type=int, default=None, metavar="N",
                        help="分步执行（place 启用 1..6；关闭 1..4），便于逐阶段调试")
    parser.add_argument("--dry-run", action="store_true", help="只打印路点，不下发控制")
    parser.add_argument("--save", action="store_true", help="录制 HDF5 数据")
    parser.add_argument("--save-hz", type=float, default=30.0, help="数据录制频率 Hz")
    parser.add_argument("--save-only-success", action=argparse.BooleanOptionalAction, default=True,
                        help="仅任务成功时保存 HDF5")
    return parser.parse_args()


def _build_move_kwargs(args):
    if args.grasp_rpy_deg is not None:
        grasp_rpy_deg = list(args.grasp_rpy_deg)
        if args.grasp_yaw_deg is not None:
            grasp_rpy_deg[2] = float(args.grasp_yaw_deg)
        grasp_rpy_world = np.deg2rad(grasp_rpy_deg).tolist()
    else:
        grasp_rpy_world = None  # 保持当前姿态，仅平移
    if args.place_rpy_deg is not None:
        place_rpy_world = np.deg2rad(args.place_rpy_deg).tolist()
    elif grasp_rpy_world is not None:
        place_rpy_world = list(grasp_rpy_world)
    else:
        place_rpy_world = None
    return dict(
        side=args.side,
        grasp_rpy_world=grasp_rpy_world,
        place_rpy_world=place_rpy_world,
        pregrasp_height=args.pregrasp_height,
        pregrasp_y_offset=args.pregrasp_y_offset,
        grasp_target_offset=args.grasp_target_offset,
        lift_height=args.lift_height,
        place_after_grasp=args.place,
        box_world_pos=args.place_box_pos,
        place_release_height=args.place_release_height,
        place_lift_height=args.place_lift_height,
        duration_per_step=args.duration,
        gripper_duration=args.gripper_duration,
        timeout=args.timeout,
        rot_weight=args.rot_weight,
        unlock_waist=args.unlock_waist,
        rot_axis_weights=tuple(args.rot_axis_weights),
        require_ik_ok=args.require_ik_ok,
        task_type=args.task_type,
        use_hierarchical=args.use_hierarchical_ik,
        step1_rot_weight=args.step1_rot_weight,
        pregrasp_pitch_down_deg=args.pregrasp_pitch_down_deg,
        grasp_retries=args.grasp_retries,
        step=args.step,
        dry_run=args.dry_run,
    )


def main():
    args = parse_args()
    rclpy.init()

    controller = WalkerS2Controller(enable_ik=True, subscribe_images=False)
    part_monitor = PartStateMonitor()

    part_names = DEFAULT_PART_SEQUENCE if args.all_parts else args.parts
    if part_names is None:
        part_names = (args.part,)

    move_kwargs = _build_move_kwargs(args)

    # ---- --save 分支：录制 HDF5 ----
    if args.save:
        from utils.recorder import WalkerS2DataRecorder  # noqa: E402

        def _parse_msg_type(name: str) -> type:
            """将 'Image1m' / 'Image6m' 等字符串转为 shm_msgs.msg.* 类型。"""
            import shm_msgs.msg
            return getattr(shm_msgs.msg, name)

        cameras = {
            name: Camera(topic=cfg["topic"], msg_type=_parse_msg_type(cfg["msg_type"]),
                         node_name=f"walker_s2_save_{name}_camera")
            for name, cfg in CAMERA_TOPICS.items()
        }
        depth_camera = Camera(topic=DEFAULT_IMAGE_DEPTH_TOPIC, node_name="walker_s2_save_depth_camera")
        recorder = WalkerS2DataRecorder(cameras, depth_camera, save_hz=args.save_hz)

        # controller + part_monitor + 4 cameras + depth_camera + recorder
        executor = MultiThreadedExecutor(num_threads=3 + len(cameras) + 1)
        executor.add_node(controller)
        executor.add_node(part_monitor)
        for cam in cameras.values():
            executor.add_node(cam)
        executor.add_node(depth_camera)
        executor.add_node(recorder)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            timeout = args.timeout
            if not controller.wait_for_state(timeout=timeout):
                raise SystemExit(1)
            if not part_monitor.wait_for_part_states(timeout=timeout):
                raise SystemExit(1)
            if not args.dry_run:
                recorder.depth_camera.wait_for_image(timeout=timeout)
                for cam in cameras.values():
                    cam.wait_for_image(timeout=timeout)

            # 前置操作（录制不包含这些阶段）
            if args.reset_scene and not reset_scene(
                controller, part_monitor, timeout=timeout, settle_time=args.reset_scene_settle_time
            ):
                raise SystemExit(1)
            if args.robot_init and not initialize_robot_pose(
                controller, duration_sec=args.robot_init_duration,
                settle_timeout=args.robot_init_settle_timeout,
                tolerance=args.robot_init_tolerance, timeout=timeout,
            ):
                raise SystemExit(1)
            if args.randomize_parts and not randomize_part_positions(
                controller, part_monitor, part_names=part_names, topic=args.randomize_parts_topic,
                timeout=timeout, settle_time=args.randomize_settle_time, seed=args.randomize_seed,
            ):
                raise SystemExit(1)

            all_ok = True
            episode_count = len(part_names)
            recorder.start_save_data()
            for index, pn in enumerate(part_names, start=1):
                controller.get_logger().info(f"Start part {index}/{episode_count}: {pn}")
                ok = move_ee_by_waypoints(
                    controller, part_monitor, part_name=pn, **move_kwargs,
                )
                if not ok:
                    controller.get_logger().error(f"Part {index}/{episode_count} failed: {pn}")
                    if args.save_only_success:
                        recorder.get_logger().warning(
                            f"Part {pn} failed, data discarded (--save-only-success is on)."
                        )
                        all_ok = False
                        continue
                    all_ok = False
                controller.get_logger().info(f"Part {index}/{episode_count} done: {pn}")
            recorder.stop_save_data()
            recorder.set_episode_metadata(
                part_name=",".join(part_names), side=args.side, auto_grasp=False,
                success=all_ok, episode_count=episode_count, episode_index=1,
            )
            recorder.save_data()
            recorder.get_logger().info(f"Saved {episode_count} parts in one episode")

            if not all_ok:
                raise SystemExit(1)
        except KeyboardInterrupt:
            controller.get_logger().warning("Interrupted by user")
        finally:
            try:
                recorder.save_timer.cancel()
            except Exception:
                pass
            executor.shutdown()
            spin_thread.join(timeout=2.0)
            recorder.destroy_node()
            depth_camera.destroy_node()
            for cam in cameras.values():
                cam.destroy_node()
            part_monitor.destroy_node()
            controller.destroy_node()
            rclpy.shutdown()
        return

    # ---- 非 --save 分支 ----
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    executor.add_node(part_monitor)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    try:
        if len(part_names) > 1:
            ok = move_parts_by_waypoints(
                controller, part_monitor, part_names=part_names,
                reset_scene_before=args.reset_scene,
                reset_scene_settle_time=args.reset_scene_settle_time,
                robot_init_before=args.robot_init,
                robot_init_duration=args.robot_init_duration,
                robot_init_settle_timeout=args.robot_init_settle_timeout,
                robot_init_tolerance=args.robot_init_tolerance,
                randomize_before=args.randomize_parts,
                randomize_topic=args.randomize_parts_topic,
                randomize_timeout=args.timeout,
                randomize_settle_time=args.randomize_settle_time,
                randomize_seed=args.randomize_seed,
                **move_kwargs,
            )
        else:
            if args.reset_scene and not reset_scene(
                controller, part_monitor, timeout=args.timeout, settle_time=args.reset_scene_settle_time
            ):
                raise SystemExit(1)
            if args.robot_init and not initialize_robot_pose(
                controller, duration_sec=args.robot_init_duration,
                settle_timeout=args.robot_init_settle_timeout,
                tolerance=args.robot_init_tolerance, timeout=args.timeout,
            ):
                raise SystemExit(1)
            if args.randomize_parts and not randomize_part_positions(
                controller, part_monitor, part_names=part_names, topic=args.randomize_parts_topic,
                timeout=args.timeout, settle_time=args.randomize_settle_time, seed=args.randomize_seed,
            ):
                raise SystemExit(1)
            ok = move_ee_by_waypoints(controller, part_monitor, part_name=args.part, **move_kwargs)
        if not ok:
            raise SystemExit(1)
    except KeyboardInterrupt:
        controller.get_logger().warning("Interrupted by user")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        part_monitor.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
