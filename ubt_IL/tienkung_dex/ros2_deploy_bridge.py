#!/usr/bin/env python3
"""ROS2 Deploy Bridge for LeRobot + TienKung Dex.

Bridges between LeRobot (Python 3.12, ZMQ) and the robot backend via ROS2 DDS.

Runs on Python 3.10 (system) with rclpy for ROS2 communication.

用法：

  # --config（推荐，由 TienKungDexRobot._start_bridge() 传入）
  python3 ros2_deploy_bridge.py --config '{"zmq_cmd_port":5559, ...}'

  # 独立调试（已弃用的旧参数）
  python3 ros2_deploy_bridge.py --zmq_cmd_port 5559 --zmq_status_port 5560

ZMQ 内部端口（LeRobot ↔ Bridge2）：

  5559: LeRobot PUB → Bridge2 SUB（动作）
  5560: Bridge2 PUB → LeRobot SUB（状态）
  5558: Bridge2 PUB → DexCamera SUB（JPEG；仅 image_enabled 时由 /ob_camera_head 转发）

只驱动**右臂 7 + 右手**。左臂/腰/头/腿由
``scripts/deploy/tienkung_dex/robot_ready.py`` 摆到预备位后保持不动——
本桥接**默认不碰 /waist/cmd 与 /leg/cmd**，腿永远不发（真机由行走控制器独占。
注意 /waist/cmd 与 /leg/cmd 与 /arm/cmd 同用 ArmCtrl 消息，只是话题不同）。

手部动作量纲（``hand_action_mode``）：

  - ``rad13``（默认，20D 老链路）：ZMQ ``right_hand`` 是 13 维关节角 rad ->
    ``SetAngle.joint_values[13]`` 的 0.1° 角码；状态也是 13 维 rad。
  - ``pico6``（r13 数据集）：ZMQ ``right_hand`` 是 **6 维归一化 0~1**
    （0=张开、1=闭合），由 :func:`pico6_to_inspire13` 展开成 13 个角码下发；
    状态回显同一 6 维指令（开环，与数据集 ``observation.state`` 同口径）。
    硬件侧仍是因时 13 通道、``hand_id=2``、``/right_hand/angle_set``，不变。
  - ``brainco``：6 维闭合度 0~1 -> ``SetMotorMulti`` 的 1~1000 码。
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import signal
import threading
import time
from typing import Any

import zmq

logger = logging.getLogger("ros2_deploy_bridge")


# ── 运行期常量副本 ──────────────────────────────────────────────────────────
# 本文件跑在系统 Python 3.10（ROS2 Humble），**不能** import 3.12 venv 里的
# lerobot_robot_tienkung_dex，也看不到 ubt_sim。因此关节表 / 限位 / 手部通道表
# 在这里留一份运行期副本：权威定义在 ubt_sim/teleoperation/control/tienkung_dex/
# constants.py，插件包 tienkung_dex/lerobot_robot_tienkung_dex/.../constants.py
# 是第二份，本文件是第三份。改动必须三处手工同步。
#
# 启动断言（_assert_hand_table）会拿插件经 --config 传下来的右手关节序与本文件
# 的通道表逐字比对，不一致直接拒启——那道断言就是防这份副本悄悄过期。

DEFAULT_VARIANT = "inspire"

# 双臂关节表：顺序 = 电机 ID 顺序 = 插件 *_arm_joints 的顺序。
# 真机命名差异：ID 15/25 在线缆上叫 wrist_yaw，URDF 里是 elbow_yaw_{l,r}_joint。
ARM_JOINTS = {
    "left": [
        "shoulder_pitch_l_joint",
        "shoulder_roll_l_joint",
        "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint",
        "elbow_yaw_l_joint",
        "wrist_pitch_l_joint",
        "wrist_roll_l_joint",
    ],
    "right": [
        "shoulder_pitch_r_joint",
        "shoulder_roll_r_joint",
        "shoulder_yaw_r_joint",
        "elbow_pitch_r_joint",
        "elbow_yaw_r_joint",
        "wrist_pitch_r_joint",
        "wrist_roll_r_joint",
    ],
}
ARM_MOTOR_IDS = {
    "left": [11, 12, 13, 14, 15, 16, 17],
    "right": [21, 22, 23, 24, 25, 26, 27],
}

# 兼容旧引用（右侧）
RIGHT_ARM_JOINTS = ARM_JOINTS["right"]
RIGHT_ARM_MOTOR_IDS = ARM_MOTOR_IDS["right"]

# 头部 2 关节（HeadCtrl，ID 1=head_yaw / 2=head_pitch）
HEAD_JOINTS = ["head_yaw_joint", "head_pitch_joint"]
HEAD_MOTOR_IDS = [1, 2]
HEAD_CMD_LIMITS = {"spd": 0.3, "cur": 10.0}

# 双臂 + 头部 URDF 限位 (lower, upper)，单位 rad。Bridge2 是最后一道机械保护。
# 左侧按镜像对称（shoulder_roll_l / elbow_pitch_l 的符号与右侧相反）。
BODY_JOINT_LIMITS = {
    "shoulder_pitch_l_joint": (-2.844887, 2.844887),
    "shoulder_roll_l_joint": (-0.191986, 3.333579),
    "shoulder_yaw_l_joint": (-2.844887, 2.844887),
    "elbow_pitch_l_joint": (-2.548181, 0.191986),
    "elbow_yaw_l_joint": (-2.844887, 2.844887),
    "wrist_pitch_l_joint": (-1.32645, 1.32645),
    "wrist_roll_l_joint": (-1.32645, 1.32645),
    "shoulder_pitch_r_joint": (-2.844887, 2.844887),
    "shoulder_roll_r_joint": (-3.333579, 0.191986),
    "shoulder_yaw_r_joint": (-2.844887, 2.844887),
    "elbow_pitch_r_joint": (-2.548181, 0.191986),
    "elbow_yaw_r_joint": (-2.844887, 2.844887),
    "wrist_pitch_r_joint": (-1.32645, 1.32645),
    "wrist_roll_r_joint": (-1.32645, 1.32645),
    "head_yaw_joint": (-1.047198, 1.047198),
    # 正值 = 低头（URDF 轴向 (0,1,0)）：低头到 +58°、仰头到 -15°。
    "head_pitch_joint": (-0.261799, 1.012291),
}

# 真机各组位置指令的 spd/cur（沿用 SDK demo 实测值：手臂 cur=40 才够扛重力）。
# 不填会让真机走 SDK 默认值，行为不可控。
GROUP_CMD_LIMITS = {"arm": {"spd": 0.5, "cur": 40.0}}

# 真机 ControlMode（HWI §7.1）：0=位置 1=阻抗 2=速度 ...
CONTROL_MODE_POSITION = 0
# 强脑 SetMotorMulti 的 SDK 默认控制模式
HAND_CMD_MODE = 1
# 因时 SetAngle.hand_id：1=左手 2=右手（预留字段，真机按话题分左右）
INSPIRE_RIGHT_HAND_ID = 2

BRAINCO_CODE_MIN, BRAINCO_CODE_MAX = 1, 1000

# 强脑 Revo2：6 电机顺序 thumb_flex / thumb_rot / index / middle / ring / pinky
BRAINCO_MOTOR_JOINTS: tuple[tuple[str, ...], ...] = (
    ("thumb_proximal",),
    ("thumb_metacarpal",),
    ("index_proximal",),
    ("middle_proximal",),
    ("ring_proximal",),
    ("pinky_proximal",),
)

# 因时 RH56：13 通道，(仿真关节短名, 码下限, 码上限, 0 rad 对应码, 方向)。
# 屈曲通道码值越大越伸直，方向 d=-1；侧摆通道零位 0、方向 d=+1。
INSPIRE_CHANNELS: tuple[tuple[str, int, int, int, int], ...] = (
    ("pinky_mcp", 900, 1650, 1650, -1),    # 0  小拇指指根
    ("ring_mcp", 900, 1650, 1650, -1),     # 1  无名指指根
    ("middle_mcp", 900, 1650, 1650, -1),   # 2  中指指根
    ("middle_yaw", -150, 150, 0, +1),      # 3  中指侧摆
    ("index_mcp", 900, 1650, 1650, -1),    # 4  食指指根
    ("index_yaw", -150, 150, 0, +1),       # 5  食指侧摆
    ("pinky_pip", 1150, 1900, 1900, -1),   # 6  小拇指指中
    ("ring_pip", 1150, 1900, 1900, -1),    # 7  无名指指中
    ("middle_pip", 1150, 1900, 1900, -1),  # 8  中指指中
    ("index_pip", 1150, 1900, 1900, -1),   # 9  食指指中
    ("thumb_yaw", 650, 1750, 1750, -1),    # 10 大拇指侧摆
    ("thumb_mcp", 1250, 1600, 1600, -1),   # 11 大拇指一关节弯曲
    ("thumb_pip", 1500, 2040, 2040, -1),   # 12 大拇指二关节弯曲
)

INSPIRE_CHANNEL_JOINTS: tuple[tuple[str, ...], ...] = tuple(
    (short,) for short, *_ in INSPIRE_CHANNELS
)

HAND_CHANNEL_COUNT = {
    "brainco": len(BRAINCO_MOTOR_JOINTS),
    "inspire": len(INSPIRE_CHANNEL_JOINTS),
}

# 因时通道码的物理单位：0.1° 一个码
INSPIRE_CODE_DEG = 0.1

# 强脑关节限位（rad）。仅用于"rad 语义"的换算（hand_rad_to_code / hand_code_to_rad，
# 遥操作/兼容路径）；**策略链路（本文件与插件）对强脑手统一用 0~1 闭合度**，
# 与训练数据集（convert 配置 scale=1/999, offset=-1/999）同口径。
HAND_JOINT_LIMITS = {
    "brainco": {
        "thumb_metacarpal": (0.0, 1.55334),
        "thumb_proximal": (0.0, 1.0472),
        "thumb_distal": (0.0, 1.15192),
        "index_proximal": (0.0, 1.41),
        "index_distal": (0.0, 1.69297),
        "middle_proximal": (0.0, 1.41),
        "middle_distal": (0.0, 1.69297),
        "ring_proximal": (0.0, 1.41),
        "ring_distal": (0.0, 1.69297),
        "pinky_proximal": (0.0, 1.41),
        "pinky_distal": (0.0, 1.69297),
    },
}


def _short_name(joint_name: str) -> str:
    """``right_index_mcp_joint`` -> ``index_mcp``。"""
    short = joint_name
    for prefix in ("left_", "right_"):
        if short.startswith(prefix):
            short = short[len(prefix):]
            break
    return short.removesuffix("_joint")


def action_limit(joint_name: str, variant: str = DEFAULT_VARIANT) -> float:
    """关节"闭合/动作"方向的极限（绝对值较大的一侧）。"""
    short = _short_name(joint_name)
    low, high = HAND_JOINT_LIMITS[variant][short]
    return low if abs(low) > abs(high) else high


def hand_channel_joints(variant: str, side: str) -> tuple[tuple[str, ...], ...]:
    """返回该变体/左右手每个真机通道对应的仿真关节全名（可能为空元组）。"""
    prefix = f"{side}_"
    if variant == "brainco":
        table = BRAINCO_MOTOR_JOINTS
    elif variant == "inspire":
        table = INSPIRE_CHANNEL_JOINTS
    else:
        raise ValueError(f"未知的灵巧手变体: {variant}")
    return tuple(tuple(f"{prefix}{joint}_joint" for joint in entry) for entry in table)


def inspire_code_to_rad(channel: int, code: int) -> float:
    """因时通道码（0.1°，带零位偏置与方向）-> 仿真关节角（rad）。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    clipped = min(max(int(code), low), high)
    return (clipped - zero) * direction * INSPIRE_CODE_DEG * math.pi / 180.0


def inspire_rad_to_code(channel: int, rad: float) -> int:
    """仿真关节角（rad）-> 因时通道码；超出真机行程时夹到通道上下限。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    code = zero + float(rad) * 180.0 / math.pi / (direction * INSPIRE_CODE_DEG)
    return int(round(min(max(code, low), high)))


def hand_percent_to_code(variant: str, percent: float) -> int:
    """闭合度 [0, 1] -> 强脑通道值（取整并夹在值域内）。"""
    ratio = min(max(float(percent), 0.0), 1.0)
    return int(round(BRAINCO_CODE_MIN + ratio * (BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)))


def hand_code_to_percent(variant: str, code: int) -> float:
    """强脑通道值 -> 闭合度 [0, 1]（hand_percent_to_code 的逆运算）。"""
    if variant != "brainco":
        raise ValueError(f"{variant} 通道不是闭合度语义，请用 inspire_code_to_rad")
    clipped = min(max(int(code), BRAINCO_CODE_MIN), BRAINCO_CODE_MAX)
    return (clipped - BRAINCO_CODE_MIN) / float(BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)


def hand_action_to_code(variant: str, channel: int, value: float, joint_name: str) -> int:
    """ZMQ 控制量 -> 真机通道码。量纲按手型区分（与训练数据集一致）。

    - ``brainco``：**闭合度 [0, 1]**（数据集 action 就是 0~1）-> ``round(1 + v*999)``；
    - ``inspire`` + ``hand_action_mode=rad13``：关节角 rad -> 0.1° 角码；
    - ``inspire`` + ``hand_action_mode=pico6``：**不走本函数**，整只手由
      :func:`pico6_to_inspire13` 把 6 维 0~1 展开成 13 个码。
    """
    if variant == "brainco":
        return hand_percent_to_code(variant, value)
    return inspire_rad_to_code(channel, value)


def hand_code_to_value(variant: str, channel: int, code: int, joint_name: str) -> float:
    """真机通道码 -> ZMQ 状态量（brainco: 闭合度 0~1；inspire: rad）。"""
    if variant == "brainco":
        return hand_code_to_percent(variant, code)
    return inspire_code_to_rad(channel, code)


def hand_code_to_rad(variant: str, channel: int, code: int, joint_name: str) -> float:
    """真机通道值 -> 关节角（rad）。**rad 语义路径**（遥操作/仿真复用）。

    策略链路（ZMQ）请用 :func:`hand_code_to_value`：brainco 返回闭合度 0~1。
    """
    if variant == "brainco":
        clipped = min(max(int(code), BRAINCO_CODE_MIN), BRAINCO_CODE_MAX)
        percent = (clipped - BRAINCO_CODE_MIN) / float(BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)
        return percent * action_limit(joint_name, variant)
    return inspire_code_to_rad(channel, code)


def hand_rad_to_code(variant: str, channel: int, rad: float, joint_name: str) -> int:
    """关节角（rad）-> 真机通道值。**rad 语义路径**（遥操作/仿真复用）。

    brainco 的策略控制量是闭合度 0~1，请用 :func:`hand_action_to_code`。
    """
    if variant == "brainco":
        limit = action_limit(joint_name, variant)
        ratio = float(rad) / limit if abs(limit) > 1e-6 else 0.0
        return hand_percent_to_code(variant, ratio)
    return inspire_rad_to_code(channel, rad)


# ── 因时 6 维归一化策略 → 13 通道码（采集端 Pico 映射）──────────────────────
# r13 数据集（dataset/tienkung_dex_real_pick_r13_1RGB）的 action[7:13] 与
# observation.state[7:13] 只有 6 维归一化 0~1：0=张开、1=闭合；真机协议却是
# SetAngle.joint_values[13]（0.1° 角码）。这里做与采集端
# pico_joint_states_callback._control_dex_inspire_hand 完全一致的 6→13 展开。
# 参考实现：dex_vla/ubt_IL/scripts/deploy/dex/act_control.py::pico_to_inspire。
#
# 输入 6 通道顺序（= 数据集列名去掉 side 前缀）：
PICO6_INPUT_JOINTS: tuple[str, ...] = (
    "thumb_proximal_joint",     # 0 thumb_pitch  拇指弯曲
    "thumb_metacarpal_joint",   # 1 thumb_roll   拇指旋转
    "index_proximal_joint",     # 2 index
    "middle_proximal_joint",    # 3 middle
    "ring_proximal_joint",      # 4 ring
    "pinky_proximal_joint",     # 5 little
)
PICO6_INPUT_DIM = len(PICO6_INPUT_JOINTS)

# (输出通道索引, 输入通道索引, 是否先 ×1.2, 固定码)
# 侧摆通道 3/5 恒为中立 0（不能写成 mapped(0, -150, 150)，那会得到 150）。
# ×1.2 只作用于四指指根（MCP），且必须写成 ``v + v * .2`` 以复现采集端的
# 浮点取整边界；PIP 与拇指通道用原值。
PICO6_TO_INSPIRE13: tuple[tuple[int, int | None, bool, int | None], ...] = (
    (0, 5, True, None),     # 小拇指指根  <- little ×1.2
    (1, 4, True, None),     # 无名指指根  <- ring   ×1.2
    (2, 3, True, None),     # 中指指根    <- middle ×1.2
    (3, None, False, 0),    # 中指侧摆    <- 固定中立
    (4, 2, True, None),     # 食指指根    <- index  ×1.2
    (5, None, False, 0),    # 食指侧摆    <- 固定中立
    (6, 5, False, None),    # 小拇指指中  <- little
    (7, 4, False, None),    # 无名指指中  <- ring
    (8, 3, False, None),    # 中指指中    <- middle
    (9, 2, False, None),    # 食指指中    <- index
    (10, 1, False, None),   # 大拇指侧摆  <- thumb_roll
    (11, 0, False, None),   # 拇指一关节  <- thumb_pitch
    (12, 0, False, None),   # 拇指二关节  <- thumb_pitch
)

# hand_action_mode：rad13 = 老口径（13 维 rad）；pico6 = 6 维 0~1 展开成 13 码。
HAND_ACTION_MODES = ("rad13", "pico6")
DEFAULT_HAND_ACTION_MODE = "rad13"


def pico6_action_joints(side: str) -> list[str]:
    """该侧手 6 维策略列名（= 数据集 action[7:13] 的名字）。"""
    return [f"{side}_{name}" for name in PICO6_INPUT_JOINTS]


def pico6_to_inspire13(values: list[float] | tuple[float, ...]) -> list[int]:
    """6 维归一化 [0,1] → 因时 13 个 0.1° 角码（输出顺序 = INSPIRE_CHANNELS）。

    0 = 张开 → 码上限，1 = 闭合 → 码下限；越界输入先夹紧。取整用 ``int``
    截断（不是 round），与采集端一致。
    """
    if len(values) != PICO6_INPUT_DIM:
        raise ValueError(f"pico6 需要 {PICO6_INPUT_DIM} 维输入，收到 {len(values)}")
    clamped = [min(max(float(v), 0.0), 1.0) for v in values]

    def mapped(value: float, low: int, high: int) -> int:
        return int(low + (1 - min(max(value, 0.0), 1.0)) * (high - low))

    codes: list[int] = []
    for channel, src, boost, fixed in PICO6_TO_INSPIRE13:
        _, low, high, _, _ = INSPIRE_CHANNELS[channel]
        if fixed is not None:
            codes.append(fixed)
            continue
        v = clamped[int(src)]
        codes.append(mapped(v + v * .2 if boost else v, low, high))
    return codes


def _assert_pico6_table() -> None:
    """启动自检：展开表结构 + 端点码值，防表被改错后静默下发错手势。"""
    n = len(INSPIRE_CHANNELS)
    if len(PICO6_TO_INSPIRE13) != n:
        raise RuntimeError("pico6 展开表长度与 INSPIRE_CHANNELS 不一致")
    seen = [row[0] for row in PICO6_TO_INSPIRE13]
    if sorted(seen) != list(range(n)):
        raise RuntimeError(f"pico6 展开表通道索引不完备: {seen}")
    for channel, src, _boost, fixed in PICO6_TO_INSPIRE13:
        if src is None and fixed is None:
            raise RuntimeError(f"pico6 通道 {channel} 既无输入也无固定值")
        if src is not None and not 0 <= src < PICO6_INPUT_DIM:
            raise RuntimeError(f"pico6 通道 {channel} 输入索引越界: {src}")
        if fixed is not None:
            _, low, high, _, _ = INSPIRE_CHANNELS[channel]
            if not low <= fixed <= high:
                raise RuntimeError(f"pico6 通道 {channel} 固定码 {fixed} 越界")
    for label, value, expected in (
        ("张开", [0.0] * PICO6_INPUT_DIM, [row[2] for row in INSPIRE_CHANNELS]),
        ("闭合", [1.0] * PICO6_INPUT_DIM, [row[1] for row in INSPIRE_CHANNELS]),
    ):
        codes = pico6_to_inspire13(value)
        for ch, code in enumerate(codes):
            if ch in (3, 5):  # 侧摆固定中立，不参与端点比对
                if code != 0:
                    raise RuntimeError(f"pico6 {label}: 侧摆通道 {ch} 应为 0，得到 {code}")
            elif code != expected[ch]:
                raise RuntimeError(
                    f"pico6 {label}: 通道 {ch} 应为 {expected[ch]}，得到 {code}"
                )


def _assert_pico6_joints(side: str, joints_from_plugin: list[str] | None) -> list[str]:
    """校验插件传下来的 6 维策略列序，防止数据集顺序与展开表错位。"""
    expected = pico6_action_joints(side)
    if joints_from_plugin:
        stripped = [name.removesuffix(".pos") for name in joints_from_plugin]
        if stripped != expected:
            raise RuntimeError(
                f"插件传入的{side}手 6 维动作列序与桥接展开表不一致：\n"
                f"  插件: {stripped}\n  桥接: {expected}"
            )
    return expected


def _assert_hand_table(
    variant: str, side: str, joints_from_plugin: list[str] | None
) -> list[str]:
    """校验手部通道表完整，并与插件传下来的硬件关节序逐字比对。

    "13 个手部关节名全部命中通道表、无重复、无遗漏"是这套映射唯一能自证的地方：
    数据集手序（URDF: thumb→pinky）与真机通道序（pinky→thumb）不是同一个排列，
    只要有一处错位，策略下发的手指就会整体错位，而画面看不出来。
    """
    if side not in ("left", "right"):
        raise RuntimeError(f"未知的手部 side: {side!r}")
    table = hand_channel_joints(variant, side)
    names = [entry[0] if entry else "" for entry in table]
    expected = HAND_CHANNEL_COUNT[variant]

    if len(table) != expected:
        raise RuntimeError(
            f"{variant}/{side} 通道表长度 {len(table)} != HAND_CHANNEL_COUNT {expected}"
        )
    empty = [i for i, name in enumerate(names) if not name]
    if empty:
        raise RuntimeError(f"{variant}/{side} 通道表有空通道: {empty}")
    if len(set(names)) != len(names):
        raise RuntimeError(f"{variant}/{side} 通道表有重复关节: {names}")

    if joints_from_plugin:
        stripped = [name.removesuffix(".pos") for name in joints_from_plugin]
        if stripped != names:
            raise RuntimeError(
                f"插件传入的{side}手关节序与本文件通道表不一致（常量副本已漂移）：\n"
                f"  插件: {stripped}\n  桥接: {names}"
            )
    return names


# ── Default config values (used when --config is not provided) ────────────────
_DEFAULT_CFG = {
    "zmq_cmd_port": 5559,
    "zmq_status_port": 5560,
    "ros_namespace": "",
    "cmd_namespace": "",
    "hand_variant": DEFAULT_VARIANT,
    # 因时手的策略侧量纲：rad13（老 20D，13 维 rad）| pico6（r13 数据集，6 维 0~1）。
    # 可用 {side}_hand_action_mode 覆盖单侧；pico6 仅对 hand_variant=inspire 有效。
    "hand_action_mode": DEFAULT_HAND_ACTION_MODE,
    # pico6 模式下插件透传的 6 维策略列名（带 .pos 后缀），用于启动逐字比对；
    # 硬件 13 通道名仍走 {side}_hand_joints + _assert_hand_table。
    "right_hand_action_joints": [],
    "left_hand_action_joints": [],
    "right_arm_motor_ids": list(RIGHT_ARM_MOTOR_IDS),
    # 左侧/头部默认关闭：老配置（13D/20D 只有右臂右手）行为完全不变。
    # 双臂数据训练时由插件 to_bridge_config() 打开对应开关。
    "enable_right_arm": True,
    "enable_left_arm": False,
    "enable_right_hand": True,
    "enable_left_hand": False,
    "enable_head": False,
    "arm_speed": GROUP_CMD_LIMITS["arm"]["spd"],
    "arm_current": GROUP_CMD_LIMITS["arm"]["cur"],
    "topic_arm_cmd": "/arm/cmd",
    "topic_head_cmd": "/head/cmd",
    "topic_right_hand_cmd": "",  # 留空 = 按 hand_variant 选
    "topic_left_hand_cmd": "",   # 同上（left）
    "topic_robot_state": "/robot_state",
    "topic_estop": "/power/board/key_status",
    "topic_reset": "/sim/cmd_reset",
    "image_enabled": False,
    "image_port": 5558,
    "topic_image_head": "/ob_camera_head/color/image_raw",
    # 编码前缩放目标（0/0 = 不缩放，仿真链路保持原样）。
    # 真机 /ob_camera_head 是 1280x720，而 dex_camera 只按 offset_x+width 切列、不缩放，
    # 原样送出会拿到"左半幅"——策略照常输出、但画面分布全错，且不报错。
    "image_width": 0,
    "image_height": 0,
    # True = 订阅 .../image_raw/compressed（真机推荐）。
    # 实测：raw 1280x720 的 rclpy 反序列化约 35 ms/帧，桥接只能转出 10~16 fps；
    # 走 compressed（约 386 KB）解码+缩放+重编码约 11 ms/帧，可稳定 ~20 fps。
    "image_compressed": False,
}

# 手型 -> side -> (指令话题, 状态话题)，与真机话题表一致
_HAND_TOPICS = {
    "inspire": {
        "left": ("/left_hand/angle_set", "/left_hand/angle_actual"),
        "right": ("/right_hand/angle_set", "/right_hand/angle_actual"),
    },
    "brainco": {
        "left": ("/left_hand/set_motor_multi", "/left_hand/motor_status"),
        "right": ("/right_hand/set_motor_multi", "/right_hand/motor_status"),
    },
}


def _status_qos(depth: int = 10):
    """状态订阅统一用 BEST_EFFORT。

    QoS 兼容性是单向的：BEST_EFFORT 订阅能收 RELIABLE 与 BEST_EFFORT 两种发布端，
    而 RELIABLE 订阅只收 RELIABLE 发布端。真机各节点的 QoS 不由我们决定，一旦
    ``/robot_state`` 是 BEST_EFFORT，默认（RELIABLE）订阅会表现为"话题列表可见、
    ``ros2 topic echo`` 有数，但桥接一行收不到"——最难查的一类故障。
    状态本来就是缓存语义，丢一帧无影响，所以这里取兼容性最大的那一侧。
    """
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ZMQInternalBridge:
    """ZMQ sockets for communication with LeRobot process.

    Bridge2 binds (server), LeRobot connects (client).
    """

    def __init__(self, cmd_port: int, status_port: int):
        self.context = zmq.Context()
        # SUB: receive actions from LeRobot
        self.cmd_socket = self.context.socket(zmq.SUB)
        self.cmd_socket.bind(f"tcp://*:{cmd_port}")
        self.cmd_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.cmd_socket.setsockopt(zmq.RCVHWM, 1)

        # PUB: forward status to LeRobot
        self.status_socket = self.context.socket(zmq.PUB)
        self.status_socket.bind(f"tcp://*:{status_port}")
        self.status_socket.setsockopt(zmq.SNDHWM, 1)

        # PUB: forward JPEG frames to DexCamera（仅 image_enabled 时创建）
        self.image_socket: zmq.Socket | None = None

        logger.info("ZMQ internal bridge: cmd=%d, status=%d", cmd_port, status_port)

    def bind_image(self, port: int) -> None:
        self.image_socket = self.context.socket(zmq.PUB)
        self.image_socket.bind(f"tcp://*:{port}")
        # 相机端 RCVHWM=1，这里同样只留最新一帧，积压无意义
        self.image_socket.setsockopt(zmq.SNDHWM, 1)
        logger.info("ZMQ image bridge: %d", port)

    def recv_action(self, timeout_ms: int = 100) -> dict | None:
        try:
            msg = self.cmd_socket.recv_json(flags=zmq.NOBLOCK)
            return msg
        except zmq.Again:
            return None

    def send_status(self, status: dict) -> None:
        try:
            self.status_socket.send_json(status, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("Status send dropped: ZMQ send buffer full (SNDHWM=1)")

    def send_image(self, jpg_bytes: bytes) -> None:
        if self.image_socket is None:
            return
        try:
            self.image_socket.send(jpg_bytes, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("Image send dropped: ZMQ send buffer full (SNDHWM=1)")

    def close(self) -> None:
        if self.image_socket is not None:
            self.image_socket.close(linger=0)
            self.image_socket = None
        self.cmd_socket.close(linger=0)
        self.status_socket.close(linger=0)
        self.context.term()


class RealRobotBridge:
    """ROS2 DDS ↔ TienKung Dex hardware.

    Subscribes /robot_state + 右手状态 + 急停（+ 头相机），发布 /arm/cmd + 手指令。
    所有硬件常量（motor id、spd/cur、话题名、手型）都从 cfg 读。
    """

    def __init__(self, zmq_bridge: ZMQInternalBridge, cfg: dict):
        self.zmq_bridge = zmq_bridge
        self._cfg = cfg
        # ---- 组开关：支持任意组合（只左臂 / 只右手 / 左因时右强脑 / 加头部）----
        # 显式开关，默认只开右侧（与 13D/20D 既有行为一致）。
        # 注意：不能用"配置里是否有 left_xxx"来判断——_DEFAULT_CFG 会给默认值，
        # 那样任何配置都会把左侧判成启用（实测踩过）。
        self._enable = {
            "right_arm": bool(cfg.get("enable_right_arm", True)),
            "left_arm": bool(cfg.get("enable_left_arm", False)),
            "right_hand": bool(cfg.get("enable_right_hand", True)),
            "left_hand": bool(cfg.get("enable_left_hand", False)),
            "head": bool(cfg.get("enable_head", False)),
        }
        if not any(self._enable.values()):
            raise RuntimeError(f"至少需要启用一个组: {self._enable}")

        # 左右手可以是不同手型（例：左因时 / 右强脑）
        self._hand_variant = {
            side: (cfg.get(f"{side}_hand_variant") or cfg.get("hand_variant") or DEFAULT_VARIANT)
            for side in ("left", "right")
        }
        for side, variant in self._hand_variant.items():
            if variant not in HAND_CHANNEL_COUNT:
                raise RuntimeError(f"未知 {side}_hand_variant: {variant!r}")

        # 因时手的策略侧动作量纲（可按侧覆盖）。pico6 只对 inspire 有效：
        # 强脑本来就是 6 维 0~1，不需要展开。
        self._hand_action_mode: dict[str, str] = {}
        for side in ("left", "right"):
            mode = (
                cfg.get(f"{side}_hand_action_mode")
                or cfg.get("hand_action_mode")
                or DEFAULT_HAND_ACTION_MODE
            )
            if mode not in HAND_ACTION_MODES:
                raise RuntimeError(
                    f"未知 {side}_hand_action_mode: {mode!r}（可选 {HAND_ACTION_MODES}）"
                )
            if (mode == "pico6" and self._enable[f"{side}_hand"]
                    and self._hand_variant[side] != "inspire"):
                raise RuntimeError(
                    f"{side}_hand_action_mode=pico6 要求 {side}_hand_variant=inspire，"
                    f"实际为 {self._hand_variant[side]!r}"
                )
            self._hand_action_mode[side] = mode
        if any(mode == "pico6" for mode in self._hand_action_mode.values()):
            _assert_pico6_table()

        self._arm_sides = [s for s in ("right", "left") if self._enable[f"{s}_arm"]]
        self._hand_sides = [s for s in ("right", "left") if self._enable[f"{s}_hand"]]
        self._sides = [s for s in ("right", "left") if s in self._arm_sides or s in self._hand_sides]
        # 兼容旧字段 self._variant：右手手型（未启用右手时取左手）
        self._variant = (
            self._hand_variant["right"] if self._enable["right_hand"]
            else self._hand_variant["left"]
        )

        self._arm_motor_ids = {
            "right": [int(m) for m in (cfg.get("right_arm_motor_ids") or ARM_MOTOR_IDS["right"])],
            "left": [int(m) for m in (cfg.get("left_arm_motor_ids") or ARM_MOTOR_IDS["left"])],
        }
        # 臂的关节级配置：{side}_arm_joints 与 {side}_arm_motor_ids 成对，可为子集。
        # 插件传下来的名字带 ``.pos`` 后缀（LeRobot 约定），这里统一剥掉。
        self._arm_joints = {
            side: [j.removesuffix(".pos") for j in (cfg.get(f"{side}_arm_joints") or ARM_JOINTS[side])]
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            joints, ids = self._arm_joints[side], self._arm_motor_ids[side]
            if len(joints) != len(ids):
                raise RuntimeError(
                    f"{side} 臂配置不一致：{side}_arm_joints={len(joints)} 个，"
                    f"{side}_arm_motor_ids={len(ids)} 个（必须一一对应）"
                )
            unknown = [j for j in joints if j not in BODY_JOINT_LIMITS]
            if unknown:
                raise RuntimeError(f"{side} 臂含未知/无限位关节: {unknown}")
            if len(set(joints)) != len(joints):
                raise RuntimeError(f"{side} 臂关节重复: {joints}")

        # 头部关节级配置：head_joints 与 head_motor_ids 成对，可只启用其中一个轴
        self._head_joints = [
            j.removesuffix(".pos") for j in (cfg.get("head_joints") or HEAD_JOINTS)
        ]
        self._head_motor_ids = [int(m) for m in (cfg.get("head_motor_ids") or HEAD_MOTOR_IDS)]
        if len(self._head_joints) != len(self._head_motor_ids):
            raise RuntimeError(
                f"头部配置不一致：head_joints={len(self._head_joints)} 个，"
                f"head_motor_ids={len(self._head_motor_ids)} 个（必须一一对应）"
            )
        unknown = [j for j in self._head_joints if j not in BODY_JOINT_LIMITS]
        if unknown:
            raise RuntimeError(f"头部含未知/无限位关节: {unknown}")

        self._arm_speed = float(cfg.get("arm_speed", GROUP_CMD_LIMITS["arm"]["spd"]))
        self._arm_current = float(cfg.get("arm_current", GROUP_CMD_LIMITS["arm"]["cur"]))

        # 手部话题 / 通道表 / 每只手的消息类：全按该 side 自己的 hand_variant
        self._hand_topic_cmd: dict[str, str] = {}
        self._hand_topic_state: dict[str, str] = {}
        self._hand_joints: dict[str, list[str]] = {}
        self._hand_channel_count: dict[str, int] = {}
        # ZMQ 里 right_hand / left_hand 的维度（动作与状态同维）：
        #   rad13  = 硬件通道数（因时 13 / 强脑 6）
        #   pico6  = 6（策略侧归一化列），硬件仍固定 13 通道
        self._hand_action_joints: dict[str, list[str]] = {}
        self._hand_action_dim: dict[str, int] = {
            side: HAND_CHANNEL_COUNT[self._hand_variant[side]] for side in ("left", "right")
        }
        self._hand_state_dim: dict[str, int] = dict(self._hand_action_dim)
        for side in self._hand_sides:
            variant = self._hand_variant[side]
            default_cmd, default_state = _HAND_TOPICS[variant][side]
            self._hand_topic_cmd[side] = cfg.get(f"topic_{side}_hand_cmd") or default_cmd
            self._hand_topic_state[side] = cfg.get(f"topic_{side}_hand_state") or default_state
            # 物理通道表永远是 13（因时）/ 6（强脑），与动作模式无关。
            self._hand_joints[side] = _assert_hand_table(
                variant, side, cfg.get(f"{side}_hand_joints")
            )
            self._hand_channel_count[side] = HAND_CHANNEL_COUNT[variant]
            if self._hand_action_mode[side] == "pico6":
                self._hand_action_joints[side] = _assert_pico6_joints(
                    side, cfg.get(f"{side}_hand_action_joints")
                )
                self._hand_action_dim[side] = PICO6_INPUT_DIM
                # 状态回显 6 维指令：r13 数据集 observation.state 手部是
                # action 的副本（fill_state_from_action），采集端也是手部开环。
                self._hand_state_dim[side] = PICO6_INPUT_DIM

        ros_namespace = (cfg.get("ros_namespace") or "").rstrip("/")
        cmd_namespace = (cfg.get("cmd_namespace") or "").rstrip("/")

        self._msgs = self._load_msgs()

        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("ros2_deploy_bridge")

        # State caches（左右两侧都建，便于未启用侧也有确定值/别名）
        self._arm_jpos = {side: [0.0] * len(self._arm_motor_ids[side]) for side in ("left", "right")}
        self._hand_pos = {
            side: [0.0] * self._hand_state_dim[side] for side in ("left", "right")
        }
        # pico6 模式下保留真机 13 个反馈码（仅日志/健康检查，不进策略状态）
        self._hand_feedback_codes: dict[str, list[int]] = {"left": [], "right": []}
        self._head_pos = [0.0] * len(self._head_motor_ids)
        # 兼容旧引用
        self._right_arm_motor_ids = self._arm_motor_ids["right"]
        self._right_arm_jpos = self._arm_jpos["right"]
        self._right_hand_pos = self._hand_pos["right"]
        self._right_hand_joints = self._hand_joints.get("right", [])
        self._estop = False
        self._state_lock = threading.Lock()

        # Publishers — command side (cmd_namespace)
        arm_topic = f"{cmd_namespace}{cfg['topic_arm_cmd']}" if cmd_namespace else cfg["topic_arm_cmd"]
        self._arm_cmd_pub = self._node.create_publisher(
            self._msgs["ArmCtrl"], arm_topic, 10
        )
        self._hand_pubs: dict[str, Any] = {}
        for side in self._hand_sides:
            topic = self._hand_topic_cmd[side]
            full = f"{cmd_namespace}{topic}" if cmd_namespace else topic
            self._hand_pubs[side] = self._node.create_publisher(
                self._hand_msgs[side]["command"], full, 10
            )
        self._head_pub = None
        if self._enable["head"]:
            head_topic = cfg.get("topic_head_cmd", "/head/cmd")
            head_full = f"{cmd_namespace}{head_topic}" if cmd_namespace else head_topic
            self._head_pub = self._node.create_publisher(
                self._msgs["HeadCtrl"], head_full, 10
            )

        # Subscribers — status side (ros_namespace)
        status_qos = _status_qos()
        self._node.create_subscription(
            self._msgs["RobotState"],
            f"{ros_namespace}{cfg['topic_robot_state']}",
            self._robot_state_callback, status_qos,
        )
        for side in self._hand_sides:
            self._node.create_subscription(
                self._hand_msgs[side]["state"],
                f"{ros_namespace}{self._hand_topic_state[side]}",
                self._make_hand_state_callback(side), status_qos,
            )
        self._create_estop_subscription(cfg, ros_namespace)

        # 头相机：只有真机才需要转发（仿真里 AsyncJpegCameraSender 直发 5758）
        self._cv_bridge = None
        self._cv2 = None
        self._last_image_ts = 0.0
        self._image_min_interval = 1.0 / 20.0
        # 多线程执行器下图像回调可能并发重入；用非阻塞锁"忙则丢帧"，
        # 保证同一时刻只有一帧在解码/编码（保持最新帧语义）。
        self._image_lock = threading.Lock()
        # 状态发布限频：/robot_state 是 200Hz，原样转发会把唯一执行线程占满
        # （实测图像被饿到 ~12fps）。策略跑 15Hz，60Hz 状态足够新。
        self._last_status_ts = 0.0
        self._status_min_interval = 1.0 / 60.0
        if cfg.get("image_enabled", False):
            self._setup_image_forwarding(cfg, ros_namespace)

        # Start executor
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="ros2_executor"
        )
        self._executor_thread.start()

        # Action forwarding thread
        self._running = True
        self._action_thread = threading.Thread(
            target=self._action_loop, daemon=True, name="action_forward"
        )
        self._action_thread.start()

        logger.info(
            "Real robot bridge started (ns=%s, cmd_ns=%s, arm_sides=%s, hand_sides=%s, "
            "head=%s, spd=%.2f, cur=%.0f)",
            ros_namespace, cmd_namespace, self._arm_sides, self._hand_sides,
            self._enable["head"], self._arm_speed, self._arm_current,
        )
        for side in self._arm_sides:
            logger.info(
                "%s臂 motors=%s", "左" if side == "left" else "右",
                self._arm_motor_ids[side],
            )
        for side in self._hand_sides:
            variant = self._hand_variant[side]
            mode = self._hand_action_mode[side]
            label = "左" if side == "left" else "右"
            logger.info(
                "%s手(%s) 通道 -> 关节: %s", label, variant,
                dict(enumerate(self._hand_joints[side])),
            )
            if mode == "pico6":
                logger.info(
                    "%s手控制量口径: 6 维归一化 0~1（0=张开/1=闭合）-> 13 个 0.1° 角码；"
                    "状态回显 6 维指令（开环，与 r13 数据集 state=action 一致）", label,
                )
                logger.info("%s手动作列序: %s", label, self._hand_action_joints[side])
                logger.info("%s手 6→13 展开表: %s", label, PICO6_TO_INSPIRE13)
            else:
                logger.info(
                    "%s手控制量口径: %s", label,
                    "闭合度 0~1 -> 码 1~1000（brainco，与训练数据集一致）"
                    if variant == "brainco"
                    else "关节角 rad -> 0.1° 角码（inspire，rad13）",
                )
        if not self._enable["head"]:
            logger.info("头部未启用（enable_head=false）")

    # ------------------------------------------------------------------ 消息包
    def _load_msgs(self) -> dict[str, Any]:
        """导入真机消息包；缺包时给出可执行的修复提示。

        左右手可以是不同手型，因此按 side 分别加载对应的手部消息类
        （brainco = SetMotorMulti/MotorStatus，inspire = SetAngle/GetAngleAct）。
        """
        try:
            from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorCtrl, RobotState
        except ImportError as exc:
            raise RuntimeError(
                f"{exc}：需要 ros2_bridge_msgs。容器内先 bash run.sh check，"
                "或 source /opt/tienkung_dex_msgs_ws/install/setup.bash"
            ) from exc

        msgs: dict[str, Any] = {
            "ArmCtrl": ArmCtrl,
            "HeadCtrl": HeadCtrl,
            "MotorCtrl": MotorCtrl,
            "RobotState": RobotState,
        }

        def hand_classes(variant: str) -> dict[str, Any]:
            if variant == "brainco":
                try:
                    from brainco_hand_msgs.msg import MotorStatus, SetMotorMulti
                except ImportError as exc:
                    raise RuntimeError(
                        f"{exc}：brainco 变体需要 brainco_hand_msgs，"
                        "容器内先 bash run.sh check"
                    ) from exc
                return {"command": SetMotorMulti, "state": MotorStatus}
            try:
                from inspire_hand_msgs.msg import GetAngleAct, SetAngle
            except ImportError as exc:
                raise RuntimeError(
                    f"{exc}：inspire 变体需要 inspire_hand_msgs，"
                    "容器内先 bash run.sh check"
                ) from exc
            return {"command": SetAngle, "state": GetAngleAct}

        self._hand_msgs = {
            side: hand_classes(self._hand_variant[side]) for side in self._hand_sides
        }
        return msgs

    def _create_estop_subscription(self, cfg: dict, ros_namespace: str) -> None:
        """急停订阅，消息包不可用时降级为"无急停"并告警（仿真里通常没有这个包）。"""
        try:
            from bodyctrl_msgs.msg import PowerBoardKeyStatus
        except ImportError:
            logger.warning(
                "bodyctrl_msgs 不可用：急停监听已关闭。真机部署前必须补上"
                "（/power/board/key_status 是唯一能在紧急时停发的软件通道）。"
            )
            return
        self._node.create_subscription(
            PowerBoardKeyStatus,
            f"{ros_namespace}{cfg['topic_estop']}",
            self._estop_callback, _status_qos(),
        )

    def _setup_image_forwarding(self, cfg: dict, ros_namespace: str) -> None:
        """真机头相机：sensor_msgs/Image → JPEG → ZMQ image_port。"""
        import cv2
        from cv_bridge import CvBridge
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage, Image

        self._cv2 = cv2
        self._cv_bridge = CvBridge()
        self._jpeg_quality = int(cfg.get("image_quality", 80))
        # 缩放目标：真机 1280x720 -> 640x360（与仿真头相机同视野，见 config/tienkung_dex/*.yaml
        # 的 focal 24 / aperture 44.76 ≈ 实机 fx 690.6 的 85.6° 水平视场）
        self._image_size = (
            int(cfg.get("image_width", 0)),
            int(cfg.get("image_height", 0)),
        )
        # JPEG 直传判定（None = 尚未探测）。目标尺寸与源尺寸一致时没有必要解码再
        # 重编码：实测 720p 的 imdecode+resize+imencode 会把帧间隔拉到 p95≈110ms、
        # 偶发 1.4s 空洞（策略 20 步取一次观测，会连续两次拿到同一帧 -> 重复动作）；
        # 直传后稳定 ≈30fps（源帧率）。
        self._image_passthrough: bool | None = None
        self._image_src_size: tuple[int, int] | None = None

        self.zmq_bridge.bind_image(int(cfg["image_port"]))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # 独立回调组：与 200Hz 的 /robot_state 解耦，否则多线程执行器里仍会被串行化
        self._image_cb_group = MutuallyExclusiveCallbackGroup()
        self._image_compressed = bool(cfg.get("image_compressed", False))
        if self._image_compressed:
            topic = f"{ros_namespace}{cfg['topic_image_head']}/compressed"
            self._node.create_subscription(
                CompressedImage, topic, self._image_compressed_callback, qos,
                callback_group=self._image_cb_group,
            )
        else:
            topic = f"{ros_namespace}{cfg['topic_image_head']}"
            self._node.create_subscription(
                Image, topic, self._image_callback, qos,
                callback_group=self._image_cb_group,
            )
        logger.info(
            "图像转发已开启：%s -> ZMQ %d（缩放目标 %s）",
            topic, cfg["image_port"],
            f"{self._image_size[0]}x{self._image_size[1]}"
            if all(self._image_size) else "原尺寸",
        )

    # ---------------------------------------------------------------- 回调
    def _robot_state_callback(self, msg: Any) -> None:
        """/robot_state：按关节 ID 取双臂 + 头部（不是按位取）。

        只更新已启用的组；某组缺 ID 时该组本帧不更新（首次告警，避免刷屏）。
        """
        pos_by_id: dict[int, float] = {}
        for group in ("head", "arm"):
            status = getattr(msg, group, None)
            for motor in getattr(status, "status", []):
                pos_by_id[int(motor.name)] = float(motor.pos)

        updated = False
        with self._state_lock:
            for side in self._arm_sides:
                ids = self._arm_motor_ids[side]
                missing = [mid for mid in ids if mid not in pos_by_id]
                if missing:
                    self._warn_missing(f"{side}_arm", missing)
                    continue
                for i, mid in enumerate(ids):
                    self._arm_jpos[side][i] = pos_by_id[mid]
                updated = True

            if self._enable["head"]:
                missing = [mid for mid in self._head_motor_ids if mid not in pos_by_id]
                if missing:
                    self._warn_missing("head", missing)
                else:
                    for i, mid in enumerate(self._head_motor_ids):
                        self._head_pos[i] = pos_by_id[mid]
                    updated = True

        if not updated:
            return
        now = time.monotonic()
        if now - self._last_status_ts >= self._status_min_interval:
            self._last_status_ts = now
            self._publish_status()

    def _warn_missing(self, group: str, missing: list[int]) -> None:
        """缺 ID 只首次告警（/robot_state 200Hz，不节流会刷屏）。"""
        warned = getattr(self, "_warned_missing_ids", None)
        if warned is None:
            warned = self._warned_missing_ids = set()
        new = [mid for mid in missing if (group, mid) not in warned]
        if new:
            warned.update((group, mid) for mid in new)
            logger.warning("/robot_state 缺少 %s 关节 ID %s，该组本帧不更新", group, new)

    def _make_hand_state_callback(self, side: str):
        """为某只手生成反馈回调（左右手可能是不同变体，消息类型也不同）。

        因时 GetAngleAct：13 个 0.1° 角码 -> rad；
        强脑 MotorStatus：6 个通道码（1~1000，**不是**闭合度）-> 闭合度 [0,1]，
        与训练数据集 ``observation.state`` 同口径。

        强脑的空闲/忙时无效帧（现场实测 ``positions=65535``、``states=255``）不参与
        换算——``(65535-1)/999`` 夹取后是 1.0，会让策略把"无数据"看成"全闭合"。
        这类通道保留上一帧有效值。

        ``hand_action_mode=pico6`` 时**不反解反馈**：6→13 不是单射（指根 ×1.2、
        侧摆恒 0、拇指两节同源），且 r13 数据集的 ``observation.state`` 手部本来
        就是 action 副本、采集端也是开环。反馈码只留作日志/健康检查，策略状态由
        :meth:`_publish_hand_command` 回显 6 维指令。
        """
        variant = self._hand_variant[side]
        count = self._hand_channel_count[side]
        joints = self._hand_joints[side]
        mode = self._hand_action_mode[side]
        label = "左" if side == "left" else "右"

        def callback(msg: Any) -> None:
            raw = list(msg.positions) if variant == "brainco" else list(msg.joint_values)
            if len(raw) < count:
                return

            if mode == "pico6":
                with self._state_lock:
                    self._hand_feedback_codes[side] = [int(code) for code in raw[:count]]
                return

            if variant == "brainco":
                # 反馈域允许 0（硬件全伸时实测就是 0），指令域才是 1~1000
                valid = [
                    (ch, int(code)) for ch, code in enumerate(raw[:count])
                    if 0 <= int(code) <= BRAINCO_CODE_MAX
                ]
                if not valid:
                    if not getattr(self, "_warned_hand_invalid", None):
                        self._warned_hand_invalid = set()
                    if side not in self._warned_hand_invalid:
                        self._warned_hand_invalid.add(side)
                        logger.warning("%s手反馈整帧无效（positions=%s），保留上一帧状态", label, raw)
                    return
                with self._state_lock:
                    for ch, code in valid:
                        self._hand_pos[side][ch] = hand_code_to_value(
                            variant, ch, code, joints[ch]
                        )
                self._publish_status()
                return

            values = [
                hand_code_to_value(variant, ch, raw[ch], joints[ch]) for ch in range(count)
            ]
            with self._state_lock:
                self._hand_pos[side][:] = values
            self._publish_status()

        return callback

    def _estop_callback(self, msg: Any) -> None:
        engaged = bool(getattr(msg, "is_estop").data)
        if engaged == self._estop:
            return
        self._estop = engaged
        if engaged:
            logger.error("急停触发（is_estop=true）：已停止下发手臂与手指令。")
        else:
            logger.warning("急停解除（is_estop=false）：恢复下发。")
        self._publish_status()

    def _image_callback(self, msg: Any) -> None:
        """raw 源：sensor_msgs/Image（仿真兼容路径）。"""
        self._image_gate_and_send(msg, compressed=False)

    def _image_compressed_callback(self, msg: Any) -> None:
        """compressed 源：sensor_msgs/CompressedImage（真机推荐，反序列化成本低 3x）。"""
        self._image_gate_and_send(msg, compressed=True)

    def _image_gate_and_send(self, msg: Any, compressed: bool) -> None:
        """限流 + 缩放 + JPEG 编码 + ZMQ 发送。

        时间戳只在"真正处理"时更新：被限流丢掉的帧若也重置窗口，会连锁丢帧。
        """
        if not self._image_lock.acquire(blocking=False):
            return  # 上一帧还在转换/编码，丢这一帧（保持最新帧语义）
        try:
            now = time.monotonic()
            if now - self._last_image_ts < self._image_min_interval:
                return
            self._last_image_ts = now

            if compressed:
                import numpy as np

                raw = bytes(msg.data)
                if self._image_passthrough is None:
                    # 首次到达：解码一帧拿源尺寸，决定后面走直传还是解码+缩放。
                    probe = self._cv2.imdecode(
                        np.frombuffer(raw, dtype=np.uint8), self._cv2.IMREAD_COLOR
                    )
                    if probe is None:
                        return
                    self._image_src_size = (probe.shape[1], probe.shape[0])
                    need_resize = all(self._image_size) and self._image_src_size != self._image_size
                    self._image_passthrough = not need_resize
                    logger.info(
                        "图像源 %dx%d -> 目标 %s：%s",
                        self._image_src_size[0], self._image_src_size[1],
                        f"{self._image_size[0]}x{self._image_size[1]}"
                        if all(self._image_size) else "原尺寸",
                        "JPEG 直传（不解码/不重编码）" if self._image_passthrough
                        else "解码+缩放+重编码",
                    )
                if self._image_passthrough:
                    # 直接把相机 JPEG 原样推给 DexCamera（它自己解码一次）。
                    self.zmq_bridge.send_image(raw)
                    return
                frame = self._cv2.imdecode(
                    np.frombuffer(raw, dtype=np.uint8), self._cv2.IMREAD_COLOR
                )
                if frame is None:
                    return
            else:
                frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 先缩放再编码：dex_camera 侧不做缩放，只按 offset_x+width 切列。
            # 真机 1280x720 必须在**发送端**变成 640x360，否则相机拿到的是左半幅。
            w, h = self._image_size
            if w > 0 and h > 0 and (frame.shape[1], frame.shape[0]) != (w, h):
                frame = self._cv2.resize(frame, (w, h), interpolation=self._cv2.INTER_LINEAR)
            ok, buf = self._cv2.imencode(
                ".jpg", frame, [int(self._cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            )
            if ok:
                self.zmq_bridge.send_image(buf.tobytes())
        except Exception as exc:  # noqa: BLE001 - 单帧失败不应打死执行器
            logger.warning("图像转换失败: %s", exc)
        finally:
            self._image_lock.release()

    # ---------------------------------------------------------------- 状态
    def _publish_status(self) -> None:
        """ZMQ 状态：只回传已启用的组（键名 = 动作空间的键名）。"""
        with self._state_lock:
            status: dict[str, Any] = {"estop": self._estop, "ts": time.time()}
            for side in self._arm_sides:
                status[f"{side}_arm"] = list(self._arm_jpos[side])
            for side in self._hand_sides:
                status[f"{side}_hand"] = list(self._hand_pos[side])
            if self._enable["head"]:
                status["head"] = list(self._head_pos)
        self.zmq_bridge.send_status(status)

    # ---------------------------------------------------------------- 指令
    def _action_loop(self) -> None:
        while self._running:
            action = self.zmq_bridge.recv_action(timeout_ms=50)
            if action is None:
                continue
            for side in self._arm_sides:
                self._publish_arm_command(action.get(f"{side}_arm", []), side)
            for side in self._hand_sides:
                self._publish_hand_command(action.get(f"{side}_hand", []), side)
            if self._enable["head"]:
                self._publish_head_command(action.get("head", []))

    def _publish_arm_command(self, values: list, side: str) -> None:
        """单侧臂位置指令；关节集合/ID/限位都可按配置裁剪（关节级颗粒度）。"""
        if not values:
            return
        if self._estop:
            return
        joints = self._arm_joints[side]
        ids = self._arm_motor_ids[side]
        if len(values) != len(joints):
            logger.warning(
                "%s_arm 维度不匹配: %d (期望 %d). Skipping.",
                side, len(values), len(joints),
            )
            return

        msg = self._msgs["ArmCtrl"]()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "arm"
        msg.mode = CONTROL_MODE_POSITION

        for joint_name, motor_id, value in zip(joints, ids, values):
            low, high = BODY_JOINT_LIMITS[joint_name]
            pos = min(max(float(value), low), high)
            if pos != float(value):
                logger.debug("%s 目标 %.4f 超限，夹到 [%.4f, %.4f]", joint_name, value, low, high)
            cmd = self._msgs["MotorCtrl"]()
            cmd.name = int(motor_id)
            cmd.pos = pos
            cmd.spd = self._arm_speed
            cmd.cur = self._arm_current
            msg.ctrl.append(cmd)

        self._arm_cmd_pub.publish(msg)

    def _publish_head_command(self, values: list) -> None:
        """头部位置指令（HeadCtrl）。head_joints 可只含一个轴。"""
        if not values:
            return
        if self._estop:
            return
        if len(values) != len(self._head_joints):
            logger.warning(
                "head 维度不匹配: %d (期望 %d). Skipping.", len(values), len(self._head_joints)
            )
            return

        msg = self._msgs["HeadCtrl"]()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "head"
        msg.mode = CONTROL_MODE_POSITION
        for joint_name, motor_id, value in zip(self._head_joints, self._head_motor_ids, values):
            low, high = BODY_JOINT_LIMITS[joint_name]
            pos = min(max(float(value), low), high)
            cmd = self._msgs["MotorCtrl"]()
            cmd.name = int(motor_id)
            cmd.pos = pos
            cmd.spd = HEAD_CMD_LIMITS["spd"]
            cmd.cur = HEAD_CMD_LIMITS["cur"]
            msg.ctrl.append(cmd)
        self._head_pub.publish(msg)

    def _publish_hand_command(self, position: list, side: str) -> None:
        """单只手的手指指令。硬件通道是定长数组，只能整只手启停。

        ``pico6`` 模式下 ZMQ 动作是 6 维 0~1，先夹紧再整只手展开成 13 个角码；
        ``rad13`` / ``brainco`` 走原来的逐通道换算（动作维度 = 硬件通道数）。
        """
        if not position:
            return
        if self._estop:
            return
        variant = self._hand_variant[side]
        count = self._hand_channel_count[side]
        joints = self._hand_joints[side]
        mode = self._hand_action_mode[side]
        action_dim = self._hand_action_dim[side]
        if len(position) != action_dim:
            logger.warning(
                "%s_hand 维度不匹配: %d (期望 %d, mode=%s). Skipping.",
                side, len(position), action_dim, mode,
            )
            return

        if mode == "pico6":
            clamped = [min(max(float(v), 0.0), 1.0) for v in position]
            if any(a != b for a, b in zip(clamped, position)):
                logger.debug("%s_hand 动作超出 [0,1]，已夹紧: %s -> %s", side, position, clamped)
            codes = pico6_to_inspire13(clamped)
            # 兜底：逐通道夹到硬件限位（展开表按限位构造，正常不会触发）
            codes = [
                min(max(code, INSPIRE_CHANNELS[ch][1]), INSPIRE_CHANNELS[ch][2])
                for ch, code in enumerate(codes)
            ]
            # 状态回显 6 维指令：与 r13 数据集 observation.state 同口径
            with self._state_lock:
                self._hand_pos[side][:] = clamped
            self._publish_status()
        else:
            # 控制量 -> 真机通道码：强脑=闭合度[0,1] -> 1~1000；因时=rad -> 0.1° 角码
            codes = [
                hand_action_to_code(variant, ch, position[ch], joints[ch])
                for ch in range(count)
            ]

        msg = self._hand_msgs[side]["command"]()
        if variant == "brainco":
            msg.mode = HAND_CMD_MODE
            msg.positions = codes[:count]
            msg.speeds = [0] * count
            msg.currents = [0] * count
            msg.pwms = [0] * count
            msg.durations = [0] * count
        else:
            # 因时 SetAngle.hand_id：1=左手 2=右手
            msg.hand_id = 1 if side == "left" else INSPIRE_RIGHT_HAND_ID
            msg.joint_values = codes
        self._hand_pubs[side].publish(msg)

    def stop(self) -> None:
        self._running = False
        if self._action_thread.is_alive():
            self._action_thread.join(timeout=2.0)
        if self._executor is not None:
            self._executor.shutdown()
        if self._executor_thread is not None and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()


def _is_alive(pid: int) -> bool:
    """Check whether a process is still running."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # process exists but we lack permission to signal it


def kill_existing_bridge(tag: str = "tienkung_dex") -> None:
    """Find and kill any already-running bridge processes of *this* robot.

    Sends SIGTERM first for graceful shutdown, then SIGKILL after 3 s if
    the process is still alive.  Waits an extra 0.5 s for ZMQ ports to be
    released before returning.

    与 tienkung_pro 的同名函数有两处不同，都是必须的：
    1. 额外要求路径里带 *tag*——三个机型的桥接脚本都叫 ros2_deploy_bridge.py，
       只按文件名匹配会把同时运行的 tienkung_pro 桥接一起杀掉；
    2. 只匹配 argv[1]（被执行的脚本），不匹配整条 cmdline。lerobot-rollout
       主进程把桥接路径作为 --robot.bridge_script= 的值带在身上，
       pgrep -f / pkill -f 会命中它，等于桥接启动时杀掉自己的父进程。
    """
    current_pid = os.getpid()
    parent_pid = os.getppid()

    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in (current_pid, parent_pid):
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                parts = f.read().split(b"\x00")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if len(parts) < 2:
            continue
        script = parts[1].decode("utf-8", "replace")
        if script.endswith("ros2_deploy_bridge.py") and tag in script:
            pids.append(pid)

    if not pids:
        return

    logger.info("Found existing bridge processes (PIDs: %s), terminating ...", pids)

    # --- graceful SIGTERM ---
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    # wait up to 3 s for them to exit
    alive = []
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        alive = [p for p in pids if _is_alive(p)]
        if not alive:
            break
        time.sleep(0.1)
    else:
        # --- force SIGKILL for stragglers ---
        for pid in alive:
            logger.warning("Force killing bridge process %d", pid)
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(0.1)

    # give the OS a moment to release ZMQ sockets
    time.sleep(0.5)
    logger.info("Previous bridge instances terminated.")


def _reset_once(topic: str, repeat: int = 3, interval: float = 0.1) -> int:
    """发 ``/sim/cmd_reset``（仿真专用，真机没有订阅者）。

    连发 *repeat* 次是为了躲 SUB 连接建立期的竞态：仿真侧的订阅者可能还没挂上，
    单发一次会静默丢失。
    """
    import rclpy
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node("tienkung_dex_reset")
    pub = node.create_publisher(Bool, topic, 1)
    try:
        for _ in range(max(1, repeat)):
            pub.publish(Bool(data=True))
            time.sleep(interval)
        logger.info("已发送 %s x%d", topic, max(1, repeat))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ROS2 Deploy Bridge for LeRobot + TienKung Dex"
    )
    parser.add_argument("--config", type=str, default=None,
                        help="JSON config string from TienKungDexRobotConfig.to_bridge_config() "
                             "(recommended). If omitted, legacy CLI args or defaults are used.")
    parser.add_argument("--variant", type=str, default=None, choices=["inspire", "brainco"],
                        help="灵巧手变体（默认取 config 里的 hand_variant）")
    parser.add_argument("--hand-action-mode", type=str, default=None,
                        choices=list(HAND_ACTION_MODES),
                        help="因时手动作量纲：rad13=13 维 rad；pico6=6 维 0~1 展开成 13 码")
    parser.add_argument("--domain-id", type=int, default=None,
                        help="覆盖 ROS_DOMAIN_ID（需与仿真/机器人所在域一致，默认 0）；必须在 rclpy.init 前设置")
    parser.add_argument("--enable-image", action="store_true",
                        help="把 /ob_camera_head 的 ROS 图像转成 JPEG 发到 image_port（真机用）")
    parser.add_argument("--reset", action="store_true",
                        help="只发一次 /sim/cmd_reset 然后退出（仿真复位）")
    # Legacy args (deprecated, kept for standalone testing)
    parser.add_argument("--zmq_cmd_port", type=int, default=None,
                        help="(deprecated) ZMQ port for receiving actions from LeRobot (bind SUB)")
    parser.add_argument("--zmq_status_port", type=int, default=None,
                        help="(deprecated) ZMQ port for sending status to LeRobot (bind PUB)")
    parser.add_argument("--ros_namespace", type=str, default=None,
                        help="(deprecated) ROS2 namespace for status topics (subscribe)")
    parser.add_argument("--cmd_namespace", type=str, default=None,
                        help="(deprecated) ROS2 namespace for command topics (publish)")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    # Build cfg dict: start from defaults, overlay --config JSON, then CLI overrides
    cfg = dict(_DEFAULT_CFG)

    if args.config:
        try:
            cfg.update(json.loads(args.config))
        except json.JSONDecodeError as e:
            logger.error("Failed to parse --config JSON: %s", e)
            return 1

    if args.variant is not None:
        cfg["hand_variant"] = args.variant
    if args.hand_action_mode is not None:
        cfg["hand_action_mode"] = args.hand_action_mode
    if args.enable_image:
        cfg["image_enabled"] = True
    if args.zmq_cmd_port is not None:
        cfg["zmq_cmd_port"] = args.zmq_cmd_port
    if args.zmq_status_port is not None:
        cfg["zmq_status_port"] = args.zmq_status_port
    if args.ros_namespace is not None:
        cfg["ros_namespace"] = args.ros_namespace
    if args.cmd_namespace is not None:
        cfg["cmd_namespace"] = args.cmd_namespace

    # Configure logging early so kill_existing_bridge() messages are visible
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    # 必须在任何 rclpy 调用之前设置
    if args.domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    if args.reset:
        return _reset_once(cfg["topic_reset"])

    # --- kill any existing bridge before starting ---
    kill_existing_bridge()

    zmq_bridge = ZMQInternalBridge(cfg["zmq_cmd_port"], cfg["zmq_status_port"])
    try:
        robot_bridge = RealRobotBridge(zmq_bridge, cfg)
    except RuntimeError as exc:
        logger.error("桥接启动失败：%s", exc)
        zmq_bridge.close()
        return 1

    stop_event = threading.Event()

    def signal_handler(sig, frame):
        logger.info("Received signal %s, shutting down...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Bridge running (ros_ns=%s, cmd_ns=%s). Press Ctrl+C to stop.",
                cfg.get("ros_namespace", ""), cfg.get("cmd_namespace", ""))
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass

    logger.info("Shutting down...")
    robot_bridge.stop()
    zmq_bridge.close()
    logger.info("Bridge stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
