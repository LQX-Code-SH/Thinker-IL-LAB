#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 预备位：四段安全到位 + 预备位手型。

在 lerobot-tienkung 容器内用系统 Python 3.10 运行（需要 rclpy 与真机/仿真消息包）：

    source /opt/ros/humble/setup.bash
    source /opt/tienkung_dex_msgs_ws/install/setup.bash
    /usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.py            # 仿真
    /usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.py --real     # 真机

通常经薄封装 :file:`robot_ready.sh` 调用（它负责 source ROS 环境与 ROS_DOMAIN_ID）。

**为什么不是"回零"、也不是一步到位**

* 预备位是**绝对**目标，不是"相对当帧读数"。站立位膝盖本就微屈（0.368/0.341），
  回 0 会跪下去；`READY_POSE` 才是数据采集时机器人待着的那个姿态。
* 正前方有桌面。从站立位直达预备位，手会贴着桌面从体侧扫过去被顶住，肘收不到位。
  因此照仿真的 ``move_to_ready_pose`` 分四段：
  纯肩外展抬臂 → 高位折前臂 → 大臂转回近垂直 → 前臂落到预备位。

**发哪些组**

| 组 | 话题 | 是否下发 | 原因 |
|---|---|---|---|
| head | `/head/cmd` (`HeadCtrl`) | 发 | `head_pitch = 0.785398`（低头 45°）决定训练视角，不发就是另一个观测分布 |
| waist | `/waist/cmd` (`ArmCtrl`) | 发 | 预备位腰部有姿态；`READY_POSE_TARGET` 也含腰 3 维 |
| arm | `/arm/cmd` (`ArmCtrl`) | 发 | **左臂也要发**：预备位左肩外展 ±0.60，数据集中左臂恒定在该姿态 |
| leg | `/leg/cmd` | **永远不发** | 真机由行走控制器独占；仿真里腿保持初始 standing 目标，正好与数据集一致 |

**为什么仿真和真机的最终目标不同**

``ready_final_pose(real=False)``（仿真）= ``{**READY_POSE, **READY_POSE_TARGET}``：
TARGET 是"让指令 = 实际"的重力前馈补偿（只覆盖双臂 14 + 腰 3，**不含 head**，head 必须
从 READY_POSE 取）。``--real`` 用 ``READY_POSE`` —— 真机驱动器闭环，本来就没有仿真那段
下垂，用 TARGET 会站成旧版仿真姿态（低 13 cm）。

**与 Bridge2 的互斥**：本脚本与 Bridge2 都往 `/arm/cmd` 写。Bridge2 在跑时默认拒绝执行
（`--force` 可跳过），否则两边会互相覆盖目标位。
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time

# 与插件 constants.py 的路径关系：deploy/tienkung_dex/ -> ../../../tienkung_dex/<pkg>/<pkg>/
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONSTANTS = os.path.normpath(
    os.path.join(
        _SCRIPT_DIR,
        "../../../tienkung_dex/lerobot_robot_tienkung_dex/lerobot_robot_tienkung_dex/constants.py",
    )
)

CONTROL_MODE_POSITION = 0   # MotorCtrl.mode：位置模式（与 ros2_deploy_bridge.py 同值）
HAND_CMD_MODE = 1           # 强脑 SetMotorMulti 的 SDK 默认控制模式

# 到位报告的判定阈值（rad）。仿真里抬臂后力臂变长，隐式 PD 稳态误差实测 0.17，
# 所以报警线取 0.25：能抓住"顶到桌面"这类 0.3 rad 级的失败，又不会天天误报。
DEFAULT_TOLERANCE = 0.25


# ── 常量加载 ──────────────────────────────────────────────────────────────────


def load_constants(path: str):
    """按文件路径加载插件的 constants.py（系统 python 装不到 lerobot venv 里的包）。

    constants.py 只 import ``math`` 与 ``enum``，纯 Python 3.10 可执行；不 import
    lerobot，所以不会被 venv 依赖挡住。三处常量副本（插件 / Bridge2 / 仿真）必须
    手工同步，这类脚本一律读**插件那份**，避免再多出第四份。
    """
    if not os.path.isfile(path):
        raise SystemExit(f"[ERROR] 找不到常量文件: {path}")
    spec = importlib.util.spec_from_file_location("tienkung_dex_constants", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"[ERROR] 无法加载常量文件: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def build_stages(C, args) -> list[tuple[str, dict, float]]:
    """四段航点：(标签, 目标关节角, 时长)。

    标签与容差沿用仿真的 ``move_to_ready_pose``。这里不再按容差判定（真机驱动器
    自己闭环），但标签在日志里保持同名，方便和仿真输出对照。
    """
    final = C.ready_final_pose(real=args.real)
    durations = list(C.READY_STAGE_DURATIONS)
    if args.stage_durations:
        parts = [p for p in args.stage_durations.replace(" ", "").split(",") if p]
        if len(parts) != len(durations):
            raise SystemExit(
                f"[ERROR] --stage-durations 需要 {len(durations)} 个数（逗号分隔），"
                f"实际 {len(parts)} 个: {args.stage_durations}"
            )
        durations = [float(p) for p in parts]
    return [
        ("1/4 肩外展抬臂过桌面", dict(C.READY_CLEARANCE_POSE), durations[0]),
        ("2/4 高位折前臂", dict(C.READY_FOREARM_POSE), durations[1]),
        ("3/4 大臂转回近垂直", dict(C.READY_UPPER_VERTICAL_POSE), durations[2]),
        ("4/4 前臂落到预备位", final, durations[3]),
    ]


def _profile(s: float) -> float:
    """quintic 缓动 6s^5-15s^4+10s^3：两端速度、加速度都为 0。

    用 quintic 而不是线性/smoothstep：真机第一段是从静止开始的大幅抬臂，
    端点加速度不连续会引起明显抖动。峰值速度 = 1.875·Δq/T（同仿真）。
    """
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0)


# ── 节点 ──────────────────────────────────────────────────────────────────────


def _status_qos(depth: int = 10):
    """状态订阅统一用 BEST_EFFORT。

    QoS 兼容性是单向的：BEST_EFFORT 订阅能收 RELIABLE 与 BEST_EFFORT 两种发布端，
    而 RELIABLE 订阅只收 RELIABLE 发布端。仿真桥接用默认（RELIABLE）发
    ``/robot_state``，真机侧未必——用 RELIABLE 订阅会出现"话题列表里有、就是收不到数"。
    """
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ReadyMover:
    """把四段航点流式下发到 /head /waist /arm，并把双手摆成预备位手型。"""

    def __init__(self, C, args):
        self.C = C
        self.args = args
        self.variant = args.variant or C.DEFAULT_VARIANT

        import rclpy
        from rclpy.node import Node
        from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorCtrl, RobotState

        self.rclpy = rclpy
        self._ArmCtrl = ArmCtrl
        self._HeadCtrl = HeadCtrl
        self._MotorCtrl = MotorCtrl

        self.node = Node("tienkung_dex_robot_ready")
        self._state_pub_qos = _status_qos()
        self._pos_by_id: dict[int, float] = {}
        self._state_seen = False
        self.node.create_subscription(
            RobotState, C.TOPIC_ROBOT_STATE, self._on_state, self._state_pub_qos
        )

        # 指令发布端用默认 QoS（RELIABLE）：RELIABLE 发布端对 RELIABLE 与
        # BEST_EFFORT 订阅者都兼容，反过来不行。
        self._head_pub = self.node.create_publisher(HeadCtrl, C.TOPIC_HEAD_CMD, 10)
        self._waist_pub = self.node.create_publisher(ArmCtrl, C.TOPIC_WAIST_CMD, 10)
        self._arm_pub = self.node.create_publisher(ArmCtrl, C.TOPIC_ARM_CMD, 10)

        # 手：按变体懒加载厂商消息包，只有 --hands 不是 none 时才需要
        self._hand_pubs = {}
        self._hand_rad: dict[str, dict[str, float]] = {}
        self._hand_sub_ready: dict[str, bool] = {}
        if args.hands != "none":
            self._setup_hands()

    # -- 手部 ----------------------------------------------------------------

    def _setup_hands(self) -> None:
        C, variant = self.C, self.variant
        sides = ["left", "right"] if self.args.hands == "both" else [self.args.hands]
        if variant == "brainco":
            from brainco_hand_msgs.msg import MotorStatus, SetMotorMulti

            cmd_cls, state_cls = SetMotorMulti, MotorStatus
        else:
            from inspire_hand_msgs.msg import GetAngleAct, SetAngle

            cmd_cls, state_cls = SetAngle, GetAngleAct

        topics = {
            "left": ("/left_hand/angle_set", "/left_hand/angle_actual")
            if variant == "inspire"
            else ("/left_hand/set_motor_multi", "/left_hand/motor_status"),
            "right": (
                C.TOPIC_HAND_CMD_BY_VARIANT[variant],
                C.TOPIC_HAND_STATUS_BY_VARIANT[variant],
            ),
        }
        self._hand_sides = sides
        self._hand_cmd_cls = cmd_cls
        self._hand_topics = topics   # side -> (cmd_topic, state_topic)，报告里用来提示话题名
        for side in sides:
            cmd_topic, state_topic = topics[side]
            self._hand_pubs[side] = self.node.create_publisher(cmd_cls, cmd_topic, 10)
            self._hand_rad[side] = {}
            self._hand_sub_ready[side] = False
            self.node.create_subscription(
                state_cls,
                state_topic,
                lambda msg, s=side: self._on_hand_state(s, msg),
                self._state_pub_qos,
            )

    def _on_hand_state(self, side: str, msg) -> None:
        """手反馈 -> {关节: rad}。因时是通道码，强脑是 1~1000 闭合度，都由 constants 换算。"""
        C = self.C
        table = C.hand_channel_joints(self.variant, side)
        codes = list(getattr(msg, "joint_values", getattr(msg, "positions", [])))
        rad: dict[str, float] = {}
        for channel, joints in enumerate(table):
            if channel >= len(codes) or not joints:
                continue
            rad[joints[0]] = C.hand_code_to_rad(
                self.variant, channel, int(codes[channel]), joints[0]
            )
        self._hand_rad[side] = rad
        self._hand_sub_ready[side] = True

    def _publish_hand(self, side: str, targets: dict[str, float]) -> None:
        """{关节: rad} -> 通道码并下发（与 Bridge2 的 `_publish_hand_command` 同算法）。"""
        C, variant = self.C, self.variant
        table = C.hand_channel_joints(variant, side)
        codes: list[int] = []
        for channel, joints in enumerate(table):
            if not joints:
                codes.append(0)
                continue
            codes.append(C.hand_rad_to_code(variant, channel, targets.get(joints[0], 0.0), joints[0]))

        msg = self._hand_cmd_cls()
        if variant == "brainco":
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
        self._hand_pubs[side].publish(msg)

    # -- 状态 ----------------------------------------------------------------

    def _on_state(self, msg) -> None:
        """RobotState -> {关节ID: 位置}。头/腰/腿/臂四组都读，缺的组跳过。"""
        updated = {}
        for group in ("head", "waist", "leg", "arm"):
            status = getattr(msg, group, None)
            for motor in getattr(status, "status", []):
                updated[int(motor.name)] = float(motor.pos)
        self._pos_by_id.update(updated)
        if updated:
            self._state_seen = True

    def _spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def _wait_state(self, timeout: float) -> dict[str, float]:
        """等首帧 /robot_state；拿不到就退回 HOME_POSE（并在日志里说清楚）。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._state_seen:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if not self._state_seen:
            print(
                f"[WARN] {timeout:.0f}s 内没收到 {self.C.TOPIC_ROBOT_STATE}，"
                "按 HOME_POSE 作为起点（只影响插值起点，不影响最终目标）"
            )
            return {}
        return {name: self._pos_by_id[jid] for jid, name in self.C.JOINT_ID_TO_SIM.items()
                if jid in self._pos_by_id}

    # -- 下发 ----------------------------------------------------------------

    def _make_msg(self, group: str, pose: dict[str, float]):
        ids = self.C.GROUP_JOINT_IDS[group]
        limits = self.C.GROUP_CMD_LIMITS[group]
        cls = self._HeadCtrl if group == "head" else self._ArmCtrl
        msg = cls()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = CONTROL_MODE_POSITION
        ctrl = []
        for jid in ids:
            name = self.C.JOINT_ID_TO_SIM[jid]
            motor = self._MotorCtrl()
            motor.name = jid
            motor.pos = float(pose[name])
            motor.spd = float(limits["spd"])   # 不填会走 SDK 默认值，行为不可控
            motor.cur = float(limits["cur"])
            ctrl.append(motor)
        msg.ctrl = ctrl
        return msg

    def publish_pose(self, pose: dict[str, float], groups=("head", "waist", "arm")) -> None:
        """按组下发。**腿永远不在 groups 里**——真机腿由行走控制器独占。"""
        for group in groups:
            if group == "leg":
                print("[ERROR] 拒绝向 /leg/cmd 下发（腿由行走控制器独占）", file=sys.stderr)
                continue
            msg = self._make_msg(group, pose)
            if group == "head":
                self._head_pub.publish(msg)
            elif group == "waist":
                self._waist_pub.publish(msg)
            else:
                self._arm_pub.publish(msg)

    def _stream(self, start: dict, end: dict, names: list[str], duration: float) -> None:
        """把 start -> end 按 quintic 插值，以 --rate 流式下发。"""
        rate = max(1.0, float(self.args.rate))
        steps = max(1, int(round(duration * rate)))
        for i in range(steps + 1):
            s = _profile(i / steps)
            self.publish_pose({n: start[n] + (end[n] - start[n]) * s for n in names})
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(1.0 / rate)


# ── 进程守护 ──────────────────────────────────────────────────────────────────


def find_bridge_pids() -> list[int]:
    """找正在跑的 tienkung_dex Bridge2 进程。

    只看 argv[1]（脚本路径）且要求路径里带 ``tienkung_dex``：tienkung_pro 的桥接
    也叫 ros2_deploy_bridge.py，按文件名匹配会误伤；lerobot-rollout 主进程的命令行
    里也有这个路径，但它出现在 ``--robot.bridge_script=`` 的值里，argv[1] 不是它。
    """
    pids = []
    own = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == own:
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if len(argv) < 2:
            continue
        script = argv[1].decode("utf-8", "replace")
        if os.path.basename(script) == "ros2_deploy_bridge.py" and "tienkung_dex" in script:
            pids.append(int(entry))
    return pids


# ── 报告 ──────────────────────────────────────────────────────────────────────


def report_positions(C, mover: ReadyMover, target: dict[str, float], tolerance: float) -> bool:
    """打印每关节 目标/实际/误差，超过阈值的标出来。返回是否全部在阈值内。"""
    actual = {
        name: mover._pos_by_id[jid]
        for jid, name in C.JOINT_ID_TO_SIM.items()
        if jid in mover._pos_by_id
    }
    if not actual:
        print("[WARN] 没有 /robot_state 数据，跳过到位报告")
        return False

    print("\n[INFO] 预备位到位情况（rad）:")
    print(f"  {'关节':<24}{'ID':>4}{'目标':>10}{'实际':>10}{'误差':>10}")
    worst = 0.0
    bad = []
    for name in sorted(target):
        jid = C.SIM_TO_JOINT_ID.get(name)
        if jid is None or name not in actual:
            continue
        err = actual[name] - target[name]
        worst = max(worst, abs(err))
        flag = "  <- 超差" if abs(err) > tolerance else ""
        if flag:
            bad.append(name)
        print(f"  {name:<24}{jid:>4}{target[name]:>10.3f}{actual[name]:>10.3f}{err:>+10.3f}{flag}")

    for side in getattr(mover, "_hand_sides", []):
        rad = mover._hand_rad.get(side) or {}
        if not rad:
            topic = getattr(mover, "_hand_topics", {}).get(side, ("?", "?"))[1]
            print(f"  [WARN] {side} 手反馈未收到（{topic}），无法核对指位")
            continue
        cmd = C.ready_hand_pose(mover.variant)
        errs = {n: rad[n] - cmd[n] for n in cmd if n in rad}
        if errs:
            print(f"  {side} 手：最大偏差 {max(abs(v) for v in errs.values()):.3f} rad"
                  f"（{len(errs)} 指）")

    print(f"\n[INFO] 最大关节偏差 {worst:.3f} rad（阈值 {tolerance:.2f}）")
    if bad:
        print(f"[WARN] 超差关节: {', '.join(bad)}")
    return not bad


# ── 入口 ──────────────────────────────────────────────────────────────────────


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="TienKung Dex 预备位（四段到位 + 预备位手型）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  robot_ready.py --dry-run          # 只打印四段目标，不发指令\n"
            "  robot_ready.py                    # 仿真（READY_POSE + READY_POSE_TARGET）\n"
            "  robot_ready.py --real             # 真机（READY_POSE）\n"
            "  robot_ready.py --stage-durations 4,4,4,4 --rate 30\n"
        ),
    )
    p.add_argument("--real", action="store_true",
                   help="真机：最终目标用 READY_POSE（不含仿真重力前馈补偿）")
    p.add_argument("--variant", choices=("inspire", "brainco"), default=None,
                   help="手型（默认取 constants.DEFAULT_VARIANT）")
    p.add_argument("--hands", choices=("both", "left", "right", "none"), default="both",
                   help="摆哪只手的预备位手型（默认 both）")
    p.add_argument("--stage-durations", default=None,
                   help="四段时长（秒，逗号分隔），默认 constants.READY_STAGE_DURATIONS")
    p.add_argument("--rate", type=float, default=50.0, help="指令下发频率 Hz（默认 50）")
    p.add_argument("--state-timeout", type=float, default=5.0,
                   help="等首帧 /robot_state 的超时秒数（默认 5）")
    p.add_argument("--settle", type=float, default=3.0,
                   help="最后一段结束后等待稳定再报告（默认 3s）")
    p.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                   help=f"到位报告的超差阈值 rad（默认 {DEFAULT_TOLERANCE}）")
    p.add_argument("--force", action="store_true",
                   help="Bridge2 正在运行时也继续（默认拒绝，避免两边抢 /arm/cmd）")
    p.add_argument("--dry-run", action="store_true", help="只打印计划，不连 ROS、不发指令")
    p.add_argument("--constants", default=DEFAULT_CONSTANTS,
                   help=f"插件 constants.py 路径（默认 {DEFAULT_CONSTANTS}）")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    C = load_constants(args.constants)
    variant = args.variant or C.DEFAULT_VARIANT
    stages = build_stages(C, args)
    final = stages[-1][1]
    names = sorted({n for _, pose, _ in stages for n in pose})

    print(f"[INFO] 常量: {args.constants}")
    print(f"[INFO] 模式: {'真机' if args.real else '仿真'}  手型: {variant}  "
          f"下发频率: {args.rate:.0f} Hz")
    print(f"[INFO] 最终目标来源: {'READY_POSE' if args.real else 'READY_POSE ∪ READY_POSE_TARGET'}")
    print(f"[INFO] 控制关节 {len(names)} 个（head {len(C.GROUP_JOINT_IDS['head'])} / "
          f"waist {len(C.GROUP_JOINT_IDS['waist'])} / arm {len(C.GROUP_JOINT_IDS['arm'])}），"
          f"腿不下发")

    if args.dry_run:
        for label, pose, duration in stages:
            print(f"\n[PLAN] {label}（{duration:.1f}s）")
            for name in sorted(pose):
                print(f"         {name:<24} -> {pose[name]:+.4f}")
        print(f"\n[PLAN] 收尾 {C.READY_TAIL_DURATION:.1f}s：头部保持 + {args.hands} 手预备位手型")
        return 0

    bridge_pids = find_bridge_pids()
    if bridge_pids and not args.force:
        print(
            f"[ERROR] Bridge2 正在运行（PID {bridge_pids}），两侧都会写 /arm/cmd。\n"
            "        先停它（或让 rollout/inference_server 不要在此时启动），确认后再跑；"
            "确要继续加 --force。",
            file=sys.stderr,
        )
        return 2

    import rclpy

    rclpy.init()
    mover = ReadyMover(C, args)
    try:
        # 起点：实测位置（缺省按 HOME_POSE）。用实测而不是常量，避免"当前在哪"与
        # 假设不符时整条轨迹从错误的位置起算。
        current = mover._wait_state(args.state_timeout)
        start = {n: float(current.get(n, C.HOME_POSE.get(n, 0.0))) for n in names}

        prev = dict(start)
        for label, pose, duration in stages:
            end = {n: float(pose.get(n, prev[n])) for n in names}
            print(f"[INFO] {label}（{duration:.1f}s）")
            mover._stream(prev, end, names, duration)
            prev = end

        # 收尾：手臂保持最终姿态（按 --rate/2 重发，真机侧保持 setpoint 最新），
        # 同时把双手摆成预备位手型。手的起点用实测，读不到就从目标直接发一次。
        print(f"[INFO] 收尾 {C.READY_TAIL_DURATION:.1f}s: 保持姿态 + 手型")
        hand_target = C.ready_hand_pose(variant)
        mover._spin(min(args.state_timeout, C.READY_TAIL_DURATION))
        # 手型起点：有反馈就用反馈，没有就从"全伸直"起（预备位手型与全伸直只差拇指
        # 侧摆，从 0 起是平滑的；直接从目标起会在第一帧瞬移）。
        open_pose = C.hand_open_pose(variant)
        hand_start = {}
        for side in getattr(mover, "_hand_sides", []):
            rad = mover._hand_rad.get(side) or {}
            if not rad:
                print(f"[WARN] {side} 手反馈未收到，手型从全伸直开始插值")
            hand_start[side] = {n: float(rad.get(n, open_pose.get(n, 0.0))) for n in hand_target}

        rate = max(1.0, float(args.rate))
        steps = max(1, int(round(C.READY_TAIL_DURATION * rate)))
        for i in range(steps + 1):
            s = _profile(i / steps)
            mover.publish_pose(prev)
            for side in hand_start:
                mover._publish_hand(
                    side,
                    {n: hand_start[side][n] + (hand_target[n] - hand_start[side][n]) * s
                     for n in hand_target},
                )
            mover.rclpy.spin_once(mover.node, timeout_sec=0.0)
            time.sleep(1.0 / rate)

        mover._spin(args.settle)
        ok = report_positions(C, mover, prev, args.tolerance)
        return 0 if ok else 1
    finally:
        mover.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
