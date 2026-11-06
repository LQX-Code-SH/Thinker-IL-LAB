#!/usr/bin/env python3
"""Move Walker C1 arms/hands to the task-ready pose via ROS2 (system Python 3.10).

Runs with the system interpreter (rclpy lives there, not in the LeRobot venv), so
joint names/poses are inlined instead of importing the lerobot plugin package.

Sequence (avoids dragging the hands across the table):
  1. lift shoulders   (shoulder_roll -> ±1.5)
  2. fold elbows      (elbow_pitch  -> -1.5)
  3. go to task pose  (TASK_POSE, hands open at 0 rad)

Only the 14 arm joints and the 12 hand channels are commanded; head/waist/legs are
left untouched (the robot keeps its current pose).

Usage:
  python3 robot_ready.py                 # 摆到预备位
  python3 robot_ready.py --dry-run       # 只打印将要发布的命令
  python3 robot_ready.py --hz 50         # 插值频率
"""

from __future__ import annotations

import argparse
import time

LEFT_ARM = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint", "L_elbow_yaw_joint", "L_wrist_pitch_joint", "L_wrist_roll_joint",
]
RIGHT_ARM = [
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint", "R_wrist_roll_joint",
]
ARM_JOINTS = LEFT_ARM + RIGHT_ARM

LEFT_HAND = [
    "left_thumb_swing", "left_thumb_mcp", "left_index_mcp",
    "left_middle_mcp", "left_ring_mcp", "left_little_mcp",
]
RIGHT_HAND = [
    "right_thumb_swing", "right_thumb_mcp", "right_index_mcp",
    "right_middle_mcp", "right_ring_mcp", "right_little_mcp",
]

# 桌面抓放预备位（与 ubt_sim TASK_RESET_BODY_POSE / 插件 READY_POSE 一致）
TASK_POSE = {
    "L_shoulder_pitch_joint": -0.291, "L_shoulder_roll_joint": 0.30,
    "L_shoulder_yaw_joint": 0.136, "L_elbow_pitch_joint": -1.155,
    "L_elbow_yaw_joint": 0.124, "L_wrist_pitch_joint": -0.361,
    "L_wrist_roll_joint": -0.194,
    "R_shoulder_pitch_joint": -0.291, "R_shoulder_roll_joint": -0.30,
    "R_shoulder_yaw_joint": -0.136, "R_elbow_pitch_joint": -1.155,
    "R_elbow_yaw_joint": -0.124, "R_wrist_pitch_joint": -0.361,
    "R_wrist_roll_joint": 0.194,
}
LIFT_POSE = {"L_shoulder_roll_joint": 1.5, "R_shoulder_roll_joint": -1.5}
ELBOW_POSE = {"L_elbow_pitch_joint": -1.5, "R_elbow_pitch_joint": -1.5}

# 头部限位（来自 ubt_sim/assets/robots/walker_c1/urdf/walker_c1.urdf）
#   head_pitch_joint: axis +Y，lower=-0.523 upper=+0.785 → 正值 = 低头
#   head_yaw_joint  : axis +Z，±1.658
# 插件 BODY_JOINT_LIMITS 只覆盖 14 个臂关节，头部限位在此单点定义并在下发前裁剪。
HEAD_LIMITS = {
    "head_pitch_joint": (-0.523, 0.785),
    "head_yaw_joint": (-1.658, 1.658),
}
# 预备位的低头角：真机实测 0.50 rad 只能看到桌面下缘，0.65 rad（37.2°）才能看到完整桌面。
# 注意：仿真/插件 READY_POSE 仍是 0.50（28.6°），真机采集/训练建议统一按 0.65 记录，
# 否则训练视角与部署视角相差约 8.6°。
HEAD_PITCH_READY = 0.65

# home 位（本文件的 home 定义）：14 个臂关节全 0，手张开。手臂竖直贴身。
ZERO_POSE = {name: 0.0 for name in ARM_JOINTS}
# 中间节点（全量 14 关节，未列出的补 0），保证跑完后所有关节都精确落到目标
LIFT_FULL = {**ZERO_POSE, **LIFT_POSE}
ELBOW_FULL = {**ZERO_POSE, **LIFT_POSE, **ELBOW_POSE}

HAND_MODE = 5
BODY_CMD_TOPIC = "/mc/sdk/robot_command"
BODY_STATE_TOPIC = "/mc/sdk/robot_state"
# 兜底状态源：主控 50 Hz 全关节 JointState。机器人重启/未重新切 SDK 时
# /mc/sdk/robot_state 会静默（publisher 存在但不发），此时用它算插值起点。
JOINT_STATE_TOPIC = "/mc/joint_states"
LEFT_HAND_CMD_TOPIC = "/mc/left_hand/command"
RIGHT_HAND_CMD_TOPIC = "/mc/right_hand/command"


def _quintic(t: float) -> float:
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def _interp(start: dict[str, float], target: dict[str, float], ratio: float) -> dict[str, float]:
    return {
        name: start.get(name, target[name]) * (1.0 - ratio) + target[name] * ratio
        for name in target
    }


def _clamp_head(pitch: float, yaw: float = 0.0) -> dict[str, float]:
    """头部目标裁剪到 URDF 限位，避免下发越界值。"""
    plo, phi = HEAD_LIMITS["head_pitch_joint"]
    ylo, yhi = HEAD_LIMITS["head_yaw_joint"]
    return {
        "head_yaw_joint": max(ylo, min(yhi, float(yaw))),
        "head_pitch_joint": max(plo, min(phi, float(pitch))),
    }


def _build_plan(
    pose: str, head: dict[str, float] | None = None
) -> list[tuple[dict[str, float], float, str]]:
    """返回 [(目标位, 时长比例, 段名)]。

    pose="ready"：zero → ready，三段「抬肩 → 收肘 → 预备位」（0.6/0.6/1.0）；
    pose="zero" ：ready → home(全 0)，严格是上面三段的反向（1.0/0.6/0.6）。
    """
    ready_target = dict(TASK_POSE)
    if head:
        # 头部（低头看桌面）放在最后一段，与手臂一起匀速插值
        ready_target.update(head)
    if pose == "ready":
        return [
            (LIFT_FULL, 0.6, "1/3 lift shoulders"),
            (ELBOW_FULL, 0.6, "2/3 fold elbows"),
            (ready_target, 1.0, "3/3 task-ready pose"),
        ]
    # ready → home(zero)：与 ready 路径互逆——先腕/肩俯仰回零并保持抬肩收肘，
    # 再展开肘，最后放下肩，全程不把手臂扫过桌面。
    home_target = dict(ZERO_POSE)
    if head:
        # 回 home 时头部同步回正（head 传的是零位）
        home_target.update(head)
    return [
        (ELBOW_FULL, 1.0, "1/3 ready -> 抬肩收肘位（其余回零）"),
        (LIFT_FULL, 0.6, "2/3 unfold elbows"),
        (home_target, 0.6, "3/3 lower shoulders -> home(零位)"),
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="插值发布频率（默认 100 Hz）。实测 300 Hz 反而更抖（发布抖动占比变大），勿盲目调高",
    )
    parser.add_argument(
        "--velocity-ff",
        action="store_true",
        help="同时下发解析速度前馈（五次多项式导数）。实测反而更抖"
        "（hf_rms 0.0129→0.0184），默认关闭，仅在 SDK 支持该字段时再评估",
    )
    parser.add_argument(
        "--pose",
        choices=["ready", "zero"],
        default="ready",
        help="目标位：ready=桌面预备位（TASK_POSE）；zero=home 位（14 臂关节全 0）",
    )
    parser.add_argument("--segment-s", type=float, default=1.0, help="Duration per segment (seconds)")
    parser.add_argument(
        "--min-segment-s",
        type=float,
        default=5.0,
        help="每段最短时长（秒），默认 5.0：真机上有效时长 = max(比例时长, 本值)，防止摆位冲得太快",
    )
    parser.add_argument(
        "--side",
        choices=["both", "left", "right"],
        default="both",
        help="只动某一侧手臂（默认 both）。未选中的一侧仍会出现在每条命令里，"
        "但目标值锁在其当前实测位以保持不动（消息始终是 14 关节，避免 SDK 丢弃子集指令）",
    )
    parser.add_argument("--hold-s", type=float, default=0.5, help="Hold the final pose (seconds)")
    parser.add_argument(
        "--head-pitch",
        type=float,
        default=HEAD_PITCH_READY,
        help=f"ready 位低头角（rad，正值低头，默认 {HEAD_PITCH_READY}=37.2°，"
        f"限位 {HEAD_LIMITS['head_pitch_joint']} 内自动裁剪）",
    )
    parser.add_argument(
        "--no-head",
        action="store_true",
        help="不动头部（只摆臂+手）；home 时也保持头部当前位",
    )
    parser.add_argument(
        "--cmd-log",
        default="",
        help="可选：把每帧下发的目标写 CSV（t,arm...），配合状态 trace 计算跟踪误差",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print commands instead of publishing")
    args = parser.parse_args(argv)

    # 头部目标：ready=低头 HEAD_PITCH_READY；zero(home)=回正 0；--no-head 则整段不动头
    if args.no_head:
        head_target = None
    elif args.pose == "ready":
        head_target = _clamp_head(args.head_pitch, 0.0)
    else:
        head_target = _clamp_head(0.0, 0.0)

    plan = _build_plan(args.pose, head_target)

    def segment_duration(ratio: float) -> float:
        """比例时长与本段最短时长取大，避免高比例段被压得过快。"""
        return max(args.segment_s * ratio, args.min_segment_s)

    if args.dry_run:
        print(
            f"[robot_ready] dry-run: pose = {args.pose}, side = {args.side}"
            + ("" if args.side == "both" else
               f"（只动{'右' if args.side == 'right' else '左'}臂，另一侧锁在当前位）")
        )
        print("[robot_ready] dry-run: ARM_JOINTS =", ARM_JOINTS)
        print("[robot_ready] dry-run: 目标位 =", plan[-1][0])
        print(
            "[robot_ready] dry-run: 头部 =",
            head_target if head_target else "保持当前（--no-head）",
        )
        print("[robot_ready] dry-run: hands ->", [0.0] * 6)
        for _, ratio, label in plan:
            print(f"[robot_ready] dry-run: {label} = {segment_duration(ratio):.1f}s")
        print(
            f"[robot_ready] dry-run: 合计 {sum(segment_duration(r) for _, r, _ in plan):.1f}s + "
            f"hold {args.hold_s:.1f}s (segment_s={args.segment_s}, min_segment_s={args.min_segment_s})"
        )
        return 0

    import rclpy
    from mc_state_msgs.msg import RobotState
    from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    qos_sub = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )
    # depth 10：真机实测 depth 1 会让 300 Hz 流随机丢帧、阶梯更不均匀，反而更抖。
    # 机器人侧订阅是 BEST_EFFORT + KEEP_LAST(5)，RELIABLE 发布与之兼容。
    qos_pub = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )

    rclpy.init()
    node = Node("walker_c1_robot_ready")
    body_pub = node.create_publisher(RobotCommand, BODY_CMD_TOPIC, qos_pub)
    left_hand_pub = node.create_publisher(JointCommand, LEFT_HAND_CMD_TOPIC, qos_pub)
    right_hand_pub = node.create_publisher(JointCommand, RIGHT_HAND_CMD_TOPIC, qos_pub)

    state = {"pos": {}}

    def on_state(msg) -> None:
        js: JointState = msg.joint_states
        state["pos"] = dict(zip(js.name, js.position))

    def on_joint_state(msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            state["pos"].setdefault(name, float(pos))

    node.create_subscription(RobotState, BODY_STATE_TOPIC, on_state, qos_sub)
    node.create_subscription(JointState, JOINT_STATE_TOPIC, on_joint_state, qos_sub)
    node.create_subscription(JointState, LEFT_HAND_CMD_TOPIC.replace("/command", "/joint_states"), on_joint_state, qos_sub)
    node.create_subscription(JointState, RIGHT_HAND_CMD_TOPIC.replace("/command", "/joint_states"), on_joint_state, qos_sub)

    print("[robot_ready] waiting for robot state ...")
    deadline = time.time() + 10.0
    # 手部 joint_states 也会写 state["pos"]，只等「非空」可能先等到手部 6 通道，
    # 导致臂关节缺失；这里等到 14 个臂关节齐全再动。
    while time.time() < deadline and any(n not in state["pos"] for n in ARM_JOINTS):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not state["pos"]:
        print("[robot_ready] 警告：未收到 /mc/sdk/robot_state，将直接从当前位置插值（可能不准确）")
    else:
        missing = [n for n in ARM_JOINTS if n not in state["pos"]]
        if missing:
            print(f"[robot_ready] 警告：状态缺少关节 {missing}（将用目标值做起点，该关节不动）")

    def publish_body(values: dict[str, float], vels: dict[str, float] | None = None,
                     stamp_ns: int | None = None) -> None:
        msg = RobotCommand()
        if stamp_ns is None:
            msg.header.stamp = node.get_clock().now().to_msg()
        else:
            msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        for name, value in values.items():
            cmd = JointCmd()
            cmd.name = name
            cmd.control_mode = JointCmd.MODE_POSITION
            cmd.position = float(value)
            if vels is not None and name in vels:
                # 速度前馈：五次多项式解析导数，SDK 若支持则显著平滑位置环
                cmd.velocity = float(vels[name])
            msg.joint_cmd.append(cmd)
        body_pub.publish(msg)

    def publish_hands() -> None:
        for pub, names in ((left_hand_pub, LEFT_HAND), (right_hand_pub, RIGHT_HAND)):
            msg = JointCommand()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.names = list(names)
            msg.position = [0.0] * len(names)
            msg.mode = [HAND_MODE] * len(names)
            pub.publish(msg)

    def run_segment(target: dict[str, float], duration: float, label: str) -> None:
        duration = max(float(duration), float(args.min_segment_s))
        # 状态缺失的关节用「目标值」当起点：该关节不动，避免从错误的起点甩过去
        start = {n: float(state["pos"].get(n, target[n])) for n in target}
        # --side：未选中的一侧锁在当前实测位（仍下发，保证 14 关节齐全）
        tgt = dict(target)
        if args.side != "both":
            freeze_prefix = "L_" if args.side == "right" else "R_"
            for n in tgt:
                if n.startswith(freeze_prefix):
                    tgt[n] = start[n]
        # 五次多项式峰值速度 = 平均速度 × 1.875，用于把实际角速度打在日志里便于判断快慢
        travel = max(abs(tgt[n] - start[n]) for n in tgt)
        peak = 1.875 * travel / duration if duration > 0 else 0.0
        steps = max(int(duration * args.hz), 1)
        step_dt = duration / steps
        print(
            f"[robot_ready] {label} ({duration:.1f}s, 最大位移 {travel:.3f} rad, "
            f"峰值角速度 ≈{peak:.2f} rad/s)"
        )
        t_begin = time.monotonic()
        base_ns = node.get_clock().now().nanoseconds
        for i in range(1, steps + 1):
            tn = i / steps
            values = _interp(start, tgt, _quintic(tn))
            vels = None
            if args.velocity_ff:
                # d/dt[10t^3-15t^4+6t^5] = 30 t^2 (1-t)^2 / T
                rate = 30.0 * tn * tn * (1.0 - tn) * (1.0 - tn) / duration
                vels = {n: (tgt[n] - start[n]) * rate for n in tgt}
            publish_body(values, vels, stamp_ns=base_ns + int(i * step_dt * 1e9))
            publish_hands()
            if cmd_log is not None:
                cmd_log.write(
                    f"{time.monotonic():.6f},"
                    + ",".join(f"{values.get(n, float('nan')):.6f}" for n in ARM_JOINTS)
                    + "\n"
                )
            # 节拍必须按「绝对时间」排程：不能把 spin_once(timeout_sec=1/hz) 当 sleep 用。
            # /mc/sdk/robot_state 约 480 Hz，spin_once 一收到消息就立刻返回，
            # 循环会被压到 ~470 Hz，设定 5 s 的段实际只跑 ~1 s（真机实测 5 倍偏快）。
            deadline = t_begin + i * step_dt
            while True:
                remain = deadline - time.monotonic()
                if remain <= 0.0:
                    break
                rclpy.spin_once(node, timeout_sec=min(remain, 0.002))
        elapsed = time.monotonic() - t_begin
        state["pos"].update(tgt)
        print(f"[robot_ready] {label} 实际耗时 {elapsed:.2f}s")
        if elapsed > duration * 1.1:
            print(
                f"[robot_ready] 警告：{label} 实际 {elapsed:.2f}s 明显超过设定 {duration:.2f}s"
                "（发布/CLI 阻塞？）"
            )
        return tgt

    final_target = plan[-1][0]
    cmd_log = None
    if args.cmd_log:
        cmd_log = open(args.cmd_log, "w", encoding="utf-8")
        cmd_log.write("t," + ",".join(ARM_JOINTS) + "\n")

    try:
        for target, ratio, label in plan:
            eff = run_segment(target, segment_duration(ratio), label)
            if eff is not None:
                final_target = eff

        print(f"[robot_ready] 保持 {args.hold_s:.1f}s ...")
        end = time.time() + args.hold_s
        while time.time() < end:
            publish_body(final_target)
            publish_hands()
            rclpy.spin_once(node, timeout_sec=0.02)
        print("[robot_ready] done.")
    finally:
        if cmd_log is not None:
            cmd_log.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
