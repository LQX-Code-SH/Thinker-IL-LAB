#!/usr/bin/env python3
"""机器人从预备位回到站立位（home），按预备位的四段航点**倒序**走。

为什么倒序而不是直接插值到 HOME_POSE：
  预备位手臂前伸在桌面上方，直接落回站立位会让前臂/手从桌沿扫下去。
  倒序 = 先高位收回前臂 → 抬到侧上方 → 再自然下垂，全程避开桌面。

段序（与 robot_ready 的镜像）：
  当前姿态 → 3/4 大臂回垂直 → 2/4 高位折前臂 → 1/4 肩外展 → HOME（站立位）

不含腿（腿由行走控制器独占）；头部与腰部在最后一段回到 HOME 值。
每段默认 8 s，50 Hz 流式下发，quintic 缓动（端点速度为 0）。

用法：
    /usr/bin/python3 robot_home.py --durations 8,8,8,8
    /usr/bin/python3 robot_home.py --dry-run
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time

DEFAULT_CONSTANTS = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "../../../../tienkung_dex/lerobot_robot_tienkung_dex/lerobot_robot_tienkung_dex/constants.py",
    )
)


def load_constants(path: str):
    spec = importlib.util.spec_from_file_location("dex_constants", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def profile(s: float) -> float:
    """quintic 6s^5-15s^4+10s^3：两端速度与加速度为 0。"""
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0)


def find_bridge_pids() -> list[int]:
    """找正在跑的 tienkung_dex Bridge2（判据与 robot_ready.py 一致，避免互相 import）。"""
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


def build_waypoints(C) -> list[tuple[str, dict]]:
    """倒序航点。头部/腰部不在前三段里，靠 carry-forward 保持，最后一段回 HOME。"""
    # deploy_home_pose() = HOME_UPPER_POSE + DEPLOY_HOME_OFFSETS（真机微偏移，
    # 避开 wrist_roll_l 在 0 点附近的 ~40Hz 摩擦极限环；仿真常量不受影响）
    home = dict(C.deploy_home_pose())
    return [
        ("3/4 大臂回垂直位", dict(C.READY_UPPER_VERTICAL_POSE)),
        ("2/4 高位折前臂", dict(C.READY_FOREARM_POSE)),
        ("1/4 肩外展（手在体侧外上方）", dict(C.READY_CLEARANCE_POSE)),
        ("HOME 站立位", home),
    ]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--durations", default="8,8,8,8", help="四段时长（秒，逗号分隔）")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--state-timeout", type=float, default=5.0)
    p.add_argument("--settle", type=float, default=3.0)
    p.add_argument("--tolerance", type=float, default=0.15)
    p.add_argument("--constants", default=DEFAULT_CONSTANTS)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--force", action="store_true", help="Bridge2 在跑时仍继续（不推荐）")
    args = p.parse_args()

    C = load_constants(args.constants)
    waypoints = build_waypoints(C)

    durations = [float(x) for x in args.durations.replace(" ", "").split(",") if x]
    if len(durations) != len(waypoints):
        raise SystemExit(f"[home] ERROR: --durations 需要 {len(waypoints)} 个数")

    if args.dry_run:
        for (label, pose), dur in zip(waypoints, durations):
            print(f"\n[PLAN] {label}（{dur:.1f}s）")
            for name in sorted(pose):
                print(f"         {name:<24} -> {pose[name]:+.4f}")
        return 0

    # Bridge2 与 robot_ready 都写 /arm/cmd，不能同时跑
    pids = find_bridge_pids()
    if pids and not args.force:
        print(f"[home] ERROR: Bridge2 正在运行（PID {pids}），会与 /arm/cmd 冲突。", file=sys.stderr)
        return 2

    import rclpy
    from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorCtrl, RobotState

    rclpy.init()
    node = rclpy.create_node("dex_robot_home")

    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)

    pos_by_id: dict[int, float] = {}
    state_seen = {"v": False}

    def on_state(msg):
        for grp in ("head", "waist", "arm"):
            for m in getattr(getattr(msg, grp, None), "status", []):
                pos_by_id[int(m.name)] = float(m.pos)
        state_seen["v"] = True

    node.create_subscription(RobotState, C.TOPIC_ROBOT_STATE, on_state, qos)
    pub_head = node.create_publisher(HeadCtrl, C.TOPIC_HEAD_CMD, 10)
    pub_waist = node.create_publisher(ArmCtrl, C.TOPIC_WAIST_CMD, 10)
    pub_arm = node.create_publisher(ArmCtrl, C.TOPIC_ARM_CMD, 10)

    deadline = time.monotonic() + args.state_timeout
    while time.monotonic() < deadline and not state_seen["v"]:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not state_seen["v"]:
        print("[home] ERROR: 收不到 /robot_state，拒绝按 HOME_POSE 起步（可能当前不在站立位）",
              file=sys.stderr)
        rclpy.shutdown()
        return 1

    names = sorted({n for _, pose in waypoints for n in pose})
    missing = [n for n in names if C.SIM_TO_JOINT_ID.get(n) not in pos_by_id]
    if missing:
        print(f"[home] ERROR: /robot_state 缺少关节 {missing}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    def publish_pose(pose: dict[str, float]) -> None:
        for group in ("head", "waist", "arm"):
            ids = C.GROUP_JOINT_IDS[group]
            msg = (HeadCtrl if group == "head" else ArmCtrl)()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.mode = 0
            ctrl = []
            for jid in ids:
                name = C.JOINT_ID_TO_SIM[jid]
                m = MotorCtrl()
                m.name = jid
                m.pos = float(pose[name])
                m.spd = float(C.GROUP_CMD_LIMITS[group]["spd"])
                m.cur = float(C.GROUP_CMD_LIMITS[group]["cur"])
                ctrl.append(m)
            msg.ctrl = ctrl
            (pub_head if group == "head" else pub_waist if group == "waist" else pub_arm).publish(msg)

    current = {n: pos_by_id[C.SIM_TO_JOINT_ID[n]] for n in names}
    print("[home] 当前姿态已读取，开始倒序回位（腿不动）")
    prev = dict(current)
    for (label, pose), dur in zip(waypoints, durations):
        end = {n: float(pose.get(n, prev[n])) for n in names}
        steps = max(1, int(round(dur * args.rate)))
        print(f"[home] {label}（{dur:.1f}s）")
        for i in range(steps + 1):
            s = profile(i / steps)
            publish_pose({n: prev[n] + (end[n] - prev[n]) * s for n in names})
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / args.rate)
        prev = end

    # settle：继续以 10 Hz 重发目标，让驱动器稳住
    end_t = time.monotonic() + args.settle
    while time.monotonic() < end_t:
        publish_pose(prev)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)

    actual = {n: pos_by_id[C.SIM_TO_JOINT_ID[n]] for n in names}
    worst_name = max(names, key=lambda n: abs(actual[n] - prev[n]))
    worst = abs(actual[worst_name] - prev[worst_name])
    print(f"[home] 最大关节偏差 {worst:.3f} rad（{worst_name}，阈值 {args.tolerance:.2f}）")
    for n in names:
        print(f"  {n:<24}{prev[n]:>10.3f}{actual[n]:>10.3f}{actual[n] - prev[n]:>+10.3f}")
    ok = worst <= args.tolerance
    print("[home] => " + ("已回到站立位" if ok else "存在超差关节，请检查"))
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
