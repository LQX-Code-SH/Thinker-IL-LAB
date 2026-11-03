#!/usr/bin/env python3
"""真机控制通路「零位移」验收：把当前实测角度原样下发回 /arm/cmd。

在不动机器人的前提下验证：
  1) 消息类型 / 关节 ID / QoS 与真机对得上；
  2) 我们发的 /arm/cmd 不引起跳变（最大偏差应 < 0.02 rad）；
  3) spd/cur 配错时偏差会立刻变大。

它**不验证**"机器人是否处于话题控制模式"——那要真的下发一个有位移的目标
（robot_ready_host.sh --real）。本脚本只保证"这条路是安全的"。

用法（系统 python，需已 source ROS；直接跑 .sh 包装更省事）：
    /usr/bin/python3 hold_test.py --duration 3 --groups arm
"""
from __future__ import annotations

import argparse
import sys
import time

ARM_IDS = [11, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26, 27]
GROUP_IDS = {"arm": ARM_IDS, "waist": [31, 32, 33], "head": [1, 2]}
TOPIC_STATE = "/robot_state"
TOPIC_CMD = {"arm": "/arm/cmd", "waist": "/waist/cmd", "head": "/head/cmd"}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--spd", type=float, default=0.2)     # 低速：即使有跳变也温和
    p.add_argument("--cur", type=float, default=10.0)    # 低电流限幅
    p.add_argument("--groups", default="arm")
    p.add_argument("--tolerance", type=float, default=0.02)
    args = p.parse_args()

    groups = [g.strip() for g in args.groups.split(",") if g.strip()]
    for g in groups:
        if g not in GROUP_IDS:
            print(f"[hold] ERROR: 未知组 {g!r}（可选 {list(GROUP_IDS)}）", file=sys.stderr)
            return 2

    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorCtrl, RobotState

    rclpy.init()
    node = rclpy.create_node("dex_hold_test")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    pos: dict[int, float] = {}

    def on_state(msg) -> None:
        for grp in ("head", "waist", "arm"):
            for m in getattr(getattr(msg, grp, None), "status", []):
                pos[int(m.name)] = float(m.pos)

    node.create_subscription(RobotState, TOPIC_STATE, on_state, qos)
    pubs = {g: node.create_publisher(HeadCtrl if g == "head" else ArmCtrl, TOPIC_CMD[g], 10)
            for g in groups}

    # 等首帧
    t0 = time.monotonic()
    while not pos and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not pos:
        print(f"[hold] ERROR: 5s 内没收到 {TOPIC_STATE}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    missing = [i for g in groups for i in GROUP_IDS[g] if i not in pos]
    if missing:
        print(f"[hold] ERROR: {TOPIC_STATE} 缺关节 ID {missing}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    baseline = {i: pos[i] for g in groups for i in GROUP_IDS[g]}
    print("[hold] 基线（当前实测，rad）:")
    for g in groups:
        print(f"  {g:6s} " + " ".join(f"{i}:{baseline[i]:+.3f}" for i in GROUP_IDS[g]))

    period = 1.0 / max(1.0, args.rate)
    worst = 0.0
    worst_id = -1
    steps = int(args.duration * args.rate)
    for _ in range(max(1, steps)):
        for g in groups:
            msg = (HeadCtrl if g == "head" else ArmCtrl)()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.mode = 0  # 位置模式
            ctrl = []
            for jid in GROUP_IDS[g]:
                m = MotorCtrl()
                m.name = jid
                m.pos = baseline[jid]   # 目标 = 当前实测，零位移
                m.spd = args.spd
                m.cur = args.cur
                ctrl.append(m)
            msg.ctrl = ctrl
            pubs[g].publish(msg)

        rclpy.spin_once(node, timeout_sec=0.0)
        for jid, base in baseline.items():
            dev = abs(pos.get(jid, base) - base)
            if dev > worst:
                worst, worst_id = dev, jid
        time.sleep(period)

    rclpy.spin_once(node, timeout_sec=0.05)
    print(f"[hold] {args.duration:.1f}s 内最大偏差 {worst:.4f} rad (id {worst_id})")
    ok = worst <= args.tolerance
    print(f"[hold] => {'零位移通过' if ok else '出现跳变，检查 spd/cur 或是否被其它控制源接管'}")
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
