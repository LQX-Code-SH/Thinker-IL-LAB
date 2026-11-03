#!/usr/bin/env python3
"""真机分步微小运动测试：头 / 手 / 臂 各做一次"小幅偏出再回位"。

跑完整备位（robot_ready --real）之前，用它逐组确认：
  指令话题有人消费、方向对、幅度可控、回位准确。

三条独立步骤（默认 20 Hz、位置模式、spd=0.2、cur=10）：
  head : head_pitch   ±0.05 rad (≈2.9°)
  hand : 右手食指闭合度 ±80 码 (8%)，通道 2 = index_proximal
  arm  : 右腕 pitch   ±0.05 rad (≈2.9°)，关节 ID 26

每步都是 基线 → 偏出 → 保持 → 回位，并从反馈核对实测幅度。
不写 /leg/cmd，不发腰。

用法：
    /usr/bin/python3 staged_test.py --step head
    /usr/bin/python3 staged_test.py --step hand --delta 60
    /usr/bin/python3 staged_test.py --step arm  --delta 0.03
"""
from __future__ import annotations

import argparse
import sys
import time

STATE_TOPIC = "/robot_state"
HAND_STATE_TOPIC = "/right_hand/motor_status"
ARM_CMD_TOPIC = "/arm/cmd"
HEAD_CMD_TOPIC = "/head/cmd"
HAND_CMD_TOPIC = "/right_hand/set_motor_multi"

HEAD_IDS = [1, 2]           # 1=yaw 2=pitch
ARM_ID = 26                 # wrist_pitch_r
ARM_IDS_CMD = [21, 22, 23, 24, 25, 26, 27]
HAND_CH = 2                 # index_proximal（0=thumb_flex 1=thumb_rot 2=index）
HAND_N = 6
HAND_CODE_MIN, HAND_CODE_MAX = 1, 1000


def smooth(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class Node:
    def __init__(self, rclpy, msgs):
        self.rclpy = rclpy
        self.msgs = msgs
        self.head = {i: 0.0 for i in HEAD_IDS}
        self.arm = {i: 0.0 for i in ARM_IDS_CMD}
        self.hand = [0] * HAND_N
        self.seen_state = False
        self.seen_hand = False
        self.node = rclpy.create_node("dex_staged_test")
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.node.create_subscription(msgs["RobotState"], STATE_TOPIC, self._on_state, qos)
        self.node.create_subscription(msgs["MotorStatus"], HAND_STATE_TOPIC, self._on_hand, qos)
        self.pub_arm = self.node.create_publisher(msgs["ArmCtrl"], ARM_CMD_TOPIC, 10)
        self.pub_head = self.node.create_publisher(msgs["HeadCtrl"], HEAD_CMD_TOPIC, 10)
        self.pub_hand = self.node.create_publisher(msgs["SetMotorMulti"], HAND_CMD_TOPIC, 10)

    def _on_state(self, msg):
        for grp, ids in (("head", HEAD_IDS), ("arm", ARM_IDS_CMD)):
            for m in getattr(getattr(msg, grp, None), "status", []):
                jid = int(m.name)
                if jid in ids:
                    (self.head if grp == "head" else self.arm)[jid] = float(m.pos)
        self.seen_state = True

    def _on_hand(self, msg):
        self.hand = [int(v) for v in msg.positions[:HAND_N]]
        self.seen_hand = True

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def publish_group(self, group: str, pose: dict, spd: float, cur: float) -> None:
        msg = (self.msgs["HeadCtrl"] if group == "head" else self.msgs["ArmCtrl"])()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = 0
        ctrl = []
        ids = HEAD_IDS if group == "head" else ARM_IDS_CMD
        for jid in ids:
            m = self.msgs["MotorCtrl"]()
            m.name = jid
            m.pos = float(pose[jid])
            m.spd = spd
            m.cur = cur
            ctrl.append(m)
        msg.ctrl = ctrl
        (self.pub_head if group == "head" else self.pub_arm).publish(msg)

    def publish_hand(self, positions: list[int]) -> None:
        msg = self.msgs["SetMotorMulti"]()
        msg.mode = 1
        msg.positions = [int(p) for p in positions]
        msg.speeds = [0] * HAND_N
        msg.currents = [0] * HAND_N
        msg.pwms = [0] * HAND_N
        msg.durations = [0] * HAND_N
        self.pub_hand.publish(msg)


def ramp_arm(node: Node, group: str, base: dict, target: dict, secs: float,
             rate: float, spd: float, cur: float) -> None:
    steps = max(1, int(secs * rate))
    for i in range(steps + 1):
        s = smooth(i / steps)
        node.publish_group(group, {k: base[k] + (target[k] - base[k]) * s for k in base}, spd, cur)
        node.spin(1.0 / rate)


def ramp_hand(node: Node, base: list[int], target: list[int], secs: float, rate: float) -> None:
    steps = max(1, int(secs * rate))
    for i in range(steps + 1):
        s = smooth(i / steps)
        node.publish_hand([int(round(base[c] + (target[c] - base[c]) * s)) for c in range(HAND_N)])
        node.spin(1.0 / rate)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--step", choices=["head", "hand", "arm", "all"], default="head")
    p.add_argument("--delta", type=float, default=None,
                   help="头/臂 rad（默认 0.05）；手为通道码（默认 80）")
    p.add_argument("--ramp", type=float, default=1.5, help="偏出/回位各用时（秒）")
    p.add_argument("--hold", type=float, default=1.0, help="偏出后保持（秒）")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--spd", type=float, default=0.2)
    p.add_argument("--cur", type=float, default=10.0)
    args = p.parse_args()

    import rclpy
    from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorCtrl, RobotState
    from brainco_hand_msgs.msg import MotorStatus, SetMotorMulti

    msgs = {"ArmCtrl": ArmCtrl, "HeadCtrl": HeadCtrl, "MotorCtrl": MotorCtrl,
            "RobotState": RobotState, "MotorStatus": MotorStatus,
            "SetMotorMulti": SetMotorMulti}

    rclpy.init()
    node = Node(rclpy, msgs)
    # DDS 发现可能要几秒（首次匹配时尤其明显），给 5s
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not node.seen_state:
        node.spin(0.1)
    if not node.seen_state:
        print("[staged] ERROR: 5s 内没收到 /robot_state", file=sys.stderr)
        rclpy.shutdown()
        return 1

    steps = ["head", "hand", "arm"] if args.step == "all" else [args.step]
    results = []
    for step in steps:
        if step == "head":
            delta = 0.05 if args.delta is None else min(abs(args.delta), 0.10)
            base = {i: node.head[i] for i in HEAD_IDS}
            target = dict(base)
            target[2] = base[2] + delta
            print(f"[staged] HEAD  head_pitch {base[2]:+.4f} -> {target[2]:+.4f} (Δ{delta:+.3f} rad)")
            ramp_arm(node, "head", base, target, args.ramp, args.rate, args.spd, args.cur)
            node.spin(args.hold)
            peak = node.head[2]
            ramp_arm(node, "head", target, base, args.ramp, args.rate, args.spd, args.cur)
            node.spin(max(0.5, args.ramp * 0.5))
            got = peak - base[2]
            resid = node.head[2] - base[2]
            ok = abs(got - delta) <= max(0.02, abs(delta) * 0.5) and abs(resid) <= max(0.02, abs(delta) * 0.5)
            print(f"[staged]   实测 Δ={got:+.4f} rad，回位残差={resid:+.4f} rad -> {'OK' if ok else 'FAIL'}")
            results.append(("head", ok))

        elif step == "hand":
            delta = 80 if args.delta is None else int(abs(args.delta))
            if not node.seen_hand:
                node.spin(1.0)
            if not node.seen_hand:
                print("[staged] ERROR: 没收到 /right_hand/motor_status", file=sys.stderr)
                results.append(("hand", False))
                continue
            base = list(node.hand)
            target = list(base)
            target[HAND_CH] = max(HAND_CODE_MIN, min(HAND_CODE_MAX, base[HAND_CH] + delta))
            print(f"[staged] HAND  通道{HAND_CH} {base[HAND_CH]} -> {target[HAND_CH]} 码 "
                  f"(真值 {base})")
            ramp_hand(node, base, target, args.ramp, args.rate)
            node.spin(args.hold)
            peak = node.hand[HAND_CH]
            ramp_hand(node, target, base, args.ramp, args.rate)
            node.spin(max(0.5, args.ramp * 0.5))
            got = peak - base[HAND_CH]
            resid = node.hand[HAND_CH] - base[HAND_CH]
            ok = abs(got) >= delta * 0.3 and abs(resid) <= max(20, delta * 0.6)
            print(f"[staged]   实测 Δ={got:+d} 码，回位残差={resid:+d} 码 -> {'OK' if ok else 'FAIL'}")
            results.append(("hand", ok))

        else:  # arm
            delta = 0.05 if args.delta is None else min(abs(args.delta), 0.10)
            base = {i: node.arm[i] for i in ARM_IDS_CMD}
            target = dict(base)
            target[ARM_ID] = base[ARM_ID] + delta
            print(f"[staged] ARM   id{ARM_ID} {base[ARM_ID]:+.4f} -> {target[ARM_ID]:+.4f} (Δ{delta:+.3f} rad)")
            ramp_arm(node, "arm", base, target, args.ramp, args.rate, args.spd, args.cur)
            node.spin(args.hold)
            peak = node.arm[ARM_ID]
            ramp_arm(node, "arm", target, base, args.ramp, args.rate, args.spd, args.cur)
            node.spin(max(0.5, args.ramp * 0.5))
            got = peak - base[ARM_ID]
            resid = node.arm[ARM_ID] - base[ARM_ID]
            ok = abs(got - delta) <= max(0.02, abs(delta) * 0.5) and abs(resid) <= max(0.02, abs(delta) * 0.5)
            print(f"[staged]   实测 Δ={got:+.4f} rad，回位残差={resid:+.4f} rad -> {'OK' if ok else 'FAIL'}")
            results.append(("arm", ok))

    print("[staged] 汇总: " + "  ".join(f"{k}={'OK' if v else 'FAIL'}" for k, v in results))
    rclpy.shutdown()
    return 0 if all(v for _, v in results) else 2


if __name__ == "__main__":
    sys.exit(main())
