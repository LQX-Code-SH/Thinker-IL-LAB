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

HAND_MODE = 5
BODY_CMD_TOPIC = "/mc/sdk/robot_command"
BODY_STATE_TOPIC = "/mc/sdk/robot_state"
LEFT_HAND_CMD_TOPIC = "/mc/left_hand/command"
RIGHT_HAND_CMD_TOPIC = "/mc/right_hand/command"


def _quintic(t: float) -> float:
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def _interp(start: dict[str, float], target: dict[str, float], ratio: float) -> dict[str, float]:
    return {
        name: start.get(name, target[name]) * (1.0 - ratio) + target[name] * ratio
        for name in target
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hz", type=float, default=100.0, help="Interpolation publish rate")
    parser.add_argument("--segment-s", type=float, default=1.0, help="Duration per segment (seconds)")
    parser.add_argument("--hold-s", type=float, default=0.5, help="Hold the final pose (seconds)")
    parser.add_argument("--dry-run", action="store_true", help="Print commands instead of publishing")
    args = parser.parse_args()

    if args.dry_run:
        print("[robot_ready] dry-run: ARM_JOINTS =", ARM_JOINTS)
        print("[robot_ready] dry-run: TASK_POSE =", TASK_POSE)
        print("[robot_ready] dry-run: hands ->", [0.0] * 6)
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
    node.create_subscription(JointState, LEFT_HAND_CMD_TOPIC.replace("/command", "/joint_states"), on_joint_state, qos_sub)
    node.create_subscription(JointState, RIGHT_HAND_CMD_TOPIC.replace("/command", "/joint_states"), on_joint_state, qos_sub)

    print("[robot_ready] waiting for robot state ...")
    deadline = time.time() + 10.0
    while time.time() < deadline and not state["pos"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not state["pos"]:
        print("[robot_ready] 警告：未收到 /mc/sdk/robot_state，将直接从当前位置插值（可能不准确）")
    else:
        missing = [n for n in ARM_JOINTS if n not in state["pos"]]
        if missing:
            print(f"[robot_ready] 警告：状态缺少关节 {missing}")

    def publish_body(values: dict[str, float]) -> None:
        msg = RobotCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        for name, value in values.items():
            cmd = JointCmd()
            cmd.name = name
            cmd.control_mode = JointCmd.MODE_POSITION
            cmd.position = float(value)
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
        start = {n: float(state["pos"].get(n, TASK_POSE[n])) for n in target}
        steps = max(int(duration * args.hz), 1)
        print(f"[robot_ready] {label} ({duration:.1f}s)")
        for i in range(1, steps + 1):
            values = _interp(start, target, _quintic(i / steps))
            publish_body(values)
            publish_hands()
            rclpy.spin_once(node, timeout_sec=1.0 / args.hz)
        state["pos"].update(target)

    try:
        run_segment(LIFT_POSE, args.segment_s * 0.6, "1/3 lift shoulders")
        run_segment(ELBOW_POSE, args.segment_s * 0.6, "2/3 fold elbows")
        run_segment(TASK_POSE, args.segment_s, "3/3 task-ready pose")

        print(f"[robot_ready] 保持 {args.hold_s:.1f}s ...")
        end = time.time() + args.hold_s
        while time.time() < end:
            publish_body(TASK_POSE)
            publish_hands()
            rclpy.spin_once(node, timeout_sec=0.02)
        print("[robot_ready] done.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
