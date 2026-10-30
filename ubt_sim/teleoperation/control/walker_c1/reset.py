#!/usr/bin/env python3
"""Move Walker C1 / Astron to the task reset pose.

There are two different reset concepts in this project:

* Pressing ``R`` in simulation resets Isaac state back to the robot home/zero
  state through ``sim_runner.py``.
* This script publishes the task initial pose used before grasping.

The official SDK/ROSA home command is available as ``--mode sdk-home`` only for
manual safety/home checks; it is not the default task reset.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

try:
    from .constants import (
        BODY_JOINT_NAMES,
        DEFAULT_TASK_RESET_CLEAR_DURATION,
        DEFAULT_TASK_RESET_DURATION,
        DEFAULT_TASK_RESET_HZ,
        LEFT_HAND_JOINT_NAMES,
        RIGHT_HAND_JOINT_NAMES,
        TASK_RESET_ARM_CLEAR_POSE,
        TASK_RESET_BODY_POSE,
        TASK_RESET_ELBOW_CLEAR_POSE,
        TASK_RESET_LEFT_HAND_POSE,
        TASK_RESET_RIGHT_HAND_POSE,
    )
except ImportError:
    from constants import (
        BODY_JOINT_NAMES,
        DEFAULT_TASK_RESET_CLEAR_DURATION,
        DEFAULT_TASK_RESET_DURATION,
        DEFAULT_TASK_RESET_HZ,
        LEFT_HAND_JOINT_NAMES,
        RIGHT_HAND_JOINT_NAMES,
        TASK_RESET_ARM_CLEAR_POSE,
        TASK_RESET_BODY_POSE,
        TASK_RESET_ELBOW_CLEAR_POSE,
        TASK_RESET_LEFT_HAND_POSE,
        TASK_RESET_RIGHT_HAND_POSE,
    )


DEFAULT_CMD_PORT = int(os.environ.get("UBT_SIM_WALKER_C1_CMD_PORT", os.environ.get("UBT_SIM_WALKER_S2_CMD_PORT", 5655)))


def _make_task_payload(body_pose: dict[str, float] | None = None) -> dict:
    pose = dict(TASK_RESET_BODY_POSE)
    if body_pose:
        pose.update(body_pose)
    return {
        "body": pose,
        "left_hand": TASK_RESET_LEFT_HAND_POSE,
        "right_hand": TASK_RESET_RIGHT_HAND_POSE,
    }


def _send_sim_messages(cmd_port: int, messages: list[dict], repeats: int, interval: float) -> bool:
    try:
        import zmq
    except ImportError:
        print("[ERROR] pyzmq is not installed; cannot send simulation command.", file=sys.stderr)
        return False

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.bind(f"tcp://*:{cmd_port}")

    try:
        # Give the simulator SUB socket time to observe this PUB socket.
        time.sleep(0.3)
        for message in messages:
            for _ in range(repeats):
                socket.send_json(message)
                time.sleep(interval)
    finally:
        socket.close(linger=0)
        context.term()

    print(f"[INFO] Sent Walker C1 simulation command on tcp://*:{cmd_port}")
    return True


def _send_sim_scene_reset(cmd_port: int, repeats: int, interval: float) -> bool:
    return _send_sim_messages(cmd_port, [{"reset": True}], repeats, interval)


def _send_sim_task_reset(cmd_port: int, repeats: int, interval: float, staged: bool) -> bool:
    messages = []
    if staged:
        messages.append(_make_task_payload(TASK_RESET_ARM_CLEAR_POSE))
        messages.append(_make_task_payload(TASK_RESET_ELBOW_CLEAR_POSE))
    messages.append(_make_task_payload())
    return _send_sim_messages(cmd_port, messages, repeats, interval)


def _run_sdk_reset(rosa_bin: str, model: str, timeout_sec: float, dry_run: bool) -> bool:
    if shutil.which(rosa_bin) is None:
        print(f"[ERROR] '{rosa_bin}' not found. Source the robot ROSA/ROS2 environment first.", file=sys.stderr)
        return False

    cmd = [
        rosa_bin,
        "run",
        "manipulation_outline_sdk",
        "robot_motion_sdk_reset_upper_body",
        "--",
        "--model",
        model,
        "--timeout_sec",
        str(timeout_sec),
    ]

    print("[INFO] Running SDK reset:")
    print("       " + " ".join(cmd))
    if dry_run:
        return True

    completed = subprocess.run(cmd, check=False)
    return completed.returncode == 0


def _make_robot_command(node, body_pose: dict[str, float]):
    from mc_task_msgs.msg import JointCmd, RobotCommand

    msg = RobotCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    for name in BODY_JOINT_NAMES:
        if name not in body_pose:
            continue
        joint_cmd = JointCmd()
        joint_cmd.name = name
        joint_cmd.control_mode = JointCmd.MODE_POSITION
        joint_cmd.position = float(body_pose[name])
        msg.joint_cmd.append(joint_cmd)
    return msg


def _make_hand_command(node, names: list[str], positions: list[float]):
    from mc_task_msgs.msg import JointCommand

    msg = JointCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.names = list(names)
    msg.position = [float(v) for v in positions]
    msg.velocity = [0.0] * len(names)
    msg.torque = [0.0] * len(names)
    msg.acceleration = [0.0] * len(names)
    # Astron SDK hand demo uses mode=5 for position control.
    msg.mode = [5] * len(names)
    msg.kp = [0.0] * len(names)
    msg.kd = [0.0] * len(names)
    return msg


def _publish_ros_pose(node, body_pub, left_hand_pub, right_hand_pub, body_pose: dict[str, float]) -> None:
    body_pub.publish(_make_robot_command(node, body_pose))
    left_hand_pub.publish(_make_hand_command(node, LEFT_HAND_JOINT_NAMES, TASK_RESET_LEFT_HAND_POSE))
    right_hand_pub.publish(_make_hand_command(node, RIGHT_HAND_JOINT_NAMES, TASK_RESET_RIGHT_HAND_POSE))


def _run_task_reset_ros(duration: float, hz: float, clear_duration: float, staged: bool) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from mc_state_msgs.msg import RobotState
        from mc_task_msgs.msg import JointCommand, RobotCommand
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"[ERROR] ROS2 Python environment is not ready: {exc}", file=sys.stderr)
        return False

    rclpy.init(args=None)
    node: Node | None = None
    try:
        node = Node("walker_c1_task_reset")
        body_pub = node.create_publisher(RobotCommand, "/mc/sdk/robot_command", 10)
        left_hand_pub = node.create_publisher(JointCommand, "/mc/left_hand/command", 10)
        right_hand_pub = node.create_publisher(JointCommand, "/mc/right_hand/command", 10)
        joint_pos: dict[str, float] = {}
        sim_step: list[int | None] = [None]

        def state_cb(msg: RobotState) -> None:
            joint_pos.update(zip(msg.joint_states.name, msg.joint_states.position))

        def object_cb(msg: String) -> None:
            try:
                sim_step[0] = json.loads(msg.data).get("sim_step")
            except (json.JSONDecodeError, TypeError):
                pass

        node.create_subscription(RobotState, "/mc/sdk/robot_state", state_cb, 10)
        node.create_subscription(String, "/sim/object_state", object_cb, 10)
        state_deadline = time.monotonic() + 1.5
        while time.monotonic() < state_deadline and not joint_pos:
            rclpy.spin_once(node, timeout_sec=0.05)

        stages: list[tuple[str, dict[str, float], float]] = []
        if staged:
            stages.append(("arm-clear", TASK_RESET_ARM_CLEAR_POSE, clear_duration))
            stages.append(("elbow-clear", TASK_RESET_ELBOW_CLEAR_POSE, clear_duration))
        stages.append(("task-reset", TASK_RESET_BODY_POSE, duration))

        interval = 1.0 / hz
        for label, pose, stage_duration in stages:
            print(f"[INFO] Moving Walker C1 through {label} for {stage_duration:.2f} simulated seconds")
            start = {name: float(joint_pos.get(name, value)) for name, value in pose.items()}
            updates = max(int(stage_duration * hz), 1)
            step_count = max(int(round(100.0 / hz)), 1)
            for index in range(updates):
                t = (index + 1) / updates
                blend = t * t * (3.0 - 2.0 * t)
                command = {
                    name: (1.0 - blend) * start[name] + blend * float(value)
                    for name, value in pose.items()
                }
                _publish_ros_pose(node, body_pub, left_hand_pub, right_hand_pub, command)

                first_step = sim_step[0]
                if first_step is None:
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(interval)
                    continue
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if sim_step[0] is not None and sim_step[0] - first_step >= step_count:
                        break
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    print("[INFO] Walker C1 task reset pose published.")
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move Walker C1 / Astron to the grasp-task initial pose."
    )
    parser.add_argument(
        "--mode",
        choices=("task", "sim-task", "sim-scene", "sdk-home", "auto"),
        default="task",
        help=(
            "task publishes the grasp-task pose over ROS; sim-task sends that pose over ZMQ; "
            "sim-scene is equivalent to pressing R in sim; sdk-home calls the vendor ROSA home command."
        ),
    )
    parser.add_argument("--no-staged", action="store_true", help="Skip Tiankung-style arm clear stages.")
    parser.add_argument("--duration", type=float, default=DEFAULT_TASK_RESET_DURATION, help="Final task-pose publish duration.")
    parser.add_argument("--clear-duration", type=float, default=DEFAULT_TASK_RESET_CLEAR_DURATION, help="Each clear-stage duration.")
    parser.add_argument("--hz", type=float, default=DEFAULT_TASK_RESET_HZ, help="ROS task-pose publish frequency.")
    parser.add_argument("--model", default="astron", help="SDK model name; SDK docs use 'astron' for Walker C1.")
    parser.add_argument("--timeout-sec", type=float, default=120.0, help="SDK reset timeout in seconds.")
    parser.add_argument("--rosa-bin", default=os.environ.get("ROSA_BIN", "rosa"), help="ROSA executable name/path.")
    parser.add_argument("--dry-run", action="store_true", help="Print the SDK command without executing it.")
    parser.add_argument("--cmd-port", type=int, default=DEFAULT_CMD_PORT, help="Simulation ZMQ command port.")
    parser.add_argument("--sim-repeats", type=int, default=20, help="Number of repeated simulation reset messages.")
    parser.add_argument("--sim-interval", type=float, default=0.05, help="Delay between simulation reset messages.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    staged = not args.no_staged

    if args.mode == "task":
        return 0 if _run_task_reset_ros(args.duration, args.hz, args.clear_duration, staged) else 1

    if args.mode == "sim-task":
        return 0 if _send_sim_task_reset(args.cmd_port, args.sim_repeats, args.sim_interval, staged) else 1

    if args.mode == "sim-scene":
        return 0 if _send_sim_scene_reset(args.cmd_port, args.sim_repeats, args.sim_interval) else 1

    if args.mode == "auto" and shutil.which(args.rosa_bin) is None:
        print("[WARN] ROSA command not found; falling back to ROS task reset.")
        return 0 if _run_task_reset_ros(args.duration, args.hz, args.clear_duration, staged) else 1

    return 0 if _run_sdk_reset(args.rosa_bin, args.model, args.timeout_sec, args.dry_run) else 1


if __name__ == "__main__":
    raise SystemExit(main())
