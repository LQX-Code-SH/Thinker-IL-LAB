#!/usr/bin/env python3
"""Play synthetic 26-joint hand poses on the Walker C1 hand in Isaac Sim.

This program has no PICO dependency.  By default it publishes full named 11DoF
``JointCommand`` messages through the existing ROS2->ZMQ simulation bridge;
``devices/walker_c1/action_process.py`` then executes those names directly.
Do not run it alongside teleoperation.
"""
from __future__ import annotations

import argparse
import os
import time
from typing import Iterable

try:
    from .pico_hand_mock import generate_mock_hand
    from .pico_hand_retarget import C1HandRetargeter
except ImportError:
    from pico_hand_mock import generate_mock_hand
    from pico_hand_retarget import C1HandRetargeter


DEMO_SEQUENCE = (
    "open_hand",
    "fist",
    "open_hand",
    "thumb_index_pinch",
    "tripod",
    "open_hand",
    "ring_flex",
    "open_hand",
    "little_flex",
    "open_hand",
    "cylindrical_grasp",
)
DEFAULT_CMD_PORT = int(
    os.environ.get(
        "UBT_SIM_WALKER_C1_CMD_PORT",
        os.environ.get("UBT_SIM_WALKER_S2_CMD_PORT", 5655),
    )
)
DEFAULT_SIM_ROS_DOMAIN_ID = int(os.environ.get("UBT_SIM_ROS_DOMAIN_ID", 146))


def selected_sides(side: str) -> tuple[str, ...]:
    side = str(side).lower()
    if side == "both":
        return ("left", "right")
    if side in ("left", "right"):
        return (side,)
    raise ValueError("side must be left, right, or both")


def require_sim_ros_domain(expected_domain_id: int) -> None:
    raw_domain = os.environ.get("ROS_DOMAIN_ID")
    try:
        actual_domain = int(raw_domain) if raw_domain is not None else None
    except ValueError as exc:
        raise RuntimeError("ROS_DOMAIN_ID must be an integer") from exc
    if actual_domain != int(expected_domain_id):
        raise RuntimeError(
            f"simulation demo requires ROS_DOMAIN_ID={expected_domain_id}; "
            f"got {raw_domain!r}. Refusing to publish on a possible real-robot domain."
        )


def make_pose_payload(
    pose: str,
    sides: Iterable[str],
    retargeter: C1HandRetargeter,
    dt_s: float,
) -> dict[str, dict[str, float]]:
    """Build a simulation payload containing full named 11DoF targets."""
    payload: dict[str, dict[str, float]] = {}
    for side in sides:
        hand = generate_mock_hand(pose, side=side)
        result = retargeter.retarget_result(hand, dt_s=dt_s)
        payload[f"{side}_hand"] = {
            name: float(value)
            for name, value in zip(result.joint_names, result.robot_hand_q)
        }
    return payload


def _format_payload(pose: str, payload: dict[str, dict[str, float]]) -> str:
    lines = [f"pose={pose}"]
    for group, values in payload.items():
        compact = ", ".join(f"{name}={value:+.3f}" for name, value in values.items())
        lines.append(f"  {group}: {compact}")
    return "\n".join(lines)


def run_dry(sides: tuple[str, ...]) -> None:
    mapper = C1HandRetargeter(smoothing_enabled=False, velocity_limits_enabled=False)
    for pose in DEMO_SEQUENCE:
        payload = make_pose_payload(pose, sides, mapper, dt_s=0.02)
        print(_format_payload(pose, payload))


class _RosSimulationPublisher:
    def __init__(self, sides: tuple[str, ...]) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from mc_task_msgs.msg import JointCommand
        except ImportError as exc:
            raise RuntimeError(
                "ROS2 and mc_task_msgs are required for the default demo transport"
            ) from exc
        self.rclpy = rclpy
        self.JointCommand = JointCommand
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)
        self.node = Node("walker_c1_hand_offline_demo")
        self.publishers = {
            side: self.node.create_publisher(
                JointCommand,
                f"/mc/{side}_hand/command",
                10,
            )
            for side in sides
        }

    def warmup(self) -> None:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish(self, payload: dict[str, dict[str, float]]) -> None:
        for group, values in payload.items():
            side = group.removesuffix("_hand")
            message = self.JointCommand()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.names = list(values)
            message.position = [float(value) for value in values.values()]
            message.velocity = [0.0] * len(values)
            message.torque = [0.0] * len(values)
            message.acceleration = [0.0] * len(values)
            message.mode = [5] * len(values)
            message.kp = [0.0] * len(values)
            message.kd = [0.0] * len(values)
            self.publishers[side].publish(message)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy and self.rclpy.ok():
            self.rclpy.shutdown()


class _ZmqSimulationPublisher:
    def __init__(self, cmd_port: int) -> None:
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError("pyzmq is required for direct ZMQ demo transport") from exc
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind(f"tcp://*:{cmd_port}")

    def warmup(self) -> None:
        time.sleep(0.4)

    def publish(self, payload: dict[str, dict[str, float]]) -> None:
        self.socket.send_json(payload)

    def close(self) -> None:
        self.socket.close(linger=0)
        self.context.term()


def run_simulation(
    sides: tuple[str, ...],
    transport: str,
    cmd_port: int,
    pose_seconds: float,
    hz: float,
    cycles: int,
    print_hz: float,
) -> None:
    if transport == "ros":
        publisher = _RosSimulationPublisher(sides)
        endpoint = "ROS2 /mc/{left,right}_hand/command -> existing ZMQ bridge"
    elif transport == "zmq":
        publisher = _ZmqSimulationPublisher(cmd_port)
        endpoint = f"direct tcp://*:{cmd_port} (requires bridge disabled)"
    else:
        raise ValueError("transport must be ros or zmq")
    mapper = C1HandRetargeter()
    dt_s = 1.0 / hz
    steps = max(1, int(round(pose_seconds * hz)))
    print_period = max(1, int(round(hz / print_hz)))
    print(
        f"C1 hand offline demo: {endpoint}, side={','.join(sides)}, "
        f"{hz:.1f} Hz; Ctrl+C stops"
    )
    publisher.warmup()

    try:
        cycle_index = 0
        while cycles == 0 or cycle_index < cycles:
            for pose in DEMO_SEQUENCE:
                print(f"\n[{cycle_index + 1}] {pose}")
                next_tick = time.monotonic()
                for step in range(steps):
                    payload = make_pose_payload(pose, sides, mapper, dt_s)
                    publisher.publish(payload)
                    if step % print_period == 0:
                        print(_format_payload(pose, payload))
                    next_tick += dt_s
                    time.sleep(max(0.0, next_tick - time.monotonic()))
            cycle_index += 1
    except KeyboardInterrupt:
        print("\nStopping C1 hand offline demo; last filtered command remains held in simulation.")
    finally:
        publisher.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument(
        "--transport",
        choices=("ros", "zmq"),
        default="ros",
        help="ros uses the normal running bridge; zmq requires starting simulation with bridge disabled",
    )
    parser.add_argument("--cmd-port", type=int, default=DEFAULT_CMD_PORT)
    parser.add_argument(
        "--sim-domain-id",
        type=int,
        default=DEFAULT_SIM_ROS_DOMAIN_ID,
        help="required ROS domain for simulation transport (default: 146)",
    )
    parser.add_argument("--pose-seconds", type=float, default=2.5)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--print-hz", type=float, default=1.0)
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="number of complete demo cycles; 0 repeats until Ctrl+C",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print all target vectors once without opening a ZMQ socket",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.cmd_port <= 0 or args.cmd_port > 65535:
        raise SystemExit("--cmd-port must be in 1..65535")
    if args.pose_seconds <= 0.0 or args.hz <= 0.0 or args.print_hz <= 0.0:
        raise SystemExit("--pose-seconds, --hz, and --print-hz must be positive")
    if args.cycles < 0:
        raise SystemExit("--cycles must be non-negative")
    sides = selected_sides(args.side)
    if args.dry_run:
        run_dry(sides)
    else:
        if args.transport == "ros":
            try:
                require_sim_ros_domain(args.sim_domain_id)
            except RuntimeError as exc:
                raise SystemExit(str(exc)) from exc
        run_simulation(
            sides,
            args.transport,
            args.cmd_port,
            args.pose_seconds,
            args.hz,
            args.cycles,
            args.print_hz,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
