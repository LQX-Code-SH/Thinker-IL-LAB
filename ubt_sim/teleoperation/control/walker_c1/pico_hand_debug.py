#!/usr/bin/env python3
"""Receive and inspect PICO/OpenXR hand skeletons without commanding C1."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from .pico_hand_tracking import (
        FINGERTIP_JOINT_NAMES,
        HandTrackingData,
        OPENXR_HAND_JOINT_NAMES,
        get_finger_flexion,
        get_thumb_opposition,
    )
    from .pico_source import PicoSource
except ImportError:
    from pico_hand_tracking import (
        FINGERTIP_JOINT_NAMES,
        HandTrackingData,
        OPENXR_HAND_JOINT_NAMES,
        get_finger_flexion,
        get_thumb_opposition,
    )
    from pico_source import PicoSource


_THIS_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = _THIS_DIR / "pico_teleop_config.json"
SUMMARY_JOINTS = ("wrist",) + FINGERTIP_JOINT_NAMES


def _format_vector(values) -> str:
    vector = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(vector)):
        return "unavailable"
    return "[" + ", ".join(f"{value:+.4f}" for value in vector) + "]"


def format_hand_summary(hand: Optional[HandTrackingData]) -> str:
    """Format the required wrist/fingertip debug view for one hand."""
    if hand is None:
        return "Hand: no hand data requested"
    title = "Left hand" if hand.side == "left" else "Right hand"
    lines = [
        f"{title}: is_tracked={hand.is_tracked} quality={hand.tracking_quality}",
    ]
    for name in SUMMARY_JOINTS:
        pose = hand.joint(name)
        if name == "wrist":
            position = _format_vector(pose.position_m) if pose.is_valid else "unavailable"
            quaternion = (
                _format_vector(pose.quaternion_xyzw)
                if pose.is_valid
                else "unavailable"
            )
            lines.append(f"  wrist position [m]: {position}")
            lines.append(
                "  wrist quaternion [xyzw]: "
                + quaternion
            )
        else:
            value = _format_vector(pose.position_m) if pose.is_valid else "unavailable"
            lines.append(f"  {name} [m]: {value}")
    try:
        lines.append(
            f"  index flexion [rad]: {get_finger_flexion(hand, 'index'):.4f}"
        )
    except ValueError:
        lines.append("  index flexion [rad]: unavailable")
    try:
        lines.append(
            f"  thumb-index distance [m]: {get_thumb_opposition(hand, 'index'):.4f}"
        )
    except ValueError:
        lines.append("  thumb-index distance [m]: unavailable")
    return "\n".join(lines)


def format_all_joints(hand: HandTrackingData) -> str:
    lines = [
        (
            f"{hand.side} hand all joints: tracked={hand.is_tracked} "
            f"quality={hand.tracking_quality} timestamp_ns={hand.timestamp_ns}"
        )
    ]
    for index, name in enumerate(OPENXR_HAND_JOINT_NAMES):
        pose = hand.joint(name)
        lines.append(
            f"  [{index:02d}] {name}: position_m={_format_vector(pose.position_m)} "
            f"quaternion_xyzw={_format_vector(pose.quaternion_xyzw)} "
            f"valid={pose.is_valid}"
        )
    return "\n".join(lines)


class RosHandMarkerPublisher:
    """Optional ROS MarkerArray publisher for wrist and five fingertips."""

    def __init__(self, topic: str, frame_id: str):
        try:
            import rclpy
            from visualization_msgs.msg import Marker, MarkerArray
        except ImportError as exc:
            raise RuntimeError(
                "--ros-markers requires ROS 2 rclpy and visualization_msgs"
            ) from exc
        self.rclpy = rclpy
        self.Marker = Marker
        self.MarkerArray = MarkerArray
        self.frame_id = str(frame_id)
        rclpy.init(args=None)
        self.node = rclpy.create_node("pico_hand_tracking_debug")
        self.publisher = self.node.create_publisher(MarkerArray, topic, 10)

    def _marker(
        self,
        side: str,
        marker_id: int,
        joint_name: str,
        hand: Optional[HandTrackingData],
    ):
        marker = self.Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = f"pico_{side}_hand"
        marker.id = marker_id
        marker.type = self.Marker.SPHERE
        pose_valid = (
            hand is not None
            and hand.is_tracked
            and bool(hand.joint_validity[joint_name])
        )
        marker.action = self.Marker.ADD if pose_valid else self.Marker.DELETE
        if not pose_valid:
            return marker
        position = hand.joint_positions[joint_name]
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        diameter = 0.030 if joint_name == "wrist" else 0.018
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        if side == "left":
            marker.color.r = 0.15
            marker.color.g = 0.90
            marker.color.b = 0.30
        else:
            marker.color.r = 0.15
            marker.color.g = 0.45
            marker.color.b = 1.00
        marker.color.a = 0.95
        return marker

    def publish(
        self,
        left: Optional[HandTrackingData],
        right: Optional[HandTrackingData],
    ) -> None:
        markers = self.MarkerArray()
        names = ("wrist",) + FINGERTIP_JOINT_NAMES
        for side, hand in (("left", left), ("right", right)):
            for marker_id, name in enumerate(names):
                markers.markers.append(
                    self._marker(side, marker_id, name, hand)
                )
        self.publisher.publish(markers)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def _load_tracking_basis(path: str):
    with open(path, "r", encoding="utf-8") as handle:
        config = json.load(handle)
    return config["tracking_to_robot"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument(
        "--hand-tracking-enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="request 26-joint hands from the existing XRoboToolkit SDK",
    )
    parser.add_argument("--read-hz", type=float, default=30.0)
    parser.add_argument("--print-hz", type=float, default=8.0)
    parser.add_argument(
        "--all-joints",
        action="store_true",
        help="print all 26 named poses instead of the compact wrist/tip view",
    )
    parser.add_argument(
        "--ros-markers",
        action="store_true",
        help="publish wrist and fingertip spheres; never publishes robot commands",
    )
    parser.add_argument("--marker-topic", default="/pico/hand_tracking/markers")
    parser.add_argument("--marker-frame-id", default="pico_tracking_robot_axes")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.read_hz <= 0.0 or args.print_hz <= 0.0:
        raise SystemExit("--read-hz and --print-hz must be positive")
    if not args.hand_tracking_enabled:
        raise SystemExit(
            "hand tracking is disabled; pass --hand-tracking-enabled to run this receiver"
        )

    basis = _load_tracking_basis(args.config)
    source = PicoSource(hand_tracking_enabled=True, tracking_to_output=basis)
    markers = (
        RosHandMarkerPublisher(args.marker_topic, args.marker_frame_id)
        if args.ros_markers
        else None
    )
    print("PICO hand tracking receiver started (robot commands are disabled)")
    print("joint count=26, position=meter, quaternion=xyzw")
    print("output frame: OpenXR origin, robot axes +X forward, +Y left, +Z up")
    print("PICO UI: enable system Hand Tracking, then Hand On and Send")

    read_period = 1.0 / args.read_hz
    print_period = 1.0 / args.print_hz
    next_print = 0.0
    last_warning = 0.0
    try:
        while True:
            started = time.monotonic()
            try:
                left_hand, right_hand = source.read_hands()
                if markers is not None:
                    markers.publish(left_hand, right_hand)
                if started >= next_print:
                    next_print = started + print_period
                    if args.all_joints:
                        print(format_all_joints(left_hand))
                        print(format_all_joints(right_hand))
                    else:
                        print(format_hand_summary(left_hand))
                        print(format_hand_summary(right_hand))
                    print("-" * 72)
            except Exception as exc:
                if started - last_warning >= 1.0:
                    print(f"warning: PICO hand read failed: {exc}")
                    last_warning = started
                if markers is not None:
                    markers.publish(None, None)
            time.sleep(max(0.0, read_period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        return 0
    finally:
        source.close()
        if markers is not None:
            markers.close()


if __name__ == "__main__":
    raise SystemExit(main())
