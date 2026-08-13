"""ROS-level smoke test; run after sourcing ROS and Walker SDK messages."""
import json
import os
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mc_task_msgs.msg import JointCommand, RobotCommand

from constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES, TASK_RESET_BODY_POSE
from pico_source import MockPicoSource, PicoFrame
from pico_teleop import WalkerC1PicoTeleop, _make_source, _validate_mode


class PicoRosSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_command_surface_and_deadman(self):
        config_path = Path(__file__).with_name("pico_teleop_config.json")
        config = json.loads(config_path.read_text(encoding="utf-8"))
        teleop = WalkerC1PicoTeleop(config, mode="sim", command_enabled=True)
        observer = Node("pico_teleop_smoke_observer")
        received = {"body": [], "left": [], "right": [], "preview": []}
        subscriptions = [
            observer.create_subscription(
                RobotCommand, "/mc/sdk/robot_command", lambda msg: received["body"].append(msg), 10
            ),
            observer.create_subscription(
                JointCommand, "/mc/left_hand/command", lambda msg: received["left"].append(msg), 10
            ),
            observer.create_subscription(
                JointCommand, "/mc/right_hand/command", lambda msg: received["right"].append(msg), 10
            ),
            observer.create_subscription(
                JointState, "/pico/joint_states", lambda msg: received["preview"].append(msg), 10
            ),
        ]
        self.assertEqual(len(subscriptions), 4)
        try:
            for name in ["head_yaw_joint", "head_pitch_joint"] + LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES:
                teleop.joint_positions[name] = float(TASK_RESET_BODY_POSE[name])
            # Allow DDS discovery before the first publication.
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.02)
                rclpy.spin_once(teleop, timeout_sec=0.02)

            frame = MockPicoSource().read()
            # The first frame establishes per-controller liveness baselines;
            # a changed pose is required before the deadman can arm.
            teleop.step(frame)
            live_frame = PicoFrame(
                frame.headset_pose,
                [frame.left_controller_pose[0] + 1e-4, *frame.left_controller_pose[1:]],
                [frame.right_controller_pose[0] + 1e-4, *frame.right_controller_pose[1:]],
                frame.controls,
                frame.timestamp_ns + 1,
            )
            teleop.step(live_frame)
            frame = live_frame
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and not all(received.values()):
                rclpy.spin_once(observer, timeout_sec=0.02)
                rclpy.spin_once(teleop, timeout_sec=0.02)

            self.assertTrue(all(received.values()))
            body_names = [command.name for command in received["body"][-1].joint_cmd]
            self.assertEqual(len(body_names), 16)
            self.assertFalse(any(name.startswith(("waist_", "L_hip", "R_hip")) for name in body_names))
            self.assertEqual(len(received["left"][-1].position), 6)
            self.assertEqual(len(received["right"][-1].position), 6)
            self.assertEqual(list(received["left"][-1].mode), [5] * 6)
            self.assertEqual(list(received["right"][-1].mode), [5] * 6)
            self.assertEqual(len(received["preview"][-1].name), 28)
            self.assertEqual(len(received["preview"][-1].name), len(received["preview"][-1].position))

            # One arm's IK failure must not freeze the other arm, head, or
            # hand commands for the whole frame.
            counts_before_failure = {key: len(value) for key, value in received.items()}
            teleop.right_ik.last_rejection_reason = "test rejection"
            with patch.object(teleop.right_ik, "solve", return_value=None):
                teleop.step(frame)
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.02)
                rclpy.spin_once(teleop, timeout_sec=0.02)
            self.assertTrue(all(
                len(received[key]) > counts_before_failure[key]
                for key in ("body", "left", "right", "preview")
            ))

            counts = {key: len(value) for key, value in received.items()}
            released_controls = dict(frame.controls)
            released_controls["RightController"] = dict(frame.controls["RightController"])
            released_controls["RightController"]["key_two"] = False
            released = PicoFrame(
                frame.headset_pose,
                frame.left_controller_pose,
                frame.right_controller_pose,
                released_controls,
                frame.timestamp_ns + 1,
            )
            teleop.step(released)
            for _ in range(5):
                rclpy.spin_once(observer, timeout_sec=0.02)
            self.assertFalse(teleop.armed)
            self.assertEqual(counts, {key: len(value) for key, value in received.items()})

            teleop.joint_positions["L_elbow_pitch_joint"] = 0.0
            teleop.joint_positions["R_elbow_pitch_joint"] = 0.0
            teleop.step(frame)
            for _ in range(5):
                rclpy.spin_once(observer, timeout_sec=0.02)
            self.assertTrue(teleop.armed)
            self.assertTrue(all(
                len(received[key]) > counts[key]
                for key in ("body", "left", "right", "preview")
            ))
        finally:
            observer.destroy_node()
            teleop.destroy_node()

    def test_real_and_domain_guards(self):
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "0"}, clear=False):
            with self.assertRaises(SystemExit):
                _validate_mode("real", True, False)
            _validate_mode("real", True, True)
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "0"}, clear=False):
            with self.assertRaises(SystemExit):
                _validate_mode("sim", True, False)
        with self.assertRaises(SystemExit):
            _make_source("mock", "real")


if __name__ == "__main__":
    unittest.main()
