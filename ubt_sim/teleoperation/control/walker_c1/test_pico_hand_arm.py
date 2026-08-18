#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import replace
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from constants import RIGHT_ARM_JOINT_NAMES, TASK_RESET_BODY_POSE
from pico_hand_arm import (
    C1WristArmRetargeter,
    RosHandArmSimulationIO,
    hand_wrist_pose,
)
from pico_hand_mock import generate_mock_hand
from pico_math import quaternion_xyzw_to_matrix


HEADSET_POSE = [0.0, 1.65, 0.0, 0.0, 0.0, 0.0, 1.0]


class FakeArmIK:
    def __init__(self, side: str):
        self.side = side
        self.joint_names = list(RIGHT_ARM_JOINT_NAMES)
        self.last_rejection_reason = ""
        self.next_solution = None
        self.solve_calls = []
        self.anchor = np.eye(4)
        self.anchor[:3, 3] = [0.40, -0.30, 0.80]

    def fk(self, joints):
        return self.anchor.copy()

    def solve(self, position, rotation, seed, *args):
        self.solve_calls.append(
            (np.asarray(position).copy(), np.asarray(rotation).copy(), list(seed), args)
        )
        if self.next_solution is None:
            return list(seed)
        if self.next_solution == "reject":
            self.last_rejection_reason = "test IK rejection"
            return None
        return list(self.next_solution)


class FakeBool:
    def __init__(self):
        self.data = False


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def moved_wrist(hand, position=None, quaternion=None):
    wrist = hand.wrist_pose
    new_position = (
        wrist.position_m.copy() if position is None else np.asarray(position, dtype=float)
    )
    new_quaternion = (
        wrist.quaternion_xyzw.copy()
        if quaternion is None
        else np.asarray(quaternion, dtype=float)
    )
    new_wrist = replace(
        wrist,
        position_m=new_position,
        quaternion_xyzw=new_quaternion,
    )
    positions = {name: value.copy() for name, value in hand.joint_positions.items()}
    orientations = {
        name: value.copy() for name, value in hand.joint_orientations_xyzw.items()
    }
    positions["wrist"] = new_position.copy()
    orientations["wrist"] = new_quaternion.copy()
    return replace(
        hand,
        wrist_pose=new_wrist,
        joint_positions=positions,
        joint_orientations_xyzw=orientations,
    )


class WristArmRetargetTest(unittest.TestCase):
    def setUp(self):
        self.solver = FakeArmIK("right")
        self.controller = C1WristArmRetargeter(
            ("right",), solvers={"right": self.solver}
        )
        self.reset_joints = [
            float(TASK_RESET_BODY_POSE[name]) for name in RIGHT_ARM_JOINT_NAMES
        ]
        self.controller.update_joint_state(RIGHT_ARM_JOINT_NAMES, self.reset_joints)
        self.hand = generate_mock_hand("open_hand", side="right")

    def test_anchor_maps_current_wrist_to_current_robot_palm_without_jump(self):
        self.controller.capture_anchors(HEADSET_POSE, {"right": self.hand})
        result = self.controller.solve(HEADSET_POSE, {"right": self.hand})
        self.assertTrue(result.valid)
        np.testing.assert_allclose(
            result.targets["right"].position,
            self.solver.anchor[:3, 3],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.targets["right"].rotation,
            self.solver.anchor[:3, :3],
            atol=1e-12,
        )
        np.testing.assert_allclose(result.commands["right"], self.reset_joints)

    def test_wrist_translation_uses_existing_scale_and_operator_frame(self):
        self.controller.capture_anchors(HEADSET_POSE, {"right": self.hand})
        initial = hand_wrist_pose(self.hand).position
        moved = moved_wrist(self.hand, position=initial + [0.10, 0.0, 0.0])
        result = self.controller.solve(HEADSET_POSE, {"right": moved})
        self.assertTrue(result.valid)
        target_position = self.solver.solve_calls[-1][0]
        np.testing.assert_allclose(
            target_position,
            self.solver.anchor[:3, 3] + [0.055, 0.0, 0.0],
            atol=1e-12,
        )

    def test_relative_wrist_rotation_drives_robot_palm_orientation(self):
        self.controller.capture_anchors(HEADSET_POSE, {"right": self.hand})
        angle = 0.30
        quaternion = [0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0)]
        moved = moved_wrist(self.hand, quaternion=quaternion)
        result = self.controller.solve(HEADSET_POSE, {"right": moved})
        self.assertTrue(result.valid)
        np.testing.assert_allclose(
            self.solver.solve_calls[-1][1],
            quaternion_xyzw_to_matrix(quaternion),
            atol=1e-12,
        )

    def test_joint_command_uses_existing_per_frame_slew_limit(self):
        self.controller.capture_anchors(HEADSET_POSE, {"right": self.hand})
        self.solver.next_solution = (np.asarray(self.reset_joints) + 0.20).tolist()
        result = self.controller.solve(HEADSET_POSE, {"right": self.hand})
        self.assertTrue(result.valid)
        np.testing.assert_allclose(
            result.commands["right"],
            np.asarray(self.reset_joints) + 0.025,
            atol=1e-12,
        )

    def test_ik_rejection_publishes_nothing_and_does_not_advance_seed(self):
        self.controller.capture_anchors(HEADSET_POSE, {"right": self.hand})
        self.solver.next_solution = "reject"
        rejected = self.controller.solve(HEADSET_POSE, {"right": self.hand})
        self.assertFalse(rejected.valid)
        self.assertEqual(rejected.commands, {})
        self.assertIn("test IK rejection", rejected.rejection_reasons["right"])

        self.solver.next_solution = None
        recovered = self.controller.solve(HEADSET_POSE, {"right": self.hand})
        np.testing.assert_allclose(recovered.commands["right"], self.reset_joints)

    def test_missing_state_and_straight_elbow_refuse_anchor(self):
        empty = C1WristArmRetargeter(
            ("right",), solvers={"right": FakeArmIK("right")}
        )
        ready, reason = empty.input_ready(HEADSET_POSE, {"right": self.hand})
        self.assertFalse(ready)
        self.assertIn("waiting", reason)

        straight = list(self.reset_joints)
        straight[RIGHT_ARM_JOINT_NAMES.index("R_elbow_pitch_joint")] = 0.0
        self.controller.update_joint_state(RIGHT_ARM_JOINT_NAMES, straight)
        ready, reason = self.controller.input_ready(
            HEADSET_POSE, {"right": self.hand}
        )
        self.assertFalse(ready)
        self.assertIn("too straight", reason)

    def test_invalid_wrist_refuses_arm_input(self):
        invalid = generate_mock_hand("open_hand", side="right", is_tracked=False)
        ready, reason = self.controller.input_ready(
            HEADSET_POSE, {"right": invalid}
        )
        self.assertFalse(ready)
        self.assertIn("tracking is invalid", reason)


class SceneTaskResetTest(unittest.TestCase):
    def make_io(self):
        solver = FakeArmIK("right")
        controller = C1WristArmRetargeter(
            ("right",), solvers={"right": solver}
        )
        joints = [
            float(TASK_RESET_BODY_POSE[name]) for name in RIGHT_ARM_JOINT_NAMES
        ]
        controller.update_joint_state(RIGHT_ARM_JOINT_NAMES, joints)
        hand = generate_mock_hand("open_hand", side="right")
        controller.capture_anchors(HEADSET_POSE, {"right": hand})

        io = RosHandArmSimulationIO.__new__(RosHandArmSimulationIO)
        io.arm_retargeter = controller
        io.last_state_monotonic = 123.0
        io.Bool = FakeBool
        io.reset_publisher = FakePublisher()
        io.spin_calls = []
        io.spin_once = lambda timeout_sec=0.0: io.spin_calls.append(timeout_sec)
        return io, controller

    @patch("pico_hand_arm.time.sleep")
    @patch("pico_hand_arm.subprocess.run")
    def test_scene_edge_then_existing_task_reset_and_fresh_state_required(
        self, run_process, sleep
    ):
        run_process.return_value = SimpleNamespace(
            returncode=0, stdout="reset ok", stderr=""
        )
        io, controller = self.make_io()

        io.reset_scene_and_robot(reset_script="/tmp/test-reset.py")

        self.assertEqual(len(io.reset_publisher.messages), 1)
        self.assertTrue(io.reset_publisher.messages[0].data)
        self.assertEqual(io.spin_calls, [0.0])
        sleep.assert_called_once_with(0.2)
        run_process.assert_called_once_with(
            [sys.executable, "/tmp/test-reset.py", "--mode", "task"],
            check=False,
            text=True,
            capture_output=True,
        )
        self.assertFalse(controller.armed)
        self.assertEqual(controller.joint_positions, {})
        self.assertEqual(io.last_state_monotonic, 0.0)

    @patch("pico_hand_arm.time.sleep")
    @patch("pico_hand_arm.subprocess.run")
    def test_task_reset_failure_stays_disarmed(self, run_process, _sleep):
        run_process.return_value = SimpleNamespace(
            returncode=7, stdout="", stderr="reset unavailable"
        )
        io, controller = self.make_io()

        with self.assertRaisesRegex(RuntimeError, "reset unavailable"):
            io.reset_scene_and_robot(reset_script="/tmp/test-reset.py")

        self.assertFalse(controller.armed)
        self.assertEqual(controller.joint_positions, {})
        self.assertEqual(io.last_state_monotonic, 0.0)


if __name__ == "__main__":
    unittest.main()
