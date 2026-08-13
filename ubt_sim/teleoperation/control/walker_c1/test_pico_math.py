import unittest

import numpy as np

from pico_math import (
    ControllerPoseLiveness,
    Pose,
    horizontal_heading,
    pose7_to_robot,
    relative_target,
    slew,
    yaw_pitch_delta,
)
from pico_source import MockPicoSource


class PicoMathTest(unittest.TestCase):
    def test_controller_pose_liveness_requires_one_change_then_allows_stillness(self):
        monitor = ControllerPoseLiveness()
        pose = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        self.assertFalse(monitor.update(pose, now=1.0))
        changed = [0.1001, *pose[1:]]
        self.assertTrue(monitor.update(changed, now=1.1))
        self.assertTrue(monitor.update(changed, now=60.0))

    def test_pose_basis_conversion(self):
        basis = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
        pose = pose7_to_robot([1, 2, 3, 0, 0, 0, 1], basis)
        np.testing.assert_allclose(pose.position, [-3, -1, 2])
        np.testing.assert_allclose(pose.rotation, np.eye(3), atol=1e-12)

    def test_openxr_axes_map_forward_right_and_up(self):
        basis = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
        np.testing.assert_allclose(basis @ np.array([0.0, 0.0, -1.0]), [1.0, 0.0, 0.0])
        np.testing.assert_allclose(basis @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0])
        np.testing.assert_allclose(basis @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])

    def test_openxr_left_yaw_becomes_robot_left_yaw(self):
        basis = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
        angle = np.pi / 2
        openxr_yaw_xyzw = [0.0, np.sin(angle / 2), 0.0, np.cos(angle / 2)]
        pose = pose7_to_robot([0, 0, 0, *openxr_yaw_xyzw], basis)
        expected_robot_yaw = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(pose.rotation, expected_robot_yaw, atol=1e-12)

    def test_rejects_improper_tracking_basis(self):
        old_left_handed_basis = [[0, 0, 1], [-1, 0, 0], [0, 1, 0]]
        with self.assertRaisesRegex(ValueError, "proper rotation"):
            pose7_to_robot([0, 0, 0, 0, 0, 0, 1], old_left_handed_basis)

    def test_relative_target_is_clamped(self):
        identity = np.eye(3)
        anchor = Pose(np.array([0.3, 0.2, 0.5]), identity)
        controller_anchor = Pose(np.zeros(3), identity)
        controller_now = Pose(np.array([1.0, 0.0, 0.0]), identity)
        target = relative_target(
            anchor, controller_anchor, controller_now, 1.0, 0.2,
            [[-1, 1], [-1, 1], [0.4, 0.6]],
        )
        np.testing.assert_allclose(target.position, [0.5, 0.2, 0.5])

    def test_heading_ignores_head_pitch(self):
        yaw_angle = 0.7
        pitch_angle = -0.4
        yaw = np.array([
            [np.cos(yaw_angle), -np.sin(yaw_angle), 0.0],
            [np.sin(yaw_angle), np.cos(yaw_angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        pitch = np.array([
            [np.cos(pitch_angle), 0.0, np.sin(pitch_angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(pitch_angle), 0.0, np.cos(pitch_angle)],
        ])
        np.testing.assert_allclose(
            horizontal_heading(Pose(np.zeros(3), yaw @ pitch)), yaw, atol=1e-12
        )

    def test_operator_heading_keeps_body_right_as_robot_right(self):
        # The operator faces tracking-world +Y (90 deg left from world +X).
        # Their body-right motion is therefore tracking-world +X, the exact
        # case that used to make the robot move forward instead of right.
        heading = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        identity = np.eye(3)
        robot_anchor = Pose(np.array([0.3, 0.2, 0.5]), identity)
        controller_anchor = Pose(np.zeros(3), identity)
        controller_now = Pose(np.array([0.1, 0.0, 0.0]), identity)
        target = relative_target(
            robot_anchor,
            controller_anchor,
            controller_now,
            1.0,
            1.0,
            [[-1, 1], [-1, 1], [-1, 1]],
            heading,
        )
        np.testing.assert_allclose(target.position, [0.3, 0.1, 0.5], atol=1e-12)

    def test_operator_heading_preserves_controller_roll_axis_and_sign(self):
        heading = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        angle = 0.3
        roll_operator = np.array([
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle), -np.sin(angle)],
            [0.0, np.sin(angle), np.cos(angle)],
        ])
        delta_world = heading @ roll_operator @ heading.T
        identity = np.eye(3)
        target = relative_target(
            Pose(np.zeros(3), identity),
            Pose(np.zeros(3), identity),
            Pose(np.zeros(3), delta_world),
            1.0,
            1.0,
            [[-1, 1], [-1, 1], [-1, 1]],
            heading,
        )
        np.testing.assert_allclose(target.rotation, roll_operator, atol=1e-12)

    def test_head_pitch_is_measured_in_operator_heading(self):
        heading = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        pitch_angle = 0.25
        pitch_operator = np.array([
            [np.cos(pitch_angle), 0.0, np.sin(pitch_angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(pitch_angle), 0.0, np.cos(pitch_angle)],
        ])
        anchor = Pose(np.zeros(3), heading)
        current = Pose(np.zeros(3), heading @ pitch_operator)
        yaw, pitch = yaw_pitch_delta(anchor, current, heading)
        self.assertAlmostEqual(yaw, 0.0)
        self.assertAlmostEqual(pitch, pitch_angle)

    def test_head_yaw(self):
        angle = 0.4
        yaw = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ])
        result, pitch = yaw_pitch_delta(Pose(np.zeros(3), np.eye(3)), Pose(np.zeros(3), yaw))
        self.assertAlmostEqual(result, angle)
        self.assertAlmostEqual(pitch, 0.0)

    def test_slew(self):
        self.assertEqual(slew([0.0, 1.0], [1.0, -1.0], 0.1), [0.1, 0.9])

    def test_mock_source_matches_sdk_frame_contract(self):
        frame = MockPicoSource().read()
        self.assertEqual(len(frame.headset_pose), 7)
        self.assertEqual(len(frame.left_controller_pose), 7)
        self.assertEqual(len(frame.right_controller_pose), 7)
        self.assertTrue(frame.controls["RightController"]["key_two"])
        self.assertGreater(frame.timestamp_ns, 0)


if __name__ == "__main__":
    unittest.main()
