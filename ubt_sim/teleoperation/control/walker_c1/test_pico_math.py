import unittest

import numpy as np

from pico_math import Pose, pose7_to_robot, relative_target, slew, yaw_pitch_delta
from pico_source import MockPicoSource


class PicoMathTest(unittest.TestCase):
    def test_pose_basis_conversion(self):
        basis = [[0, 0, 1], [-1, 0, 0], [0, 1, 0]]
        pose = pose7_to_robot([1, 2, 3, 0, 0, 0, 1], basis)
        np.testing.assert_allclose(pose.position, [3, -1, 2])
        np.testing.assert_allclose(pose.rotation, np.eye(3), atol=1e-12)

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
