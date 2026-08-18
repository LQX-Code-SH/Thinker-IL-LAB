#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from pico_hand_tracking import (
    OPENXR_HAND_JOINT_NAMES,
    get_finger_flexion,
    get_fingertip_positions,
    get_thumb_opposition,
    parse_hand_tracking_state,
    retarget_hand,
)
from pico_math import quaternion_xyzw_to_matrix
from pico_source import PicoSource


def _raw_hand() -> np.ndarray:
    poses = np.zeros((26, 7), dtype=float)
    poses[:, 0] = np.arange(26, dtype=float) * 0.01
    poses[:, 1] = 0.20
    poses[:, 2] = -0.30
    poses[:, 6] = 1.0
    return poses


class PicoHandTrackingTest(unittest.TestCase):
    def test_openxr_order_uses_palm_zero_and_wrist_one(self):
        raw = _raw_hand()
        raw[0, :3] = [1.0, 2.0, 3.0]
        raw[1, :3] = [4.0, 5.0, 6.0]

        hand = parse_hand_tracking_state("left", raw, 1, 123)

        self.assertEqual(len(OPENXR_HAND_JOINT_NAMES), 26)
        self.assertEqual(OPENXR_HAND_JOINT_NAMES[0], "palm")
        self.assertEqual(OPENXR_HAND_JOINT_NAMES[1], "wrist")
        np.testing.assert_allclose(hand.palm_pose.position_m, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(hand.wrist_pose.position_m, [4.0, 5.0, 6.0])
        self.assertEqual(hand.quaternion_order, "xyzw")
        self.assertEqual(hand.position_unit, "meter")
        self.assertTrue(hand.is_tracked)

    def test_hand_uses_the_proven_controller_basis_for_pose(self):
        raw = _raw_hand()
        raw[1, :3] = [0.20, 0.30, -0.40]
        angle = np.deg2rad(70.0)
        raw[1, 3:] = [0.0, np.sin(angle / 2.0), 0.0, np.cos(angle / 2.0)]
        basis = np.asarray(
            [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        )

        hand = parse_hand_tracking_state("right", raw, 1, 456, basis)

        np.testing.assert_allclose(hand.wrist_pose.position_m, [0.40, -0.20, 0.30])
        raw_rotation = quaternion_xyzw_to_matrix(raw[1, 3:])
        converted_rotation = quaternion_xyzw_to_matrix(
            hand.wrist_pose.quaternion_xyzw
        )
        np.testing.assert_allclose(
            converted_rotation, basis @ raw_rotation @ basis.T, atol=1e-7
        )
        self.assertIn("+X forward", hand.reference_space)

    def test_inactive_or_invalid_skeleton_is_not_safe_to_consume(self):
        inactive = parse_hand_tracking_state("left", _raw_hand(), 0, 10)
        self.assertFalse(inactive.is_tracked)
        self.assertEqual(get_fingertip_positions(inactive), {})

        raw = _raw_hand()
        raw[10, 3:] = 0.0
        partial = parse_hand_tracking_state("left", raw, 1, 11)
        self.assertFalse(partial.is_tracked)
        self.assertFalse(partial.joint_validity["index_tip"])
        self.assertNotIn("index_tip", get_fingertip_positions(partial))

    def test_flexion_and_thumb_opposition_are_geometric_observables(self):
        raw = _raw_hand()
        index_indices = [6, 7, 8, 9, 10]
        straight = [
            [0.00, 0.00, 0.00],
            [0.01, 0.00, 0.00],
            [0.02, 0.00, 0.00],
            [0.03, 0.00, 0.00],
            [0.04, 0.00, 0.00],
        ]
        for index, position in zip(index_indices, straight):
            raw[index, :3] = position
        raw[5, :3] = [0.00, 0.03, 0.00]
        hand_straight = parse_hand_tracking_state("right", raw, 1, 20)

        raw_bent = raw.copy()
        bent = [
            [0.00, 0.00, 0.00],
            [0.01, 0.00, 0.00],
            [0.02, 0.00, 0.00],
            [0.02, 0.01, 0.00],
            [0.02, 0.02, 0.00],
        ]
        for index, position in zip(index_indices, bent):
            raw_bent[index, :3] = position
        raw_bent[5, :3] = [0.02, 0.021, 0.00]
        hand_bent = parse_hand_tracking_state("right", raw_bent, 1, 21)

        self.assertAlmostEqual(get_finger_flexion(hand_straight, "index"), 0.0)
        self.assertGreater(get_finger_flexion(hand_bent, "index"), 1.4)
        self.assertLess(
            get_thumb_opposition(hand_bent),
            get_thumb_opposition(hand_straight),
        )

    def test_serialization_retains_all_named_joints(self):
        hand = parse_hand_tracking_state("left", _raw_hand(), 1, 99)
        payload = hand.as_dict()
        self.assertEqual(len(payload["joints"]), 26)
        self.assertEqual(
            [joint["name"] for joint in payload["joints"]],
            list(OPENXR_HAND_JOINT_NAMES),
        )
        robot_hand_q = retarget_hand(hand)
        self.assertEqual(robot_hand_q.shape, (11,))
        self.assertTrue(np.all(np.isfinite(robot_hand_q)))


def _fake_sdk():
    pose = [0.0, 1.0, -0.5, 0.0, 0.0, 0.0, 1.0]
    hands = _raw_hand().tolist()
    return SimpleNamespace(
        init=Mock(),
        close=Mock(),
        get_left_trigger=Mock(return_value=0.0),
        get_left_grip=Mock(return_value=0.0),
        get_X_button=Mock(return_value=False),
        get_Y_button=Mock(return_value=False),
        get_left_axis=Mock(return_value=[0.0, 0.0]),
        get_left_axis_click=Mock(return_value=False),
        get_right_trigger=Mock(return_value=0.0),
        get_right_grip=Mock(return_value=0.0),
        get_A_button=Mock(return_value=False),
        get_B_button=Mock(return_value=False),
        get_right_axis=Mock(return_value=[0.0, 0.0]),
        get_right_axis_click=Mock(return_value=False),
        get_time_stamp_ns=Mock(return_value=123456),
        get_headset_pose=Mock(return_value=pose),
        get_left_controller_pose=Mock(return_value=pose),
        get_right_controller_pose=Mock(return_value=pose),
        get_left_hand_tracking_state=Mock(return_value=hands),
        get_right_hand_tracking_state=Mock(return_value=hands),
        get_left_hand_is_active=Mock(return_value=1),
        get_right_hand_is_active=Mock(return_value=1),
    )


class PicoSourceCompatibilityTest(unittest.TestCase):
    def test_hand_reads_are_opt_in_and_controller_frame_stays_compatible(self):
        sdk = _fake_sdk()
        with patch.dict(sys.modules, {"xrobotoolkit_sdk": sdk}):
            controller_only = PicoSource()
            frame = controller_only.read()
            controller_only.close()

        self.assertIsNone(frame.left_hand)
        self.assertIsNone(frame.right_hand)
        sdk.get_left_hand_tracking_state.assert_not_called()
        sdk.get_right_hand_tracking_state.assert_not_called()
        self.assertEqual(len(frame.left_controller_pose), 7)

        sdk = _fake_sdk()
        with patch.dict(sys.modules, {"xrobotoolkit_sdk": sdk}):
            with_hands = PicoSource(hand_tracking_enabled=True)
            frame = with_hands.read()
            with_hands.close()

        self.assertTrue(frame.left_hand.is_tracked)
        self.assertTrue(frame.right_hand.is_tracked)
        self.assertEqual(frame.left_hand.wrist_pose.name, "wrist")
        sdk.get_left_hand_tracking_state.assert_called_once_with()
        sdk.get_right_hand_tracking_state.assert_called_once_with()

    def test_hand_only_read_does_not_require_controller_getters(self):
        sdk = _fake_sdk()
        with patch.dict(sys.modules, {"xrobotoolkit_sdk": sdk}):
            source = PicoSource(hand_tracking_enabled=True)
            left, right = source.read_hands()
            source.close()

        self.assertTrue(left.is_tracked)
        self.assertTrue(right.is_tracked)
        sdk.get_headset_pose.assert_not_called()
        sdk.get_left_controller_pose.assert_not_called()
        sdk.get_right_controller_pose.assert_not_called()
        sdk.get_left_trigger.assert_not_called()
        sdk.get_right_trigger.assert_not_called()


if __name__ == "__main__":
    unittest.main()
