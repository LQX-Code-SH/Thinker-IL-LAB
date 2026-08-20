#!/usr/bin/env python3
from __future__ import annotations

import os
import unittest
from unittest.mock import patch

import numpy as np

from pico_hand_live import (
    SimulationCommandGate,
    format_live_results,
    initial_open_hand_ready,
    parse_args,
    results_to_payload,
    retarget_live_hands,
    validate_args,
)
from pico_hand_mock import generate_mock_hand
from pico_hand_retarget import C1HandRetargeter


class SimulationCommandGateTest(unittest.TestCase):
    def test_gate_requires_explicit_arm_and_continuous_valid_dwell(self):
        gate = SimulationCommandGate(("left", "right"), valid_dwell_s=0.5)
        valid = {"left": True, "right": True}
        self.assertFalse(gate.update(valid, 0.0).allow_publish)
        self.assertTrue(gate.request_arm(valid, 1.0))
        self.assertEqual(gate.state, "pending")
        self.assertFalse(gate.update(valid, 1.49).allow_publish)
        update = gate.update(valid, 1.50)
        self.assertTrue(update.just_armed)
        self.assertTrue(update.allow_publish)
        self.assertEqual(gate.state, "armed")

    def test_arm_refuses_when_any_selected_hand_is_invalid(self):
        gate = SimulationCommandGate(("left", "right"))
        self.assertFalse(gate.request_arm({"left": True, "right": False}, 0.0))
        self.assertEqual(gate.state, "disarmed")

    def test_tracking_loss_stops_immediately_then_disarms(self):
        gate = SimulationCommandGate(
            ("right",), valid_dwell_s=0.0, tracking_timeout_s=0.30
        )
        self.assertTrue(gate.request_arm({"right": True}, 0.0))
        self.assertTrue(gate.update({"right": True}, 0.0).allow_publish)
        first_loss = gate.update({"right": False}, 1.0)
        self.assertFalse(first_loss.allow_publish)
        self.assertEqual(gate.state, "armed")
        timeout = gate.update({"right": False}, 1.30)
        self.assertFalse(timeout.allow_publish)
        self.assertTrue(timeout.just_disarmed)
        self.assertEqual(gate.state, "disarmed")
        self.assertFalse(gate.update({"right": True}, 2.0).allow_publish)

    def test_operator_disarm_never_auto_rearms(self):
        gate = SimulationCommandGate(("right",), valid_dwell_s=0.0)
        gate.request_arm({"right": True}, 0.0)
        gate.update({"right": True}, 0.0)
        self.assertTrue(gate.disarm())
        self.assertFalse(gate.update({"right": True}, 1.0).allow_publish)


class LiveRetargetTest(unittest.TestCase):
    def setUp(self):
        self.mapper = C1HandRetargeter(
            smoothing_enabled=False,
            velocity_limits_enabled=False,
        )

    def test_both_live_hands_produce_full_named_11d_payload(self):
        hands = {
            "left": generate_mock_hand("tripod", side="left"),
            "right": generate_mock_hand("index_flex", side="right"),
        }
        results = retarget_live_hands(hands, ("left", "right"), self.mapper, 0.02)
        payload = results_to_payload(results)
        self.assertEqual(set(payload), {"left_hand", "right_hand"})
        self.assertEqual(list(payload["left_hand"]), list(self.mapper.joint_names("left")))
        self.assertEqual(list(payload["right_hand"]), list(self.mapper.joint_names("right")))
        self.assertEqual(len(payload["left_hand"]), 11)
        self.assertEqual(len(payload["right_hand"]), 11)
        self.assertTrue(np.all(np.isfinite(list(payload["right_hand"].values()))))

    def test_invalid_selected_hand_makes_payload_empty(self):
        hands = {
            "left": generate_mock_hand("open_hand", side="left"),
            "right": generate_mock_hand("fist", side="right", is_tracked=False),
        }
        results = retarget_live_hands(hands, ("left", "right"), self.mapper, 0.02)
        self.assertEqual(results_to_payload(results), {})
        self.assertIn("tracking invalid", format_live_results(results, mode="preview"))

    def test_preview_rejects_command_flag(self):
        args = parse_args(["--mode", "preview", "--enable-command"])
        with self.assertRaisesRegex(ValueError, "only valid with --mode sim"):
            validate_args(args)

    def test_sim_requires_double_ack_and_sim_domain(self):
        args = parse_args(["--mode", "sim"])
        with self.assertRaisesRegex(ValueError, "requires explicit"):
            validate_args(args)

        args = parse_args(["--mode", "sim", "--enable-command"])
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "0"}, clear=False):
            with self.assertRaisesRegex(RuntimeError, "possible real-robot domain"):
                validate_args(args)
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "146"}, clear=False):
            self.assertEqual(validate_args(args), ("left", "right"))

    def test_first_hand_arm_version_is_explicitly_right_only(self):
        preview = parse_args(["--control", "hand-arm", "--side", "right"])
        with self.assertRaisesRegex(ValueError, "requires --mode sim"):
            validate_args(preview)

        both = parse_args([
            "--control", "hand-arm", "--mode", "sim", "--enable-command",
            "--side", "both",
        ])
        with self.assertRaisesRegex(ValueError, "right-side only"):
            with patch.dict(os.environ, {"ROS_DOMAIN_ID": "146"}, clear=False):
                validate_args(both)

        right = parse_args([
            "--control", "hand-arm", "--mode", "sim", "--enable-command",
            "--side", "right",
        ])
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "0"}, clear=False):
            with self.assertRaisesRegex(RuntimeError, "possible real-robot domain"):
                validate_args(right)
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "146"}, clear=False):
            self.assertEqual(validate_args(right), ("right",))

    def test_recording_is_only_available_for_right_hand_arm_simulation(self):
        hand_only = parse_args([
            "--control", "hand", "--mode", "sim", "--enable-command",
            "--record",
        ])
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "146"}, clear=False):
            with self.assertRaisesRegex(ValueError, "requires --control hand-arm"):
                validate_args(hand_only)

        hand_arm = parse_args([
            "--control", "hand-arm", "--mode", "sim", "--side", "right",
            "--enable-command", "--record",
        ])
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "146"}, clear=False):
            self.assertEqual(validate_args(hand_arm), ("right",))

    def test_hand_arm_anchor_requires_an_initially_open_hand(self):
        mapper = C1HandRetargeter(smoothing_enabled=False, velocity_limits_enabled=False)
        open_results = retarget_live_hands(
            {"right": generate_mock_hand("open_hand", side="right")},
            ("right",),
            mapper,
            0.02,
        )
        self.assertTrue(initial_open_hand_ready(open_results, ("right",), 0.45)[0])

        fist_results = retarget_live_hands(
            {"right": generate_mock_hand("fist", side="right")},
            ("right",),
            mapper,
            0.02,
        )
        ready, reason = initial_open_hand_ready(fist_results, ("right",), 0.45)
        self.assertFalse(ready)
        self.assertIn("must be open", reason)


if __name__ == "__main__":
    unittest.main()
