#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import patch

import numpy as np

from pico_hand_mock import MOCK_HAND_POSES, generate_mock_hand
from pico_hand_offline_demo import DEMO_SEQUENCE, make_pose_payload, require_sim_ros_domain
from pico_hand_retarget import (
    DEFAULT_CONFIG_PATH,
    C1HandRetargeter,
    extract_finger_geometry,
    load_retarget_config,
)
from pico_hand_tracking import get_thumb_opposition, retarget_hand


class C1HandAssetModelTest(unittest.TestCase):
    def test_verified_main_usd_control_order_and_limits(self):
        config = load_retarget_config()
        self.assertTrue(config["asset"]["usd_path"].endswith("walker_astron_v1_sensorKpkd_hands.usd"))
        self.assertEqual(
            config["joint_order"],
            [
                "thumb_cmp", "thumb_mpp", "thumb_ip",
                "index_mpp", "index_ip", "middle_mpp", "middle_ip",
                "ring_mpp", "ring_ip", "little_mpp", "little_ip",
            ],
        )
        mapper = C1HandRetargeter()
        self.assertEqual(
            mapper.joint_names("left"),
            (
                "L_thumb_cmp_joint", "L_thumb_mpp_joint", "L_thumb_ip_joint",
                "L_index_mpp_joint", "L_index_ip_joint",
                "L_middle_mpp_joint", "L_middle_ip_joint",
                "L_ring_mpp_joint", "L_ring_ip_joint",
                "L_little_mpp_joint", "L_little_ip_joint",
            ),
        )
        left_lower, left_upper = mapper.joint_limits("left")
        right_lower, right_upper = mapper.joint_limits("right")
        np.testing.assert_allclose(left_lower, [-0.96] + [0.0] * 10)
        np.testing.assert_allclose(left_upper, [0.0, 1.04, 1.05, 1.46, 1.62, 1.46, 1.62, 1.46, 1.62, 1.46, 1.62])
        np.testing.assert_allclose(right_lower, np.zeros(11))
        np.testing.assert_allclose(right_upper, [0.96, 1.04, 1.05, 1.46, 1.62, 1.46, 1.62, 1.46, 1.62, 1.46, 1.62])
        np.testing.assert_allclose(mapper.velocity_limits("left"), np.full(11, 2.09))

    def test_all_tuning_parameters_are_standalone_json(self):
        config = json.loads(Path(DEFAULT_CONFIG_PATH).read_text(encoding="utf-8"))
        for joint in config["joints"].values():
            for key in ("sign", "scale", "offset_rad", "joint_range_rad", "velocity_limit_rad_s"):
                self.assertIn(key, joint)
        self.assertIn("smoothing", config["mapping"])
        self.assertIn("flexion_sensitivity", config["mapping"])


class MockHandRetargetTest(unittest.TestCase):
    def setUp(self):
        self.mapper = C1HandRetargeter(
            smoothing_enabled=False,
            velocity_limits_enabled=False,
        )

    def assertWithinLimits(self, command: np.ndarray, side: str):
        lower, upper = self.mapper.joint_limits(side)
        self.assertEqual(command.shape, (11,))
        self.assertTrue(np.all(np.isfinite(command)))
        self.assertTrue(np.all(command >= lower - 1e-12))
        self.assertTrue(np.all(command <= upper + 1e-12))

    def test_every_mock_pose_outputs_limited_11d_for_both_hands(self):
        self.assertEqual(len(MOCK_HAND_POSES), 8)
        for side in ("left", "right"):
            for pose in MOCK_HAND_POSES:
                with self.subTest(side=side, pose=pose):
                    command = self.mapper.retarget(generate_mock_hand(pose, side=side))
                    self.assertWithinLimits(command, side)

    def test_open_and_fist_are_clearly_different(self):
        opened = self.mapper.retarget(generate_mock_hand("open_hand"))
        fist = self.mapper.retarget(generate_mock_hand("fist"))
        np.testing.assert_allclose(opened, np.zeros(11), atol=1e-10)
        self.assertGreater(float(np.linalg.norm(fist - opened)), 3.0)
        self.assertTrue(np.all(fist > 0.4))

    def test_index_flex_primarily_moves_index_pair(self):
        opened = self.mapper.retarget(generate_mock_hand("open_hand"))
        index = self.mapper.retarget(generate_mock_hand("index_flex"))
        delta = np.abs(index - opened)
        self.assertGreater(float(np.sum(delta[3:5])), 2.0)
        np.testing.assert_allclose(delta[[0, 1, 2, 5, 6, 7, 8, 9, 10]], 0.0, atol=1e-10)

    def test_ring_flex_only_moves_ring_pair(self):
        opened = self.mapper.retarget(generate_mock_hand("open_hand"))
        ring = self.mapper.retarget(generate_mock_hand("ring_flex"))
        delta = np.abs(ring - opened)
        self.assertGreater(float(np.sum(delta[7:9])), 2.0)
        np.testing.assert_allclose(delta[[0, 1, 2, 3, 4, 5, 6, 9, 10]], 0.0, atol=1e-10)

    def test_little_flex_only_moves_little_pair(self):
        opened = self.mapper.retarget(generate_mock_hand("open_hand"))
        little = self.mapper.retarget(generate_mock_hand("little_flex"))
        delta = np.abs(little - opened)
        self.assertGreater(float(np.sum(delta[9:11])), 2.0)
        np.testing.assert_allclose(delta[:9], 0.0, atol=1e-10)

    def test_pinch_primarily_moves_thumb_and_index(self):
        opened = self.mapper.retarget(generate_mock_hand("open_hand"))
        pinch_hand = generate_mock_hand("thumb_index_pinch")
        pinch = self.mapper.retarget(pinch_hand)
        delta = np.abs(pinch - opened)
        self.assertGreater(float(np.sum(delta[:5])), 3.0)
        np.testing.assert_allclose(delta[5:], 0.0, atol=1e-10)
        self.assertLess(get_thumb_opposition(pinch_hand), 0.01)

    def test_tripod_adds_middle_and_cylindrical_closes_all_fingers(self):
        tripod = self.mapper.retarget(generate_mock_hand("tripod"))
        cylinder = self.mapper.retarget(generate_mock_hand("cylindrical_grasp"))
        self.assertTrue(np.all(tripod[:7] > 0.2))
        np.testing.assert_allclose(tripod[7:], 0.0, atol=1e-10)
        self.assertTrue(np.all(cylinder > 0.4))

    def test_human_joint_angles_are_compressed_not_copied(self):
        hand = generate_mock_hand("fist")
        geometry = extract_finger_geometry(hand, "index", self.mapper.config)
        command = self.mapper.retarget(hand)
        self.assertEqual(geometry.bend_angles_rad.shape, (3,))
        self.assertFalse(np.isclose(command[3], geometry.bend_angles_rad[0]))
        self.assertFalse(np.isclose(command[4], geometry.bend_angles_rad[1]))

    def test_left_thumb_opposition_uses_mirrored_usd_sign(self):
        left = self.mapper.retarget(generate_mock_hand("thumb_index_pinch", side="left"))
        right = self.mapper.retarget(generate_mock_hand("thumb_index_pinch", side="right"))
        self.assertLess(left[0], -0.9)
        self.assertGreater(right[0], 0.9)
        np.testing.assert_allclose(left[1:], right[1:], atol=1e-10)

    def test_public_retarget_helper_returns_numpy_11d(self):
        command = retarget_hand(
            generate_mock_hand("tripod"),
            retargeter=self.mapper,
        )
        self.assertIsInstance(command, np.ndarray)
        self.assertEqual(command.shape, (11,))


class RetargetSafetyFilterTest(unittest.TestCase):
    def test_invalid_tracking_holds_previous_valid_command_exactly(self):
        mapper = C1HandRetargeter()
        valid = mapper.retarget(generate_mock_hand("fist"), dt_s=0.02)
        invalid = mapper.retarget(
            generate_mock_hand("open_hand", is_tracked=False),
            dt_s=0.02,
        )
        np.testing.assert_array_equal(invalid, valid)

    def test_invalid_before_first_valid_frame_falls_back_to_open(self):
        mapper = C1HandRetargeter()
        command = mapper.retarget(generate_mock_hand("fist", is_tracked=False))
        np.testing.assert_allclose(command, mapper.open_command("right"))

    def test_velocity_limit_bounds_each_valid_transition(self):
        mapper = C1HandRetargeter(smoothing_enabled=False, velocity_limits_enabled=True)
        opened = mapper.retarget(generate_mock_hand("open_hand"), dt_s=0.01)
        fist = mapper.retarget(generate_mock_hand("fist"), dt_s=0.01)
        maximum = mapper.velocity_limits("right") * 0.01
        self.assertTrue(np.all(np.abs(fist - opened) <= maximum + 1e-12))

    def test_first_valid_frame_is_rate_limited_from_open(self):
        mapper = C1HandRetargeter(smoothing_enabled=False, velocity_limits_enabled=True)
        fist = mapper.retarget(generate_mock_hand("fist"), dt_s=0.01)
        opened = mapper.open_command("right")
        maximum = mapper.velocity_limits("right") * 0.01
        self.assertTrue(np.all(np.abs(fist - opened) <= maximum + 1e-12))

    def test_smoothing_reduces_step_before_velocity_limit(self):
        filtered = C1HandRetargeter(smoothing_enabled=True, velocity_limits_enabled=False)
        direct = C1HandRetargeter(smoothing_enabled=False, velocity_limits_enabled=False)
        filtered.retarget(generate_mock_hand("open_hand"), dt_s=0.02)
        smoothed = filtered.retarget(generate_mock_hand("fist"), dt_s=0.02)
        raw = direct.retarget(generate_mock_hand("fist"), dt_s=0.02)
        self.assertGreater(float(np.linalg.norm(smoothed)), 0.0)
        self.assertLess(float(np.linalg.norm(smoothed)), float(np.linalg.norm(raw)))

    def test_optional_open_fallback_closes_no_faster_than_velocity_limit(self):
        mapper = C1HandRetargeter(
            smoothing_enabled=False,
            velocity_limits_enabled=True,
            invalid_behavior="open",
        )
        fist = mapper.retarget(generate_mock_hand("fist"), dt_s=0.01)
        fallback = mapper.retarget(generate_mock_hand("open_hand", is_tracked=False), dt_s=0.01)
        maximum = mapper.velocity_limits("right") * 0.01
        self.assertTrue(np.all(np.abs(fallback - fist) <= maximum + 1e-12))
        self.assertLess(float(np.linalg.norm(fallback)), float(np.linalg.norm(fist)))

    def test_corrupt_joint_is_treated_as_tracking_loss(self):
        mapper = C1HandRetargeter()
        prior = mapper.retarget(generate_mock_hand("tripod"))
        corrupt = generate_mock_hand("fist")
        corrupt.joint_positions["index_tip"][0] = np.nan
        held = mapper.retarget(corrupt)
        np.testing.assert_array_equal(held, prior)


class OfflineDemoPayloadTest(unittest.TestCase):
    def test_ros_transport_refuses_non_simulation_domain(self):
        with patch.dict("os.environ", {"ROS_DOMAIN_ID": "146"}, clear=False):
            require_sim_ros_domain(146)
        with patch.dict("os.environ", {"ROS_DOMAIN_ID": "0"}, clear=False):
            with self.assertRaisesRegex(RuntimeError, "possible real-robot domain"):
                require_sim_ros_domain(146)

    def test_sequence_and_payload_use_full_named_11d_commands(self):
        self.assertEqual(
            DEMO_SEQUENCE,
            (
                "open_hand", "fist", "open_hand", "thumb_index_pinch", "tripod",
                "open_hand", "ring_flex", "open_hand", "little_flex", "open_hand",
                "cylindrical_grasp",
            ),
        )
        mapper = C1HandRetargeter(smoothing_enabled=False, velocity_limits_enabled=False)
        payload = make_pose_payload("tripod", ("left", "right"), mapper, 0.02)
        self.assertEqual(set(payload), {"left_hand", "right_hand"})
        self.assertEqual(list(payload["left_hand"]), list(mapper.joint_names("left")))
        self.assertEqual(list(payload["right_hand"]), list(mapper.joint_names("right")))
        self.assertEqual(len(payload["left_hand"]), 11)
        self.assertEqual(len(payload["right_hand"]), 11)

    def test_existing_action_process_accepts_demo_as_full_11d_dict(self):
        mapper = C1HandRetargeter(smoothing_enabled=False, velocity_limits_enabled=False)
        payload = make_pose_payload("cylindrical_grasp", ("right",), mapper, 0.02)

        fake_config = ModuleType("ubt_sim.devices.walker_c1.config")
        fake_config.WALKER_C1_LEFT_HAND_JOINTS = list(mapper.joint_names("left"))
        fake_config.WALKER_C1_RIGHT_HAND_JOINTS = list(mapper.joint_names("right"))
        fake_config.WALKER_C1_HEAD_JOINTS = []
        fake_config.WALKER_C1_WAIST_JOINTS = []
        fake_config.WALKER_C1_LEFT_ARM_JOINTS = []
        fake_config.WALKER_C1_RIGHT_ARM_JOINTS = []
        fake_config.WALKER_C1_LEFT_LEG_JOINTS = []
        fake_config.WALKER_C1_RIGHT_LEG_JOINTS = []
        all_hand_names = mapper.joint_names("left") + mapper.joint_names("right")
        fake_config.WALKER_C1_HOME_POSE = {name: 0.0 for name in all_hand_names}

        fake_torch = ModuleType("torch")
        fake_torch.Tensor = type("Tensor", (), {})
        module_stubs = {
            "torch": fake_torch,
            "ubt_sim": ModuleType("ubt_sim"),
            "ubt_sim.devices": ModuleType("ubt_sim.devices"),
            "ubt_sim.devices.walker_c1": ModuleType("ubt_sim.devices.walker_c1"),
            "ubt_sim.devices.walker_c1.config": fake_config,
        }
        repo_root = Path(__file__).resolve().parents[4]
        action_path = repo_root / "ubt_sim/source/ubt_sim/devices/walker_c1/action_process.py"
        spec = importlib.util.spec_from_file_location("_retarget_action_process_test", action_path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, module_stubs):
            spec.loader.exec_module(module)

        targets = module._extract_command_targets(payload)
        self.assertEqual(list(targets), list(mapper.joint_names("right")))
        self.assertEqual(targets, payload["right_hand"])


if __name__ == "__main__":
    unittest.main()
