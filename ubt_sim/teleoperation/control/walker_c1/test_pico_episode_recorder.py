"""Offline HDF5 format checks for PICO episode recording."""
from __future__ import annotations

import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import h5py
import numpy as np

from pico_episode_recorder import BUFFER_KEYS, PicoEpisodeRecorder


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


class PicoEpisodeRecorderFormatTest(unittest.TestCase):
    def test_space_uses_the_same_completion_entry_point(self):
        recorder = object.__new__(PicoEpisodeRecorder)
        recorder.request_complete = Mock(return_value=True)

        recorder._complete_callback(SimpleNamespace(data=True))

        recorder.request_complete.assert_called_once_with("Space key")

    def test_episode_snapshot_is_consumed_only_once(self):
        recorder = object.__new__(PicoEpisodeRecorder)
        recorder._lock = threading.Lock()
        recorder.active = True
        recorder.finishing = False
        recorder.last_record_stamp = 1.0
        recorder.buffers = {key: [key] for key in BUFFER_KEYS}

        episode = recorder._take_episode_snapshot()

        self.assertEqual(episode, {key: [key] for key in BUFFER_KEYS})
        self.assertFalse(recorder.active)
        self.assertTrue(recorder.finishing)
        self.assertIsNone(recorder.last_record_stamp)
        self.assertTrue(all(not values for values in recorder.buffers.values()))
        self.assertIsNone(recorder._take_episode_snapshot())

    def test_recording_gate_requires_full_hand_body_and_recent_camera_state(self):
        recorder = object.__new__(PicoEpisodeRecorder)
        recorder.body_positions = {}
        recorder.left_hand_positions = {}
        recorder.right_hand_positions = {}
        recorder.hand_joint_names = {
            "left": ("L0", "L1"),
            "right": ("R0", "R1"),
        }
        recorder.camera_topic = "/camera"
        recorder.last_camera_monotonic = 0.0

        ready, reason = recorder.inputs_ready()
        self.assertFalse(ready)
        self.assertIn("arm state", reason)

        from constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES
        recorder.body_positions.update(
            {name: 0.0 for name in LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES}
        )
        ready, reason = recorder.inputs_ready()
        self.assertFalse(ready)
        self.assertIn("left hand", reason)

        recorder.left_hand_positions.update({"L0": 0.0, "L1": 0.0})
        recorder.right_hand_positions.update({"R0": 0.0, "R1": 0.0})
        ready, reason = recorder.inputs_ready()
        self.assertFalse(ready)
        self.assertIn("camera", reason)

        recorder.last_camera_monotonic = time.monotonic()
        self.assertEqual(recorder.inputs_ready(), (True, ""))

    def test_saved_episode_matches_lerobot_hdf5_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = object.__new__(PicoEpisodeRecorder)
            recorder.node = _Node()
            recorder.record_root = Path(directory)
            recorder.camera_topic = "/sensor/camera/head/color/raw"
            recorder.record_hz = 30.0
            recorder.skipped_frames = 0
            frames = 3
            episode = {key: [] for key in BUFFER_KEYS}
            for index in range(frames):
                episode["arm_right"].append([0.1 * index] * 7)
                episode["hand_right"].append([0.2 * index] * 6)
                episode["arm_left"].append([0.3] * 7)
                episode["hand_left"].append([0.0] * 6)
                episode["action_arm_right"].append([0.1 * index] * 7)
                episode["action_hand_right"].append([0.2 * index] * 6)
                episode["action_arm_left"].append([0.3] * 7)
                episode["action_hand_left"].append([0.0] * 6)
                episode["img"].append(np.asarray([255, 216, 255, 217], dtype=np.uint8))
                episode["timestamp"].append(index / 30.0)

            filename = recorder._save_episode(episode)
            self.assertIsNotNone(filename)
            with h5py.File(filename, "r") as data:
                state_parts = [
                    data["puppet/arm_left_position_align/data"],
                    data["puppet/arm_right_position_align/data"],
                    data["puppet/end_effector_left_position_align/data"],
                    data["puppet/end_effector_right_position_align/data"],
                ]
                action_parts = [
                    data["action/arm_left_position_align/data"],
                    data["action/arm_right_position_align/data"],
                    data["action/end_effector_left_position_align/data"],
                    data["action/end_effector_right_position_align/data"],
                ]
                self.assertEqual(sum(part.shape[1] for part in state_parts), 26)
                self.assertEqual(sum(part.shape[1] for part in action_parts), 26)
                self.assertEqual(data["observations/timestamp"].shape, (frames,))
                self.assertEqual(
                    data["camera_observations/color_images/camera_head"].shape,
                    (frames,),
                )
                self.assertEqual(data.attrs["recording_source"], "pico_teleop")
                self.assertAlmostEqual(data.attrs["record_hz"], 30.0)
                self.assertEqual(data.attrs["requested_record_hz"], 30.0)
                self.assertEqual(data.attrs["timestamp_clock"], "ros_image_header")

    def test_hand_tracking_episode_preserves_full_11d_state_and_action(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = object.__new__(PicoEpisodeRecorder)
            recorder.node = _Node()
            recorder.record_root = Path(directory)
            recorder.camera_topic = "/sensor/camera/head/color/raw"
            recorder.record_hz = 30.0
            recorder.skipped_frames = 0
            recorder.task = "walker_c1_pico_hand_tracking_right_pick_place"
            recorder.recording_source = "pico_hand_tracking"
            recorder.hand_joint_names = {
                "left": tuple(f"L_joint_{index}" for index in range(11)),
                "right": tuple(f"R_joint_{index}" for index in range(11)),
            }
            episode = {key: [] for key in BUFFER_KEYS}
            for index in range(2):
                episode["arm_right"].append([0.1 * index] * 7)
                episode["hand_right"].append([0.2 * index] * 11)
                episode["arm_left"].append([0.3] * 7)
                episode["hand_left"].append([0.0] * 11)
                episode["action_arm_right"].append([0.1 * index] * 7)
                episode["action_hand_right"].append([0.2 * index] * 11)
                episode["action_arm_left"].append([0.3] * 7)
                episode["action_hand_left"].append([0.0] * 11)
                episode["img"].append(
                    np.asarray([255, 216, 255, 217], dtype=np.uint8)
                )
                episode["timestamp"].append(index / 30.0)

            filename = recorder._save_episode(episode, "Space key")

            with h5py.File(filename, "r") as data:
                state_width = sum(
                    data[key].shape[1]
                    for key in (
                        "puppet/arm_left_position_align/data",
                        "puppet/arm_right_position_align/data",
                        "puppet/end_effector_left_position_align/data",
                        "puppet/end_effector_right_position_align/data",
                    )
                )
                action_width = sum(
                    data[key].shape[1]
                    for key in (
                        "action/arm_left_position_align/data",
                        "action/arm_right_position_align/data",
                        "action/end_effector_left_position_align/data",
                        "action/end_effector_right_position_align/data",
                    )
                )
                self.assertEqual(state_width, 36)
                self.assertEqual(action_width, 36)
                self.assertEqual(data.attrs["recording_source"], "pico_hand_tracking")
                self.assertEqual(data.attrs["completion_trigger"], "Space key")
                self.assertEqual(len(data.attrs["right_hand_joint_names"]), 11)


if __name__ == "__main__":
    unittest.main()
