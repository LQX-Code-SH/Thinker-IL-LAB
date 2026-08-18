#!/usr/bin/env python3
"""Synchronized HDF5 episode recording for Walker C1 PICO teleoperation."""
from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import h5py
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool

from mc_task_msgs.msg import JointCommand

try:
    from .constants import (
        LEFT_ARM_JOINT_NAMES,
        LEFT_HAND_JOINT_NAMES,
        RIGHT_ARM_JOINT_NAMES,
        RIGHT_HAND_JOINT_NAMES,
        TASK_RESET_BODY_POSE,
    )
except ImportError:
    from constants import (
        LEFT_ARM_JOINT_NAMES,
        LEFT_HAND_JOINT_NAMES,
        RIGHT_ARM_JOINT_NAMES,
        RIGHT_HAND_JOINT_NAMES,
        TASK_RESET_BODY_POSE,
    )

if TYPE_CHECKING:
    from pico_teleop import WalkerC1PicoTeleop


BUFFER_KEYS = (
    "arm_right", "hand_right", "arm_left", "hand_left",
    "action_arm_right", "action_arm_left",
    "action_hand_right", "action_hand_left",
    "img", "timestamp",
)


class PicoEpisodeRecorder:
    """Record one episode at a time and reset after the simulator Space event."""

    def __init__(
        self,
        node: "WalkerC1PicoTeleop",
        record_root: str,
        camera_topic: str,
        record_hz: float,
    ) -> None:
        import cv2

        if record_hz <= 0.0:
            raise ValueError("record_hz must be positive")
        self.node = node
        self.record_root = Path(record_root)
        self.camera_topic = camera_topic
        self.record_hz = float(record_hz)
        self.cv2 = cv2
        self.active = False
        self.finishing = False
        self.buffers = {key: [] for key in BUFFER_KEYS}
        self.left_hand_positions: dict[str, float] = {}
        self.right_hand_positions: dict[str, float] = {}
        self.commanded_body: dict[str, float] = {}
        self.commanded_hands: dict[str, dict[str, float]] = {"left": {}, "right": {}}
        self.skipped_frames = 0
        self.last_record_stamp: Optional[float] = None
        self._lock = threading.Lock()
        self._worker: Optional[threading.Thread] = None

        node.create_subscription(
            JointState, "/mc/left_hand/joint_states",
            lambda msg: self.left_hand_positions.update(zip(msg.name, msg.position)), 10,
        )
        node.create_subscription(
            JointState, "/mc/right_hand/joint_states",
            lambda msg: self.right_hand_positions.update(zip(msg.name, msg.position)), 10,
        )
        node.create_subscription(Image, camera_topic, self._image_callback, qos_profile_sensor_data)
        node.create_subscription(Bool, "/sim/episode_complete", self._complete_callback, 10)
        self.reset_pub = node.create_publisher(Bool, "/sim/cmd_reset", 1)
        self.ready_pub = node.create_publisher(Bool, "/sim/episode_ready", 1)
        node.get_logger().info(
            f"episode recorder ready: Space saves to {self.record_root} and resets"
        )

    def note_body_command(self, positions: dict[str, float]) -> None:
        self.commanded_body.update({name: float(value) for name, value in positions.items()})

    def note_hand_command(self, side: str, positions: list[float]) -> None:
        names = LEFT_HAND_JOINT_NAMES if side == "left" else RIGHT_HAND_JOINT_NAMES
        self.commanded_hands[side].update(zip(names, map(float, positions)))

    def start_episode(self) -> None:
        with self._lock:
            if self.active or self.finishing:
                return
            for values in self.buffers.values():
                values.clear()
            self.skipped_frames = 0
            self.last_record_stamp = None
            self.commanded_body.update(
                {name: float(TASK_RESET_BODY_POSE[name]) for name in LEFT_ARM_JOINT_NAMES}
            )
            for name in RIGHT_ARM_JOINT_NAMES:
                if name in self.node.joint_positions:
                    self.commanded_body[name] = float(self.node.joint_positions[name])
            self.commanded_hands["left"].update(
                {name: 0.0 for name in LEFT_HAND_JOINT_NAMES}
            )
            self.commanded_hands["right"].update(
                {name: float(self.right_hand_positions.get(name, 0.0)) for name in RIGHT_HAND_JOINT_NAMES}
            )
            self.active = True
        self.node.get_logger().info("episode recording started")

    def _image_callback(self, msg: Image) -> None:
        if not self.active or self.finishing:
            return
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            stamp = self.node.get_clock().now().nanoseconds / 1e9
        if self.last_record_stamp is not None and stamp - self.last_record_stamp < 1.0 / self.record_hz:
            return

        body_names = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        state_ready = all(name in self.node.joint_positions for name in body_names)
        state_ready = state_ready and all(name in self.left_hand_positions for name in LEFT_HAND_JOINT_NAMES)
        state_ready = state_ready and all(name in self.right_hand_positions for name in RIGHT_HAND_JOINT_NAMES)
        action_ready = all(name in self.commanded_body for name in body_names)
        action_ready = action_ready and all(
            name in self.commanded_hands["left"] for name in LEFT_HAND_JOINT_NAMES
        )
        action_ready = action_ready and all(
            name in self.commanded_hands["right"] for name in RIGHT_HAND_JOINT_NAMES
        )
        if not state_ready or not action_ready:
            self.skipped_frames += 1
            return

        try:
            channels = 4 if msg.encoding in ("rgba8", "bgra8") else 3
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw[: int(msg.height) * int(msg.step)].reshape(int(msg.height), int(msg.step))
            image = rows[:, : int(msg.width) * channels].reshape(int(msg.height), int(msg.width), channels)
            if msg.encoding == "rgb8":
                rgb = image
            elif msg.encoding == "bgr8":
                rgb = image[..., ::-1]
            elif msg.encoding == "rgba8":
                rgb = image[..., :3]
            elif msg.encoding == "bgra8":
                rgb = image[..., 2::-1]
            else:
                raise ValueError(f"unsupported RGB encoding {msg.encoding!r}")
            ok, encoded = self.cv2.imencode(
                ".jpg", self.cv2.cvtColor(np.ascontiguousarray(rgb), self.cv2.COLOR_RGB2BGR)
            )
            if not ok:
                raise ValueError("JPEG encoding failed")
        except Exception as exc:
            self.skipped_frames += 1
            if self.skipped_frames <= 3:
                self.node.get_logger().warn(f"skipping recorder camera frame: {exc}")
            return

        snapshot = {
            "arm_right": [self.node.joint_positions[name] for name in RIGHT_ARM_JOINT_NAMES],
            "hand_right": [self.right_hand_positions[name] for name in RIGHT_HAND_JOINT_NAMES],
            "arm_left": [self.node.joint_positions[name] for name in LEFT_ARM_JOINT_NAMES],
            "hand_left": [self.left_hand_positions[name] for name in LEFT_HAND_JOINT_NAMES],
            "action_arm_right": [self.commanded_body[name] for name in RIGHT_ARM_JOINT_NAMES],
            "action_arm_left": [self.commanded_body[name] for name in LEFT_ARM_JOINT_NAMES],
            "action_hand_right": [self.commanded_hands["right"][name] for name in RIGHT_HAND_JOINT_NAMES],
            "action_hand_left": [self.commanded_hands["left"][name] for name in LEFT_HAND_JOINT_NAMES],
            "img": encoded.reshape(-1).copy(),
            "timestamp": stamp,
        }
        with self._lock:
            if not self.active or self.finishing:
                return
            for key, value in snapshot.items():
                self.buffers[key].append(value)
            self.last_record_stamp = stamp

    def _take_episode_snapshot(self) -> Optional[dict[str, list]]:
        """Atomically close the active episode and consume its buffers once."""
        with self._lock:
            if self.finishing or not self.active:
                return None
            self.finishing = True
            self.active = False
            episode = {key: list(values) for key, values in self.buffers.items()}
            for values in self.buffers.values():
                values.clear()
            self.last_record_stamp = None
        return episode

    def _complete_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.request_complete("Space key")

    def request_complete(self, trigger: str) -> bool:
        """Finish the active episode once, from Space or a controller gesture."""
        episode = self._take_episode_snapshot()
        if episode is None:
            return False
        self.node.episode_reset_blocked = True
        self.node.episode_reset_complete = False
        self.node.disarm(f"{trigger}; saving episode")
        self._worker = threading.Thread(
            target=self._save_and_reset, args=(episode, trigger), daemon=True,
            name="pico_episode_save_reset",
        )
        self._worker.start()
        return True

    def _save_episode(
        self, episode: dict[str, list], completion_trigger: str = "unknown"
    ) -> Optional[Path]:
        frame_count = len(episode["timestamp"])
        if frame_count == 0:
            self.node.get_logger().warn(
                "episode completed with no recorded frames; resetting without save"
            )
            return None
        lengths = {key: len(values) for key, values in episode.items()}
        if len(set(lengths.values())) != 1:
            raise RuntimeError(f"record buffer length mismatch: {lengths}")

        episode_dir = self.record_root / str(int(time.time() * 1000))
        episode_dir.mkdir(parents=True, exist_ok=False)
        filename = episode_dir / "trajectory.hdf5"
        timestamps = np.asarray(episode["timestamp"], dtype=np.float64)
        elapsed = float(timestamps[-1] - timestamps[0]) if frame_count > 1 else 0.0
        measured_hz = float((frame_count - 1) / elapsed) if elapsed > 0.0 else 0.0
        with h5py.File(filename, "w") as output:
            output.attrs["task"] = "walker_c1_pico_right_hand_pick_place"
            output.attrs["recording_source"] = "pico_teleop"
            output.attrs["success"] = True
            output.attrs["camera_topic"] = self.camera_topic
            output.attrs["record_hz"] = measured_hz
            output.attrs["requested_record_hz"] = self.record_hz
            output.attrs["timestamp_clock"] = "ros_image_header"
            output.attrs["completion_trigger"] = completion_trigger
            output.create_dataset("puppet/arm_right_position_align/data", data=np.asarray(episode["arm_right"], dtype=np.float32))
            output.create_dataset("puppet/end_effector_right_position_align/data", data=np.asarray(episode["hand_right"], dtype=np.float32))
            output.create_dataset("puppet/arm_left_position_align/data", data=np.asarray(episode["arm_left"], dtype=np.float32))
            output.create_dataset("puppet/end_effector_left_position_align/data", data=np.asarray(episode["hand_left"], dtype=np.float32))
            output.create_dataset("action/arm_right_position_align/data", data=np.asarray(episode["action_arm_right"], dtype=np.float32))
            output.create_dataset("action/arm_left_position_align/data", data=np.asarray(episode["action_arm_left"], dtype=np.float32))
            output.create_dataset("action/end_effector_right_position_align/data", data=np.asarray(episode["action_hand_right"], dtype=np.float32))
            output.create_dataset("action/end_effector_left_position_align/data", data=np.asarray(episode["action_hand_left"], dtype=np.float32))
            output.create_dataset("observations/timestamp", data=timestamps)
            image_type = h5py.vlen_dtype(np.dtype("uint8"))
            images = output.create_dataset(
                "camera_observations/color_images/camera_head", (frame_count,), dtype=image_type
            )
            for index, encoded in enumerate(episode["img"]):
                images[index] = encoded
        try:
            os.chmod(episode_dir, 0o777)
            os.chmod(filename, 0o666)
        except PermissionError:
            pass
        self.node.get_logger().info(
            f"episode saved: {filename} ({frame_count} frames, skipped={self.skipped_frames})"
        )
        return filename

    def _save_and_reset(self, episode: dict[str, list], completion_trigger: str) -> None:
        try:
            self._save_episode(episode, completion_trigger)
            reset_msg = Bool()
            reset_msg.data = True
            # This publisher/subscription pair uses reliable ROS QoS.  Sending
            # the same edge-triggered command repeatedly makes Isaac reset the
            # environment several times and delays the following task pose.
            self.reset_pub.publish(reset_msg)
            time.sleep(0.2)
            reset_script = Path(__file__).with_name("reset.py")
            result = subprocess.run(
                [sys.executable, str(reset_script), "--mode", "task"],
                check=False,
                text=True,
                capture_output=True,
            )
            if result.returncode != 0:
                raise RuntimeError(
                    f"task reset failed ({result.returncode}): {result.stderr.strip()}"
                )
            deadman_label = getattr(self.node, "deadman_label", "right B")
            self.node.get_logger().info(
                f"episode reset complete; release {deadman_label} for the next episode"
            )
        except Exception as exc:
            self.node.get_logger().error(f"episode save/reset failed: {exc}")
        finally:
            self.finishing = False
            self.node.episode_reset_complete = True
            ready = Bool()
            ready.data = True
            self.ready_pub.publish(ready)
