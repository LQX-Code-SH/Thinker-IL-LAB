#!/usr/bin/env python3
"""Walker C1 online pick-place task over ROS SDK topics.

This is the non-replay path: start from the same ready pose as reset.py, read
the apple's current simulator-reported world position, plan palm poses with IK,
grasp, place into the plate, then return to ready. The only sim-only shortcut is
the optional initial apple placement for fixed-position validation; all motion
targets are planned from the state read back from /sim/object_state.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional, Sequence

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    from .constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES, TASK_RESET_BODY_POSE
    from .robot_controller import (
        LEFT_HAND_SDK_NAMES,
        RIGHT_HAND_SDK_NAMES,
        WalkerC1RobotController,
    )
except ImportError:
    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES, TASK_RESET_BODY_POSE
    from robot_controller import LEFT_HAND_SDK_NAMES, RIGHT_HAND_SDK_NAMES, WalkerC1RobotController


# Table top is z=0.902 and the apple collision radius is 0.040 m. Keep only
# 2 mm clearance so initialization does not create a rolling impact.
APPLE_SPAWN_W = (8.17, 5.86, 0.944)
APPLE_RANDOM_HALF_EXTENT_M = 0.025
PLATE_CENTER_W = (8.19, 6.083, 0.930)
PLATE_RADIUS = 0.12

PRESHAPE_HAND = [0.2] * 6
CLOSED_HAND = [0.7, 0.85, 0.8, 0.8, 0.8, 0.8]
HOLD_HAND = CLOSED_HAND
GRASP_MOUTH_OFFSET_XY_B = np.array([-0.010, 0.013], dtype=float)
GRASP_CENTER_DZ = 0.059
# Natural, tucked-arm solution for the fixed level palm-down attitude at the
# nominal approach point.  In particular, wrist roll is -0.14 rad instead of
# the old trajectory seed's -1.38 rad.
PREGRASP_IK_SEED = [0.204, -0.115, 0.148, -1.671, 1.295, 0.132, -0.144]
READY_RIGHT_ARM_IK_SEED = [TASK_RESET_BODY_POSE[name] for name in RIGHT_ARM_JOINT_NAMES]
PLACE_YAW_DEG = 35.0

# Palm-origin offset from the live apple center for the calibrated palm-down
# cage. This is a hand geometry/task calibration, not a replayed joint
# trajectory; every episode still plans from the apple position read online.
PALM_GRASP_OFFSET_B = np.array([-0.058, -0.018, 0.044], dtype=float)
APPROACH_PALM_Z = 0.20
HOVER_PALM_Z = 0.14
LIFT_PALM_Z = 0.18
CARRY_PALM_Z = 0.20
RELEASE_PALM_Z = 0.12
PLATE_RELEASE_CLEARANCE_B = 0.110
PLATE_CARRY_ABOVE_RELEASE_B = 0.040

DEFAULT_CAMERA_TOPIC = "/sensor/camera/head/color/raw"
DEFAULT_RECORD_ROOT = "/ubt_sim/dataset/walker_c1_ros"
SIM_PHYSICS_HZ = 100.0
DEFAULT_RECORD_HZ = 30.0
DEFAULT_MOTION_SPEED = 5.0
_RECORD_BUFFER_KEYS = (
    "arm_right", "hand_right", "arm_left", "hand_left",
    "action_arm_right", "action_arm_left", "action_hand_right", "action_hand_left",
    "img", "timestamp",
)


def _format_vec(values: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(v):+.3f}" for v in values) + "]"


def _base_z_rotation(degrees: float) -> np.ndarray:
    angle = np.deg2rad(degrees)
    cosine, sine = float(np.cos(angle)), float(np.sin(angle))
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


class WalkerC1PickPlace(WalkerC1RobotController):
    def __init__(
        self,
        record: bool = False,
        record_root: str = DEFAULT_RECORD_ROOT,
        save_on_failure: bool = False,
        camera_topic: str = DEFAULT_CAMERA_TOPIC,
        record_hz: float = DEFAULT_RECORD_HZ,
        motion_speed: float = DEFAULT_MOTION_SPEED,
    ):
        super().__init__(node_name="walker_c1_pick_place")
        self.record_enabled = bool(record)
        self.record_root = record_root
        self.save_on_failure = bool(save_on_failure)
        self.camera_topic = camera_topic
        self.record_hz = float(record_hz)
        self.motion_speed = float(motion_speed)
        if self.record_hz <= 0.0:
            raise ValueError("record_hz must be positive")
        if self.motion_speed <= 0.0:
            raise ValueError("motion_speed must be positive")
        self._record_active = False
        self._record_buffers = {key: [] for key in _RECORD_BUFFER_KEYS}
        self._record_skipped_frames = 0
        self._next_record_sim_step: Optional[float] = None
        self._last_record_wall_stamp: Optional[float] = None
        self._record_uses_sim_time = False
        self._cv2 = None
        self.ready_pose_converged = False
        if self.record_enabled:
            import cv2

            self._cv2 = cv2
            self.create_subscription(Image, camera_topic, self._record_image_cb, qos_profile_sensor_data)
            self.get_logger().info(f"ROS trajectory recording enabled on {camera_topic}")

    def _motion_duration(self, nominal_seconds: float) -> float:
        """Scale arm interpolation time without shortening grasp/hold checks."""
        return float(nominal_seconds) / self.motion_speed

    def _go_ready(self, right_arm_only: bool = False) -> bool:
        self.go_ready(
            clear_duration=self._motion_duration(0.6),
            final_duration=self._motion_duration(1.0),
            hand_repeats=2,
            right_arm_only=right_arm_only,
        )
        ready_target = TASK_RESET_BODY_POSE
        if right_arm_only:
            ready_target = {
                name: TASK_RESET_BODY_POSE[name]
                for name in RIGHT_ARM_JOINT_NAMES
            }
        self.ready_pose_converged = self.hold_body_pose_until_converged(ready_target)
        return self.ready_pose_converged

    # ── synchronized ROS trajectory recording ──
    def prepare_recording(self) -> None:
        if not self.record_enabled:
            return
        self._record_active = False
        for values in self._record_buffers.values():
            values.clear()
        self._record_skipped_frames = 0
        self._next_record_sim_step = None
        self._last_record_wall_stamp = None
        self._record_uses_sim_time = False

    def start_recording(self) -> None:
        if not self.record_enabled:
            return
        self.prepare_recording()
        # When reset.py ran in a separate process, this controller has not
        # published the ready target itself.  Seed the recorded action with
        # the live ready state until the first task command is sent.
        for name in LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES:
            if name in self.joint_pos:
                self.commanded_body.setdefault(name, float(self.joint_pos[name]))
        for name in LEFT_HAND_SDK_NAMES:
            if name in self.left_hand_pos:
                self.commanded_hand["left"].setdefault(name, float(self.left_hand_pos[name]))
        for name in RIGHT_HAND_SDK_NAMES:
            if name in self.right_hand_pos:
                self.commanded_hand["right"].setdefault(name, float(self.right_hand_pos[name]))
        self._record_active = True
        self.get_logger().info(
            "recording synchronized state/action/RGB frames from the ready pose ..."
        )

    def _record_image_cb(self, msg: Image) -> None:
        if not self._record_active:
            return
        sim_step = self.sim_step()
        if sim_step is not None:
            sim_step = float(sim_step)
            if self._next_record_sim_step is None:
                self._next_record_sim_step = sim_step
            if sim_step < self._next_record_sim_step:
                return
            period_steps = SIM_PHYSICS_HZ / self.record_hz
            while self._next_record_sim_step <= sim_step:
                self._next_record_sim_step += period_steps
        else:
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if stamp <= 0.0:
                stamp = self.get_clock().now().nanoseconds / 1e9
            if (
                self._last_record_wall_stamp is not None
                and stamp - self._last_record_wall_stamp < 1.0 / self.record_hz
            ):
                return
            self._last_record_wall_stamp = stamp
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
            self._record_snapshot(rgb, msg)
        except Exception as exc:
            self._record_skipped_frames += 1
            if self._record_skipped_frames <= 3:
                self.get_logger().warn(f"skipping camera frame: {exc}")

    def _record_snapshot(self, rgb: np.ndarray, msg: Image) -> None:
        body_names = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        state_ready = all(name in self.joint_pos for name in body_names)
        state_ready = state_ready and all(name in self.left_hand_pos for name in LEFT_HAND_SDK_NAMES)
        state_ready = state_ready and all(name in self.right_hand_pos for name in RIGHT_HAND_SDK_NAMES)
        action_ready = all(name in self.commanded_body for name in body_names)
        action_ready = action_ready and all(name in self.commanded_hand["left"] for name in LEFT_HAND_SDK_NAMES)
        action_ready = action_ready and all(name in self.commanded_hand["right"] for name in RIGHT_HAND_SDK_NAMES)
        if not state_ready or not action_ready:
            self._record_skipped_frames += 1
            return

        assert self._cv2 is not None
        image_bgr = self._cv2.cvtColor(np.ascontiguousarray(rgb), self._cv2.COLOR_RGB2BGR)
        encoded_ok, encoded = self._cv2.imencode(".jpg", image_bgr)
        if not encoded_ok:
            self._record_skipped_frames += 1
            return

        sim_step = self.sim_step()
        if sim_step is not None:
            stamp = float(sim_step) / SIM_PHYSICS_HZ
            self._record_uses_sim_time = True
        else:
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if stamp <= 0.0:
                stamp = self.get_clock().now().nanoseconds / 1e9
        snapshot = {
            "arm_right": [self.joint_pos[name] for name in RIGHT_ARM_JOINT_NAMES],
            "hand_right": [self.right_hand_pos[name] for name in RIGHT_HAND_SDK_NAMES],
            "arm_left": [self.joint_pos[name] for name in LEFT_ARM_JOINT_NAMES],
            "hand_left": [self.left_hand_pos[name] for name in LEFT_HAND_SDK_NAMES],
            "action_arm_right": [self.commanded_body[name] for name in RIGHT_ARM_JOINT_NAMES],
            "action_arm_left": [self.commanded_body[name] for name in LEFT_ARM_JOINT_NAMES],
            "action_hand_right": [self.commanded_hand["right"][name] for name in RIGHT_HAND_SDK_NAMES],
            "action_hand_left": [self.commanded_hand["left"][name] for name in LEFT_HAND_SDK_NAMES],
            "img": encoded.reshape(-1).copy(),
            "timestamp": stamp,
        }
        for key, value in snapshot.items():
            self._record_buffers[key].append(value)

    def finish_recording(self, success: bool) -> Optional[str]:
        if not self.record_enabled:
            return None
        self._record_active = False
        frame_count = len(self._record_buffers["timestamp"])
        if not success and not self.save_on_failure:
            self.get_logger().info(f"discarding {frame_count} recorded frames because the task failed")
            return None
        if frame_count == 0:
            self.get_logger().error(
                f"no synchronized frames recorded from {self.camera_topic}; trajectory not saved"
            )
            return None
        lengths = {key: len(values) for key, values in self._record_buffers.items()}
        if len(set(lengths.values())) != 1:
            self.get_logger().error(f"record buffer length mismatch: {lengths}")
            return None
        return self._save_recording(success)

    def _save_recording(self, success: bool) -> str:
        import h5py

        episode_dir = os.path.join(self.record_root, str(int(time.time() * 1000)))
        os.makedirs(episode_dir, exist_ok=False)
        filename = os.path.join(episode_dir, "trajectory.hdf5")
        frame_count = len(self._record_buffers["timestamp"])
        self.get_logger().info(f"saving {frame_count} ROS frames to {filename} ...")
        with h5py.File(filename, "w") as output:
            output.attrs["task"] = "walker_c1_online_ik_pick_place"
            output.attrs["recording_source"] = "ros2"
            output.attrs["success"] = bool(success)
            output.attrs["camera_topic"] = self.camera_topic
            output.attrs["record_hz"] = self.record_hz
            output.attrs["motion_speed"] = self.motion_speed
            output.attrs["timestamp_clock"] = "simulation" if self._record_uses_sim_time else "ros"
            output.create_dataset("puppet/arm_right_position_align/data", data=np.asarray(self._record_buffers["arm_right"], dtype=np.float32))
            output.create_dataset("puppet/end_effector_right_position_align/data", data=np.asarray(self._record_buffers["hand_right"], dtype=np.float32))
            output.create_dataset("puppet/arm_left_position_align/data", data=np.asarray(self._record_buffers["arm_left"], dtype=np.float32))
            output.create_dataset("puppet/end_effector_left_position_align/data", data=np.asarray(self._record_buffers["hand_left"], dtype=np.float32))
            output.create_dataset("action/arm_right_position_align/data", data=np.asarray(self._record_buffers["action_arm_right"], dtype=np.float32))
            output.create_dataset("action/arm_left_position_align/data", data=np.asarray(self._record_buffers["action_arm_left"], dtype=np.float32))
            output.create_dataset("action/end_effector_right_position_align/data", data=np.asarray(self._record_buffers["action_hand_right"], dtype=np.float32))
            output.create_dataset("action/end_effector_left_position_align/data", data=np.asarray(self._record_buffers["action_hand_left"], dtype=np.float32))
            output.create_dataset("observations/timestamp", data=np.asarray(self._record_buffers["timestamp"], dtype=np.float64))
            image_type = h5py.vlen_dtype(np.dtype("uint8"))
            images = output.create_dataset(
                "camera_observations/color_images/camera_head", (frame_count,), dtype=image_type
            )
            for index, encoded in enumerate(self._record_buffers["img"]):
                images[index] = encoded
        try:
            os.chmod(episode_dir, 0o777)
            os.chmod(filename, 0o666)
        except PermissionError:
            pass
        self.get_logger().info(
            f"trajectory saved: {filename} ({frame_count} frames, "
            f"skipped={self._record_skipped_frames})"
        )
        return filename

    def _root_rotation_wb(self) -> np.ndarray:
        root = self.object_state.get("robot_root_pose_w")
        if not root:
            raise RuntimeError("no robot_root_pose_w yet (is the sim bridge up?)")
        w, x, y, z = root[3:7]
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])

    def world_to_base(self, pos_w: Sequence[float]) -> np.ndarray:
        root = self.object_state.get("robot_root_pose_w")
        if not root:
            raise RuntimeError("no robot_root_pose_w yet (is the sim bridge up?)")
        return self._root_rotation_wb().T @ (np.array(pos_w, dtype=float) - np.array(root[:3]))

    def world_delta_to_base(self, delta_w: Sequence[float]) -> np.ndarray:
        return self._root_rotation_wb().T @ np.array(delta_w, dtype=float)

    def wait_for_object_state(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.object_state.get("object_pos_w") and self.object_state.get("robot_root_pose_w"):
                return True
        return False

    def mouth_pos_in_base(self) -> Optional[np.ndarray]:
        mouth_w = self.mouth_center_w()
        if mouth_w is None:
            return None
        return self.world_to_base(mouth_w)

    def grasp_center_w(self) -> Optional[np.ndarray]:
        links = self.object_state.get("right_hand_links_w") or {}
        positions = [
            links.get("R_thumb_mpp_link"),
            links.get("R_index_ip_link"),
            links.get("R_middle_ip_link"),
        ]
        if any(pos is None for pos in positions):
            return None
        return np.mean(np.array(positions, dtype=float), axis=0)

    def align_grasp(
        self,
        palm_target_b: np.ndarray,
        rot_mat: np.ndarray,
        max_iters: int = 3,
        tol: float = 0.004,
    ) -> np.ndarray:
        target = np.array(palm_target_b, dtype=float)
        for _ in range(max_iters):
            mouth_w = self.mouth_center_w()
            grasp_center_w = self.grasp_center_w()
            apple_w = self.object_state.get("object_pos_w")
            if mouth_w is None or grasp_center_w is None or apple_w is None:
                self.get_logger().warn("no hand-link/object state; skipping grasp alignment")
                return target
            mouth_rel_b = self.world_delta_to_base(np.array(mouth_w) - np.array(apple_w))
            grasp_rel_b = self.world_delta_to_base(np.array(grasp_center_w) - np.array(apple_w))
            xy_err_b = GRASP_MOUTH_OFFSET_XY_B - mouth_rel_b[:2]
            z_err_b = GRASP_CENTER_DZ - float(grasp_rel_b[2])
            self.get_logger().info(
                f"grasp align: mouth offset={_format_vec(mouth_rel_b[:2] * 1000.0)} mm, "
                f"center dz={grasp_rel_b[2] * 1000.0:.0f} mm "
                f"(target {GRASP_CENTER_DZ * 1000.0:.0f} mm)"
            )
            if float(np.linalg.norm(xy_err_b)) < tol and abs(z_err_b) < tol:
                return target
            correction_b = np.array([xy_err_b[0], xy_err_b[1], z_err_b])
            correction_norm = float(np.linalg.norm(correction_b))
            if correction_norm > 0.025:
                correction_b *= 0.025 / correction_norm
            target = self.fk_palm()[:3, 3] + correction_b
            if not self.move_right_arm(
                target,
                rot_mat=rot_mat,
                duration=self._motion_duration(0.6),
                corrections=0,
            ):
                return target
        return target

    def log_hand_geometry(self, label: str) -> None:
        apple_w = self.object_state.get("object_pos_w")
        links = self.object_state.get("right_hand_links_w") or {}
        if not apple_w or not links:
            self.get_logger().warn(f"{label}: no hand geometry state")
            return
        apple = np.array(apple_w, dtype=float)
        names = (
            "R_thumb_ip_link",
            "R_index_ip_link",
            "R_middle_ip_link",
            "R_ring_ip_link",
            "R_little_ip_link",
            "R_palm_link",
        )
        parts = []
        for name in names:
            pos = links.get(name)
            if pos is None:
                continue
            rel_b = self.world_delta_to_base(np.array(pos, dtype=float) - apple)
            parts.append(f"{name.replace('_link', '')}:{_format_vec(rel_b)}")
        mouth_w = self.mouth_center_w()
        if mouth_w is not None:
            mouth_rel_b = self.world_delta_to_base(mouth_w - apple)
            parts.append(f"mouth:{_format_vec(mouth_rel_b)}")
        grasp_links = [
            links.get("R_thumb_mpp_link"),
            links.get("R_index_ip_link"),
            links.get("R_middle_ip_link"),
        ]
        if all(pos is not None for pos in grasp_links):
            grasp_center_w = np.mean(np.array(grasp_links, dtype=float), axis=0)
            grasp_rel_b = self.world_delta_to_base(grasp_center_w - apple)
            parts.append(f"grasp_center:{_format_vec(grasp_rel_b)}")
        arm = _format_vec(self.current_arm())
        self.get_logger().info(
            f"{label} hand rel apple base xyz: " + " | ".join(parts) + f" | arm:{arm}"
        )

    def close_hand_decisive(self) -> None:
        # Close all six commands together over 60 physics steps, then let the
        # grasp stabilize before arm motion.  This also holds the restored
        # 8 cm task apple with the level palm-down pose.
        start = PRESHAPE_HAND
        updates = 60
        for step in range(1, updates + 1):
            t = step / updates
            cmd = [(1.0 - t) * a + t * b for a, b in zip(start, CLOSED_HAND)]
            self.move_hand("right", cmd, repeats=1, wait_steps=1)
        self.wait_sim_steps(20, timeout=10.0)

    def verify_static_hold(self, apple0_b: np.ndarray) -> bool:
        # Preserve the proven grasp shape after clearing the table. The static
        # hold depends on fingertip friction, not additional finger curl.
        updates = 20
        for step in range(1, updates + 1):
            t = step / updates
            cmd = [(1.0 - t) * a + t * b for a, b in zip(CLOSED_HAND, HOLD_HAND)]
            self.move_hand("right", cmd, repeats=1, wait_steps=1)

        # Hold still long enough to expose the slow-slip failure mode. Refresh
        # the target for real-robot compatibility; the simulator also retains
        # the most recent hand command internally.
        for _ in range(6):
            self.move_hand("right", HOLD_HAND, repeats=1, wait_steps=10)

        held = self.object_pos_in_base()
        mouth_b = self.mouth_pos_in_base()
        if held is None or mouth_b is None:
            self.get_logger().warn("missing state for static hold check")
            return False
        lift_delta = float(held[2] - apple0_b[2])
        mouth_dist = float(np.linalg.norm(held - mouth_b))
        stable = lift_delta >= 0.05 and mouth_dist <= 0.10
        self.get_logger().info(
            f"static hold check: dz={lift_delta:.3f} m, apple-mouth={mouth_dist:.3f} m -> "
            f"{'STABLE' if stable else 'SLIPPING'}"
        )
        return stable

    def prepare_palm_down_cage(self) -> np.ndarray:
        self.move_hand("right", PRESHAPE_HAND, repeats=4)
        self.wait_sim_steps(5, timeout=6.0)
        self.get_logger().info("using fixed level palm-down grasp attitude")
        return self.grasp_attitude.copy()

    def try_grasp_once(
        self,
        apple0_b: np.ndarray,
        grasp_rot: np.ndarray,
    ) -> bool:
        apple_b = self.object_pos_in_base()
        if apple_b is None:
            self.get_logger().error("no apple position for grasp attempt")
            return False
        self.get_logger().info(f"planning grasp from live apple base pos {_format_vec(apple_b)}")

        anchor = np.array(apple_b[:3], dtype=float) + PALM_GRASP_OFFSET_B
        approach = np.array([anchor[0], anchor[1], max(APPROACH_PALM_Z, anchor[2] + 0.12)])
        hover = np.array([anchor[0], anchor[1], max(HOVER_PALM_Z, anchor[2] + 0.05)])
        grasp = np.array([anchor[0], anchor[1], max(0.075, anchor[2])])
        self.get_logger().info(
            f"palm plan approach={_format_vec(approach)} hover={_format_vec(hover)} "
            f"grasp={_format_vec(grasp)}"
        )

        self.get_logger().info("approach above apple ...")
        if not self.move_right_arm(
            approach,
            rot_mat=grasp_rot,
            duration=self._motion_duration(2.0),
            corrections=0,
            seed_arms=(PREGRASP_IK_SEED, self.current_arm(), READY_RIGHT_ARM_IK_SEED),
            reference_arm=READY_RIGHT_ARM_IK_SEED,
        ):
            return False
        self.get_logger().info("hover over apple ...")
        if not self.move_right_arm(
            hover,
            rot_mat=grasp_rot,
            duration=self._motion_duration(1.2),
            corrections=0,
        ):
            return False
        self.get_logger().info("descend around apple ...")
        if not self.move_right_arm(
            grasp,
            rot_mat=grasp_rot,
            duration=self._motion_duration(3.0),
            corrections=0,
        ):
            return False
        palm_target = self.align_grasp(grasp, grasp_rot)
        self.get_logger().info("settling at the grasp pose for 0.1 simulated seconds ...")
        self.wait_sim_steps(10, timeout=8.0)
        self.log_hand_geometry("preclose")

        self.get_logger().info("closing hand ...")
        self.close_hand_decisive()
        self.log_hand_geometry("postclose")

        self.get_logger().info("lifting for grasp check ...")
        lift_palm = np.array([palm_target[0], palm_target[1], max(LIFT_PALM_Z, palm_target[2] + 0.08)])
        if not self.move_right_arm(
            lift_palm,
            rot_mat=grasp_rot,
            duration=self._motion_duration(3.0),
            corrections=0,
            smooth=True,
        ):
            return False
        self.wait_sim_steps(10, timeout=6.0)

        lifted = self.object_pos_in_base()
        mouth_b = self.mouth_pos_in_base()
        if lifted is None or mouth_b is None:
            self.get_logger().warn("missing state for lift check")
            return False
        lift_delta = float(lifted[2] - apple0_b[2])
        mouth_dist = float(np.linalg.norm(lifted - mouth_b))
        held = lift_delta >= 0.05 and mouth_dist <= 0.18
        self.get_logger().info(
            f"lift check: dz={lift_delta:.3f} m, apple-mouth={mouth_dist:.3f} m -> "
            f"{'HELD' if held else 'NOT HELD'}"
        )
        if not held:
            return False

        self.get_logger().info("verifying static hold for 0.8 simulated seconds ...")
        return self.verify_static_hold(apple0_b)

    def place_and_return(self, grasp_rot: np.ndarray) -> bool:
        plate_b = self.world_to_base(PLATE_CENTER_W)
        plate_x, plate_y, plate_z = [float(v) for v in plate_b[:3]]

        # Preserve the level inclination through lift, carry and release.  The
        # plate is on the body centerline, where +35 deg horizontal yaw gives a
        # fully reachable release pose; yaw does not change palm inclination.
        place_rot = _base_z_rotation(PLACE_YAW_DEG) @ grasp_rot
        plate_anchor = np.array([plate_x, plate_y, plate_z], dtype=float) + PALM_GRASP_OFFSET_B
        release_z = max(RELEASE_PALM_Z, plate_anchor[2] + PLATE_RELEASE_CLEARANCE_B)
        carry_palm = np.array(
            [
                plate_anchor[0],
                plate_anchor[1],
                max(CARRY_PALM_Z, release_z + PLATE_CARRY_ABOVE_RELEASE_B),
            ]
        )
        lower_palm = np.array(
            [
                plate_anchor[0],
                plate_anchor[1],
                release_z,
            ]
        )
        retreat_palm = np.array([plate_anchor[0] - 0.06, plate_anchor[1] - 0.06, max(CARRY_PALM_Z, plate_anchor[2] + 0.15)])

        self.get_logger().info(
            f"carry directly over plate with {PLACE_YAW_DEG:.1f} deg place yaw ..."
        )
        if not self.move_right_arm(
            carry_palm,
            rot_mat=place_rot,
            duration=self._motion_duration(3.0),
            corrections=0,
            smooth=True,
        ):
            return False
        self.get_logger().info("lower into plate ...")
        if not self.move_right_arm(
            lower_palm,
            rot_mat=place_rot,
            duration=self._motion_duration(2.0),
            corrections=0,
            smooth=True,
        ):
            return False
        self.log_hand_geometry("pre-release")
        self.get_logger().info("release apple ...")
        self.open_hand("right")
        self.get_logger().info("waiting 0.3 simulated seconds for the apple to settle ...")
        self.wait_sim_steps(30, timeout=10.0)

        self.get_logger().info("retreat from plate ...")
        self.move_right_arm(
            retreat_palm,
            rot_mat=place_rot,
            duration=self._motion_duration(1.2),
            corrections=0,
            smooth=True,
        )

        self.get_logger().info("back to ready ...")
        self._go_ready(right_arm_only=True)

        final_w = self.object_state.get("object_pos_w")
        if not final_w:
            self.get_logger().warn("no object state for final success check")
            return False
        dist = float(np.hypot(final_w[0] - PLATE_CENTER_W[0], final_w[1] - PLATE_CENTER_W[1]))
        ok = dist <= PLATE_RADIUS and final_w[2] > 0.9
        self.get_logger().info(
            f"final: apple=({_format_vec(final_w)}) plate_dist={dist:.3f} "
            f"(limit {PLATE_RADIUS}) -> {'SUCCESS' if ok else 'FAILURE'}"
        )
        return ok

    def run_task(
        self,
        randomize: bool = False,
        set_apple: bool = True,
        max_grasp_attempts: int = 2,
        reset_at_start: bool = True,
        apple_xy: Optional[Sequence[float]] = None,
    ) -> bool:
        rng = np.random.default_rng()

        self.get_logger().info("waiting for robot/object state ...")
        if not self.wait_for_state():
            self.get_logger().error("no /mc/sdk/robot_state; is the stack running?")
            return False
        if not self.wait_for_object_state():
            self.get_logger().error("no /sim/object_state; is the bridge running?")
            return False

        if reset_at_start:
            self.get_logger().info("going to reset.py ready pose ...")
            self._go_ready()
        else:
            self.get_logger().info("already at ready pose from the previous episode; skipping reset ...")

        if set_apple:
            apple_w = list(APPLE_SPAWN_W)
            if apple_xy is not None:
                apple_w[:2] = [float(apple_xy[0]), float(apple_xy[1])]
            elif randomize:
                offset_xy = rng.uniform(
                    -APPLE_RANDOM_HALF_EXTENT_M,
                    APPLE_RANDOM_HALF_EXTENT_M,
                    size=2,
                )
                apple_w[0] += float(offset_xy[0])
                apple_w[1] += float(offset_xy[1])
            self.get_logger().info(f"setting apple near fixed spot {np.round(apple_w, 3).tolist()}")
            self.set_object_world_pos(*apple_w)
            self.get_logger().info("waiting 0.1 simulated seconds for the apple to settle ...")
            self.wait_sim_steps(10, timeout=5.0)

        apple0_b = self.object_pos_in_base()
        if apple0_b is None:
            self.get_logger().error("no apple position after ready")
            return False
        self.get_logger().info(
            f"observed apple world pos: {_format_vec(self.object_state['object_pos_w'])}; "
            f"base pos: {_format_vec(apple0_b)}"
        )

        # The initial move/reset and apple settling are deliberately excluded.
        # The final return-to-ready remains inside the recorded episode.
        self.start_recording()
        grasp_rot = self.prepare_palm_down_cage()

        held = False
        self.ready_pose_converged = False
        for attempt in range(max(1, int(max_grasp_attempts))):
            self.get_logger().info(f"grasp attempt {attempt + 1}/{max_grasp_attempts}")
            held = self.try_grasp_once(apple0_b, grasp_rot)
            if held:
                break
            apple_w = self.object_state.get("object_pos_w")
            if not apple_w or apple_w[2] < 0.5:
                self.get_logger().warn("apple fell off the table; aborting")
                break
            self.get_logger().info("grasp missed; reopen and replan from live apple state ...")
            self.move_hand("right", PRESHAPE_HAND, repeats=4)
            self.wait_sim_steps(60, timeout=10.0)

        if not held:
            self.get_logger().warn("failed to grasp; returning to ready")
            self.open_hand("right")
            self._go_ready(right_arm_only=True)
            return False

        return self.place_and_return(grasp_rot)


def main() -> int:
    parser = argparse.ArgumentParser(description="Walker C1 online IK pick-place task")
    parser.add_argument("--randomize", action="store_true", help="Randomize the apple spot around the fixed validation pose")
    parser.add_argument(
        "--apple-xy",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="Place the apple at an exact world XY position (validation only)",
    )
    parser.add_argument("--use-existing-apple", action="store_true", help="Do not command the sim-only apple placement topic")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--max-grasp-attempts", type=int, default=2)
    parser.add_argument(
        "--skip-initial-reset",
        action="store_true",
        help="Assume reset.py already placed the robot at ready; do not repeat the first reset",
    )
    parser.add_argument("--record", dest="record", action="store_true", help="Record synchronized ROS state/action/RGB to HDF5")
    parser.add_argument("--no-record", dest="record", action="store_false", help="Disable HDF5 recording")
    parser.add_argument("--record-root", default=DEFAULT_RECORD_ROOT, help="Root directory for ROS-recorded trajectories")
    parser.add_argument("--save-on-failure", action="store_true", help="Save recorded frames even when the task fails")
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument("--record-hz", type=float, default=DEFAULT_RECORD_HZ, help="Trajectory recording rate in Hz")
    parser.add_argument(
        "--motion-speed",
        type=float,
        default=DEFAULT_MOTION_SPEED,
        help="Arm interpolation speed multiplier; grasp close profile and safety checks are retained",
    )
    parser.set_defaults(record=False)
    args = parser.parse_args()
    if args.use_existing_apple and args.apple_xy is not None:
        parser.error("--apple-xy cannot be combined with --use-existing-apple")
    if args.randomize and args.apple_xy is not None:
        parser.error("--apple-xy cannot be combined with --randomize")

    rclpy.init()
    node = WalkerC1PickPlace(
        record=args.record,
        record_root=args.record_root,
        save_on_failure=args.save_on_failure,
        camera_topic=args.camera_topic,
        record_hz=args.record_hz,
        motion_speed=args.motion_speed,
    )
    ok_count = 0
    saved_count = 0
    previous_ok = False
    try:
        for ep in range(args.episodes):
            node.get_logger().info(f"=== episode {ep + 1}/{args.episodes} ===")
            node.prepare_recording()
            ok = node.run_task(
                randomize=args.randomize,
                set_apple=not args.use_existing_apple,
                max_grasp_attempts=args.max_grasp_attempts,
                reset_at_start=not (
                    (ep == 0 and args.skip_initial_reset)
                    or (ep > 0 and previous_ok and node.ready_pose_converged)
                ),
                apple_xy=args.apple_xy,
            )
            previous_ok = ok
            ok_count += int(ok)
            saved_count += int(node.finish_recording(ok) is not None)
            node.wait_sim_steps(20, timeout=10.0)
    except KeyboardInterrupt:
        node.finish_recording(False)
    finally:
        record_summary = f", {saved_count} trajectory file(s) saved" if args.record else ""
        node.get_logger().info(f"done: {ok_count}/{args.episodes} success{record_summary}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    complete = ok_count == args.episodes
    if args.record:
        complete = complete and saved_count == args.episodes
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
