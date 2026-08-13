#!/usr/bin/env python3
"""PICO head + dual-controller teleoperation for Walker C1.

This node does not use GMR.  It publishes only the two head joints, fourteen
arm joints, and the two six-value SDK hand commands.  The same SDK topics are
used in simulation and on hardware; ROS_DOMAIN_ID selects the destination.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from mc_state_msgs.msg import RobotState
from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand

try:
    from .constants import (
        LEFT_ARM_JOINT_NAMES,
        LEFT_HAND_JOINT_NAMES,
        RIGHT_ARM_JOINT_NAMES,
        RIGHT_HAND_JOINT_NAMES,
        TASK_RESET_BODY_POSE,
    )
    from .dual_arm_ik import ArmIK, DEFAULT_FULL_URDF
    from .pico_math import (
        ControllerPoseLiveness,
        Pose,
        horizontal_heading,
        pose7_to_robot,
        relative_target,
        slew,
        yaw_pitch_delta,
    )
    from .pico_source import MockPicoSource, PicoFrame, PicoSource
    from .pico_episode_recorder import PicoEpisodeRecorder
except ImportError:
    from constants import (
        LEFT_ARM_JOINT_NAMES,
        LEFT_HAND_JOINT_NAMES,
        RIGHT_ARM_JOINT_NAMES,
        RIGHT_HAND_JOINT_NAMES,
        TASK_RESET_BODY_POSE,
    )
    from dual_arm_ik import ArmIK, DEFAULT_FULL_URDF
    from pico_math import (
        ControllerPoseLiveness,
        Pose,
        horizontal_heading,
        pose7_to_robot,
        relative_target,
        slew,
        yaw_pitch_delta,
    )
    from pico_source import MockPicoSource, PicoFrame, PicoSource
    from pico_episode_recorder import PicoEpisodeRecorder


_THIS_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = _THIS_DIR / "pico_teleop_config.json"
HEAD_JOINT_NAMES = ["head_yaw_joint", "head_pitch_joint"]
PREVIEW_JOINT_NAMES = (
    HEAD_JOINT_NAMES + LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
    + LEFT_HAND_JOINT_NAMES + RIGHT_HAND_JOINT_NAMES
)


@dataclass
class TeleopAnchors:
    headset: Pose
    operator_to_tracking: np.ndarray
    left_controller: Pose
    right_controller: Pose
    left_palm: Pose
    right_palm: Pose
    head: list[float]


class StatusWriter:
    def __init__(self):
        data_dir = Path(os.getenv("ROBOT_INSIGHT_DATA_DIR", "/tmp"))
        self.path = data_dir / "pico_connection_status.json"
        self.last_write = 0.0
        self.warned = False

    def update(self, connected: bool, armed: bool, mode: str, force: bool = False) -> None:
        now = time.time()
        if not force and now - self.last_write < 1.0:
            return
        self.last_write = now
        payload = {
            "has_data": bool(connected),
            "armed": bool(armed),
            "mode": mode,
            "timestamp_ms": int(now * 1000),
            # Preserve Thinker Studio's existing status-file contract.
            "service": "teleop",
            "robot": "walker_c1",
        }
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(payload), encoding="utf-8")
        except OSError as exc:
            if not self.warned:
                print(f"warning: cannot write Thinker Studio status {self.path}: {exc}", file=sys.stderr)
                self.warned = True


class WalkerC1PicoTeleop(Node):
    def __init__(
        self,
        config: dict,
        mode: str,
        command_enabled: bool,
        urdf_path: str = DEFAULT_FULL_URDF,
        record: bool = False,
        record_root: str = "/ubt_sim/dataset/walker_c1_pico",
        camera_topic: str = "/sensor/camera/head/color/raw",
        record_hz: float = 30.0,
    ):
        super().__init__("walker_c1_pico_teleop")
        self.config = config
        self.mode = mode
        self.command_enabled = bool(command_enabled and mode != "preview")

        self.body_pub = self.create_publisher(RobotCommand, "/mc/sdk/robot_command", 10)
        self.left_hand_pub = self.create_publisher(JointCommand, "/mc/left_hand/command", 10)
        self.right_hand_pub = self.create_publisher(JointCommand, "/mc/right_hand/command", 10)
        self.preview_pub = self.create_publisher(JointState, "/pico/joint_states", 10)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            RobotState, "/mc/sdk/robot_state", self._state_callback, state_qos
        )

        self.left_ik = ArmIK("left", urdf_path)
        self.right_ik = ArmIK("right", urdf_path)
        self.joint_positions: dict[str, float] = {}
        self.anchors: Optional[TeleopAnchors] = None
        self.last_left: Optional[list[float]] = None
        self.last_right: Optional[list[float]] = None
        self.last_head: Optional[list[float]] = None
        self.last_warning_time = 0.0
        self.emergency_latched = False
        self.episode_reset_blocked = False
        self.episode_reset_complete = False
        self.controller_liveness = {
            "left": ControllerPoseLiveness(),
            "right": ControllerPoseLiveness(),
        }
        self.recorder = (
            PicoEpisodeRecorder(self, record_root, camera_topic, record_hz)
            if record else None
        )

        action = "COMMAND" if self.command_enabled else "PREVIEW ONLY"
        self.get_logger().info(f"PICO teleop mode={mode}, {action}; hold right B to move")

    @property
    def armed(self) -> bool:
        return self.anchors is not None

    def _state_callback(self, msg: RobotState) -> None:
        self.joint_positions.update(zip(msg.joint_states.name, msg.joint_states.position))

    def _current(self, names: list[str]) -> list[float]:
        return [float(self.joint_positions[name]) for name in names]

    def _state_ready(self) -> bool:
        required = HEAD_JOINT_NAMES + LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        return all(name in self.joint_positions for name in required)

    def _arm_posture_ready(self, side: str) -> bool:
        # Near-zero elbow pitch is the straight-arm singular posture used by
        # the simulator's HOME pose.  Full-pose 7-DoF IK is not safe there.
        limit = float(self.config["max_elbow_pitch_for_teleop_rad"])
        prefix = "L" if side == "left" else "R"
        return float(self.joint_positions[f"{prefix}_elbow_pitch_joint"]) <= limit

    def disarm(self, reason: Optional[str] = None) -> None:
        if self.anchors is not None and reason:
            self.get_logger().info(f"teleop disarmed: {reason}")
        self.anchors = None

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_time > 1.0:
            self.get_logger().warn(message)
            self.last_warning_time = now

    def _capture_anchors(self, head: Pose, left: Pose, right: Pose) -> None:
        left_joints = self._current(LEFT_ARM_JOINT_NAMES)
        right_joints = self._current(RIGHT_ARM_JOINT_NAMES)
        left_fk = self.left_ik.fk(left_joints)
        right_fk = self.right_ik.fk(right_joints)
        self.anchors = TeleopAnchors(
            headset=head,
            operator_to_tracking=horizontal_heading(head),
            left_controller=left,
            right_controller=right,
            left_palm=Pose(left_fk[:3, 3].copy(), left_fk[:3, :3].copy()),
            right_palm=Pose(right_fk[:3, 3].copy(), right_fk[:3, :3].copy()),
            head=(
                self._current(HEAD_JOINT_NAMES)
                if self.config.get("head_tracking_enabled", True)
                else [float(TASK_RESET_BODY_POSE[name]) for name in HEAD_JOINT_NAMES]
            ),
        )
        self.last_left = (
            left_joints
            if "left" in set(self.config.get("teleop_sides", ("left", "right")))
            else [float(TASK_RESET_BODY_POSE[name]) for name in LEFT_ARM_JOINT_NAMES]
        )
        self.last_right = right_joints
        self.last_head = list(self.anchors.head)
        self.get_logger().info("teleop armed; controller and robot anchors captured")
        if self.recorder is not None:
            self.recorder.start_episode()

    def _publish_body(self, positions: dict[str, float]) -> None:
        msg = RobotCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, position in positions.items():
            command = JointCmd()
            command.name = name
            command.control_mode = JointCmd.MODE_POSITION
            command.position = float(position)
            msg.joint_cmd.append(command)
        self.body_pub.publish(msg)
        if self.recorder is not None:
            self.recorder.note_body_command(positions)

    def _publish_hand(self, side: str, positions: list[float]) -> None:
        names = LEFT_HAND_JOINT_NAMES if side == "left" else RIGHT_HAND_JOINT_NAMES
        publisher = self.left_hand_pub if side == "left" else self.right_hand_pub
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.names = list(names)
        msg.position = [float(value) for value in positions]
        # Astron hand controllers use their SDK-specific position mode 5.
        # This is intentionally different from JointCmd.MODE_POSITION == 2.
        msg.mode = [5] * len(names)
        publisher.publish(msg)
        if self.recorder is not None:
            self.recorder.note_hand_command(side, positions)

    def _publish_preview(
        self,
        head: list[float],
        left: list[float],
        right: list[float],
        left_hand: list[float],
        right_hand: list[float],
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(PREVIEW_JOINT_NAMES)
        msg.position = head + left + right + left_hand + right_hand
        self.preview_pub.publish(msg)

    def step(self, frame: PicoFrame) -> None:
        teleop_sides = set(self.config.get("teleop_sides", ("left", "right")))
        basis = self.config["tracking_to_robot"]
        try:
            headset = pose7_to_robot(frame.headset_pose, basis)
            left_controller = pose7_to_robot(frame.left_controller_pose, basis)
            right_controller = pose7_to_robot(frame.right_controller_pose, basis)
        except (TypeError, ValueError) as exc:
            self.disarm("invalid PICO pose")
            self._warn_throttled(f"invalid PICO pose: {exc}")
            return

        controls = frame.controls
        left_controls = controls.get("LeftController", {})
        right_controls = controls.get("RightController", {})
        now = time.monotonic()
        controllers_live = {
            "left": self.controller_liveness["left"].update(
                frame.left_controller_pose, now
            ),
            "right": self.controller_liveness["right"].update(
                frame.right_controller_pose, now
            ),
        }
        if bool(left_controls.get("axis_click", False)):
            self.emergency_latched = True
            self.disarm("left stick emergency latch")
            self._warn_throttled("emergency latched; release buttons, then press right A to clear")
            return
        if self.emergency_latched:
            if bool(right_controls.get("key_one", False)) and not bool(right_controls.get("key_two", False)):
                self.emergency_latched = False
                self.get_logger().info("emergency latch cleared")
            return

        deadman = bool(right_controls.get("key_two", False))
        if self.episode_reset_blocked:
            if self.episode_reset_complete and not deadman:
                self.episode_reset_blocked = False
                self.episode_reset_complete = False
                self.get_logger().info("ready for next episode; press and hold right B")
            return
        if not deadman:
            self.disarm("right B released")
            return
        stale_controllers = [
            side for side, live in controllers_live.items()
            if side in teleop_sides and not live
        ]
        if stale_controllers:
            self.disarm("controller pose stopped")
            self._warn_throttled(
                "controller pose has not updated since startup: " + ", ".join(stale_controllers)
            )
            return
        if not self._state_ready():
            self.disarm()
            self._warn_throttled("waiting for all head and arm joints on /mc/sdk/robot_state")
            return
        if self.anchors is None:
            self._capture_anchors(headset, left_controller, right_controller)

        anchors = self.anchors
        left_target = None
        if "left" in teleop_sides:
            left_target = relative_target(
                anchors.left_palm,
                anchors.left_controller,
                left_controller,
                self.config["translation_scale"],
                self.config["max_controller_displacement_m"],
                self.config["left_workspace"],
                anchors.operator_to_tracking,
            )
        right_target = relative_target(
            anchors.right_palm,
            anchors.right_controller,
            right_controller,
            self.config["translation_scale"],
            self.config["max_controller_displacement_m"],
            self.config["right_workspace"],
            anchors.operator_to_tracking,
        )
        def solve_arm(side: str, target: Pose) -> Optional[list[float]]:
            solver = self.left_ik if side == "left" else self.right_ik
            seed = self.last_left if side == "left" else self.last_right
            if not self._arm_posture_ready(side):
                solver.last_rejection_reason = "arm is too straight for stable IK"
                return None
            return solver.solve(
                target.position,
                target.rotation,
                seed,
                self.config["max_ik_position_error_m"],
                self.config["max_ik_rotation_error_rad"],
                self.config["max_ik_solution_jump_rad"],
                self.config["ik_orientation_weight_m_per_rad"],
                self.config["ik_seed_regularization_weight"],
                self.config["ik_max_nfev"],
                self.config["max_ik_target_position_step_m"],
                self.config["max_ik_target_rotation_step_rad"],
                self.config["max_commanded_elbow_pitch_rad"],
            )

        left_solution = (
            solve_arm("left", left_target)
            if "left" in teleop_sides
            else [float(TASK_RESET_BODY_POSE[name]) for name in LEFT_ARM_JOINT_NAMES]
        )
        right_solution = (
            solve_arm("right", right_target)
            if "right" in teleop_sides
            else list(self.last_right)
        )
        if left_solution is None or right_solution is None:
            reasons = []
            if left_solution is None:
                reasons.append(f"left: {self.left_ik.last_rejection_reason or 'unknown'}")
            if right_solution is None:
                reasons.append(f"right: {self.right_ik.last_rejection_reason or 'unknown'}")
            self._warn_throttled(
                "IK rejected this arm for the frame; holding only that arm; " + "; ".join(reasons)
            )
        if left_solution is None:
            left_solution = list(self.last_left)
        if right_solution is None:
            right_solution = list(self.last_right)

        max_joint_step = float(self.config["max_joint_step_rad"])
        left_command = slew(self.last_left, left_solution, max_joint_step)
        right_command = slew(self.last_right, right_solution, max_joint_step)
        if self.config.get("head_tracking_enabled", True):
            yaw_delta, pitch_delta = yaw_pitch_delta(
                anchors.headset, headset, anchors.operator_to_tracking
            )
            head_target = [
                anchors.head[0] + self.config["head_yaw_scale"] * yaw_delta,
                anchors.head[1] + self.config["head_pitch_scale"] * pitch_delta,
            ]
            head_target[0] = float(np.clip(head_target[0], *self.config["head_yaw_limits"]))
            head_target[1] = float(np.clip(head_target[1], *self.config["head_pitch_limits"]))
            head_command = slew(
                self.last_head, head_target, float(self.config["max_head_step_rad"])
            )
        else:
            head_command = list(anchors.head)

        close_pose = np.asarray(self.config["hand_close_pose"], dtype=float)
        left_trigger = (
            float(np.clip(left_controls.get("index_trig", 0.0), 0.0, 1.0))
            if "left" in teleop_sides else 0.0
        )
        right_trigger = (
            float(np.clip(right_controls.get("index_trig", 0.0), 0.0, 1.0))
            if "right" in teleop_sides else 0.0
        )
        left_hand = (close_pose * left_trigger).tolist()
        right_hand = (close_pose * right_trigger).tolist()

        self._publish_preview(head_command, left_command, right_command, left_hand, right_hand)
        if self.command_enabled:
            body = dict(zip(
                HEAD_JOINT_NAMES + LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES,
                head_command + left_command + right_command,
            ))
            self._publish_body(body)
            self._publish_hand("left", left_hand)
            self._publish_hand("right", right_hand)

        self.last_head = head_command
        self.last_left = left_command
        self.last_right = right_command


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _validate_mode(mode: str, enable_command: bool, confirm_real_robot: bool) -> None:
    if not enable_command or mode == "preview":
        return
    domain = os.getenv("ROS_DOMAIN_ID", "0")
    expected = "146" if mode == "sim" else "0"
    if domain != expected:
        raise SystemExit(f"mode={mode} requires ROS_DOMAIN_ID={expected}, got {domain!r}")
    if mode == "real" and not confirm_real_robot:
        raise SystemExit("real commands require --confirm-real-robot")


def _make_source(source_name: str, mode: str):
    if source_name == "mock":
        if mode == "real":
            raise SystemExit("mock PICO data is forbidden in real mode")
        return MockPicoSource()
    return PicoSource()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("preview", "sim", "real"), default="preview")
    parser.add_argument("--source", choices=("sdk", "mock"), default="sdk")
    parser.add_argument("--enable-command", action="store_true", help="publish Walker SDK commands")
    parser.add_argument("--confirm-real-robot", action="store_true")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--urdf", default=DEFAULT_FULL_URDF)
    parser.add_argument("--record", action="store_true", help="record Space-delimited HDF5 episodes")
    parser.add_argument("--record-root", default="/ubt_sim/dataset/walker_c1_pico")
    parser.add_argument("--camera-topic", default="/sensor/camera/head/color/raw")
    parser.add_argument("--record-hz", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    _validate_mode(args.mode, args.enable_command, args.confirm_real_robot)
    config = _load_config(args.config)
    status = StatusWriter()
    rclpy.init()
    node: Optional[WalkerC1PicoTeleop] = None
    source = None
    try:
        node = WalkerC1PicoTeleop(
            config, args.mode, args.enable_command, args.urdf,
            args.record, args.record_root, args.camera_topic, args.record_hz,
        )
        source = _make_source(args.source, args.mode)
        period = 1.0 / float(config["rate_hz"])
        last_source_timestamp: Optional[int] = None
        last_source_change = time.monotonic()
        while rclpy.ok():
            started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            connected = False
            try:
                frame = source.read()
                now = time.monotonic()
                if frame.timestamp_ns != last_source_timestamp:
                    last_source_timestamp = frame.timestamp_ns
                    last_source_change = now
                stale_for = now - last_source_change
                stale_timeout = float(config["source_stale_timeout_s"])
                timed_out = stale_timeout > 0.0 and stale_for > stale_timeout
                if frame.timestamp_ns <= 0 or timed_out:
                    node.disarm("PICO timestamp stopped")
                    node._warn_throttled(f"PICO data stale for {stale_for:.3f} s")
                else:
                    node.step(frame)
                    connected = True
            except Exception as exc:
                node.disarm("PICO read failed")
                node._warn_throttled(f"PICO read failed: {exc}")
            status.update(connected, node.armed, args.mode)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        return 0
    finally:
        status.update(False, False, args.mode, force=True)
        if source is not None:
            try:
                source.close()
            except Exception as exc:
                print(f"warning: cannot close PICO source cleanly: {exc}", file=sys.stderr)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
