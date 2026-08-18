"""Human wrist relative pose to Walker C1 arm targets.

This module reuses the established controller teleoperation math and IK
parameters without importing or modifying ``pico_teleop.py``.  Human wrist
tracking and robot palm FK are anchored together when control is armed, so no
absolute PICO tracking-origin offset is ever sent to the robot.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

import numpy as np

try:
    from .constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES
    from .pico_math import (
        Pose,
        horizontal_heading,
        pose7_to_robot,
        quaternion_xyzw_to_matrix,
        relative_target,
        slew,
    )
except ImportError:
    from constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES
    from pico_math import (
        Pose,
        horizontal_heading,
        pose7_to_robot,
        quaternion_xyzw_to_matrix,
        relative_target,
        slew,
    )


_THIS_DIR = Path(__file__).resolve().parent
DEFAULT_ARM_CONFIG = _THIS_DIR / "pico_teleop_config.json"
DEFAULT_TASK_RESET_SCRIPT = _THIS_DIR / "reset.py"
ARM_JOINT_NAMES = {
    "left": tuple(LEFT_ARM_JOINT_NAMES),
    "right": tuple(RIGHT_ARM_JOINT_NAMES),
}


@dataclass(frozen=True)
class HandArmAnchors:
    headset: Pose
    operator_to_tracking: np.ndarray
    wrists: Mapping[str, Pose]
    robot_palms: Mapping[str, Pose]


@dataclass(frozen=True)
class ArmRetargetResult:
    commands: Mapping[str, tuple[float, ...]]
    targets: Mapping[str, Pose]
    rejection_reasons: Mapping[str, str]
    valid: bool


def load_arm_config(path: str | Path | None = None) -> dict[str, Any]:
    config_path = Path(path) if path is not None else DEFAULT_ARM_CONFIG
    with config_path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    required = (
        "tracking_to_robot",
        "translation_scale",
        "max_controller_displacement_m",
        "left_workspace",
        "right_workspace",
        "max_joint_step_rad",
        "max_ik_position_error_m",
        "max_ik_rotation_error_rad",
        "max_ik_solution_jump_rad",
        "ik_orientation_weight_m_per_rad",
        "ik_seed_regularization_weight",
        "ik_max_nfev",
        "max_ik_target_position_step_m",
        "max_ik_target_rotation_step_rad",
        "max_elbow_pitch_for_teleop_rad",
        "max_commanded_elbow_pitch_rad",
    )
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError("arm teleop config missing: " + ", ".join(missing))
    return config


def hand_wrist_pose(hand) -> Pose:
    """Return one valid wrist pose already expressed in robot-style axes."""
    if not bool(hand.is_tracked):
        raise ValueError(f"{hand.side} hand tracking is invalid")
    wrist = hand.wrist_pose
    if not bool(wrist.is_valid):
        raise ValueError(f"{hand.side} wrist tracking is invalid")
    position = np.asarray(wrist.position_m, dtype=float)
    quaternion = np.asarray(wrist.quaternion_xyzw, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError(f"{hand.side} wrist position is invalid")
    return Pose(position.copy(), quaternion_xyzw_to_matrix(quaternion))


class C1WristArmRetargeter:
    """Stateful relative wrist-pose controller around the existing C1 ArmIK."""

    def __init__(
        self,
        sides: Sequence[str],
        *,
        config_path: str | Path | None = None,
        urdf_path: str | None = None,
        solvers: Mapping[str, object] | None = None,
    ) -> None:
        selected = tuple(str(side).lower() for side in sides)
        if not selected or any(side not in ARM_JOINT_NAMES for side in selected):
            raise ValueError("sides must contain left and/or right")
        if len(set(selected)) != len(selected):
            raise ValueError("sides must be unique")
        self.sides = selected
        self.config = load_arm_config(config_path)
        if solvers is None:
            try:
                from .dual_arm_ik import ArmIK, DEFAULT_FULL_URDF
            except ImportError:
                from dual_arm_ik import ArmIK, DEFAULT_FULL_URDF
            resolved_urdf = urdf_path or DEFAULT_FULL_URDF
            self.solvers = {side: ArmIK(side, resolved_urdf) for side in self.sides}
        else:
            self.solvers = {side: solvers[side] for side in self.sides}
        for side, solver in self.solvers.items():
            if tuple(solver.joint_names) != ARM_JOINT_NAMES[side]:
                raise ValueError(f"{side} IK joint order does not match C1 control order")

        self.joint_positions: dict[str, float] = {}
        self.anchors: HandArmAnchors | None = None
        self._last_commands: dict[str, list[float]] = {}

    @property
    def armed(self) -> bool:
        return self.anchors is not None

    def update_joint_state(self, names: Sequence[str], positions: Sequence[float]) -> None:
        for name, position in zip(names, positions):
            value = float(position)
            if np.isfinite(value):
                self.joint_positions[str(name)] = value

    def current_joints(self, side: str) -> list[float]:
        return [float(self.joint_positions[name]) for name in ARM_JOINT_NAMES[side]]

    def state_ready(self) -> bool:
        return all(
            name in self.joint_positions
            for side in self.sides
            for name in ARM_JOINT_NAMES[side]
        )

    def posture_ready(self) -> tuple[bool, str]:
        if not self.state_ready():
            return False, "waiting for selected arm joints on /mc/sdk/robot_state"
        limit = float(self.config["max_elbow_pitch_for_teleop_rad"])
        for side in self.sides:
            prefix = "L" if side == "left" else "R"
            elbow = float(self.joint_positions[f"{prefix}_elbow_pitch_joint"])
            if elbow > limit:
                return False, (
                    f"{side} arm elbow {elbow:.3f} rad is too straight; "
                    f"requires <= {limit:.3f} rad"
                )
        return True, ""

    def input_ready(self, headset_pose, hands: Mapping[str, object]) -> tuple[bool, str]:
        ready, reason = self.posture_ready()
        if not ready:
            return ready, reason
        try:
            pose7_to_robot(headset_pose, self.config["tracking_to_robot"])
            for side in self.sides:
                hand_wrist_pose(hands[side])
        except (KeyError, TypeError, ValueError) as exc:
            return False, str(exc)
        return True, ""

    def capture_anchors(self, headset_pose, hands: Mapping[str, object]) -> None:
        ready, reason = self.input_ready(headset_pose, hands)
        if not ready:
            raise ValueError(reason)
        headset = pose7_to_robot(headset_pose, self.config["tracking_to_robot"])
        wrists: dict[str, Pose] = {}
        palms: dict[str, Pose] = {}
        last: dict[str, list[float]] = {}
        for side in self.sides:
            joints = self.current_joints(side)
            fk = self.solvers[side].fk(joints)
            wrists[side] = hand_wrist_pose(hands[side])
            palms[side] = Pose(fk[:3, 3].copy(), fk[:3, :3].copy())
            last[side] = joints
        self.anchors = HandArmAnchors(
            headset=headset,
            operator_to_tracking=horizontal_heading(headset),
            wrists=wrists,
            robot_palms=palms,
        )
        self._last_commands = last

    def disarm(self) -> None:
        self.anchors = None
        self._last_commands.clear()

    def _solve_side(self, side: str, target: Pose) -> list[float] | None:
        solver = self.solvers[side]
        seed = self._last_commands[side]
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

    def solve(self, headset_pose, hands: Mapping[str, object]) -> ArmRetargetResult:
        if self.anchors is None:
            return ArmRetargetResult({}, {}, {"all": "arm anchors not captured"}, False)
        ready, reason = self.input_ready(headset_pose, hands)
        if not ready:
            return ArmRetargetResult({}, {}, {"all": reason}, False)

        targets: dict[str, Pose] = {}
        raw_solutions: dict[str, list[float]] = {}
        rejections: dict[str, str] = {}
        for side in self.sides:
            target = relative_target(
                self.anchors.robot_palms[side],
                self.anchors.wrists[side],
                hand_wrist_pose(hands[side]),
                self.config["translation_scale"],
                self.config["max_controller_displacement_m"],
                self.config[f"{side}_workspace"],
                self.anchors.operator_to_tracking,
            )
            targets[side] = target
            solution = self._solve_side(side, target)
            if solution is None:
                rejections[side] = (
                    self.solvers[side].last_rejection_reason or "unknown IK rejection"
                )
            else:
                raw_solutions[side] = solution

        # Treat the selected sides as one command transaction.  A rejected IK
        # frame publishes neither arm nor hand and does not advance any seed.
        if rejections:
            return ArmRetargetResult({}, targets, rejections, False)

        commands = {
            side: tuple(
                slew(
                    self._last_commands[side],
                    raw_solutions[side],
                    float(self.config["max_joint_step_rad"]),
                )
            )
            for side in self.sides
        }
        self._last_commands = {side: list(command) for side, command in commands.items()}
        return ArmRetargetResult(commands, targets, {}, True)


class RosHandArmSimulationIO:
    """ROS state and atomic arm+hand publishing for Isaac simulation only."""

    def __init__(self, sides: tuple[str, ...], arm_retargeter: C1WristArmRetargeter) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from mc_state_msgs.msg import RobotState
            from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
            from std_msgs.msg import Bool
        except ImportError as exc:
            raise RuntimeError(
                "hand-arm mode requires ROS2, mc_state_msgs, and mc_task_msgs"
            ) from exc
        self.rclpy = rclpy
        self.JointCmd = JointCmd
        self.JointCommand = JointCommand
        self.RobotCommand = RobotCommand
        self.Bool = Bool
        self.arm_retargeter = arm_retargeter
        self.last_state_monotonic = 0.0
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)
        self.node = Node("walker_c1_pico_hand_arm_teleop")
        self.body_publisher = self.node.create_publisher(
            RobotCommand, "/mc/sdk/robot_command", 10
        )
        self.reset_publisher = self.node.create_publisher(
            Bool, "/sim/cmd_reset", 1
        )
        self.hand_publishers = {
            side: self.node.create_publisher(
                JointCommand, f"/mc/{side}_hand/command", 10
            )
            for side in sides
        }
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.node.create_subscription(
            RobotState, "/mc/sdk/robot_state", self._state_callback, state_qos
        )

    def _state_callback(self, message) -> None:
        self.arm_retargeter.update_joint_state(
            message.joint_states.name,
            message.joint_states.position,
        )
        self.last_state_monotonic = time.monotonic()

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def warmup(self, duration_s: float = 0.6) -> None:
        deadline = time.monotonic() + float(duration_s)
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.02)

    def state_fresh(self, timeout_s: float) -> bool:
        return (
            self.last_state_monotonic > 0.0
            and time.monotonic() - self.last_state_monotonic <= float(timeout_s)
            and self.arm_retargeter.state_ready()
        )

    def reset_scene_and_robot(
        self,
        *,
        reset_script: str | Path = DEFAULT_TASK_RESET_SCRIPT,
    ) -> None:
        """Reset the Isaac scene once, then run the existing staged task reset.

        The scene command is edge-triggered and must be published only once.
        Teleoperation anchors and cached state are invalid across a reset, so
        this method clears them before doing any blocking reset work.
        """
        self.arm_retargeter.disarm()
        self.arm_retargeter.joint_positions.clear()
        self.last_state_monotonic = 0.0

        message = self.Bool()
        message.data = True
        self.reset_publisher.publish(message)
        self.spin_once(timeout_sec=0.0)
        # Match the already proven episode-reset ordering: let Isaac consume
        # the scene edge before publishing the staged task pose.
        time.sleep(0.2)

        result = subprocess.run(
            [sys.executable, str(Path(reset_script)), "--mode", "task"],
            check=False,
            text=True,
            capture_output=True,
        )
        if result.returncode != 0:
            details = (result.stderr or result.stdout or "unknown reset error").strip()
            raise RuntimeError(
                f"task reset failed ({result.returncode}): {details}"
            )

        # Discard any RobotState queued during the blocking subprocess.  A new
        # post-reset state frame is required before the operator can arm again.
        self.arm_retargeter.joint_positions.clear()
        self.last_state_monotonic = 0.0

    def publish(
        self,
        arm_commands: Mapping[str, Sequence[float]],
        hand_payload: Mapping[str, Mapping[str, float]],
    ) -> None:
        body = self.RobotCommand()
        body.header.stamp = self.node.get_clock().now().to_msg()
        for side, values in arm_commands.items():
            for name, value in zip(ARM_JOINT_NAMES[side], values):
                command = self.JointCmd()
                command.name = name
                command.control_mode = self.JointCmd.MODE_POSITION
                command.position = float(value)
                body.joint_cmd.append(command)
        if body.joint_cmd:
            self.body_publisher.publish(body)

        for group, values in hand_payload.items():
            side = group.removesuffix("_hand")
            message = self.JointCommand()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.names = list(values)
            message.position = [float(value) for value in values.values()]
            message.velocity = [0.0] * len(values)
            message.torque = [0.0] * len(values)
            message.acceleration = [0.0] * len(values)
            message.mode = [5] * len(values)
            message.kp = [0.0] * len(values)
            message.kd = [0.0] * len(values)
            self.hand_publishers[side].publish(message)
        self.spin_once(timeout_sec=0.0)

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy and self.rclpy.ok():
            self.rclpy.shutdown()
