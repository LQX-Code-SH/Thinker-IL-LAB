"""Pure math helpers for controller-relative PICO teleoperation."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation: np.ndarray


@dataclass
class ControllerPoseLiveness:
    """Require each controller to prove that it has delivered live pose data.

    The vendor SDK exposes one global timestamp, which can continue advancing
    when only the headset is live.  Requiring one observed controller-pose
    change after process startup prevents a stale cached pose from immediately
    becoming a teleoperation anchor.  The SDK legitimately repeats an exactly
    unchanged pose while a controller is held still, so repetition alone cannot
    be used as an ongoing disconnect timeout.
    """

    last_pose: Optional[np.ndarray] = None
    last_change_time: float = 0.0
    observed_change: bool = False

    def update(self, raw_pose: Sequence[float], now: float) -> bool:
        pose = np.asarray(raw_pose, dtype=float).reshape(-1)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            return False
        if self.last_pose is None:
            self.last_pose = pose.copy()
            self.last_change_time = float(now)
            return False
        if not np.array_equal(pose, self.last_pose):
            self.last_pose = pose.copy()
            self.last_change_time = float(now)
            self.observed_change = True
        return self.observed_change


def quaternion_xyzw_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite xyzw values")
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose7_to_robot(raw_pose: Sequence[float], tracking_to_robot: Sequence[Sequence[float]]) -> Pose:
    """Convert an SDK OpenXR pose into robot-base axes.

    Live PICO calibration shows the SDK uses the OpenXR right-handed convention:
    +X right, +Y up, and -Z forward.  ``tracking_to_robot`` is therefore a proper
    rotation, so position and orientation use the same ordinary basis change.
    """
    values = np.asarray(raw_pose, dtype=float).reshape(-1)
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise ValueError("PICO pose must contain seven finite values")
    basis = np.asarray(tracking_to_robot, dtype=float)
    if basis.shape != (3, 3) or not np.allclose(basis @ basis.T, np.eye(3), atol=1e-5):
        raise ValueError("tracking_to_robot must be a 3x3 orthonormal matrix")
    if not np.isclose(np.linalg.det(basis), 1.0, atol=1e-5):
        raise ValueError("tracking_to_robot must be a proper rotation matrix")
    source_rotation = quaternion_xyzw_to_matrix(values[3:])
    return Pose(basis @ values[:3], basis @ source_rotation @ basis.T)


def horizontal_heading(pose: Pose) -> np.ndarray:
    """Return the yaw-only operator-to-tracking rotation for a headset pose.

    Poses have already been expressed using robot-style axes: +X forward, +Y
    left, and +Z up.  The headset's local +X is therefore its forward axis.
    Projecting it onto the horizontal plane makes controller commands follow
    the operator's facing direction without allowing head pitch or roll to
    tilt the command frame.
    """
    rotation = np.asarray(pose.rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("pose rotation must be a finite 3x3 matrix")
    forward_xy = rotation[:2, 0]
    norm = float(np.linalg.norm(forward_xy))
    if norm < 1e-6:
        raise ValueError("headset forward axis is too close to vertical")
    forward_x, forward_y = forward_xy / norm
    return np.array([
        [forward_x, -forward_y, 0.0],
        [forward_y, forward_x, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _rotation_in_reference(delta_world: np.ndarray, reference_to_world: Optional[np.ndarray]) -> np.ndarray:
    if reference_to_world is None:
        return delta_world
    reference = np.asarray(reference_to_world, dtype=float)
    if (
        reference.shape != (3, 3)
        or not np.all(np.isfinite(reference))
        or not np.allclose(reference @ reference.T, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(reference), 1.0, atol=1e-5)
    ):
        raise ValueError("reference_to_world must be a proper 3x3 rotation matrix")
    return reference.T @ delta_world @ reference


def relative_target(
    robot_anchor: Pose,
    controller_anchor: Pose,
    controller_now: Pose,
    translation_scale: float,
    max_displacement: float,
    workspace: Sequence[Sequence[float]],
    reference_to_world: Optional[np.ndarray] = None,
) -> Pose:
    delta_world = controller_now.position - controller_anchor.position
    if reference_to_world is not None:
        # Validate once even when the controller has not rotated.
        _rotation_in_reference(np.eye(3), reference_to_world)
        delta_world = np.asarray(reference_to_world, dtype=float).T @ delta_world
    delta = delta_world * float(translation_scale)
    distance = float(np.linalg.norm(delta))
    if distance > max_displacement > 0.0:
        delta *= float(max_displacement) / distance
    position = robot_anchor.position + delta
    bounds = np.asarray(workspace, dtype=float)
    if bounds.shape != (3, 2):
        raise ValueError("workspace must be [[xmin,xmax],[ymin,ymax],[zmin,zmax]]")
    position = np.clip(position, bounds[:, 0], bounds[:, 1])
    world_delta = controller_now.rotation @ controller_anchor.rotation.T
    reference_delta = _rotation_in_reference(world_delta, reference_to_world)
    return Pose(position, reference_delta @ robot_anchor.rotation)


def rotation_error(target: np.ndarray, reached: np.ndarray) -> float:
    delta = np.asarray(target).T @ np.asarray(reached)
    return float(np.arccos(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)))


def yaw_pitch_delta(
    anchor: Pose,
    current: Pose,
    reference_to_world: Optional[np.ndarray] = None,
) -> tuple[float, float]:
    """Return base-frame yaw and pitch change, in radians."""
    delta = current.rotation @ anchor.rotation.T
    delta = _rotation_in_reference(delta, reference_to_world)
    yaw = float(np.arctan2(delta[1, 0], delta[0, 0]))
    pitch = float(np.arctan2(-delta[2, 0], np.hypot(delta[2, 1], delta[2, 2])))
    return yaw, pitch


def slew(previous: Sequence[float], target: Sequence[float], max_step: float) -> list[float]:
    old = np.asarray(previous, dtype=float)
    new = np.asarray(target, dtype=float)
    if old.shape != new.shape:
        raise ValueError("previous and target must have the same shape")
    return np.clip(new, old - max_step, old + max_step).astype(float).tolist()
