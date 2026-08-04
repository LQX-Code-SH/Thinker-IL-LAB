"""Pure math helpers for controller-relative PICO teleoperation."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation: np.ndarray


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
    """Convert SDK ``[x,y,z,qx,qy,qz,qw]`` into robot-base axes."""
    values = np.asarray(raw_pose, dtype=float).reshape(-1)
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise ValueError("PICO pose must contain seven finite values")
    basis = np.asarray(tracking_to_robot, dtype=float)
    if basis.shape != (3, 3) or not np.allclose(basis @ basis.T, np.eye(3), atol=1e-5):
        raise ValueError("tracking_to_robot must be a 3x3 orthonormal matrix")
    source_rotation = quaternion_xyzw_to_matrix(values[3:])
    return Pose(basis @ values[:3], basis @ source_rotation @ basis.T)


def relative_target(
    robot_anchor: Pose,
    controller_anchor: Pose,
    controller_now: Pose,
    translation_scale: float,
    max_displacement: float,
    workspace: Sequence[Sequence[float]],
) -> Pose:
    delta = (controller_now.position - controller_anchor.position) * float(translation_scale)
    distance = float(np.linalg.norm(delta))
    if distance > max_displacement > 0.0:
        delta *= float(max_displacement) / distance
    position = robot_anchor.position + delta
    bounds = np.asarray(workspace, dtype=float)
    if bounds.shape != (3, 2):
        raise ValueError("workspace must be [[xmin,xmax],[ymin,ymax],[zmin,zmax]]")
    position = np.clip(position, bounds[:, 0], bounds[:, 1])
    world_delta = controller_now.rotation @ controller_anchor.rotation.T
    return Pose(position, world_delta @ robot_anchor.rotation)


def rotation_error(target: np.ndarray, reached: np.ndarray) -> float:
    delta = np.asarray(target).T @ np.asarray(reached)
    return float(np.arccos(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)))


def yaw_pitch_delta(anchor: Pose, current: Pose) -> tuple[float, float]:
    """Return base-frame yaw and pitch change, in radians."""
    delta = current.rotation @ anchor.rotation.T
    yaw = float(np.arctan2(delta[1, 0], delta[0, 0]))
    pitch = float(np.arctan2(-delta[2, 0], np.hypot(delta[2, 1], delta[2, 2])))
    return yaw, pitch


def slew(previous: Sequence[float], target: Sequence[float], max_step: float) -> list[float]:
    old = np.asarray(previous, dtype=float)
    new = np.asarray(target, dtype=float)
    if old.shape != new.shape:
        raise ValueError("previous and target must have the same shape")
    return np.clip(new, old - max_step, old + max_step).astype(float).tolist()
