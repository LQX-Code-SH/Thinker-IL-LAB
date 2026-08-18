"""PICO/OpenXR hand-skeleton data types and geometry helpers.

This module deliberately does not command the robot.  It preserves all 26
OpenXR hand joints so acquisition, coordinate validation, and retargeting can
be tested independently from the existing controller teleoperation path.

XRoboToolkit pose arrays use ``[x, y, z, qx, qy, qz, qw]``:

* position unit: meter
* quaternion order: ``xyzw``
* source axes: OpenXR right-handed, +X right, +Y up, -Z forward
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

try:
    from .pico_math import pose7_to_robot, quaternion_xyzw_to_matrix
except ImportError:
    from pico_math import pose7_to_robot, quaternion_xyzw_to_matrix


OPENXR_HAND_JOINT_NAMES = (
    "palm",
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_metacarpal",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)

FINGERTIP_JOINT_NAMES = (
    "thumb_tip",
    "index_tip",
    "middle_tip",
    "ring_tip",
    "little_tip",
)

FINGER_JOINT_CHAINS = {
    "thumb": (
        "thumb_metacarpal",
        "thumb_proximal",
        "thumb_distal",
        "thumb_tip",
    ),
    "index": (
        "index_metacarpal",
        "index_proximal",
        "index_intermediate",
        "index_distal",
        "index_tip",
    ),
    "middle": (
        "middle_metacarpal",
        "middle_proximal",
        "middle_intermediate",
        "middle_distal",
        "middle_tip",
    ),
    "ring": (
        "ring_metacarpal",
        "ring_proximal",
        "ring_intermediate",
        "ring_distal",
        "ring_tip",
    ),
    "little": (
        "little_metacarpal",
        "little_proximal",
        "little_intermediate",
        "little_distal",
        "little_tip",
    ),
}

OPENXR_REFERENCE_SPACE = (
    "PICO/OpenXR local tracking space: right-handed, +X right, +Y up, -Z forward"
)
ROBOT_AXIS_REFERENCE_SPACE = (
    "OpenXR tracking origin with robot axes: +X forward, +Y left, +Z up"
)
QUATERNION_ORDER = "xyzw"
POSITION_UNIT = "meter"


class HandTrackingFormatError(ValueError):
    """Raised when an SDK hand-skeleton frame has the wrong shape or type."""


@dataclass(frozen=True)
class HandJointPose:
    """One named hand joint pose.

    ``quaternion_xyzw`` is kept explicit in the field name to prevent a later
    retargeter from silently treating it as ``wxyz``.
    """

    name: str
    position_m: np.ndarray
    quaternion_xyzw: np.ndarray
    is_valid: bool

    def as_pose7(self) -> list[float]:
        return np.concatenate((self.position_m, self.quaternion_xyzw)).astype(float).tolist()


@dataclass(frozen=True)
class HandTrackingData:
    """A complete left or right OpenXR hand skeleton for one SDK frame."""

    side: str
    wrist_pose: HandJointPose
    palm_pose: HandJointPose
    joint_positions: Mapping[str, np.ndarray]
    joint_orientations_xyzw: Mapping[str, np.ndarray]
    joint_validity: Mapping[str, bool]
    is_tracked: bool
    tracking_quality: int
    timestamp_ns: int
    reference_space: str
    quaternion_order: str = QUATERNION_ORDER
    position_unit: str = POSITION_UNIT

    def joint(self, name: str) -> HandJointPose:
        if name not in self.joint_positions:
            raise KeyError(f"unknown OpenXR hand joint: {name}")
        return HandJointPose(
            name=name,
            position_m=np.asarray(self.joint_positions[name], dtype=float).copy(),
            quaternion_xyzw=np.asarray(
                self.joint_orientations_xyzw[name], dtype=float
            ).copy(),
            is_valid=bool(self.joint_validity[name]),
        )

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable representation retaining all 26 joints."""
        return {
            "side": self.side,
            "is_tracked": bool(self.is_tracked),
            "tracking_quality": int(self.tracking_quality),
            "timestamp_ns": int(self.timestamp_ns),
            "reference_space": self.reference_space,
            "position_unit": self.position_unit,
            "quaternion_order": self.quaternion_order,
            "joints": [
                {
                    "name": name,
                    "position_m": np.asarray(
                        self.joint_positions[name], dtype=float
                    ).tolist(),
                    "quaternion_xyzw": np.asarray(
                        self.joint_orientations_xyzw[name], dtype=float
                    ).tolist(),
                    "is_valid": bool(self.joint_validity[name]),
                }
                for name in OPENXR_HAND_JOINT_NAMES
            ],
        }


def matrix_to_quaternion_xyzw(matrix: Sequence[Sequence[float]]) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to a normalized ``xyzw`` quaternion."""
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5):
        raise ValueError("rotation matrix must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError("rotation matrix must be proper")

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale

    quaternion = np.asarray([x, y, z, w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    # q and -q encode the same rotation.  A stable sign makes logs/tests easier
    # to compare without changing the represented orientation.
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def _invalid_pose_values(
    raw_pose: np.ndarray,
    tracking_to_output: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray]:
    position = raw_pose[:3].copy()
    if tracking_to_output is not None and np.all(np.isfinite(position)):
        position = tracking_to_output @ position
    quaternion = raw_pose[3:].copy()
    return position, quaternion


def parse_hand_tracking_state(
    side: str,
    raw_joint_poses: Sequence[Sequence[float]],
    tracking_quality: int,
    timestamp_ns: int,
    tracking_to_output: Sequence[Sequence[float]] | None = None,
) -> HandTrackingData:
    """Validate and name one XRoboToolkit 26x7 hand-skeleton frame.

    The current Python binding exposes only hand-level ``isActive`` quality;
    PICO's per-joint OpenXR location flags are not present in version 1.0.2.
    Therefore ``joint_validity`` is conservatively derived from hand quality,
    finite values, and a non-zero quaternion.  No confidence is fabricated.
    """
    side = str(side).lower()
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    try:
        values = np.asarray(raw_joint_poses, dtype=float)
    except (TypeError, ValueError) as exc:
        raise HandTrackingFormatError("hand poses must be numeric") from exc
    expected_shape = (len(OPENXR_HAND_JOINT_NAMES), 7)
    if values.shape != expected_shape:
        raise HandTrackingFormatError(
            f"{side} hand must contain 26 poses of 7 values, got {values.shape}"
        )

    basis = None
    reference_space = OPENXR_REFERENCE_SPACE
    if tracking_to_output is not None:
        basis = np.asarray(tracking_to_output, dtype=float)
        if basis.shape != (3, 3) or not np.all(np.isfinite(basis)):
            raise ValueError("tracking_to_output must be a finite 3x3 matrix")
        if not np.allclose(basis @ basis.T, np.eye(3), atol=1e-5):
            raise ValueError("tracking_to_output must be orthonormal")
        if not np.isclose(np.linalg.det(basis), 1.0, atol=1e-5):
            raise ValueError("tracking_to_output must be a proper rotation")
        reference_space = ROBOT_AXIS_REFERENCE_SPACE

    quality = int(tracking_quality)
    quality_valid = quality > 0
    positions: dict[str, np.ndarray] = {}
    orientations: dict[str, np.ndarray] = {}
    validity: dict[str, bool] = {}

    for name, raw_pose in zip(OPENXR_HAND_JOINT_NAMES, values):
        finite = bool(np.all(np.isfinite(raw_pose)))
        quaternion_norm = float(np.linalg.norm(raw_pose[3:])) if finite else 0.0
        joint_valid = quality_valid and finite and quaternion_norm > 1e-8
        if joint_valid:
            if basis is None:
                position = raw_pose[:3].copy()
                quaternion = raw_pose[3:] / quaternion_norm
            else:
                converted = pose7_to_robot(raw_pose, basis)
                position = converted.position
                quaternion = matrix_to_quaternion_xyzw(converted.rotation)
        else:
            position, quaternion = _invalid_pose_values(raw_pose, basis)
        positions[name] = np.asarray(position, dtype=float)
        orientations[name] = np.asarray(quaternion, dtype=float)
        validity[name] = joint_valid

    # The binding supplies no per-joint status, so require a complete valid
    # skeleton before declaring it safe for a future robot-control consumer.
    is_tracked = quality_valid and all(validity.values())

    def make_pose(name: str) -> HandJointPose:
        return HandJointPose(
            name=name,
            position_m=positions[name].copy(),
            quaternion_xyzw=orientations[name].copy(),
            is_valid=validity[name],
        )

    return HandTrackingData(
        side=side,
        wrist_pose=make_pose("wrist"),
        palm_pose=make_pose("palm"),
        joint_positions=positions,
        joint_orientations_xyzw=orientations,
        joint_validity=validity,
        is_tracked=is_tracked,
        tracking_quality=quality,
        timestamp_ns=int(timestamp_ns),
        reference_space=reference_space,
    )


def get_fingertip_positions(
    hand: HandTrackingData,
    valid_only: bool = True,
) -> dict[str, np.ndarray]:
    """Return named fingertip positions in meters in ``hand.reference_space``."""
    result = {}
    for name in FINGERTIP_JOINT_NAMES:
        if valid_only and not hand.joint_validity[name]:
            continue
        result[name] = np.asarray(hand.joint_positions[name], dtype=float).copy()
    return result


def get_finger_flexion(hand: HandTrackingData, finger: str) -> float:
    """Estimate finger flexion as the sum of inter-segment bend angles.

    The result is in radians, is zero for a straight synthetic chain, and does
    not assume a robot-hand kinematic model.  It is intended as an observable
    for MVP validation and a later retargeting feature, not as a robot command.
    """
    finger = str(finger).lower()
    if finger not in FINGER_JOINT_CHAINS:
        raise ValueError(f"finger must be one of {tuple(FINGER_JOINT_CHAINS)}")
    chain = FINGER_JOINT_CHAINS[finger]
    if not all(hand.joint_validity[name] for name in chain):
        raise ValueError(f"{finger} joint chain is not valid")
    points = [np.asarray(hand.joint_positions[name], dtype=float) for name in chain]
    segments = [second - first for first, second in zip(points[:-1], points[1:])]
    norms = [float(np.linalg.norm(segment)) for segment in segments]
    if any(norm < 1e-8 for norm in norms):
        raise ValueError(f"{finger} joint chain contains a zero-length segment")
    directions = [segment / norm for segment, norm in zip(segments, norms)]
    angles = [
        float(np.arccos(np.clip(np.dot(first, second), -1.0, 1.0)))
        for first, second in zip(directions[:-1], directions[1:])
    ]
    return float(sum(angles))


def get_thumb_opposition(
    hand: HandTrackingData,
    target_finger: str = "index",
) -> float:
    """Return thumb-tip to target-fingertip distance in meters."""
    target_finger = str(target_finger).lower()
    if target_finger not in ("index", "middle", "ring", "little"):
        raise ValueError("target_finger must be index, middle, ring, or little")
    target_name = f"{target_finger}_tip"
    if not hand.joint_validity["thumb_tip"] or not hand.joint_validity[target_name]:
        raise ValueError("thumb opposition fingertips are not valid")
    thumb = np.asarray(hand.joint_positions["thumb_tip"], dtype=float)
    target = np.asarray(hand.joint_positions[target_name], dtype=float)
    return float(np.linalg.norm(thumb - target))


def retarget_hand(
    human_hand_data: HandTrackingData,
    *,
    retargeter: Any | None = None,
    config_path: str | None = None,
    dt_s: float | None = None,
) -> np.ndarray:
    """Map a complete hand skeleton to the C1 11DoF joint order.

    The implementation lives in the independent ``pico_hand_retarget`` module
    so the acquisition-only receiver does not import mapping state unless a
    caller explicitly invokes this function.  Real-time callers should pass a
    persistent ``C1HandRetargeter`` to retain smoothing and tracking-loss hold.
    """
    try:
        from .pico_hand_retarget import retarget_hand as _retarget_impl
    except ImportError:
        from pico_hand_retarget import retarget_hand as _retarget_impl

    return _retarget_impl(
        human_hand_data,
        retargeter=retargeter,
        config_path=config_path,
        dt_s=dt_s,
    )
