"""Synthetic OpenXR-compatible hand skeletons for offline retarget testing."""
from __future__ import annotations

import time
from typing import Mapping, Sequence

import numpy as np

try:
    from .pico_hand_tracking import (
        FINGER_JOINT_CHAINS,
        OPENXR_HAND_JOINT_NAMES,
        HandJointPose,
        HandTrackingData,
    )
except ImportError:
    from pico_hand_tracking import (
        FINGER_JOINT_CHAINS,
        OPENXR_HAND_JOINT_NAMES,
        HandJointPose,
        HandTrackingData,
    )


MOCK_HAND_POSES = (
    "open_hand",
    "fist",
    "index_flex",
    "ring_flex",
    "little_flex",
    "thumb_index_pinch",
    "tripod",
    "cylindrical_grasp",
)

_ALIASES = {
    "open": "open_hand",
    "pinch": "thumb_index_pinch",
    "ring": "ring_flex",
    "little": "little_flex",
    "cylindrical": "cylindrical_grasp",
    "cylinder": "cylindrical_grasp",
}

_LONG_FINGER_BASES = {
    "index": np.asarray([0.0, 0.030, 0.0]),
    "middle": np.asarray([0.0, 0.010, 0.0]),
    "ring": np.asarray([0.0, -0.010, 0.0]),
    "little": np.asarray([0.0, -0.030, 0.0]),
}

_LONG_FINGER_LENGTHS = {
    "index": (0.035, 0.026, 0.019, 0.013),
    "middle": (0.037, 0.028, 0.020, 0.014),
    "ring": (0.035, 0.026, 0.019, 0.013),
    "little": (0.031, 0.023, 0.017, 0.012),
}


def _long_finger_points(
    base: np.ndarray,
    lengths: Sequence[float],
    bends_rad: Sequence[float],
) -> list[np.ndarray]:
    if len(lengths) != 4 or len(bends_rad) != 3:
        raise ValueError("long-finger mock requires four bones and three bend angles")
    points = [np.asarray(base, dtype=float).copy()]
    angle = 0.0
    for index, length in enumerate(lengths):
        if index > 0:
            angle -= float(bends_rad[index - 1])
        direction = np.asarray([np.cos(angle), 0.0, np.sin(angle)], dtype=float)
        points.append(points[-1] + float(length) * direction)
    return points


def _thumb_points(base: np.ndarray, tip: np.ndarray, curve: float) -> list[np.ndarray]:
    """Build a non-degenerate, curved four-joint thumb chain ending at ``tip``."""
    base = np.asarray(base, dtype=float)
    tip = np.asarray(tip, dtype=float)
    delta = tip - base
    if float(np.linalg.norm(delta)) <= 1e-4:
        raise ValueError("mock thumb base and tip must be distinct")
    curve = float(np.clip(curve, 0.0, 1.0))
    offset = np.asarray([0.0, 0.0, -0.018 * curve], dtype=float)
    proximal = base + 0.38 * delta + offset
    distal = base + 0.72 * delta + 1.15 * offset
    return [base.copy(), proximal, distal, tip.copy()]


def _pose_bends(pose: str) -> dict[str, tuple[float, float, float]]:
    opened = (0.0, 0.0, 0.0)
    result = {finger: opened for finger in _LONG_FINGER_BASES}
    if pose == "fist":
        return {finger: (0.85, 1.0, 0.7) for finger in result}
    if pose == "index_flex":
        result["index"] = (0.85, 1.0, 0.7)
    elif pose == "ring_flex":
        result["ring"] = (0.85, 1.0, 0.7)
    elif pose == "little_flex":
        result["little"] = (0.85, 1.0, 0.7)
    elif pose == "thumb_index_pinch":
        result["index"] = (0.38, 0.55, 0.34)
    elif pose == "tripod":
        result["index"] = (0.34, 0.48, 0.30)
        result["middle"] = (0.30, 0.44, 0.28)
    elif pose == "cylindrical_grasp":
        result.update(
            {
                "index": (0.55, 0.78, 0.48),
                "middle": (0.62, 0.85, 0.52),
                "ring": (0.68, 0.92, 0.58),
                "little": (0.74, 0.98, 0.62),
            }
        )
    return result


def _right_hand_positions(pose: str) -> dict[str, np.ndarray]:
    positions: dict[str, np.ndarray] = {
        "wrist": np.asarray([-0.055, 0.0, 0.0]),
        "palm": np.asarray([0.0, 0.0, 0.0]),
    }
    bends = _pose_bends(pose)
    for finger, base in _LONG_FINGER_BASES.items():
        points = _long_finger_points(base, _LONG_FINGER_LENGTHS[finger], bends[finger])
        for name, point in zip(FINGER_JOINT_CHAINS[finger], points):
            positions[name] = point

    thumb_base = np.asarray([0.015, 0.052, 0.0])
    curve = 0.0
    if pose in ("open_hand", "index_flex", "ring_flex", "little_flex"):
        thumb_tip = np.asarray([0.060, 0.108, 0.0])
    elif pose == "fist":
        thumb_tip = positions["index_tip"] + np.asarray([0.002, 0.010, 0.003])
        curve = 0.9
    elif pose == "thumb_index_pinch":
        thumb_tip = positions["index_tip"] + np.asarray([0.0, 0.003, 0.0])
        curve = 0.72
    elif pose == "tripod":
        thumb_tip = 0.5 * (positions["index_tip"] + positions["middle_tip"])
        thumb_tip += np.asarray([0.0, 0.003, 0.0])
        curve = 0.68
    elif pose == "cylindrical_grasp":
        thumb_tip = 0.55 * positions["index_tip"] + 0.45 * positions["middle_tip"]
        thumb_tip += np.asarray([-0.003, 0.006, 0.004])
        curve = 0.82
    else:
        raise ValueError(f"unsupported mock hand pose {pose!r}")
    for name, point in zip(
        FINGER_JOINT_CHAINS["thumb"],
        _thumb_points(thumb_base, thumb_tip, curve),
    ):
        positions[name] = point
    return positions


def _mirror_for_left(positions: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    mirrored = {}
    for name, position in positions.items():
        value = np.asarray(position, dtype=float).copy()
        value[1] *= -1.0
        mirrored[name] = value
    return mirrored


def generate_mock_hand(
    pose: str,
    *,
    side: str = "right",
    is_tracked: bool = True,
    timestamp_ns: int | None = None,
) -> HandTrackingData:
    """Generate one complete 26-joint synthetic hand in meters.

    The skeleton is palm-local and uses robot-style axes (+X finger-forward,
    +Y left, +Z up).  It is intended for geometry and command-path tests, not
    for validating the PICO/OpenXR reference-space transform.
    """
    pose = _ALIASES.get(str(pose).lower(), str(pose).lower())
    if pose not in MOCK_HAND_POSES:
        raise ValueError(f"pose must be one of {MOCK_HAND_POSES}")
    side = str(side).lower()
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    positions = _right_hand_positions(pose)
    if side == "left":
        positions = _mirror_for_left(positions)
    if set(positions) != set(OPENXR_HAND_JOINT_NAMES):
        missing = sorted(set(OPENXR_HAND_JOINT_NAMES) - set(positions))
        raise RuntimeError(f"mock skeleton is incomplete: {missing}")

    identity_xyzw = np.asarray([0.0, 0.0, 0.0, 1.0])
    orientations = {name: identity_xyzw.copy() for name in OPENXR_HAND_JOINT_NAMES}
    validity = {name: bool(is_tracked) for name in OPENXR_HAND_JOINT_NAMES}

    def joint(name: str) -> HandJointPose:
        return HandJointPose(
            name=name,
            position_m=positions[name].copy(),
            quaternion_xyzw=orientations[name].copy(),
            is_valid=validity[name],
        )

    return HandTrackingData(
        side=side,
        wrist_pose=joint("wrist"),
        palm_pose=joint("palm"),
        joint_positions=positions,
        joint_orientations_xyzw=orientations,
        joint_validity=validity,
        is_tracked=bool(is_tracked),
        tracking_quality=1 if is_tracked else 0,
        timestamp_ns=time.monotonic_ns() if timestamp_ns is None else int(timestamp_ns),
        reference_space="synthetic palm-local robot axes: +X forward, +Y left, +Z up",
    )
