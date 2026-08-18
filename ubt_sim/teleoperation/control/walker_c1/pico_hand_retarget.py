"""Explainable OpenXR 26-joint to Walker C1 11DoF hand retargeting.

The mapping is deliberately geometric and side-effect free: this module does
not import ROS, XRoboToolkit, or Isaac Sim and never publishes a command.  The
stateful :class:`C1HandRetargeter` adds smoothing, rate limits, and tracking
loss behavior around the pure geometry mapping.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

try:
    from .pico_hand_tracking import FINGER_JOINT_CHAINS, HandTrackingData
except ImportError:
    from pico_hand_tracking import FINGER_JOINT_CHAINS, HandTrackingData


DEFAULT_CONFIG_PATH = Path(__file__).with_name("pico_hand_retarget_config.json")
SIDES = ("left", "right")
LONG_FINGERS = ("index", "middle", "ring", "little")


class RetargetConfigError(ValueError):
    """Raised when the standalone retarget configuration is inconsistent."""


@dataclass(frozen=True)
class FingerGeometry:
    """Rotation-invariant geometric observables for one human finger."""

    bend_angles_rad: np.ndarray
    total_bend_rad: float
    tip_reach_ratio: float
    total_flexion: float
    proximal_flexion: float
    distal_flexion: float


@dataclass(frozen=True)
class RetargetResult:
    """One filtered robot-hand target and the features that produced it."""

    side: str
    joint_names: tuple[str, ...]
    robot_hand_q: np.ndarray
    normalized_targets: np.ndarray
    features: Mapping[str, float]
    used_tracking: bool
    fallback: str | None


def load_retarget_config(path: str | Path | None = None) -> dict[str, Any]:
    config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    with config_path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    _validate_config(config)
    return config


def _validate_config(config: Mapping[str, Any]) -> None:
    if int(config.get("schema_version", -1)) != 1:
        raise RetargetConfigError("retarget config schema_version must be 1")
    order = config.get("joint_order")
    if not isinstance(order, list) or len(order) != 11 or len(set(order)) != 11:
        raise RetargetConfigError("joint_order must contain 11 unique semantic joints")
    joints = config.get("joints")
    if not isinstance(joints, dict) or set(joints) != set(order):
        raise RetargetConfigError("joints must exactly match joint_order")
    for semantic in order:
        joint = joints[semantic]
        for side in SIDES:
            lower = float(joint["q_min_rad"][side])
            upper = float(joint["q_max_rad"][side])
            if not lower < upper:
                raise RetargetConfigError(f"{semantic}/{side} has invalid q limits")
            if not str(joint["joint_name"][side]).endswith("_joint"):
                raise RetargetConfigError(f"{semantic}/{side} has invalid joint name")
        if float(joint["joint_range_rad"]) <= 0.0:
            raise RetargetConfigError(f"{semantic} joint_range_rad must be positive")
        if float(joint["velocity_limit_rad_s"]) <= 0.0:
            raise RetargetConfigError(f"{semantic} velocity limit must be positive")
    mapping = config.get("mapping", {})
    behavior = mapping.get("invalid_tracking", {}).get("behavior")
    if behavior not in ("hold", "open", "neutral"):
        raise RetargetConfigError("invalid tracking behavior must be hold/open/neutral")


def _unit_segments(hand: HandTrackingData, finger: str) -> tuple[list[np.ndarray], list[float]]:
    chain = FINGER_JOINT_CHAINS[finger]
    if not hand.is_tracked or not all(hand.joint_validity.get(name, False) for name in chain):
        raise ValueError(f"{finger} skeleton is not valid")
    points = [np.asarray(hand.joint_positions[name], dtype=float) for name in chain]
    if not all(point.shape == (3,) and np.all(np.isfinite(point)) for point in points):
        raise ValueError(f"{finger} skeleton contains an invalid point")
    segments = [second - first for first, second in zip(points[:-1], points[1:])]
    lengths = [float(np.linalg.norm(segment)) for segment in segments]
    if any(length <= 1e-7 for length in lengths):
        raise ValueError(f"{finger} skeleton contains a zero-length bone")
    return [segment / length for segment, length in zip(segments, lengths)], lengths


def _smoothstep01(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def extract_finger_geometry(
    hand: HandTrackingData,
    finger: str,
    config: Mapping[str, Any],
) -> FingerGeometry:
    """Compress a human finger chain into normalized proximal/distal flexion.

    Angles and metacarpal-to-tip reach are invariant to wrist translation and
    rotation.  Human MCP/PIP/DIP are not copied directly: their combined bend
    and fingertip curl are blended, then compressed into the C1's two joints.
    """
    finger = str(finger).lower()
    if finger not in FINGER_JOINT_CHAINS:
        raise ValueError(f"unknown finger {finger!r}")
    directions, lengths = _unit_segments(hand, finger)
    bend_angles = np.asarray(
        [
            math.acos(float(np.clip(np.dot(first, second), -1.0, 1.0)))
            for first, second in zip(directions[:-1], directions[1:])
        ],
        dtype=float,
    )
    total_bend = float(np.sum(bend_angles))
    chain = FINGER_JOINT_CHAINS[finger]
    base = np.asarray(hand.joint_positions[chain[0]], dtype=float)
    tip = np.asarray(hand.joint_positions[chain[-1]], dtype=float)
    reach_ratio = float(np.linalg.norm(tip - base) / sum(lengths))
    reach_ratio = float(np.clip(reach_ratio, 0.0, 1.0))

    mapping = config["mapping"]
    references = mapping["human_flexion_reference_rad"][finger]
    weights = mapping["geometry_weights"]
    sensitivity = float(mapping["flexion_sensitivity"][finger])
    bend_norm = total_bend / float(references["total"])
    curl_norm = (1.0 - reach_ratio) / float(weights["tip_curl_reference"])
    total_norm = sensitivity * (
        float(weights["bend"]) * bend_norm + float(weights["tip_curl"]) * curl_norm
    )
    total_norm = float(np.clip(total_norm, 0.0, 1.0))

    proximal_angle = float(bend_angles[0]) if len(bend_angles) else 0.0
    distal_angle = float(np.sum(bend_angles[1:])) if len(bend_angles) > 1 else proximal_angle
    proximal_norm = float(np.clip(proximal_angle / float(references["proximal"]), 0.0, 1.0))
    distal_norm = float(np.clip(distal_angle / float(references["distal"]), 0.0, 1.0))
    proximal = float(
        np.clip(
            float(weights["mpp_total"]) * total_norm
            + float(weights["mpp_proximal"]) * proximal_norm,
            0.0,
            1.0,
        )
    )
    distal = float(
        np.clip(
            float(weights["ip_total"]) * total_norm
            + float(weights["ip_distal"]) * distal_norm,
            0.0,
            1.0,
        )
    )
    return FingerGeometry(
        bend_angles_rad=bend_angles,
        total_bend_rad=total_bend,
        tip_reach_ratio=reach_ratio,
        total_flexion=total_norm,
        proximal_flexion=proximal,
        distal_flexion=distal,
    )


def _palm_width_m(hand: HandTrackingData) -> float:
    names = ("index_metacarpal", "little_metacarpal")
    if not all(hand.joint_validity.get(name, False) for name in names):
        raise ValueError("palm-width joints are not valid")
    width = float(
        np.linalg.norm(
            np.asarray(hand.joint_positions[names[0]], dtype=float)
            - np.asarray(hand.joint_positions[names[1]], dtype=float)
        )
    )
    if not np.isfinite(width) or width <= 1e-4:
        raise ValueError("palm width is invalid")
    return width


def _thumb_index_opposition(hand: HandTrackingData, config: Mapping[str, Any]) -> tuple[float, float]:
    names = ("thumb_tip", "index_tip")
    if not all(hand.joint_validity.get(name, False) for name in names):
        raise ValueError("thumb/index tips are not valid")
    distance = float(
        np.linalg.norm(
            np.asarray(hand.joint_positions[names[0]], dtype=float)
            - np.asarray(hand.joint_positions[names[1]], dtype=float)
        )
    )
    ratio = distance / _palm_width_m(hand)
    settings = config["mapping"]["thumb_opposition"]
    closed = float(settings["closed_distance_over_palm_width"])
    opened = float(settings["open_distance_over_palm_width"])
    if not closed < opened:
        raise RetargetConfigError("thumb opposition closed threshold must be below open threshold")
    opposition = 1.0 - _smoothstep01((ratio - closed) / (opened - closed))
    return float(np.clip(opposition, 0.0, 1.0)), distance


def normalized_hand_targets(
    hand: HandTrackingData,
    config: Mapping[str, Any],
) -> tuple[np.ndarray, dict[str, float]]:
    """Return the 11 normalized C1 joint targets before signs and limits."""
    if hand.side not in SIDES:
        raise ValueError("hand side must be left or right")
    if not hand.is_tracked:
        raise ValueError("hand tracking is invalid")

    geometry = {
        finger: extract_finger_geometry(hand, finger, config)
        for finger in ("thumb",) + LONG_FINGERS
    }
    opposition, pinch_distance = _thumb_index_opposition(hand, config)
    thumb_settings = config["mapping"]["thumb_opposition"]
    thumb = geometry["thumb"]
    thumb_mpp = max(
        thumb.proximal_flexion,
        opposition * float(thumb_settings["mpp_opposition_assist"]),
    )
    thumb_ip = max(
        thumb.distal_flexion,
        opposition * float(thumb_settings["ip_opposition_assist"]),
    )
    index = geometry["index"]
    index_assist = opposition * float(thumb_settings["index_pinch_assist"])

    targets = np.asarray(
        [
            opposition * float(thumb_settings["cmp_opposition_weight"]),
            thumb_mpp,
            thumb_ip,
            max(index.proximal_flexion, index_assist),
            max(index.distal_flexion, index_assist),
            geometry["middle"].proximal_flexion,
            geometry["middle"].distal_flexion,
            geometry["ring"].proximal_flexion,
            geometry["ring"].distal_flexion,
            geometry["little"].proximal_flexion,
            geometry["little"].distal_flexion,
        ],
        dtype=float,
    )
    targets = np.clip(targets, 0.0, 1.0)
    features: dict[str, float] = {
        "thumb_index_distance_m": pinch_distance,
        "thumb_opposition": opposition,
    }
    for finger, values in geometry.items():
        features[f"{finger}_total_flexion"] = values.total_flexion
        features[f"{finger}_tip_reach_ratio"] = values.tip_reach_ratio
    return targets, features


def _arrays_for_side(config: Mapping[str, Any], side: str) -> dict[str, np.ndarray | tuple[str, ...]]:
    if side not in SIDES:
        raise ValueError("side must be left or right")
    order = config["joint_order"]
    joints = config["joints"]
    return {
        "names": tuple(str(joints[key]["joint_name"][side]) for key in order),
        "lower": np.asarray([float(joints[key]["q_min_rad"][side]) for key in order]),
        "upper": np.asarray([float(joints[key]["q_max_rad"][side]) for key in order]),
        "range": np.asarray([float(joints[key]["joint_range_rad"]) for key in order]),
        "sign": np.asarray([float(joints[key]["sign"][side]) for key in order]),
        "scale": np.asarray([float(joints[key]["scale"]) for key in order]),
        "offset": np.asarray([float(joints[key]["offset_rad"][side]) for key in order]),
        "velocity": np.asarray([float(joints[key]["velocity_limit_rad_s"]) for key in order]),
    }


def normalized_to_robot_q(
    normalized: np.ndarray,
    side: str,
    config: Mapping[str, Any],
) -> np.ndarray:
    values = np.asarray(normalized, dtype=float)
    if values.shape != (11,) or not np.all(np.isfinite(values)):
        raise ValueError("normalized hand target must be a finite 11D vector")
    arrays = _arrays_for_side(config, side)
    command = (
        arrays["offset"]
        + arrays["sign"] * arrays["scale"] * arrays["range"] * np.clip(values, 0.0, 1.0)
    )
    return np.clip(command, arrays["lower"], arrays["upper"]).astype(float)


class C1HandRetargeter:
    """Stateful safe wrapper around the pure normalized geometry mapping."""

    def __init__(
        self,
        config_path: str | Path | None = None,
        *,
        smoothing_enabled: bool | None = None,
        velocity_limits_enabled: bool | None = None,
        invalid_behavior: str | None = None,
    ) -> None:
        self.config = load_retarget_config(config_path)
        mapping = self.config["mapping"]
        self.smoothing_enabled = (
            bool(mapping["smoothing"]["enabled"])
            if smoothing_enabled is None
            else bool(smoothing_enabled)
        )
        self.velocity_limits_enabled = (
            bool(mapping["velocity_limits_enabled"])
            if velocity_limits_enabled is None
            else bool(velocity_limits_enabled)
        )
        behavior = invalid_behavior or str(mapping["invalid_tracking"]["behavior"])
        if behavior not in ("hold", "open", "neutral"):
            raise ValueError("invalid_behavior must be hold, open, or neutral")
        self.invalid_behavior = behavior
        self._previous: dict[str, np.ndarray] = {}
        self._invalid_elapsed_s = {side: 0.0 for side in SIDES}

    def reset(self, side: str | None = None) -> None:
        if side is None:
            self._previous.clear()
            self._invalid_elapsed_s = {name: 0.0 for name in SIDES}
            return
        if side not in SIDES:
            raise ValueError("side must be left or right")
        self._previous.pop(side, None)
        self._invalid_elapsed_s[side] = 0.0

    def joint_names(self, side: str) -> tuple[str, ...]:
        return _arrays_for_side(self.config, side)["names"]

    def joint_limits(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        arrays = _arrays_for_side(self.config, side)
        return arrays["lower"].copy(), arrays["upper"].copy()

    def velocity_limits(self, side: str) -> np.ndarray:
        return _arrays_for_side(self.config, side)["velocity"].copy()

    def previous_command(self, side: str) -> np.ndarray | None:
        previous = self._previous.get(side)
        return None if previous is None else previous.copy()

    def open_command(self, side: str) -> np.ndarray:
        return normalized_to_robot_q(np.zeros(11), side, self.config)

    def neutral_command(self, side: str) -> np.ndarray:
        value = float(self.config["mapping"]["neutral_normalized"])
        return normalized_to_robot_q(np.full(11, value), side, self.config)

    def _dt(self, dt_s: float | None) -> float:
        mapping = self.config["mapping"]
        value = float(mapping["default_dt_s"] if dt_s is None else dt_s)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError("dt_s must be positive and finite")
        return min(value, float(mapping["smoothing"]["max_dt_s"]))

    def _filter(self, side: str, target: np.ndarray, dt_s: float) -> np.ndarray:
        arrays = _arrays_for_side(self.config, side)
        target = np.clip(np.asarray(target, dtype=float), arrays["lower"], arrays["upper"])
        previous = self._previous.get(side)
        if (
            previous is None
            and bool(self.config["mapping"].get("initialize_from_open", True))
            and (self.smoothing_enabled or self.velocity_limits_enabled)
        ):
            previous = self.open_command(side)
        if previous is None:
            result = target
        else:
            result = target
            if self.smoothing_enabled:
                tau = float(self.config["mapping"]["smoothing"]["time_constant_s"])
                if tau <= 0.0:
                    raise RetargetConfigError("smoothing time_constant_s must be positive")
                alpha = 1.0 - math.exp(-dt_s / tau)
                result = previous + alpha * (result - previous)
            if self.velocity_limits_enabled:
                maximum_delta = arrays["velocity"] * dt_s
                result = previous + np.clip(result - previous, -maximum_delta, maximum_delta)
        result = np.clip(result, arrays["lower"], arrays["upper"]).astype(float)
        self._previous[side] = result.copy()
        return result

    def _fallback_target(self, side: str, mode: str) -> np.ndarray:
        if mode == "open":
            return self.open_command(side)
        if mode == "neutral":
            return self.neutral_command(side)
        raise ValueError(f"unsupported fallback {mode!r}")

    def _invalid_result(self, side: str, dt_s: float) -> RetargetResult:
        settings = self.config["mapping"]["invalid_tracking"]
        self._invalid_elapsed_s[side] += dt_s
        mode = self.invalid_behavior
        timeout = settings.get("hold_timeout_s")
        if mode == "hold" and timeout is not None and self._invalid_elapsed_s[side] >= float(timeout):
            mode = str(settings["timeout_fallback"])

        previous = self._previous.get(side)
        if mode == "hold" and previous is not None:
            command = previous.copy()
        elif mode == "hold":
            command = self.open_command(side)
            self._previous[side] = command.copy()
        else:
            command = self._filter(side, self._fallback_target(side, mode), dt_s)
        return RetargetResult(
            side=side,
            joint_names=self.joint_names(side),
            robot_hand_q=command,
            normalized_targets=np.full(11, np.nan),
            features={},
            used_tracking=False,
            fallback=mode,
        )

    def retarget_result(self, hand: HandTrackingData, dt_s: float | None = None) -> RetargetResult:
        side = str(hand.side).lower()
        if side not in SIDES:
            raise ValueError("hand side must be left or right")
        dt = self._dt(dt_s)
        try:
            normalized, features = normalized_hand_targets(hand, self.config)
        except (KeyError, TypeError, ValueError, FloatingPointError):
            return self._invalid_result(side, dt)

        target = normalized_to_robot_q(normalized, side, self.config)
        command = self._filter(side, target, dt)
        self._invalid_elapsed_s[side] = 0.0
        return RetargetResult(
            side=side,
            joint_names=self.joint_names(side),
            robot_hand_q=command,
            normalized_targets=normalized,
            features=features,
            used_tracking=True,
            fallback=None,
        )

    def retarget(self, hand: HandTrackingData, dt_s: float | None = None) -> np.ndarray:
        """Return a finite, limited robot hand target with shape ``(11,)``."""
        return self.retarget_result(hand, dt_s=dt_s).robot_hand_q.copy()


def retarget_hand(
    human_hand_data: HandTrackingData,
    *,
    retargeter: C1HandRetargeter | None = None,
    config_path: str | Path | None = None,
    dt_s: float | None = None,
) -> np.ndarray:
    """Convenience 26-joint human hand -> C1 11DoF interface.

    Pass a persistent ``retargeter`` in a real-time loop so filtering and
    tracking-loss hold state carry across frames.
    """
    mapper = retargeter or C1HandRetargeter(config_path)
    return mapper.retarget(human_hand_data, dt_s=dt_s)
