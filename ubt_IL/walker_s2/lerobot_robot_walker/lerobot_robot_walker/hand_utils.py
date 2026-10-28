"""V4 hand and 1-DOF gripper clipping utilities (walker plugin side).

The ROS2 bridge runs in Python 3.10 and cannot import this module, so keep
bridge-side clipping logic synchronized with the behavior here.
"""

from .constants import V4_HAND_JOINT_LIMITS


def _clamp(value: float, limits: tuple[float, float]) -> float:
    lo, hi = limits
    return max(lo, min(hi, float(value)))


def v4_clip_position(position: list, joint_names: list) -> list:
    """Clamp V4 hand joints by per-joint limits."""
    result = []
    for pos, name in zip(position, joint_names):
        short = name.removeprefix("left_").removeprefix("right_")
        if short in V4_HAND_JOINT_LIMITS:
            pos = _clamp(pos, V4_HAND_JOINT_LIMITS[short])
        result.append(pos)
    return result


def v4_clip_value(pos: float, joint_name: str) -> float:
    """Single-value V4 clamp used when returning the sent action."""
    short = joint_name.removeprefix("left_").removeprefix("right_")
    if short in V4_HAND_JOINT_LIMITS:
        return _clamp(pos, V4_HAND_JOINT_LIMITS[short])
    return float(pos)


def gripper_clip_position(position: list, limits: tuple[float, float]) -> list:
    """Clamp 1-DOF gripper positions in meters."""
    return [_clamp(pos, limits) for pos in position]


def gripper_clip_value(pos: float, limits: tuple[float, float]) -> float:
    """Clamp one 1-DOF gripper position in meters."""
    return _clamp(pos, limits)


def clip_hand_position(
    position: list,
    joint_names: list,
    hand_type: str = "v4",
    gripper_limits: tuple[float, float] = (0.0, 0.05),
) -> list:
    """Dispatch clipping by end-effector type."""
    if hand_type == "v4":
        return v4_clip_position(position, joint_names)
    if hand_type == "pgc_gripper_1dof":
        return gripper_clip_position(position, gripper_limits)
    raise ValueError(f"Unsupported hand_type: {hand_type!r}")


def clip_hand_value(
    pos: float,
    joint_name: str,
    hand_type: str = "v4",
    gripper_limits: tuple[float, float] = (0.0, 0.05),
) -> float:
    """Single-value clipping dispatcher."""
    if hand_type == "v4":
        return v4_clip_value(pos, joint_name)
    if hand_type == "pgc_gripper_1dof":
        return gripper_clip_value(pos, gripper_limits)
    raise ValueError(f"Unsupported hand_type: {hand_type!r}")
