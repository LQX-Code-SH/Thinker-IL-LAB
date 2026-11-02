"""Walker C1 dexterous-hand clipping utilities (walker_c1 plugin side).

C1 手是真机 SDK 的 6 个压缩通道（thumb_swing/thumb_mcp/index_mcp/
middle_mcp/ring_mcp/little_mcp），限位来源为 ``config.hand_joint_limits``
（默认 ``constants.C1_HAND_JOINT_LIMITS``）。ROS2 bridge 运行在 Python 3.10，
不 import 本模块——限位通过 ``to_bridge_config()`` 的 ``hand_joint_limits``
字段单点下发，bridge 侧不再重复定义。
"""

from __future__ import annotations

from .constants import C1_HAND_JOINT_LIMITS


def _clamp(value: float, limits: tuple[float, float]) -> float:
    lo, hi = limits
    return max(float(lo), min(float(hi), float(value)))


def _short_name(joint_name: str) -> str:
    return joint_name.removeprefix("left_").removeprefix("right_")


def clip_hand_position(
    position: list,
    joint_names: list,
    limits: dict[str, tuple[float, float]] | None = None,
) -> list:
    """按关节名裁剪 C1 手部 6 通道位置。"""
    table = limits if limits is not None else C1_HAND_JOINT_LIMITS
    result = []
    for pos, name in zip(position, joint_names):
        short = _short_name(name)
        if short in table:
            pos = _clamp(pos, table[short])
        result.append(float(pos))
    return result


def clip_hand_value(
    pos: float,
    joint_name: str,
    limits: dict[str, tuple[float, float]] | None = None,
) -> float:
    """单值裁剪（用于回传实际下发的 action）。"""
    table = limits if limits is not None else C1_HAND_JOINT_LIMITS
    short = _short_name(joint_name)
    if short in table:
        return _clamp(pos, table[short])
    return float(pos)
