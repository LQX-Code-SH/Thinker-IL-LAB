"""通用 URDF 解析工具（机型无关）。

限位 / 速度 / 力矩 / mimic 关系以 URDF 为唯一来源，避免手写常量与模型漂移。
原先位于 ``devices/tienkung_dex/urdf_utils.py``，提升到 ``utils`` 后由各机型复用；
原模块保留转发导入，历史调用不需要改动。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path


@lru_cache(maxsize=8)
def read_urdf_joints(urdf_path: str) -> dict[str, dict]:
    """返回 ``{joint_name: {type, lower, upper, velocity, effort, mimic}}``（不含 fixed）。"""
    root = ET.parse(urdf_path).getroot()
    table: dict[str, dict] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        if not name or jtype == "fixed":
            continue
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        table[name] = {
            "type": jtype,
            "lower": float(limit.get("lower", 0.0)) if limit is not None else 0.0,
            "upper": float(limit.get("upper", 0.0)) if limit is not None else 0.0,
            "velocity": float(limit.get("velocity", 0.0)) if limit is not None else 0.0,
            "effort": float(limit.get("effort", 0.0)) if limit is not None else 0.0,
            "mimic": (
                {
                    "joint": mimic.get("joint"),
                    "multiplier": float(mimic.get("multiplier", 1.0)),
                    "offset": float(mimic.get("offset", 0.0)),
                }
                if mimic is not None
                else None
            ),
        }
    return table


def safe_urdf_table(urdf_path: str | Path, label: str = "URDF") -> dict[str, dict]:
    """URDF 缺失/损坏时返回空表并告警（资产未落盘时仍可 import 模块）。"""
    try:
        return read_urdf_joints(str(urdf_path))
    except (FileNotFoundError, ET.ParseError) as exc:  # pragma: no cover
        print(f"[WARN] 无法读取 {label} URDF（{exc}），关节限位回退为 0")
        return {}


def joint_limits(table: dict[str, dict]) -> dict[str, tuple[float, float, float, float]]:
    """``name -> (lower[rad], upper[rad], velocity[rad/s], effort[Nm])``"""
    return {
        name: (info["lower"], info["upper"], info["velocity"], info["effort"])
        for name, info in table.items()
    }


def mimic_joints(table: dict[str, dict]) -> dict[str, dict]:
    """``name -> {joint, multiplier, offset}``"""
    return {name: info["mimic"] for name, info in table.items() if info.get("mimic") is not None}
