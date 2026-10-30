"""tienkung_dex 各变体共用的 URDF 解析工具（转发到 ubt_sim.utils.urdf_utils）。

限位 / 速度 / 力矩 / mimic 关系以 URDF 为唯一来源，避免手写常量与模型漂移。
实现已提升为机型无关的公共工具，本模块仅保留导入兼容。
"""

from __future__ import annotations

from ...utils.urdf_utils import (
    joint_limits,
    mimic_joints,
    read_urdf_joints,
    safe_urdf_table,
)

__all__ = ["read_urdf_joints", "safe_urdf_table", "joint_limits", "mimic_joints"]
