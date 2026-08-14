# -*- coding: utf-8 -*-
"""机器人适配层注册表。

适配层封装两类差异：
1. HDF5 schema（键路径、维度、fps 来源、关节命名）
2. ROS2 发布接口（消息类型、话题、电机 ID）

新增机器人：实现 Adapter 接口（见 walker_s2.py 结构）后在此注册。
"""
from __future__ import annotations

import h5py

from .tienkung_pro import TienkungProAdapter
from .walker_s2 import WalkerS2Adapter

ADAPTERS = {
    "walker_s2": WalkerS2Adapter,
    "tienkung_pro": TienkungProAdapter,
}


def detect_robot_type(path: str) -> str:
    """根据 HDF5 内容推断机器人类型（root attr 优先，其次结构特征）。"""
    with h5py.File(path, "r") as f:
        rt = f.attrs.get("robot_type")
        if rt and str(rt) in ADAPTERS:
            return str(rt)
        if "puppet" in f:
            return "tienkung_pro"
        if "action/joint_state/position" in f or "action/joint_state/position/data" in f:
            return "walker_s2"
    raise ValueError(f"无法识别数据集机器人类型: {path}")
