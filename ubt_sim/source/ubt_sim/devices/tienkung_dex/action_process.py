"""tienkung_dex 的 action 组装与状态回传（支持 inspire / brainco 两种手部变体）。

与 ``tienkung_pro`` 保持相同的调用约定：

* ``to_controller_data``：把 ZMQ 收到的 ``{关节名: 目标}`` 组装成 Isaac Lab action 张量，
  并按 URDF 的 ``mimic`` 关系展开被动关节；
* ``to_ros_data``：把仿真状态打包成桥接进程可消费的 dict。

手部通道支持两种输入（优先级：``*_percent`` > 绝对弧度）：

* ``"right_index_mcp_joint": 0.42``            -> 绝对目标（rad）
* ``"right_index_mcp_joint_percent": 0.8``     -> 归一化闭合度（0=张开，1=闭合）

变体差异只在手部关节集合/mimic 关系，由 ``variant`` 参数选择。
"""

from __future__ import annotations

from typing import Any

import torch

from .config import (
    TIENKUNG_DEX_HAND_JOINTS,
    TIENKUNG_DEX_HOME_POSE,
    TIENKUNG_DEX_JOINT_LIMITS,
    TIENKUNG_DEX_MIMIC_JOINTS,
)
from .config_brainco import (
    TIENKUNG_DEX_BRAINCO_HAND_JOINTS,
    TIENKUNG_DEX_BRAINCO_HOME_POSE,
    TIENKUNG_DEX_BRAINCO_JOINT_LIMITS,
    TIENKUNG_DEX_BRAINCO_MIMIC_JOINTS,
)

DEFAULT_VARIANT = "inspire"

_VARIANTS: dict[str, dict[str, Any]] = {
    "inspire": {
        "hand_joints": TIENKUNG_DEX_HAND_JOINTS,
        "joint_limits": TIENKUNG_DEX_JOINT_LIMITS,
        "mimic": TIENKUNG_DEX_MIMIC_JOINTS,
        "home_pose": TIENKUNG_DEX_HOME_POSE,
    },
    "brainco": {
        "hand_joints": TIENKUNG_DEX_BRAINCO_HAND_JOINTS,
        "joint_limits": TIENKUNG_DEX_BRAINCO_JOINT_LIMITS,
        "mimic": TIENKUNG_DEX_BRAINCO_MIMIC_JOINTS,
        "home_pose": TIENKUNG_DEX_BRAINCO_HOME_POSE,
    },
}

_action_joint_names: dict[str, list[str]] = {}


def _spec(variant: str) -> dict[str, Any]:
    if variant not in _VARIANTS:
        raise ValueError(f"未知的 tienkung_dex 变体: {variant}（可选 {sorted(_VARIANTS)}）")
    return _VARIANTS[variant]


def _target_limit(joint_name: str, limits: dict[str, tuple[float, float, float, float]]) -> float:
    """返回关节从零位出发的"闭合/动作"极限（绝对值较大的一侧）。"""
    low, high = limits[joint_name][0], limits[joint_name][1]
    return low if abs(low) > abs(high) else high


def _resolve_value(
    joint_name: str,
    joint_pos_dict: dict[str, float],
    limits: dict[str, tuple[float, float, float, float]],
    mimic: dict[str, dict],
    home_pose: dict[str, float],
    depth: int = 0,
) -> float:
    """解析单个关节目标，mimic 关节按源关节倍率展开。

    没收到指令的关节保持站姿（``home_pose``）而不是回零：action 是绝对角度
    （``use_default_offset=False``），回零会让未下发的那几组关节（例如只发
    ``/arm/cmd`` 时的腿、腰、头）立刻向 0 运动，与真机"不动就保持当前位姿"
    的语义不符。
    """
    if depth > 4:  # 防御 URDF 中意外的 mimic 环
        return 0.0

    percent_key = f"{joint_name}_percent"
    if percent_key in joint_pos_dict and joint_name in limits:
        return float(joint_pos_dict[percent_key]) * _target_limit(joint_name, limits)

    if joint_name in mimic:
        info = mimic[joint_name]
        source = info["joint"]
        return info["multiplier"] * _resolve_value(
            source, joint_pos_dict, limits, mimic, home_pose, depth + 1
        ) + info.get("offset", 0.0)

    return float(joint_pos_dict.get(joint_name, home_pose.get(joint_name, 0.0)))


def to_controller_data(joint_pos_dict: dict[str, Any], env, variant: str = DEFAULT_VARIANT) -> torch.Tensor:
    """把 ``{关节名: 目标}`` 组装成 ``(num_envs, num_actions)`` 的 action 张量。"""
    spec = _spec(variant)
    if variant not in _action_joint_names:
        _action_joint_names[variant] = get_joint_names(env, variant)

    limits = spec["joint_limits"]
    mimic = spec["mimic"]
    home_pose = spec["home_pose"]
    results = [
        _resolve_value(name, joint_pos_dict, limits, mimic, home_pose)
        for name in _action_joint_names[variant]
    ]
    actions = torch.tensor(results, dtype=torch.float32, device=env.device)
    return actions.unsqueeze(0).repeat(env.num_envs, 1)


def to_ros_data(env, variant: str = DEFAULT_VARIANT) -> dict[str, Any]:
    """打包当前仿真状态：关节名/位置/速度 + 手部归一化闭合度。"""
    spec = _spec(variant)
    limits = spec["joint_limits"]

    robot = env.scene["robot"]
    joint_names = list(robot.data.joint_names)
    joint_pos = robot.data.joint_pos[0].detach().cpu().numpy().tolist()
    joint_vel = robot.data.joint_vel[0].detach().cpu().numpy().tolist()

    pos_map = dict(zip(joint_names, joint_pos))
    hand_percentages: dict[str, float] = {}
    for name in spec["hand_joints"]:
        if name not in pos_map:
            continue
        limit = _target_limit(name, limits)
        if abs(limit) > 1e-6:
            ratio = pos_map[name] / limit
            hand_percentages[name] = float(min(max(ratio, 0.0), 1.0))
        else:
            hand_percentages[name] = 0.0

    return {
        "joint_names": joint_names,
        "joint_pos": joint_pos,
        "joint_vel": joint_vel,
        "hand_percentages": hand_percentages,
    }


def get_joint_names(env, variant: str = DEFAULT_VARIANT) -> list[str]:
    """按 action manager 的 term 顺序取出受控关节名（与 action 张量一一对应）。"""
    robot = env.unwrapped.scene["robot"]
    action_manager = env.unwrapped.action_manager

    terms: dict[str, Any] = {}
    for attr_name in ("_terms", "_action_terms", "terms"):
        if hasattr(action_manager, attr_name):
            value = getattr(action_manager, attr_name)
            if isinstance(value, dict):
                terms = value
                break
            if isinstance(value, list):
                terms = {f"term_{i}": term for i, term in enumerate(value)}
                break

    indices: list[int] = []
    for term in terms.values():
        for attr in ("joint_indices", "_joint_indices", "joint_ids", "_joint_ids"):
            if hasattr(term, attr):
                indices += list(getattr(term, attr))
                break

    joint_names = [robot.joint_names[idx] for idx in indices]
    print(f"[INFO] tienkung_dex[{variant}] action joints ({len(joint_names)}): {joint_names}")
    return joint_names
