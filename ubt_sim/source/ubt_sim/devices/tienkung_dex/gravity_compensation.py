"""tienkung_dex 重力补偿前馈模块。

仿真里的手臂是 ``ImplicitActuator``（k=80, d=20；2026-10-27 起肩 200、其余手臂
150、腰 400，见 ``config.py`` 的 ``TIENKUNG_DEX_*_KP``），没有重力前馈时稳态必然
差一个 ``τ_g / k``。实测（k=80 时，2026-10-27，pelvis 系）：

* 预备位 READY：末端下垂 105 mm（FK 复算 104.6 mm）
* 2 号路径点：末端下垂 70.7 mm（FK 复算 70.2 mm，分量级吻合）

本模块用 pinocchio RNEA 从 URDF 算关节空间重力矩，换算成等效位置偏移，叠加
到 action 的位置指令上：

    τ = k × (θ_cmd + τ_g/k − θ) = k × (θ_cmd − θ) + τ_g
    稳态 τ = 0 → θ = θ_cmd

与 :mod:`ubt_sim.devices.walker_s2.gravity_compensation` 同源，两处差别：

* **关节索引表不硬编码**。tienkung_dex 的关节名在 ``config`` 的 actuator 分组
  里本来就是显式列表（腿 12 + 腰 3 + 头 2 + 臂 14 + 手 36 = 67），直接用
  ``getJointId(name)`` 建表即可。walker_s2 之所以硬编码，是因为它**枚举**
  ``model.names``（把 ``std::vector<string>`` 取出来）会撞 pybind11 的类型转换
  失败；只把字符串**传进去**、取回 int 不受影响。
* **刚度直接从 ArticulationCfg 读**，不复制一份常量表，执行器参数改了这里自动
  跟着走。

补偿 17 个关节：14 个手臂 + 3 个腰。腰在预备位扛着整条上半身（约 30 N·m），
k=600 仍有 0.042 rad 下垂，而肩离腰枢轴约 0.5 m——就是末端 20 mm 的下沉，
比手臂侧残差还大。手指（k=10）不补：重力矩很小，位置偏移还会改变合手力度。

**模型精度**（2026-10-27 标定，六个姿态）：腰部与肩 roll 准到 2%，但手臂差得多，
且比值随姿态漂——「真值重力矩 / URDF 模型重力矩」在肩 pitch 上是 0.98~1.38，
elbow_yaw 上 2.6~6.5。这说明 URDF 与 USD 的臂/手惯量分布不一致（不是单一缩放），
所以**没有**像 walker_s2 那样加一个全局 ``scale``：它在某个姿态调准了，换个姿态
就会过补。前馈按模型原样走，吃掉 85~92% 的下垂，剩下的这十几毫米只能逐姿态标定
（预备位在 ``constants.READY_POSE_TARGET`` 里扣掉了）。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any

import numpy as np

from .config import (
    TIENKUNG_DEX_CFG,
    TIENKUNG_DEX_LEFT_ARM_JOINTS,
    TIENKUNG_DEX_RIGHT_ARM_JOINTS,
    TIENKUNG_DEX_URDF_PATH,
)
from .config_brainco import (
    TIENKUNG_DEX_BRAINCO_CFG,
    TIENKUNG_DEX_BRAINCO_URDF_PATH,
)

DEFAULT_VARIANT = "inspire"

ARM_JOINTS: list[str] = list(TIENKUNG_DEX_LEFT_ARM_JOINTS) + list(TIENKUNG_DEX_RIGHT_ARM_JOINTS)
WAIST_JOINTS: list[str] = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
COMPENSATED_JOINTS: list[str] = ARM_JOINTS + WAIST_JOINTS

# 经验缩放系数：URDF 的质量/惯量与 USD 的转换偏差。tienkung_dex 实测比值
# 0.98~1.05（见模块 docstring 的实测对比），所以默认 1.0，需要时用环境变量微调。
_GRAVITY_SCALE = float(os.environ.get("TIENKUNG_DEX_GRAVITY_SCALE", "1.0"))


@dataclass
class _Model:
    """一个变体的 pinocchio 模型与索引表（构建一次，之后复用）。"""

    pin: Any
    model: Any
    data: Any
    q_idx: dict[str, int]
    v_idx: dict[str, int]
    comp_v_idx: list[int]
    stiffness: dict[str, float]


_CACHE: dict[str, _Model] = {}
_pin = None
_pin_ready = False


def _ensure_pinocchio() -> Any:
    """延迟导入 pinocchio。

    Isaac Sim 启动阶段 hpp-fcl 解析不到 Assimp 符号，需要先按 RTLD_GLOBAL
    预载 cmeel 自带的 libassimp（与 walker_s2 相同）。失败不致命：某些镜像
    版本里符号本来就齐全。
    """
    global _pin, _pin_ready
    if _pin_ready:
        return _pin

    import ctypes

    for path in (
        "/isaac-sim/kit/python/lib/python3.11/site-packages/"
        "cmeel.prefix/lib/libassimp.so.5",
        "/usr/lib/x86_64-linux-gnu/libassimp.so.5",
    ):
        try:
            ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
            break
        except OSError:
            continue

    import pinocchio

    _pin = pinocchio
    _pin_ready = True
    return _pin


def _actuator_joint_names(cfg: Any) -> list[str]:
    """cfg 里所有 actuator 分组显式列出的关节名（去重保序）。"""
    names: list[str] = []
    seen: set[str] = set()
    for actuator in cfg.actuators.values():
        expr = actuator.joint_names_expr
        for name in expr if isinstance(expr, (list, tuple)) else [expr]:
            if isinstance(name, str) and name not in seen:
                names.append(name)
                seen.add(name)
    return names


def _compensated_stiffness(cfg: Any) -> dict[str, float]:
    """被补偿关节的刚度，直接取执行器 cfg 里的值（保持单一来源）。"""
    stiffness: dict[str, float] = {}
    for group in ("left_arm", "right_arm", "waist"):
        actuator = cfg.actuators[group]
        for name in actuator.joint_names_expr:
            value = actuator.stiffness
            stiffness[name] = float(value[name] if isinstance(value, dict) else value)
    return stiffness


def _build(variant: str) -> _Model:
    spec = {
        "inspire": (TIENKUNG_DEX_CFG, str(TIENKUNG_DEX_URDF_PATH)),
        "brainco": (TIENKUNG_DEX_BRAINCO_CFG, str(TIENKUNG_DEX_BRAINCO_URDF_PATH)),
    }
    if variant not in spec:
        raise ValueError(f"未知的 tienkung_dex 变体: {variant}（可选 {sorted(spec)}）")
    cfg, urdf = spec[variant]

    pin = _ensure_pinocchio()
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()

    q_idx: dict[str, int] = {}
    v_idx: dict[str, int] = {}
    for name in _actuator_joint_names(cfg):
        if not model.existJointName(name):
            continue
        joint = model.joints[model.getJointId(name)]
        if joint.nq != 1:  # 只处理 revolute（本机全是一自由度关节）
            continue
        q_idx[name] = joint.idx_q
        v_idx[name] = joint.idx_v

    missing = [name for name in COMPENSATED_JOINTS if name not in v_idx]
    if missing:
        raise RuntimeError(f"URDF {urdf} 缺少补偿关节 {missing}，无法做重力补偿")

    cached = _Model(
        pin=pin,
        model=model,
        data=data,
        q_idx=q_idx,
        v_idx=v_idx,
        comp_v_idx=[v_idx[name] for name in COMPENSATED_JOINTS],
        stiffness=_compensated_stiffness(cfg),
    )
    print(
        f"[INFO] tienkung_dex[{variant}] 重力补偿已加载: "
        f"{len(COMPENSATED_JOINTS)} 个关节（{len(ARM_JOINTS)} 臂 + {len(WAIST_JOINTS)} 腰）"
        f" / URDF nq={model.nq} / scale={_GRAVITY_SCALE}",
        flush=True,  # 日志被重定向到文件时是块缓冲，不 flush 会看不到这行
    )
    return cached


def _model(variant: str) -> _Model:
    if variant not in _CACHE:
        _CACHE[variant] = _build(variant)
    return _CACHE[variant]


def compute_gravity_offsets(
    joint_pos_dict: dict[str, float],
    variant: str = DEFAULT_VARIANT,
) -> dict[str, float]:
    """算手臂关节的重力补偿位置偏移（rad）。

    Args:
        joint_pos_dict: ``{关节名: 实测角度}``，整机一起给（腿/腰的姿态影响质心）。
        variant: ``inspire`` / ``brainco``，决定用哪份 URDF 和刚度。

    Returns:
        ``{关节名: offset_rad}``，键为 :data:`COMPENSATED_JOINTS`（14 臂 + 3 腰）。
    """
    cached = _model(variant)

    q = np.zeros(cached.model.nq)
    for name, value in joint_pos_dict.items():
        q_idx = cached.q_idx.get(name)
        if q_idx is not None:
            q[q_idx] = float(value)

    cached.pin.computeGeneralizedGravity(cached.model, cached.data, q)
    tau = np.asarray(cached.data.g)

    return {
        name: float(tau[cached.v_idx[name]]) * _GRAVITY_SCALE / cached.stiffness[name]
        for name in COMPENSATED_JOINTS
    }
