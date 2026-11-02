"""动作块融合 / 插值 / 滤波（纯 numpy，桥接侧 chunk 消费算法）。

本模块不依赖 lerobot / ROS2，仅用 numpy，便于在桥接（系统 Python 3.10）中
直接 import，也可独立单测。被 ros2_walker_bridge.py 的 chunk 消费流水线调用：

    延迟补偿 -> ChunkFuser.blend -> ChunkInterpolator.densify -> smooth_window
              -> transition_ramp

设计要点
--------
- ChunkFuser：新 chunk 前缀与上一轨迹未执行后缀按权重混合，消除 chunk 边界
  跳变（C0 连续，w[0]=0 全旧 -> w[L-1]=1 全新）。
- ChunkInterpolator：把 fps 间距的稀疏 chunk 点加密到 control_hz（300Hz）。
  默认 hermite（Catmull-Rom，C1 连续、不在 waypoints 处停顿），适合密集 chunk；
  另提供 linear（C0）与 quintic（C2 但端点零速度，会在每个点停顿，仅适合稀疏
  单动作 retarget 场景）。
- smooth_window：滑窗均值滤波（可选，默认 window=1 关闭）。
- transition_ramp：从实测当前位置 quintic 缓动到 chunk 首点，跨 chunk 兜底。
"""

from __future__ import annotations

from enum import Enum

import numpy as np


class BlendSchedule(Enum):
    """融合权重曲线。w(0)=0（全旧）-> w(1)=1（全新）。"""

    LINEAR = "linear"
    SMOOTHSTEP = "smoothstep"   # 3t²-2t³，端点导数 0（C1）
    EXP = "exp"                 # 慢起快收，k 控制弯曲度


def _blend_weights(length: int, schedule: BlendSchedule, exp_k: float = 3.0) -> np.ndarray:
    """生成 [0,1] 长度 length 的单调递增权重序列，w[0]=0, w[-1]=1。"""
    if length <= 0:
        return np.zeros(0, dtype=float)
    if length == 1:
        # 单点重叠：取全旧（w=0）以保持 C0 连续（新 chunk 首点被旧轨迹首点取代）。
        return np.zeros(1, dtype=float)
    t = np.linspace(0.0, 1.0, length, dtype=float)
    if schedule is BlendSchedule.LINEAR:
        w = t
    elif schedule is BlendSchedule.SMOOTHSTEP:
        w = 3.0 * t * t - 2.0 * t * t * t
    elif schedule is BlendSchedule.EXP:
        denom = np.expm1(exp_k)  # exp(k)-1
        w = np.expm1(exp_k * t) / denom if denom != 0 else t
    else:  # pragma: no cover - defensive
        w = t
    # 数值兜底：确保端点精确
    w[0] = 0.0
    w[-1] = 1.0
    return w


class ChunkFuser:
    """重叠前缀混合：新 chunk 前缀与上一轨迹未执行后缀按权重混合。

    blend(new_chunk, prev_leftover) 返回与 new_chunk 同长度的数组：
      - 前 L = min(blend_horizon, len(prev_leftover), len(new)) 点为混合段；
      - 其余点直接取 new_chunk。
    若 prev_leftover 为 None（首块），原样返回 new_chunk。
    """

    def __init__(
        self,
        blend_horizon: int = 10,
        schedule: BlendSchedule | str = BlendSchedule.SMOOTHSTEP,
        exp_k: float = 3.0,
    ) -> None:
        self.blend_horizon = max(0, int(blend_horizon))
        self.exp_k = float(exp_k)
        if isinstance(schedule, str):
            schedule = BlendSchedule(schedule)
        self.schedule = schedule

    def blend(
        self,
        new_chunk: np.ndarray,
        prev_leftover: np.ndarray | None,
    ) -> np.ndarray:
        """混合 new_chunk 与 prev_leftover 的重叠前缀。

        Args:
            new_chunk: [C, D] 新动作块。
            prev_leftover: [Lp, D] 上一轨迹未执行后缀（chunk 点分辨率），或 None。

        Returns:
            [C, D] 融合后的动作块。
        """
        new_chunk = np.asarray(new_chunk, dtype=float)
        if new_chunk.ndim != 2:
            raise ValueError(f"new_chunk must be 2D [C, D], got shape {new_chunk.shape}")
        if prev_leftover is None or self.blend_horizon <= 0 or len(prev_leftover) == 0:
            return new_chunk.copy()

        prev_leftover = np.asarray(prev_leftover, dtype=float)
        L = min(self.blend_horizon, len(prev_leftover), len(new_chunk))
        if L <= 0:
            return new_chunk.copy()

        w = _blend_weights(L, self.schedule, self.exp_k)[:, None]  # [L, 1]
        out = new_chunk.copy()
        out[:L] = (1.0 - w) * prev_leftover[:L] + w * new_chunk[:L]
        return out


class ChunkInterpolator:
    """把 fps 间距的稀疏 chunk 点加密到 control_hz（默认 300Hz）。

    method:
      - "hermite"（默认）：Catmull-Rom 三次 Hermite，C1 连续，不在 waypoints
        处停顿，适合密集 chunk（推荐）。
      - "linear"：线性插值，C0。
      - "quintic"：s=10τ³-15τ⁴+6τ⁵，端点零速度（C2），会在每个 waypoint 处
        停顿，仅适合稀疏单动作 retarget。
    """

    def __init__(self, control_hz: float = 300.0, method: str = "hermite") -> None:
        self.control_hz = float(control_hz)
        if method not in ("hermite", "linear", "quintic"):
            raise ValueError(f"unknown interp method: {method!r}")
        self.method = method

    def densify(self, chunk: np.ndarray, point_dt: float) -> np.ndarray:
        """把 [C, D] chunk 加密为 [N, D] 的 control_hz 轨迹。

        Args:
            chunk: [C, D] 稀疏动作块（C >= 1）。
            point_dt: chunk 相邻点间距（秒），= 1/fps。

        Returns:
            [N, D] 密轨迹，N = 1 + (C-1)*points_per_segment。
        """
        chunk = np.asarray(chunk, dtype=float)
        if chunk.ndim != 2:
            raise ValueError(f"chunk must be 2D [C, D], got shape {chunk.shape}")
        C = len(chunk)
        if C == 1:
            return chunk.copy()

        pps = max(1, int(round(self.control_hz * float(point_dt))))
        if self.method == "linear":
            return self._densify_linear(chunk, pps)
        if self.method == "quintic":
            return self._densify_quintic(chunk, pps)
        return self._densify_hermite(chunk, pps)

    @staticmethod
    def _densify_linear(chunk: np.ndarray, pps: int) -> np.ndarray:
        C, D = chunk.shape
        out = np.empty((1 + (C - 1) * pps, D), dtype=float)
        out[0] = chunk[0]
        idx = 1
        for i in range(C - 1):
            p0, p1 = chunk[i], chunk[i + 1]
            for k in range(1, pps + 1):
                tau = k / pps
                out[idx] = p0 + tau * (p1 - p0)
                idx += 1
        return out

    @staticmethod
    def _densify_quintic(chunk: np.ndarray, pps: int) -> np.ndarray:
        """s(τ)=10τ³-15τ⁴+6τ⁵，端点零速度（与桥接单动作 quintic 一致）。"""
        C, D = chunk.shape
        out = np.empty((1 + (C - 1) * pps, D), dtype=float)
        out[0] = chunk[0]
        idx = 1
        for i in range(C - 1):
            p0, p1 = chunk[i], chunk[i + 1]
            for k in range(1, pps + 1):
                tau = k / pps
                s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
                out[idx] = p0 + s * (p1 - p0)
                idx += 1
        return out

    @staticmethod
    def _densify_hermite(chunk: np.ndarray, pps: int) -> np.ndarray:
        """Catmull-Rom 三次 Hermite：切线由相邻 waypoint 估计，C1 连续无停顿。"""
        C, D = chunk.shape
        out = np.empty((1 + (C - 1) * pps, D), dtype=float)
        out[0] = chunk[0]
        idx = 1
        for i in range(C - 1):
            p0 = chunk[i - 1] if i - 1 >= 0 else chunk[i]
            p1 = chunk[i]
            p2 = chunk[i + 1]
            p3 = chunk[i + 2] if i + 2 < C else chunk[i + 1]
            # 切线（uniform Catmull-Rom，除以 2 对应 tau∈[0,1] 的段长）
            m1 = (p2 - p0) * 0.5
            m2 = (p3 - p1) * 0.5
            for k in range(1, pps + 1):
                t = k / pps
                t2 = t * t
                t3 = t2 * t
                # Hermite 基函数
                h00 = 2.0 * t3 - 3.0 * t2 + 1.0
                h10 = t3 - 2.0 * t2 + t
                h01 = -2.0 * t3 + 3.0 * t2
                h11 = t3 - t2
                out[idx] = h00 * p1 + h10 * m1 + h01 * p2 + h11 * m2
                idx += 1
        return out


def smooth_window(chunk: np.ndarray, window: int = 1) -> np.ndarray:
    """滑窗均值滤波（对 [C, D] 的每个 DOF 沿时间轴）。

    Args:
        chunk: [C, D]。
        window: 窗口大小（奇数更对称）；<=1 原样返回。
    """
    if window <= 1:
        return np.asarray(chunk, dtype=float).copy()
    chunk = np.asarray(chunk, dtype=float)
    if chunk.ndim != 2:
        raise ValueError(f"chunk must be 2D [C, D], got shape {chunk.shape}")
    C, D = chunk.shape
    half = window // 2
    out = chunk.copy()
    for d in range(D):
        col = chunk[:, d]
        for t in range(C):
            lo = max(0, t - half)
            hi = min(C, t + half + 1)
            out[t, d] = float(np.mean(col[lo:hi]))
    return out


def transition_ramp(
    current_pos: np.ndarray,
    chunk_first: np.ndarray,
    n_pts: int,
) -> np.ndarray:
    """从当前位置 quintic 缓动到 chunk 首点（跨 chunk 兜底，防跳变）。

    返回 [n_pts, D]，ramp[0]=current_pos，ramp[-1]=chunk_first。
    n_pts<=1 时返回单行 current_pos。
    """
    current_pos = np.asarray(current_pos, dtype=float)
    chunk_first = np.asarray(chunk_first, dtype=float)
    n_pts = max(1, int(n_pts))
    if n_pts == 1:
        return current_pos.reshape(1, -1).copy()
    t = np.linspace(0.0, 1.0, n_pts, dtype=float)[:, None]
    s = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5   # [n_pts, 1]
    return current_pos[None, :] + s * (chunk_first - current_pos)[None, :]
