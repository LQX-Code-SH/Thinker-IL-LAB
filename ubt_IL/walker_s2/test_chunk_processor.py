"""chunk_processor 单测。纯 numpy，可独立运行：python3 test_chunk_processor.py"""

from __future__ import annotations

import sys
import os

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from chunk_processor import (  # noqa: E402
    BlendSchedule,
    ChunkFuser,
    ChunkInterpolator,
    smooth_window,
    transition_ramp,
)


def _almost(a: np.ndarray, b: np.ndarray, tol: float = 1e-9) -> bool:
    return np.allclose(a, b, atol=tol)


def test_fuser_none_leftover_returns_copy() -> None:
    f = ChunkFuser(blend_horizon=5)
    new = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    out = f.blend(new, None)
    assert _almost(out, new)
    # 修改 out 不影响 new（copy）
    out[0, 0] = 999.0
    assert new[0, 0] == 1.0


def test_fuser_c0_continuity() -> None:
    """out[0] == prev_leftover[0]（旧），blend 段末 == new[L-1]（新）。"""
    f = ChunkFuser(blend_horizon=4, schedule=BlendSchedule.SMOOTHSTEP)
    prev = np.zeros((4, 2))
    prev[:, 0] = 10.0
    new = np.zeros((6, 2))
    new[:, 1] = 5.0
    out = f.blend(new, prev)
    L = 4
    assert _almost(out[0], prev[0])           # w[0]=0 -> 全旧
    assert _almost(out[L - 1], new[L - 1])    # w[-1]=1 -> 全新
    assert _almost(out[L:], new[L:])          # 非混合段直通
    # 权重单调递增：out[:,1] 从 prev[:,1]=0 升到 new[:,1]=5
    col = out[:L, 1]
    assert np.all(np.diff(col) >= -1e-12)


def test_fuser_blend_horizon_clamped() -> None:
    f = ChunkFuser(blend_horizon=100, schedule=BlendSchedule.LINEAR)
    prev = np.ones((3, 1))
    new = np.zeros((5, 1)) * 0.0
    out = f.blend(new, prev)
    L = min(100, 3, 5)  # =3
    assert _almost(out[0], prev[0])   # 1.0
    assert _almost(out[L - 1], new[L - 1])  # 0.0
    assert _almost(out[L:], new[L:])  # 0.0


def test_fuser_zero_horizon_passthrough() -> None:
    f = ChunkFuser(blend_horizon=0)
    new = np.array([[1.0], [2.0]])
    prev = np.array([[9.0], [9.0]])
    out = f.blend(new, prev)
    assert _almost(out, new)


def test_fuser_linear_weights() -> None:
    f = ChunkFuser(blend_horizon=2, schedule=BlendSchedule.LINEAR)
    prev = np.array([[0.0]])
    new = np.array([[10.0], [20.0]])
    out = f.blend(new, prev)
    # L=1 (min(2,1,2))，单点 w[0]=0 -> 全旧
    assert _almost(out[0], prev[0])


def test_interp_single_point() -> None:
    interp = ChunkInterpolator(control_hz=300, method="hermite")
    chunk = np.array([[1.0, 2.0, 3.0]])
    out = interp.densify(chunk, point_dt=1.0 / 15.0)
    assert out.shape == (1, 3)
    assert _almost(out[0], chunk[0])


def test_interp_length_and_endpoints() -> None:
    """输出长度 = 1 + (C-1)*pps，端点值匹配 chunk 端点。"""
    for method in ("linear", "quintic", "hermite"):
        interp = ChunkInterpolator(control_hz=300, method=method)
        chunk = np.linspace(0, 1, 4)[:, None].repeat(2, axis=1)  # [4,2]
        out = interp.densify(chunk, point_dt=1.0 / 15.0)
        pps = 20
        assert out.shape == (1 + 3 * pps, 2), f"{method}: {out.shape}"
        assert _almost(out[0], chunk[0]), f"{method} start"
        assert _almost(out[-1], chunk[-1]), f"{method} end"


def test_interp_linear_monotone() -> None:
    interp = ChunkInterpolator(control_hz=300, method="linear")
    chunk = np.array([[0.0], [1.0], [2.0]])
    out = interp.densify(chunk, point_dt=1.0 / 15.0)
    assert np.all(np.diff(out[:, 0]) >= -1e-9)
    # chunk[j] 落在输出索引 j*pps：chunk[1]=1.0 在 index pps=20
    pps = 20
    assert _almost(out[pps], np.array([1.0]))
    assert _almost(out[2 * pps], np.array([2.0]))


def test_interp_hermite_no_stall() -> None:
    """Catmull-Rom 不在内部 waypoint 处停顿（速度不为 0）。"""
    interp = ChunkInterpolator(control_hz=300, method="hermite")
    chunk = np.array([[0.0], [1.0], [2.0], [3.0]])
    out = interp.densify(chunk, point_dt=1.0 / 15.0)
    pps = 20
    vel = np.diff(out[:, 0])
    # 内部 waypoint 在 pps, 2*pps（chunk[1], chunk[2]）。其两侧速度应同号且非零。
    for wp in (pps, 2 * pps):
        assert vel[wp - 1] > 1e-6, f"stall before waypoint {wp}: {vel[wp-1]}"
        assert vel[wp] > 1e-6, f"stall after waypoint {wp}: {vel[wp]}"


def test_interp_quintic_stalls_at_waypoints() -> None:
    """quintic 端点零速度：内部 waypoint 处速度≈0（对照，说明为何默认 hermite）。"""
    interp = ChunkInterpolator(control_hz=300, method="quintic")
    chunk = np.array([[0.0], [1.0], [2.0]])
    out = interp.densify(chunk, point_dt=1.0 / 15.0)
    pps = 20
    vel = np.diff(out[:, 0])
    # waypoint pps（chunk[1]）处两侧速度均接近 0（quintic 端点零速度；hermite 约 0.05）
    assert abs(vel[pps - 1]) < 2e-3, f"quintic should ease-out: {vel[pps-1]}"
    assert abs(vel[pps]) < 2e-3, f"quintic should ease-in: {vel[pps]}"


def test_interp_c1_continuity_hermite() -> None:
    """hermite 在内部 waypoint 处速度近似连续（有限差分代理，解析导数严格 C1）。"""
    interp = ChunkInterpolator(control_hz=300, method="hermite")
    chunk = np.array([[0.0], [1.0], [2.0], [3.0]])
    out = interp.densify(chunk, point_dt=1.0 / 15.0)
    pps = 20
    # waypoint pps（chunk[1]）：左速度 |out[pps]-out[pps-1]|，右速度 |out[pps+1]-out[pps]|
    # 有限差分采样偏离 waypoint，受段内曲率影响，故用 5% 宽限（解析导数两侧严格相等）。
    left = abs(out[pps] - out[pps - 1])
    right = abs(out[pps + 1] - out[pps])
    assert abs(left - right) < 0.05 * max(left, right, 1e-6), f"C1 discontinuity: {left} vs {right}"


def test_smooth_window_passthrough_and_filter() -> None:
    chunk = np.array([[0.0], [1.0], [0.0], [1.0]])
    out0 = smooth_window(chunk, window=1)
    assert _almost(out0, chunk)
    out3 = smooth_window(chunk, window=3)
    # 中间两点应被邻域平均拉向 0.5 附近
    assert out3[1, 0] < 1.0
    assert out3[2, 0] > 0.0


def test_transition_ramp_endpoints() -> None:
    cur = np.array([0.0, 0.0])
    first = np.array([1.0, 2.0])
    ramp = transition_ramp(cur, first, n_pts=10)
    assert ramp.shape == (10, 2)
    assert _almost(ramp[0], cur)
    assert _almost(ramp[-1], first)
    # 单调
    assert np.all(np.diff(ramp[:, 0]) >= -1e-9)


def test_transition_ramp_single_point() -> None:
    cur = np.array([1.0])
    ramp = transition_ramp(cur, np.array([5.0]), n_pts=1)
    assert ramp.shape == (1, 1)
    assert _almost(ramp[0], cur)


def _run() -> None:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {t.__name__}: {e}")
            raise
    print(f"\n{passed}/{len(tests)} tests passed.")


if __name__ == "__main__":
    _run()
