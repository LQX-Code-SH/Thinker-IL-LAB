"""桥接 chunk 消费管线端到端仿真（无 ROS2 依赖）。

镜像 ros2_walker_bridge.py 的 _handle_chunk + _body_publish_loop 数学：
  chunk -> 延迟补偿 -> ChunkFuser.blend(prev_leftover) -> ChunkInterpolator.densify
        -> smooth -> transition_ramp -> 300Hz 轨迹 -> 指针推进 + leftover 采样

验证：
  1. 连续两块 chunk 在边界无跳变（fused[0] ≈ 当前执行点）。
  2. 轨迹指针推进正确，q_des 序列连续。
  3. hand chunk 按 fps 节拍推进。
  4. 延迟补偿跳过过期前缀。
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from chunk_processor import (  # noqa: E402
    ChunkFuser,
    ChunkInterpolator,
    smooth_window,
    transition_ramp,
)

CONTROL_HZ = 300.0
FPS = 15.0
POINT_DT = 1.0 / FPS
PPS = int(round(CONTROL_HZ * POINT_DT))  # 20


class FakeBridge:
    """复刻桥接 chunk 消费的纯数学状态机（无 ROS2；桥接已去 PD，仅余 v_max
    rate-limit，此处简化为 q_cmd=q_des，测试数据不触发限速）。"""

    def __init__(self, blend_horizon=10, ramp_pts=10, smoothing_window=1,
                 latency_compensation=True, method="hermite"):
        self.fuser = ChunkFuser(blend_horizon=blend_horizon, schedule="smoothstep")
        self.interp = ChunkInterpolator(control_hz=CONTROL_HZ, method=method)
        self.ramp_pts = ramp_pts
        self.smoothing_window = smoothing_window
        self.latency_compensation = latency_compensation

        self.body_traj = None
        self.traj_index = 0
        self.q_cmd = None
        self.hand_traj = {"left": None, "right": None}
        self.last_hand_cp = -1

    def _leftover(self):
        if self.body_traj is None or len(self.body_traj) == 0:
            return None
        idx = self.traj_index
        if idx >= len(self.body_traj):
            return self.body_traj[-1:]
        return self.body_traj[idx:][::PPS]

    def handle_chunk(self, body_chunk, inference_time_sec=0.0):
        body = np.asarray(body_chunk, dtype=float).copy()
        skip = 0
        if self.latency_compensation and inference_time_sec > 0:
            skip = min(int(np.ceil(inference_time_sec / POINT_DT)), len(body) - 1)
            body = body[skip:]
        fused = self.fuser.blend(body, self._leftover())
        densified = self.interp.densify(fused, POINT_DT)
        if self.smoothing_window > 1:
            densified = smooth_window(densified, self.smoothing_window)
        if self.q_cmd is not None and self.ramp_pts > 0:
            ramp = transition_ramp(self.q_cmd, densified[0], self.ramp_pts)
            traj = np.vstack([ramp, densified]) if len(ramp) > 1 else densified
        else:
            traj = densified
        self.body_traj = traj
        self.traj_index = 0
        self.q_cmd = np.array(traj[0], dtype=float)  # traj[0]=ramp 起点=当前位姿
        return traj

    def tick(self):
        """推进一个 300Hz tick，返回 (q_des, hand_advanced)。"""
        if self.body_traj is None:
            return None, (None, None)
        idx = min(self.traj_index, len(self.body_traj) - 1)
        q_des = np.array(self.body_traj[idx], dtype=float)
        if self.traj_index < len(self.body_traj) - 1:
            self.traj_index += 1
        self.q_cmd = q_des  # q_cmd 跟随 q_des（桥接有 v_max rate-limit，测试数据不触发故省略）
        # hand 推进
        cp = self.traj_index // PPS
        hands = (None, None)
        if cp != self.last_hand_cp:
            self.last_hand_cp = cp
            def pick(side):
                ht = self.hand_traj.get(side)
                if ht is None or len(ht) == 0:
                    return None
                return list(ht[min(cp, len(ht) - 1)])
            hands = (pick("left"), pick("right"))
        return q_des, hands


def test_two_chunks_no_seam_jump() -> None:
    """第二块 chunk 接入时，fused[0] ≈ 当前执行点（无跳变）。"""
    b = FakeBridge(blend_horizon=8, ramp_pts=5)
    # 第一块：从 0 线性升到 1（10 点）
    c1 = np.linspace(0, 1, 10)[:, None].repeat(3, axis=1)
    b.handle_chunk(c1)
    # 执行若干 tick（推进到中间）
    for _ in range(PPS * 3 + 5):
        b.tick()
    exec_point = b.body_traj[b.traj_index].copy()
    # 第二块：从 0.5 升到 1.5（首点 0.5 与当前执行点不同 -> 测试融合是否拉回执行点）
    c2 = np.linspace(0.5, 1.5, 10)[:, None].repeat(3, axis=1)
    traj2 = b.handle_chunk(c2)
    # fused[0]（ramp 之后 densified 首点）应 ≈ exec_point（融合 w[0]=0 全旧）
    # traj2 = ramp + densified；densified[0] = fused[0]
    densified_start = traj2[b.ramp_pts] if b.ramp_pts > 0 else traj2[0]
    assert np.allclose(densified_start, exec_point, atol=1e-9), (
        f"seam jump: densified_start={densified_start} exec_point={exec_point}"
    )


def test_pointer_advances_and_holds_last() -> None:
    """指针推进到末尾后保持末帧（不越界）。"""
    b = FakeBridge(ramp_pts=0)
    chunk = np.linspace(0, 1, 5)[:, None]
    b.handle_chunk(chunk)
    N = len(b.body_traj)
    # 执行 N + 10 个 tick，应全部不报错且末段保持末帧
    last_q = None
    for _ in range(N + 10):
        q, _ = b.tick()
        last_q = q
    assert np.allclose(last_q, b.body_traj[-1])


def test_hand_advances_at_fps() -> None:
    """hand 在每个 chunk 点（每 PPS 个 body tick）切换一次。"""
    b = FakeBridge(ramp_pts=0)
    body = np.zeros((6, 2))
    hand = np.array([[i, -i] for i in range(6)])  # [6,2]
    b.hand_traj = {"left": hand, "right": hand}
    b.handle_chunk(body)
    hand_emitted = []
    for _ in range(PPS * 6 + 5):
        _, (lh, rh) = b.tick()
        if lh is not None:
            hand_emitted.append(lh)
    # 应至少下发 hand[0]..hand[5]
    assert len(hand_emitted) >= 6
    assert hand_emitted[0] == [0, 0]
    assert hand_emitted[5] == [5, -5]


def test_latency_compensation_skips_prefix() -> None:
    """延迟补偿跳过 inference_time_sec 对应的过期前缀。"""
    b = FakeBridge(latency_compensation=True, ramp_pts=0)
    chunk = np.arange(10)[:, None].astype(float)  # 0..9
    # inference_time = 3 个 point_dt -> 跳过前 3 点 -> fused 从 3 开始
    traj = b.handle_chunk(chunk, inference_time_sec=3 * POINT_DT)
    # traj 首点应 = 3（跳过 0,1,2）
    assert np.allclose(traj[0], [3.0]), f"latency skip failed: {traj[0]}"

    # 关闭延迟补偿 -> 不跳过
    b2 = FakeBridge(latency_compensation=False, ramp_pts=0)
    traj2 = b2.handle_chunk(chunk, inference_time_sec=3 * POINT_DT)
    assert np.allclose(traj2[0], [0.0])


def test_continuous_chunks_smooth_trajectory() -> None:
    """多块 chunk 连续接入，q_des 序列整体连续（相邻 tick 差分有界）。"""
    b = FakeBridge(blend_horizon=6, ramp_pts=4)
    # 模拟 5 块 chunk，每块是同一段上升轨迹（receding horizon）
    base = np.linspace(0, 1, 12)[:, None].repeat(2, axis=1)
    q_des_seq = []
    for k in range(5):
        chunk = base + 0.2 * k  # 每块整体上移 0.2
        b.handle_chunk(chunk)
        # 每块执行 PPS*4 个 tick（少于整块，留 leftover 给下一块融合）
        for _ in range(PPS * 4):
            q, _ = b.tick()
            if q is not None:
                q_des_seq.append(q[0])
    q_des_seq = np.array(q_des_seq)
    # 相邻 tick 差分应无巨大跳变（融合 + ramp 消除边界跳变）
    diffs = np.abs(np.diff(q_des_seq))
    # 单 tick 最大跳变应远小于 chunk 间整体偏移 0.2
    assert diffs.max() < 0.05, f"trajectory discontinuity: max diff={diffs.max()}"
    # 整体趋势上升
    assert q_des_seq[-1] > q_des_seq[0] + 0.5


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
