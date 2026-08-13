# -*- coding: utf-8 -*-
"""静止帧判据校准诊断:采样 episode,统计 action 位移分布与游程。
action 构造同 walker_s2_sim_10d_2RGB.json 的 hdf5_mapping:
  joint[7..13] + joint[14,15] + grip_right
"""
import glob, os
import numpy as np
import h5py

ROOT = "/home/qingxiangliu/work/UBTECH-IL-LAB/ubt_IL/dataset/Walker-s2-pick-part-sim"


def build_action(f):
    jpos = np.array(f["observation/joint_state/position/data"])      # (T,17)
    grip = np.array(f["action/grip_right_position/data"])            # (T,)
    a = np.concatenate([jpos[:, [7,8,9,10,11,12,13,14,15]], grip[:, None]], axis=1)
    return a.astype(np.float64)                                       # (T,10)


def windowed_disp(a, W):
    T = len(a)
    disp = np.empty(T)
    for t in range(T):
        j = min(t + W, T - 1)
        disp[t] = np.abs(a[j] - a[t]).max()
    return disp


def run_lengths(mask):
    """返回连续 True 游程长度列表。"""
    runs, n, cur = [], len(mask), 0
    for i in range(n):
        if mask[i]:
            cur += 1
        else:
            if cur:
                runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    return runs


def main():
    eps = sorted(glob.glob(os.path.join(ROOT, "*/trajectory.hdf5")))
    sample = eps[::5][:32]   # ~32 episodes deterministic
    print(f"[diag] sampling {len(sample)}/{len(eps)} episodes\n")

    all_disp = {1: [], 5: []}      # W=1 (单帧差分) vs W=5 (窗口)
    all_fpv = []                    # frames per episode
    all_fps = []
    all_ranges = []                 # per-dim range
    lead_runs, trail_runs = [], []  # 首尾静止游程
    internal_runs_by_thr = {0.005: [], 0.01: [], 0.02: []}

    for p in sample:
        with h5py.File(p, "r") as f:
            a = build_action(f)
            ts = np.array(f["observation/timestamp/data"]).ravel()
        T = len(a)
        all_fpv.append(T)
        if ts.max() > 1e15:
            ts = ts / 1e9
        dur = ts[-1] - ts[0]
        if dur > 0:
            all_fps.append(T / dur)
        all_ranges.append(a.max(0) - a.min(0))

        for W in (1, 5):
            d = windowed_disp(a, W)
            all_disp[W].append(d)

        # 首尾静止游程 (W=5, thr=0.01)
        d5 = windowed_disp(a, 5)
        stat = d5 < 0.01
        # leading
        l = 0
        while l < T and stat[l]:
            l += 1
        lead_runs.append(l)
        # trailing
        r = T - 1
        while r >= 0 and stat[r]:
            r -= 1
        trail_runs.append(T - 1 - r)

        # 内部游程 (去掉首尾) 在不同阈值下
        inner = stat.copy()
        inner[:l] = False
        inner[r+1:] = False
        for thr in internal_runs_by_thr:
            inner_thr = (d5 < thr).copy()
            inner_thr[:l] = False
            inner_thr[r+1:] = False
            internal_runs_by_thr[thr].extend(run_lengths(inner_thr))

    fpv = np.array(all_fpv)
    print(f"=== episode 规模 ===")
    print(f"  frames/ep: mean={fpv.mean():.0f} min={fpv.min()} max={fpv.max()} total={fpv.sum()}")
    if all_fps:
        fps = np.array(all_fps)
        print(f"  实测 fps:  mean={fps.mean():.1f} min={fps.min():.1f} max={fps.max():.1f}")
    print()

    ranges = np.array(all_ranges)   # (n_ep, 10)
    dim_names = ["R_elbow_roll","R_elbow_yaw","R_sh_pitch","R_sh_roll","R_sh_yaw",
                 "R_wr_pitch","R_wr_roll","head_pitch","head_yaw","grip_right"]
    print(f"=== 每维行程 (max-min, 跨 episode 均值) ===")
    for i, nm in enumerate(dim_names):
        print(f"  {nm:14s}: mean={ranges[:,i].mean():.4f}  min={ranges[:,i].min():.4f}  max={ranges[:,i].max():.4f}")
    print()

    for W in (1, 5):
        d = np.concatenate(all_disp[W])
        print(f"=== 位移分布  W={W} ({len(d)} 帧) ===")
        edges = [0, 1e-5, 1e-4, 5e-4, 1e-3, 2e-3, 5e-3, 1e-2, 2e-2, 5e-2, 1e-1, 0.5, 1e9]
        h, _ = np.histogram(d, bins=edges)
        for i, c in enumerate(h):
            lo, hi = edges[i], edges[i+1]
            print(f"  [{lo:>8.1e}, {hi:>8.1e}): {c:6d}  ({100*c/len(d):5.1f}%)")
        print(f"  分位: p1={np.percentile(d,1):.2e} p5={np.percentile(d,5):.2e} "
              f"p50={np.percentile(d,50):.2e} p95={np.percentile(d,95):.2e}")
        print()

    print(f"=== 首尾静止游程 (W=5, thr=0.01, 单位=帧) ===")
    lr, tr = np.array(lead_runs), np.array(trail_runs)
    print(f"  leading:  mean={lr.mean():.1f} med={np.median(lr):.0f} max={lr.max()}  (>10帧的episode: {(lr>10).sum()}/{len(lr)})")
    print(f"  trailing: mean={tr.mean():.1f} med={np.median(tr):.0f} max={tr.max()}  (>10帧的episode: {(tr>10).sum()}/{len(tr)})")
    print()

    print(f"=== 内部长静止游程分布 (W=5) ===")
    for thr, runs in internal_runs_by_thr.items():
        if not runs:
            print(f"  thr={thr}: 无内部游程")
            continue
        r = np.array(runs)
        print(f"  thr={thr}: 游程数={len(r)} mean={r.mean():.1f} med={np.median(r):.0f} "
              f"max={r.max()}  (>=5帧: {(r>=5).sum()}, >=10帧: {(r>=10).sum()})")


if __name__ == "__main__":
    main()
