#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tienkung_dex 采集数据体检（只读，不写盘）。

转换前先跑一遍，回答四个问题：

1. 30 集的 attrs 是否一致（body/hand 关节名、fps、hand_variant、task）
2. 每一维的运动量 —— 找出恒定维（死维）。死维进 LeRobot 数据集会触发已知的
   QUANTILES 归一化炸裂，必须在转换配置里剔除
3. 转换配置选的 indices 是否与数据实际有效维一致（选中死维 → 报错退出）
4. 相邻重复帧比例（15 Hz 录制跑在 ~11 Hz 仿真主循环上的副产物）

退出码：0 通过；1 发现阻断性问题（选中死维 / attrs 不一致 / 配置引用了不存在的 key）。

用法（宿主机 conda 或容器内均可）：

    python ubt_IL/scripts/convert/tienkung_dex/inspect_hdf5.py
    python ubt_IL/scripts/convert/tienkung_dex/inspect_hdf5.py --verbose
    python ubt_IL/scripts/convert/tienkung_dex/inspect_hdf5.py --all-fields
    SRC_ROOT=/path/to/dataset/tienkung_dex python .../inspect_hdf5.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import h5py
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent

# 判定「死维」的阈值：整段 30 集里 max-min 小于它即视为恒定
DEFAULT_DEAD_EPS = 1e-3
# 判定「算得上有运动」的阈值：用于提示配置漏选的维度。取 0.05 rad（≈2.9°），
# 低于它的多是静止站立时的被动下沉（腿 ~0.003、左臂 ~0.02、腰 ~0.037）
DEFAULT_MOVING_EPS = 0.05


def load_config(config_path: Path) -> tuple[dict, dict]:
    """读转换配置，返回 (features, hdf5_mapping)。"""
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    mapping = cfg.pop("hdf5_mapping", {})
    if not mapping:
        raise ValueError(f"{config_path} 缺少 hdf5_mapping 段")
    return cfg, mapping


def find_episodes(src_root: Path, hdf5_rel_path: str) -> list[Path]:
    """列出 `src_root/<episode>/<hdf5_rel_path>` 形式的所有 episode 文件。"""
    episodes = []
    for ep_dir in sorted(src_root.iterdir()):
        if not ep_dir.is_dir():
            continue
        candidate = ep_dir / hdf5_rel_path
        if candidate.is_file():
            episodes.append(candidate)
    return episodes


def collect_selected_indices(mapping: dict) -> dict[str, set[int] | None]:
    """从 hdf5_mapping 反推每个 HDF5 key 被选中的列。

    值是 None 表示整段取用（mapping 里给的是字符串或没有 indices 的 dict）。
    """
    selected: dict[str, set[int] | None] = {}
    for spec in mapping.values():
        if isinstance(spec, dict):  # 图像等单段字段
            continue
        for part in spec:
            if isinstance(part, str):
                selected[part] = None
            elif isinstance(part, dict) and "hdf5_key" in part:
                key = part["hdf5_key"]
                if "indices" not in part:
                    selected[key] = None
                    continue
                if selected.get(key, set()) is None:
                    continue  # 已整段取用，不用再记列号
                selected.setdefault(key, set()).update(part["indices"])
    return selected


def names_for(key: str, attrs: dict) -> list[str]:
    """按字段名挑对应的关节名清单（HDF5 attrs 里自带）。"""
    if "hand_left" in key:
        return [str(n) for n in attrs["left_hand_joint_names"]]
    if "hand_right" in key:
        return [str(n) for n in attrs["right_hand_joint_names"]]
    return [str(n) for n in attrs["body_joint_names"]]


def attr_signature(attrs: dict) -> dict:
    """只取需要跨集比对的那几个 attr。"""
    return {
        "robot": attrs.get("robot"),
        "hand_variant": attrs.get("hand_variant"),
        "task": attrs.get("task"),
        "fps": float(attrs["fps"]),
        "body_joint_names": tuple(str(n) for n in attrs["body_joint_names"]),
        "right_hand_joint_names": tuple(str(n) for n in attrs["right_hand_joint_names"]),
        "left_hand_joint_names": tuple(str(n) for n in attrs["left_hand_joint_names"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="tienkung_dex HDF5 数据体检（只读）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--src-root",
        default=os.environ.get("SRC_ROOT", str(PROJECT_ROOT / "dataset" / "tienkung_dex")),
        help=f"源数据根目录（默认 $SRC_ROOT 或 {PROJECT_ROOT / 'dataset' / 'tienkung_dex'}）",
    )
    parser.add_argument(
        "--config",
        default=os.environ.get("CONFIG", str(SCRIPT_DIR / "configs" / "tienkung_dex_20d_1RGB.json")),
        help="转换配置 JSON（用于交叉核对 indices）",
    )
    parser.add_argument(
        "--hdf5-rel-path",
        default=os.environ.get("HDF5_REL_PATH", "trajectory.hdf5"),
        help="HDF5 相对于 episode 目录的路径（默认 trajectory.hdf5）",
    )
    parser.add_argument(
        "--dead-eps",
        type=float,
        default=DEFAULT_DEAD_EPS,
        help=f"死维判定阈值，max-min 小于它即视为恒定（默认 {DEFAULT_DEAD_EPS}）",
    )
    parser.add_argument(
        "--moving-eps",
        type=float,
        default=DEFAULT_MOVING_EPS,
        help=f"「算得上有运动」的阈值，用于提示漏选维（默认 {DEFAULT_MOVING_EPS} rad ≈ 2.9°）",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="逐维展开全部维度")
    parser.add_argument(
        "--all-fields", action="store_true", help="连未被转换配置引用的字段（速度/力矩/左手）一起统计"
    )
    args = parser.parse_args()

    src_root = Path(args.src_root).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    if not src_root.is_dir():
        print(f"[ERROR] 源目录不存在: {src_root}", file=sys.stderr)
        print(
            f"        采集产物默认在 ubt_sim/dataset/tienkung_dex，"
            f"可复制/软链到 {PROJECT_ROOT / 'dataset' / 'tienkung_dex'}，"
            "或用 SRC_ROOT=<路径> 直接指定。",
            file=sys.stderr,
        )
        return 1
    if not config_path.is_file():
        print(f"[ERROR] 配置文件不存在: {config_path}", file=sys.stderr)
        return 1

    features, mapping = load_config(config_path)
    selected = collect_selected_indices(mapping)

    episodes = find_episodes(src_root, args.hdf5_rel_path)
    if not episodes:
        print(f"[ERROR] {src_root} 下没有找到 */{args.hdf5_rel_path}", file=sys.stderr)
        return 1

    # ---------------------------------------------------------------- 采集
    fields: dict[str, list[np.ndarray]] = {}
    frame_counts: list[int] = []
    dropped: list[int] = []
    timestamps: list[np.ndarray] = []
    ref_sig: dict | None = None
    sig_mismatch: list[str] = []
    attrs_ref: dict = {}

    for ep in episodes:
        with h5py.File(ep, "r") as f:
            attrs = dict(f.attrs)
            sig = attr_signature(attrs)
            if ref_sig is None:
                ref_sig, attrs_ref = sig, attrs
            elif sig != ref_sig:
                diff = [k for k in sig if sig[k] != ref_sig[k]]
                sig_mismatch.append(f"{ep.parent.name}: {', '.join(diff)}")

            keys = selected.keys() if not args.all_fields else [
                k for k, v in _iter_float_2d(f) if v
            ]
            for key in keys:
                if key not in f:
                    print(f"[ERROR] 配置引用的 HDF5 key 不存在: {key}（{ep}）", file=sys.stderr)
                    return 1
                fields.setdefault(key, []).append(f[key][()].astype(np.float64))

            # 真机 HDF5 不一定带 timestamp/num_frames（不同录制器的产物），缺了就跳过，
            # 不能因为体检脚本本身 KeyError 而看不到结构报告——它正是拿来诊断陌生数据集的。
            has_ts = "timestamp" in f
            frame_counts.append(
                int(attrs.get("num_frames", f["timestamp"].shape[0] if has_ts else 0))
            )
            dropped.append(int(attrs.get("dropped_frames", 0)))
            if has_ts:
                timestamps.append(f["timestamp"][()].astype(np.float64))

    total_frames = sum(frame_counts)

    print("=" * 78)
    print(f"tienkung_dex 数据体检  {src_root}")
    print("=" * 78)
    print(f"配置        : {config_path}")
    print(f"episodes    : {len(episodes)}")
    print(f"总帧数      : {total_frames}")
    print(f"帧数 min/max: {min(frame_counts)} / {max(frame_counts)}")
    print(f"fps (attr)  : {ref_sig['fps']:g}")
    print(f"robot       : {ref_sig['robot']}    hand_variant: {ref_sig['hand_variant']}")
    print(f"task        : {ref_sig['task']}")
    print(f"丢帧        : {sum(dropped)}（各集合计）")

    # 时间戳实际步长
    diffs = [np.diff(t) for t in timestamps if len(t) > 1]
    dts = np.concatenate(diffs) if diffs else np.array([])
    if dts.size:
        print(f"ts step     : mean {dts.mean()*1e3:.2f} ms  median {np.median(dts)*1e3:.2f} ms  "
              f"=> {1.0/np.median(dts):.2f} Hz")

    if sig_mismatch:
        print()
        print("[ERROR] 以下 episode 的 attrs 与第一集不一致：")
        for line in sig_mismatch:
            print(f"        {line}")
    else:
        print("attrs 一致性: OK（关节名 / fps / 手型 30 集全同）")

    # ------------------------------------------------------ 相邻重复帧比例
    # 按相机 JPEG 字节比对（策略实际看到的画面）；action 向量几乎每帧都在末位
    # 浮动，拿它比对会得到接近 0 的假阴性。
    image_key = next(
        (k for k, v in mapping.items() if isinstance(v, dict) and "hdf5_key" in v), None
    )
    if image_key is not None:
        dup_total = 0
        dup_base = 0
        for ep in episodes:
            with h5py.File(ep, "r") as f:
                frames = f[mapping[image_key]["hdf5_key"]][()]
            prev = None
            same = 0
            for raw in frames:
                cur = bytes(raw) if isinstance(raw, np.ndarray) else raw
                if prev is not None and cur == prev:
                    same += 1
                prev = cur
            dup_total += same
            dup_base += max(len(frames) - 1, 0)
        if dup_base:
            print(
                f"相邻重复帧  : {dup_total}/{dup_base} = {100.0*dup_total/dup_base:.1f}%"
                "（按相机 JPEG 字节比对；--trim-stationary 只裁长的静止游程，消除不了这部分）"
            )

    # ------------------------------------------------------------ 逐维统计
    errors: list[str] = []
    warnings: list[str] = []

    for key, chunks in fields.items():
        data = np.concatenate(chunks, axis=0)
        names = names_for(key, attrs_ref)
        lo = data.min(axis=0)
        hi = data.max(axis=0)
        rng = hi - lo
        std = data.std(axis=0)
        sel = selected.get(key, set())
        dead_idx = [i for i in range(data.shape[1]) if rng[i] < args.dead_eps]

        print()
        print(f"-- {key}  ({data.shape[1]} 维, {data.shape[0]} 帧)")
        print(f"   {'idx':>3}  {'name':<34s} {'min':>9s} {'max':>9s} {'range':>8s} {'std':>8s}")
        for i in range(data.shape[1]):
            names_i = names[i] if i < len(names) else f"<{i}>"
            is_sel = (sel is None) or (i in sel)
            mark = "  <-- 选中" if is_sel else ""
            if rng[i] < args.dead_eps:
                mark += "  [死维]"
            if not args.verbose and not is_sel and rng[i] < args.dead_eps:
                continue  # 未选中且是死维，默认不铺开
            print(f"   {i:>3}  {names_i:<34s} {lo[i]:>9.4f} {hi[i]:>9.4f} {rng[i]:>8.4f} {std[i]:>8.4f}{mark}")

        # 检查一：选中的维不能是死维
        if sel is None:
            bad = dead_idx
            scope = "全部"
        else:
            bad = [i for i in dead_idx if i in sel]
            scope = f"选中的 {len(sel)} 维"
        if bad:
            names_bad = ", ".join(f"[{i}]{names[i]}" for i in bad if i < len(names))
            errors.append(f"{key}: {scope}里有死维（range<{args.dead_eps:g}）: {names_bad}")

        # 检查二：没被选中但有明显运动的维，提示一遍防止漏维
        if sel is not None:
            missed = [i for i in range(data.shape[1]) if i not in sel and rng[i] >= args.moving_eps]
            if missed:
                names_missed = ", ".join(f"[{i}]{names[i]}(r={rng[i]:.3f})" for i in missed if i < len(names))
                warnings.append(f"{key}: 未被配置选中但有运动: {names_missed}")

        # 检查三：配置里出现了超出维度的下标
        if sel is not None:
            oob = [i for i in sel if i >= data.shape[1] or i < 0]
            if oob:
                errors.append(f"{key}: 配置 indices 越界 {oob}（该字段只有 {data.shape[1]} 维）")

    # 检查四：每个 LeRobot 字段的组合宽度必须等于配置声明的 shape
    widths = {k: v[0].shape[1] for k, v in fields.items()}
    n_declared = 0
    for lkey, spec in mapping.items():
        feat = features.get(lkey)
        if not isinstance(feat, dict) or "shape" not in feat or not isinstance(spec, list):
            continue
        total = 0
        for part in spec:
            if isinstance(part, str):
                total += widths.get(part, 0)
            elif isinstance(part, dict) and "hdf5_key" in part:
                w = widths.get(part["hdf5_key"], 0)
                total += len(part["indices"]) if "indices" in part else w
        declared = int(np.prod(feat["shape"]))
        n_declared = max(n_declared, declared)
        if total != declared:
            errors.append(
                f"{lkey}: hdf5_mapping 组合出 {total} 维，与配置声明的 shape {tuple(feat['shape'])} 不符"
            )

    # ------------------------------------------------------------ 结论汇总
    print()
    print("=" * 78)
    if warnings:
        print(f"[WARN] {len(warnings)} 条：")
        for w in warnings:
            print(f"       {w}")
    if errors:
        print(f"[ERROR] {len(errors)} 条阻断性问题：")
        for e in errors:
            print(f"        {e}")
        print("=" * 78)
        return 1

    print(f"[OK] 配置选中的 {n_declared} 维全部有运动，attrs 一致，可以开始转换。")
    print("=" * 78)
    return 0


def _iter_float_2d(f: h5py.File):
    """遍历文件里所有二维 float dataset（--all-fields 用）。"""
    found: list[tuple[str, bool]] = []

    def visit(name, obj):
        if isinstance(obj, h5py.Dataset) and obj.ndim == 2 and np.issubdtype(obj.dtype, np.floating):
            found.append((name, True))

    f.visititems(visit)
    return found


if __name__ == "__main__":
    sys.exit(main())
