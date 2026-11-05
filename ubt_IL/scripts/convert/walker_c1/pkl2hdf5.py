#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker C1 真机 pkl 逐帧文件 -> 单 HDF5（字段布局与 thinker-studio 采集产物一致）。

采集侧每帧一个 pkl（文件名即该帧时间戳，如 ``2026-11-05T14_59_46.010.pkl``）：:

    arm_joints      {left:(7,), right:(7,)}      本体反馈  -> puppet/arm_*
    hand_joints     {left:(6,), right:(6,)}      手部反馈  -> puppet/end_effector_*
    control.arm.position  / control.hand.position  下发指令 -> master/*
    images          {head_left: JPEG, head_right: JPEG}
    depths          {head: None}                 该机型无深度
    force_sensor    {left:(0,), right:(0,)}      未接，恒空 -> 每帧 6 个 0

输出与 ``ubt_IL/dataset/20261105_145920/hdf5/*.hdf5`` 同构：master/puppet 八个数值字段
（float64/float32）、camera_observations/color_images/camera_head、force_sensor/{left,right}、
timestamps（|S23 文件名），attrs 里 n_frames / source。

与原产物的两点差异：
1. 原产物 camera_head 是 (T,720,1280,3) 全零（图像根本没写进去）；这里写真实 JPEG 解码后的
   原生分辨率（960x768），需要别的尺寸用 --resize。
2. 深度：pkl 里 depths.head 为 None，默认不写空深度数组（省 2.7GB），需要占位用 --write-depth。

用法::

    /usr/bin/python3 ubt_IL/scripts/convert/walker_c1/pkl2hdf5.py \
        --src /data/thinker-studio/walker_c1/tasks/lqx/collect_data/20261105_145920 \
        --dst-root ubt_IL/dataset --subdir hdf5_fixed \
        --label ubt_IL/dataset/20261105_145920/label.json
"""

from __future__ import annotations

import argparse
import io
import json
import sys
import time
from pathlib import Path

import h5py
import numpy as np

# (HDF5 键, pkl 取值路径, 维数, dtype)
NUM_FIELDS = [
    ("master/arm_left_position_align/data", ("control", "arm", "position", "left"), 7, np.float64),
    ("master/arm_right_position_align/data", ("control", "arm", "position", "right"), 7, np.float64),
    ("master/end_effector_left_position_align/data", ("control", "hand", "position", "left"), 6, np.float64),
    ("master/end_effector_right_position_align/data", ("control", "hand", "position", "right"), 6, np.float64),
    ("puppet/arm_left_position_align/data", ("arm_joints", "left"), 7, np.float32),
    ("puppet/arm_right_position_align/data", ("arm_joints", "right"), 7, np.float32),
    ("puppet/end_effector_left_position_align/data", ("hand_joints", "left"), 6, np.float32),
    ("puppet/end_effector_right_position_align/data", ("hand_joints", "right"), 6, np.float32),
]
CAMERA_KEYS = {"left": "head_left", "right": "head_right"}


def dig(obj, path):
    cur = obj
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            raise KeyError("pkl 缺少字段: " + ".".join(path))
        cur = cur[key]
    return cur


def decode_jpeg(buf: bytes) -> np.ndarray:
    """JPEG bytes -> (H,W,3) uint8 RGB。"""
    try:
        from PIL import Image

        with Image.open(io.BytesIO(buf)) as im:
            return np.asarray(im.convert("RGB"), dtype=np.uint8)
    except ImportError:
        import cv2

        arr = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            raise ValueError("JPEG 解码失败")
        return cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)


def load_segments(label_path: Path):
    """读 pkl 帧号空间的 label.json -> [(start, end)]（end 开区间）。"""
    with open(label_path, "r", encoding="utf-8") as f:
        label = json.load(f)
    if label.get("result") != "pass":
        raise ValueError(f"{label_path}: result={label.get('result')!r}，非 pass 不予转换")
    datas = sorted(label.get("datas", []), key=lambda d: int(d["start_frame"]))
    if not datas:
        raise ValueError(f"{label_path}: 没有 datas 段")
    return [(int(d["start_frame"]), int(d["end_frame"])) for d in datas]


def build_frame_index(segs, total: int) -> np.ndarray:
    if segs is None:
        return np.arange(total, dtype=np.int64)
    idx = []
    for start, end in segs:
        if not (0 <= start < end <= total):
            raise ValueError(f"段 [{start},{end}) 超出 pkl 帧数 {total}")
        idx.extend(range(start, end))
    return np.asarray(idx, dtype=np.int64)


def source_attr(segs) -> str:
    if not segs:
        return "converted from pkl files"
    return "converted from pkl files, segments: " + "、".join(f"{s}~{e - 1}帧" for s, e in segs)


def main() -> int:
    ap = argparse.ArgumentParser(description="Walker C1 pkl -> HDF5")
    ap.add_argument("--src", required=True, help="pkl 帧所在目录")
    ap.add_argument("--out", default=None, help="输出 HDF5 路径（优先）")
    ap.add_argument("--dst-root", default=None, help="数据集根目录（默认 <repo>/ubt_IL/dataset）")
    ap.add_argument("--subdir", default="hdf5", help="数据集内子目录（默认 hdf5）")
    ap.add_argument("--name", default=None, help="输出名（默认 --src 目录名）")
    ap.add_argument("--label", default=None, help="label.json（pkl 帧号空间），给定时只保留标注段")
    ap.add_argument("--camera", default="left", choices=["left", "right"], help="哪路相写成 camera_head")
    ap.add_argument("--resize", default=None, help="可选缩放 WxH，默认原生分辨率")
    ap.add_argument("--write-depth", action="store_true", help="额外写全零 uint16 深度数组（格式占位）")
    ap.add_argument("--compression", default="gzip", choices=["none", "gzip", "lzf"], help="图像压缩方式")
    ap.add_argument("--overwrite", action="store_true", help="覆盖已存在的输出文件")
    ap.add_argument("--limit", type=int, default=0, help="只转前 N 帧（调试）")
    args = ap.parse_args()

    src = Path(args.src).expanduser().resolve()
    if not src.is_dir():
        print(f"[ERROR] 源目录不存在: {src}", file=sys.stderr)
        return 1
    pkls = sorted(p for p in src.iterdir() if p.suffix == ".pkl" and p.is_file())
    if not pkls:
        print(f"[ERROR] {src} 下没有 *.pkl", file=sys.stderr)
        return 1

    name = args.name or src.name
    if args.out:
        out = Path(args.out).expanduser().resolve()
    else:
        repo_root = Path(__file__).resolve().parent.parent.parent.parent
        dst_root = Path(args.dst_root).expanduser().resolve() if args.dst_root else repo_root / "dataset"
        out = dst_root / name / args.subdir / f"{name}.hdf5"
    if out.exists() and not args.overwrite:
        print(f"[ERROR] 输出已存在（加 --overwrite 覆盖）: {out}", file=sys.stderr)
        return 1

    segs = load_segments(Path(args.label).expanduser().resolve()) if args.label else None
    frame_idx = build_frame_index(segs, len(pkls))
    if args.limit:
        frame_idx = frame_idx[: args.limit]
    n = len(frame_idx)

    resize = None
    if args.resize:
        w, h = (int(x) for x in args.resize.lower().split("x"))
        resize = (w, h)
    comp = {} if args.compression == "none" else {"compression": args.compression}
    if args.compression == "gzip":
        comp["compression_opts"] = 1

    print(f"[pkl2hdf5] src      = {src}")
    print(f"[pkl2hdf5] pkl 帧数 = {len(pkls)}，写入 {n} 帧（label {len(segs) if segs else 0} 段）")
    print(f"[pkl2hdf5] out      = {out}")

    out.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    with h5py.File(out, "w") as f:
        # 先取首帧确定图像尺寸（也能尽早暴露坏 pkl）
        first = pkls[frame_idx[0]]
        import pickle

        with open(first, "rb") as fh:
            d0 = pickle.load(fh)
        img0 = decode_jpeg(d0["images"][CAMERA_KEYS[args.camera]])
        if resize:
            import cv2

            img0 = cv2.resize(img0, resize)
        ih, iw = img0.shape[:2]
        print(f"[pkl2hdf5] 图像     = {iw}x{ih}，相机键 {CAMERA_KEYS[args.camera]} -> camera_head")

        f.create_dataset(
            "camera_observations/color_images/camera_head",
            shape=(n, ih, iw, 3), dtype=np.uint8, chunks=(1, ih, iw, 3), **comp,
        )
        if args.write_depth:
            f.create_dataset(
                "camera_observations/depth_images/camera_head",
                shape=(n, ih, iw), dtype=np.uint16, chunks=(1, ih, iw), **comp,
            )
        for key, _, width, dtype in NUM_FIELDS:
            f.create_dataset(key, shape=(n, width), dtype=dtype, chunks=(min(n, 1024), width))
        vlen = h5py.vlen_dtype(np.float64)
        for side in ("left", "right"):
            f.create_dataset(f"force_sensor/{side}", shape=(n,), dtype=vlen, chunks=(min(n, 4096),))
        f.create_dataset("timestamps", shape=(n,), dtype="S32", chunks=(min(n, 4096),))

        color = f["camera_observations/color_images/camera_head"]
        color[0] = img0
        for key, path, _, _ in NUM_FIELDS:
            f[key][0] = np.asarray(dig(d0, path), dtype=f[key].dtype)
        for side in ("left", "right"):
            fx = np.asarray(d0["force_sensor"][side], dtype=np.float64).ravel()
            f[f"force_sensor/{side}"][0] = fx if fx.size else np.zeros(6, dtype=np.float64)
        f["timestamps"][0] = first.stem.encode()

        for i in range(1, n):
            pkl_path = pkls[frame_idx[i]]
            with open(pkl_path, "rb") as fh:
                d = pickle.load(fh)
            img = decode_jpeg(d["images"][CAMERA_KEYS[args.camera]])
            if img.shape[:2] != (ih, iw):
                import cv2

                img = cv2.resize(img, (iw, ih))
            color[i] = img
            for key, path, _, _ in NUM_FIELDS:
                f[key][i] = np.asarray(dig(d, path), dtype=f[key].dtype)
            for side in ("left", "right"):
                fx = np.asarray(d["force_sensor"][side], dtype=np.float64).ravel()
                f[f"force_sensor/{side}"][i] = fx if fx.size else np.zeros(6, dtype=np.float64)
            f["timestamps"][i] = pkl_path.stem.encode()

            if (i + 1) % 200 == 0 or i + 1 == n:
                el = time.time() - t0
                rate = (i + 1) / el
                eta = (n - i - 1) / rate if rate > 0 else 0
                print(f"[pkl2hdf5] {i + 1}/{n} 帧  {rate:.1f} fps  剩余 {eta:.0f}s", flush=True)

        f.attrs["n_frames"] = n
        f.attrs["source"] = source_attr(segs)

    size_gb = out.stat().st_size / 1e9
    print(f"[pkl2hdf5] 完成: {out}  ({size_gb:.2f} GB, {time.time() - t0:.0f}s)")

    # 写盘后自检：帧数、时间戳、图像非零、关节范围
    with h5py.File(out, "r") as f:
        color = f["camera_observations/color_images/camera_head"]
        means = [float(color[i].mean()) for i in range(0, n, max(1, n // 8))]
        print(f"[check] n_frames={int(f.attrs['n_frames'])}  timestamps={f['timestamps'].shape[0]}")
        print(f"[check] 图像抽样均值: {[round(m, 1) for m in means]}")
        for key, _, _, _ in NUM_FIELDS:
            a = f[key][()]
            print(f"[check] {key}: {a.shape} {a.dtype} range={np.ptp(a, axis=0).round(3)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
