#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pkl 帧号空间的 label.json -> HDF5 拼接帧号空间的 label.json。

真机采集目录（如 ubt_IL/dataset/20261102_144321）里的 label.json 是按**原始 pkl
帧号**标注的（start_frame=5453 指的是第 5453 个 pkl）。robot_insight 的
convert_pkl_to_hdf5 只把 label 覆盖的段拼进 HDF5（段间间隙丢弃），所以 HDF5 的
帧号 = 各段长度的累计，与原 pkl 帧号相差一个随段累积的偏移。

转换器（--label-root）按 HDF5 帧号切段，因此喂给它的 label.json 必须换算成
HDF5 空间。本脚本做这个换算（只做累计偏移，逐段验证长度一致性）。

用法：

    python make_hdf5_label.py <pkl空间label.json> <输出目录>
    # 例：python make_hdf5_label.py ../../dataset/20261102_144321/label.json labels/
    # 输出 labels/20261102_144321/label.json（转换时 LABEL_ROOT=labels/）
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def convert(src: Path, out_root: Path, hdf5_frames: int | None = None) -> Path:
    with open(src, "r", encoding="utf-8") as f:
        label = json.load(f)

    datas = label.get("datas", [])
    if not datas:
        raise ValueError(f"{src} 没有 datas 段")

    out_datas = []
    cursor = 0
    for seg in datas:
        start, end = int(seg["start_frame"]), int(seg["end_frame"])
        n = end - start
        out_datas.append({**seg, "start_frame": cursor, "end_frame": cursor + n})
        cursor += n

    if hdf5_frames is not None and cursor != hdf5_frames:
        raise ValueError(
            f"段长度累计 {cursor} 与 HDF5 帧数 {hdf5_frames} 不一致 —— "
            "label.json 与 HDF5 不配套（段有增删？）"
        )

    # 目录名与采集目录同名，转换器按 <label-root>/<episode_dir>/label.json 查找
    out_dir = out_root / src.parent.name
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "label.json"

    new_label = dict(label)
    new_label["datas"] = out_datas
    new_label["src"] = f"{src} (frames remapped to HDF5 space)"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(new_label, f, ensure_ascii=False, indent=4)

    print(f"[make_hdf5_label] {len(out_datas)} 段, 共 {cursor} 帧 -> {out_path}")
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("src_label", help="pkl 帧号空间的 label.json 路径")
    parser.add_argument("out_root", help="输出根目录（按采集目录名建子目录）")
    parser.add_argument(
        "--hdf5-frames", type=int, default=None,
        help="HDF5 实际帧数，用于校验段长度累计一致（可选）",
    )
    args = parser.parse_args()

    src = Path(args.src_label).expanduser().resolve()
    if not src.is_file():
        print(f"[ERROR] 找不到 {src}", file=sys.stderr)
        return 1
    out = convert(src, Path(args.out_root), args.hdf5_frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
