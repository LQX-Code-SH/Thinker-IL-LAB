#!/usr/bin/env python3
"""Walker C1 回 home（零位）—— ``robot_ready.py --pose zero`` 的薄封装。

home 定义：14 个臂关节全 0（手臂竖直贴身），手张开 0 rad。
路径：ready → home，**严格等于 ready 三段动作的逆序**：

  1/3  ready → 抬肩收肘位（腕/肩俯仰回零，保持 shoulder_roll=±1.5、elbow=-1.5）
  2/3  展开肘（elbow_pitch -1.5 → 0），肩仍抬起
  3/3  放下肩（shoulder_roll ±1.5 → 0）→ 全 0

这样手臂不会贴着桌面横扫，每段默认 ≥5 s（`--min-segment-s`）。

用法:
  python3 robot_home.py                        # 回 home（3 段 × 5 s）
  python3 robot_home.py --dry-run              # 只打印分段与时长
  python3 robot_home.py --min-segment-s 8      # 每段 8 s
  python3 robot_home.py --hold-s 1.0           # 末端保持 1 s

容器内更常用入口：robot_home.sh（负责 source ROS2 环境）。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_ready import main as _ready_main  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    return _ready_main(["--pose", "zero", *args])


if __name__ == "__main__":
    raise SystemExit(main())
