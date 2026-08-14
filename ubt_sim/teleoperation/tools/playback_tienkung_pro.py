#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""天工 Pro 数据集回放入口（预览 / 控制回放）。

用法（容器内，/usr/bin/python3）：

    # 预览（Rerun 查看相机 + 关节曲线）
    python3 teleoperation/tools/playback_tienkung_pro.py \
        --episode dataset/tienkung_pro/1786638182 --mode preview

    # 控制（回放录制动作；仿真 / 真机，详见 playback/README.md）
    source /opt/ros/humble/setup.bash
    python3 teleoperation/tools/playback_tienkung_pro.py \
        --episode dataset/tienkung_pro/1786638182 --mode control --dry-run
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # tools/ 目录（支持直接运行）

from playback.common import main
from playback.robots.tienkung_pro import TienkungProAdapter

if __name__ == "__main__":
    main(TienkungProAdapter)
