#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 数据集回放入口（预览 / 控制回放）。

用法（容器内，/usr/bin/python3）：

    # 预览（Rerun 查看相机 + 关节曲线）
    python3 teleoperation/tools/playback_tienkung_dex.py \
        --episode dataset/tienkung_dex/episode_0001.hdf5 --mode preview

    # 控制（向 /tienkung_dex/*_command 回放录制动作）
    source /opt/ros/humble/setup.bash
    python3 teleoperation/tools/playback_tienkung_dex.py \
        --episode dataset/tienkung_dex/episode_0001.hdf5 --mode control
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # tools/ 目录

from playback.common import main
from playback.robots.tienkung_dex import TienkungDexAdapter

if __name__ == "__main__":
    main(TienkungDexAdapter)
