#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 数据集回放入口（预览 / 控制回放）。

用法（容器内，/usr/bin/python3）：

    # 预览（Rerun 查看相机 + 关节曲线）
    python3 teleoperation/tools/playback_walker_s2.py \
        --episode dataset/walker_s2/1786681892 --mode preview

    # 控制（回放录制动作；仿真 / 真机，详见 playback/README.md）
    source /opt/ros/humble/setup.bash
    source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
    python3 teleoperation/tools/playback_walker_s2.py \
        --episode dataset/walker_s2/1786681892 --mode control --dry-run
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # tools/ 目录（支持直接运行）

from playback.common import main
from playback.robots.walker_s2 import WalkerS2Adapter

if __name__ == "__main__":
    main(WalkerS2Adapter)
