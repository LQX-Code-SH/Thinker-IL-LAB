#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker C1 数据集回放入口（预览 / 控制回放）。

用法（容器内，/usr/bin/python3）：

    # 预览（Rerun 查看相机 + 关节曲线）；不给 --episode 就用最新的时间戳目录
    python3 teleoperation/tools/playback_walker_c1.py --mode preview

    # 控制（向 /mc/sdk/robot_command 与 /mc/*_hand/command 回放录制动作）
    source /opt/ros/humble/setup.bash
    source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
    python3 teleoperation/tools/playback_walker_c1.py \
        --episode dataset/walker_c1/<epoch秒>/trajectory.hdf5 --mode control
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # tools/ 目录

from playback.common import main
from playback.robots.walker_c1 import WalkerC1Adapter

if __name__ == "__main__":
    main(WalkerC1Adapter)
