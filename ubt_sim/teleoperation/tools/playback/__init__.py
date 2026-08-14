# -*- coding: utf-8 -*-
"""数据集回放包：预览（Rerun）+ 控制（ROS2 直发录制动作）。

结构：
- common.py      —— 机器人无关核心（HDF5 加载、节奏、CLI、Rerun 预览）
- robots/*.py    —— 机器人适配层（schema + ROS2 发布器 + 首帧对齐）

入口脚本在上级目录：playback_walker_s2.py / playback_tienkung_pro.py。
本包不重导出 rclpy / rerun 等重依赖，宿主机 --help / --dry-run 无感。
"""
