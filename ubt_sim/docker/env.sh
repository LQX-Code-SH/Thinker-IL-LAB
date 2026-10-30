#!/bin/bash
# Docker environment variables for Isaac Sim container
CONTAINER_NAME="ubt-sim"
IMAGE="ubt-sim-isaac:latest"
BASE_IMAGE="nvcr.nju.edu.cn/nvidia/isaac-lab:2.2.0"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="${PROJECT_DIR}/shell/isaac-sim"
PIP_MIRROR="-i https://pypi.tuna.tsinghua.edu.cn/simple"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# 默认桥接脚本；walker_c1 等机型由 UBT_SIM_ROBOT 在 run.sh bridge-start 中分派
BRIDGE_SCRIPT="${UBT_SIM_BRIDGE_SCRIPT:-/ubt_sim/teleoperation/bridges/tienkung_pro/tienkung_pro_ros2_zmq_bridge.py}"
FASTRTPS_DEFAULT_PROFILES_FILE="/ubt_sim/docker/fastdds_no_shm.xml"
