#!/bin/bash
# 编译 Walker S2 的 ROS2 SDK 消息包（rosa/shm/mc_task/mc_state/ecat_task）。
#
# 产物落在独立工作空间 /opt/ubt_sim/walker_sdk_ros2_msgs，与其它机型的消息包
# 互不覆盖；只编译本目录下的包，不依赖 tienkung_pro / tienkung_dex 的任何东西。
#
# 使用：
#   bash /ubt_sim/teleoperation/msgs/walker_s2/build.sh
#   source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
#
# 环境变量：
#   WALKER_MSGS_OUT   输出工作空间（默认 /opt/ubt_sim/walker_sdk_ros2_msgs）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/walker_sdk_ros2_msgs_src/src"
OUT_DIR="${WALKER_MSGS_OUT:-/opt/ubt_sim/walker_sdk_ros2_msgs}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] 找不到 /opt/ros/humble/setup.bash，需在 ROS2 Humble 容器内运行" >&2
    exit 1
fi

# ROS 的 setup.bash 会引用未定义变量，source 时临时关掉 -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

echo "[INFO] Walker S2 消息包 -> $OUT_DIR"
colcon --log-base "$OUT_DIR/log" build \
    --base-paths "$SRC_DIR" \
    --build-base "$OUT_DIR/build" \
    --install-base "$OUT_DIR/install" \
    --merge-install \
    --packages-up-to rosa_msgs shm_msgs mc_task_msgs mc_state_msgs ecat_task_msgs
