#!/bin/bash
# 编译 Walker C1 的 ROS2 SDK 消息包（rosa/shm/mc_task/mc_state/ecat_task）。
#
# 产物落在独立工作空间 /opt/ubt_sim/walker_c1_msgs，与 Walker S2 的
# walker_sdk_ros2_msgs 完全隔离：两边各自演进、互不覆盖。
# 注意二者含同名包（mc_task_msgs 等），同一 shell 里不要同时 source。
#
# 使用：
#   bash /ubt_sim/teleoperation/msgs/walker_c1/build.sh
#   source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
#
# 环境变量：
#   WALKER_C1_MSGS_OUT   输出工作空间（默认 /opt/ubt_sim/walker_c1_msgs）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/walker_sdk_ros2_msgs_src/src"
OUT_DIR="${WALKER_C1_MSGS_OUT:-/opt/ubt_sim/walker_c1_msgs}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] 找不到 /opt/ros/humble/setup.bash，需在 ROS2 Humble 容器内运行" >&2
    exit 1
fi

# ROS 的 setup.bash 会引用未定义变量，source 时临时关掉 -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

echo "[INFO] Walker C1 消息包 -> $OUT_DIR"
colcon --log-base "$OUT_DIR/log" build \
    --base-paths "$SRC_DIR" \
    --build-base "$OUT_DIR/build" \
    --install-base "$OUT_DIR/install" \
    --merge-install \
    --packages-up-to rosa_msgs shm_msgs mc_task_msgs mc_state_msgs ecat_task_msgs
