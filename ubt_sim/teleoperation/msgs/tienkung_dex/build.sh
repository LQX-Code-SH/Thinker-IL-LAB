#!/bin/bash
# 编译天工行者 Dex 的真机同款消息包（ros2_bridge_msgs + 因时/强脑灵巧手）。
#
# 产物落在独立工作空间 /opt/ubt_sim/tienkung_dex_msgs，与其它机型的消息包
# 互不覆盖；只编译本目录下的包，不依赖 walker_s2 / tienkung_pro 的任何东西。
#
# 使用：
#   bash /ubt_sim/teleoperation/msgs/tienkung_dex/build.sh
#   source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash
#
# 环境变量：
#   TIENKUNG_DEX_MSGS_OUT   输出工作空间（默认 /opt/ubt_sim/tienkung_dex_msgs）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${TIENKUNG_DEX_MSGS_OUT:-/opt/ubt_sim/tienkung_dex_msgs}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] 找不到 /opt/ros/humble/setup.bash，需在 ROS2 Humble 容器内运行" >&2
    exit 1
fi

# ROS 的 setup.bash 会引用未定义变量，source 时临时关掉 -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

echo "[INFO] TienKung Dex 消息包 -> $OUT_DIR"
colcon --log-base "$OUT_DIR/log" build \
    --base-paths "$SCRIPT_DIR/ros2_bridge_msgs_src/src" \
                 "$SCRIPT_DIR/dex_hand_msgs_src/src" \
    --build-base "$OUT_DIR/build" \
    --install-base "$OUT_DIR/install" \
    --merge-install \
    --packages-up-to ros2_bridge_msgs brainco_hand_msgs inspire_hand_msgs
