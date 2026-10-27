#!/bin/bash
# TienKung Dex 数据采集启动脚本
#
# 用法：
#   bash teleoperation/control/tienkung_dex/save_data.sh                       # 手动 Enter 控制
#   bash teleoperation/control/tienkung_dex/save_data.sh --auto-start --duration 30 --episodes 5
set -euo pipefail

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[ERROR] 未找到 ROS2 Humble 环境"
    exit 1
fi

# 真机同款消息包（仿真机上由 run.sh init 编译；真机上用厂商自带的）
if [ -f /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash ]; then
    set +u
    source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash
    set -u
fi

# 仿真 146 / 真机 0：采集代码本身完全一样
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-146}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

OUTPUT_DIR="${UBT_SIM_DEX_OUTPUT_DIR:-$PROJECT_DIR/dataset/tienkung_dex}"
mkdir -p "$OUTPUT_DIR"

echo "[INFO] 输出目录: $OUTPUT_DIR"
exec /usr/bin/python3 "$SCRIPT_DIR/save_data.py" --output-dir "$OUTPUT_DIR" "$@"
