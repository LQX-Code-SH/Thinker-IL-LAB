#!/bin/bash
# 机器人从预备位回到站立位（HOME_POSE，不含腿），按预备位四段航点倒序走。
#
#   bash robot_home_host.sh                 # 四段各 8s（约 32s + 3s 稳定）
#   bash robot_home_host.sh --dry-run       # 只打印航点
#   bash robot_home_host.sh --durations 6,6,6,6
#
# 与 Bridge2 互斥（都写 /arm/cmd）。腿不下发。
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

dex_source_ros
dex_check_msgs

if pgrep -f "ros2_deploy_bridge.py --config" >/dev/null 2>&1; then
    echo "[home] ERROR: Bridge2 在跑，会与 /arm/cmd 冲突，先 pkill -f ros2_deploy_bridge.py" >&2
    exit 1
fi

cd "$(dirname "${BASH_SOURCE[0]}")"
exec "$SYSPY" ./robot_home.py "$@"
