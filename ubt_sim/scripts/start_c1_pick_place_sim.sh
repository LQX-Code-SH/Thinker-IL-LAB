#!/bin/bash
# 启动 Walker C1 抓放仿真 + ROS2 桥接（前台运行）。
# 默认 GUI；无窗口时显式传 --headless。
#
# 注意：C1 与 Walker S2 的 ZMQ 端口区间相同（5655-5658），不要同时启动。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

export UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
C1_STEP_HZ="${UBT_SIM_C1_STEP_HZ:-100}"

cd "$PROJECT_DIR"
exec bash scripts/start_sim.sh --device cpu --step_hz "$C1_STEP_HZ" "$@"
