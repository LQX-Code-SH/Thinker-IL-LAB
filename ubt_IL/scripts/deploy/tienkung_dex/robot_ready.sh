#!/bin/bash
# TienKung Dex 机器人预备位（四段到位 + 预备位手型）
# 模型推理前把机器人摆到数据采集时的预备姿态。决策与依据见同目录 README.md。
#
# 用法:
#   bash robot_ready.sh                     # 仿真（ROS_DOMAIN_ID=146 需自己 export）
#   bash robot_ready.sh --real              # 真机（最终目标用 READY_POSE）
#   bash robot_ready.sh --dry-run           # 只打印四段目标，不发指令
#   bash robot_ready.sh --stage-durations 4,4,4,4 --rate 30
#
# 环境变量:
#   ROS_DOMAIN_ID   默认 0（真机）；仿真用 146
#   TIENKUNG_DEX_MSGS_WS  dex 消息工作空间（默认 /opt/tienkung_dex_msgs_ws）
#
# 注意：本脚本与 Bridge2 都写 /arm/cmd，Bridge2 在跑时 robot_ready.py 会拒绝执行
#       （--force 可跳过）。顺序永远是 先 robot_ready，再 rollout / inference_server。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MSGS_WS="${TIENKUNG_DEX_MSGS_WS:-/opt/tienkung_dex_msgs_ws}"

# ROS2 Humble 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "[robot_ready] ERROR: /opt/ros/humble/setup.bash 不存在" >&2
    exit 1
fi

# dex 消息包（ros2_bridge_msgs + inspire/brainco_hand_msgs），由镜像构建时编译
if [ -f "$MSGS_WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$MSGS_WS/install/setup.bash"
else
    echo "[robot_ready] ERROR: 找不到 $MSGS_WS/install/setup.bash" >&2
    echo "               容器内先编译: bash /ubt_IL/tienkung_dex/ros2_dex_msgs/build.sh" >&2
    exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 系统 python3（3.10，有 rclpy 与消息包）。不能跟着 PATH 里的 /lerobot/.venv（3.12），
# 那里面没有 rclpy，import 就会失败。
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

echo "[robot_ready] python: $PYTHON_BIN | ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

exec "$PYTHON_BIN" "$SCRIPT_DIR/robot_ready.py" "$@"
