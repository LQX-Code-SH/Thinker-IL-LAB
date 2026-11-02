#!/bin/bash
# Walker C1 预备位摆位（system python3 + rclpy，需 source ROS2 与消息工作空间）。
#
# 用法（容器内）：
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh --dry-run
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh --mode both   # 先仿真复位再摆位
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="${WALKER_C1_ROS_WS:-/ubt_IL/walker_s2/walker_sdk_ros2}"
MODE="task"

ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode) MODE="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROS_WS/install/setup.bash"
else
    echo "[robot_ready] 警告：$ROS_WS/install/setup.bash 不存在，mc_task_msgs 可能不可用" >&2
fi

exec /usr/bin/python3 "$SCRIPT_DIR/reset.py" --mode "$MODE" "${ARGS[@]}"
