#!/bin/bash
# Walker C1 复位包装（必须用系统 python3：rclpy 装在 3.10，不在 LeRobot venv）。
#
# 用法（容器内）：
#   bash /ubt_IL/scripts/deploy/walker_c1/reset.sh --mode sim-scene   # 等价按下仿真窗口 R
#   bash /ubt_IL/scripts/deploy/walker_c1/reset.sh --mode task        # 摆到任务预备位
#   bash /ubt_IL/scripts/deploy/walker_c1/reset.sh --mode both
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="${WALKER_C1_ROS_WS:-/ubt_IL/walker_s2/walker_sdk_ros2}"

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROS_WS/install/setup.bash"
else
    echo "[reset] 警告：$ROS_WS/install/setup.bash 不存在，mc_task_msgs 可能不可用" >&2
fi

exec /usr/bin/python3 "$SCRIPT_DIR/reset.py" "$@"
