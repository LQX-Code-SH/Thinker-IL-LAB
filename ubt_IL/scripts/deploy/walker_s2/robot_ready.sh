#!/bin/bash
# Walker S2 机器人预备姿态/回零（分步安全到位，对齐仿真 3 段流程）
# 用法：
#   robot_ready.sh            # 预备姿态（--init，分3步，默认13s）
#   robot_ready.sh --home     # 回零（分3步，默认15s）
#   robot_ready.sh --legacy-init / --legacy-home   # 旧4段流程回退
# 其余参数原样透传给 robot_control.py（如 --init-duration 20）
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_CTRL="$SCRIPT_DIR/../../../walker/walker_sdk_ros2/robot_control/robot_control.py"

if [ $# -eq 0 ]; then
    exec /usr/bin/python3 "$ROBOT_CTRL" --init
else
    exec /usr/bin/python3 "$ROBOT_CTRL" "$@"
fi
