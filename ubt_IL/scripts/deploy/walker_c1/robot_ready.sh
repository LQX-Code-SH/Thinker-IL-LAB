#!/bin/bash
# Walker C1 预备位摆位（system python3 + rclpy，需 source ROS2 与消息工作空间）。
#
# 用法（容器内）：
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh --dry-run
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh --mode both   # 先仿真复位再摆位
#
# 真机前置：必须在 half_sdk（开发者模式 + 上半身 SDK 权限）下运行，否则被
# sdk_guard.sh 拦截（`bash sdk_mode.sh half_sdk` 切换；仿真设 SDK_GUARD=0）。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="${WALKER_C1_ROS_WS:-/ubt_IL/walker_s2/walker_sdk_ros2}"
# 默认按真机处理（half_sdk 门禁生效）；仿真请显式 WALKER_C1_REAL=0
WALKER_C1_REAL="${WALKER_C1_REAL:-1}"
export WALKER_C1_REAL
MODE="task"

ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode) MODE="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

# 真机门禁：手臂指令只有在 half_sdk 下才会被执行（--dry-run 不动机器人，跳过）
DRY_RUN=0
for arg in "${ARGS[@]-}"; do
    if [[ "$arg" == "--dry-run" ]]; then
        DRY_RUN=1
    fi
done
if [[ "$DRY_RUN" -eq 0 ]]; then
    # shellcheck source=./sdk_guard.sh
    source "$SCRIPT_DIR/sdk_guard.sh"
    walker_c1_require_half_sdk "robot_ready" || exit 1
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROS_WS/install/setup.bash"
else
    echo "[robot_ready] 警告：$ROS_WS/install/setup.bash 不存在，mc_task_msgs 可能不可用" >&2
fi

exec /usr/bin/python3 "$SCRIPT_DIR/reset.py" --mode "$MODE" "${ARGS[@]}"
