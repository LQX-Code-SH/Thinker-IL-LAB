#!/bin/bash
# Walker C1 回 home（零位；system python3 + rclpy，需 source ROS2 与消息工作空间）。
#
# home = 14 个臂关节全 0、手张开；路径是 robot_ready 三段动作的逆序
# （腕/肩俯仰回零 → 展开肘 → 放下肩），每段默认 ≥5 s。
#
# 用法（容器内）：
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_home.sh
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_home.sh --dry-run
#   bash /ubt_IL/scripts/deploy/walker_c1/robot_home.sh --min-segment-s 8 --hold-s 1.0
#
# 注意：插件里的 ``home_position``（rollout 断开时自动回位用）仍是 READY_POSE，
# 与本脚本的零位不同；需要统一时改 constants.HOME_POSITION。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="${WALKER_C1_ROS_WS:-/ubt_IL/walker_s2/walker_sdk_ros2}"
# 默认按真机处理（half_sdk 门禁生效）；仿真请显式 WALKER_C1_REAL=0
WALKER_C1_REAL="${WALKER_C1_REAL:-1}"
export WALKER_C1_REAL

# 真机门禁：手臂指令只有 half_sdk 下才执行；--dry-run 不动机器人，跳过
DRY_RUN=0
for arg in "$@"; do
    if [[ "$arg" == "--dry-run" ]]; then
        DRY_RUN=1
    fi
done
if [[ "$DRY_RUN" -eq 0 ]]; then
    # shellcheck source=./sdk_guard.sh
    source "$SCRIPT_DIR/sdk_guard.sh"
    walker_c1_require_half_sdk "robot_home" || exit 1
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROS_WS/install/setup.bash"
else
    echo "[robot_home] 提示：$ROS_WS/install/setup.bash 不存在，mc_task_msgs 用镜像内预置路径" >&2
fi

exec /usr/bin/python3 "$SCRIPT_DIR/robot_home.py" "$@"
