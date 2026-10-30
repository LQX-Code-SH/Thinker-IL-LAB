#!/bin/bash
# 对已运行的 C1 仿真执行并录制一条在线 IK 抓放轨迹。
# 成功的轨迹写到 dataset/walker_c1/<epoch秒>/trajectory.hdf5。
# 本脚本不启动/停止/重启仿真器；只跑任务时传 --no-record。
#
# 用法：
#   bash scripts/run_c1_pick_place_once.sh
#   bash scripts/run_c1_pick_place_once.sh --no-record

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

set +u
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
    if [ -f /opt/ros/humble/lib/librmw_fastrtps_cpp.so ]; then
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    else
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
fi

exec /usr/bin/python3 \
    "$PROJECT_DIR/teleoperation/control/walker_c1/pick_place_controller.py" \
    --episodes 1 \
    --max-grasp-attempts 1 \
    --save \
    "$@"
