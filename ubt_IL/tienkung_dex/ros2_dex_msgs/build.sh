#!/bin/bash
# 编译天工行者 Dex 的真机同款消息包（ros2_bridge_msgs + 因时/强脑灵巧手）。
#
# 源码复制自 ubt_sim/teleoperation/msgs/tienkung_dex/（那边是仿真与真机共用的
# 权威定义）。**不跨仓库引用**：容器里看不到 ubt_sim，两边各自维护，改 .msg 后
# 必须手工同步（与三个机型的插件包同一做法）。
#
# 产物落在独立工作空间 /opt/tienkung_dex_msgs_ws，与其它机型的消息包互不覆盖
# （bodyctrl_msgs 在 /opt/ros/humble，walker/tienkung 的包在各自工作空间）。
#
# 使用（容器内）：
#   bash /ubt_IL/tienkung_dex/ros2_dex_msgs/build.sh
#   source /opt/tienkung_dex_msgs_ws/install/setup.bash
#
# 环境变量：
#   TIENKUNG_DEX_MSGS_OUT   输出工作空间（默认 /opt/tienkung_dex_msgs_ws）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${TIENKUNG_DEX_MSGS_OUT:-/opt/tienkung_dex_msgs_ws}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] 找不到 /opt/ros/humble/setup.bash，需在 ROS2 Humble 容器内运行" >&2
    exit 1
fi

# 编译必须用系统 Python 3.10。基镜像把 /lerobot/.venv/bin 放在 PATH 最前面，
# CMake 的 find_package(Python3) 会挑中 venv 里的 3.12，而 rosidl_adapter 依赖的
# em(python3-empy) 只装在系统 python 上，症状是
#   ModuleNotFoundError: No module named 'em'
# 于是整批消息包 configure 阶段就失败。这里清掉 venv 环境再让 CMake 探测，
# 与 docker/entrypoint.sh 的 build_tienkung_dex_ros2_msgs 同一套做法。
unset VIRTUAL_ENV PYTHONHOME
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
export Python3_EXECUTABLE=/usr/bin/python3
export PYTHON_EXECUTABLE=/usr/bin/python3

# ROS 的 setup.bash 会引用未定义变量，source 时临时关掉 -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

echo "[INFO] TienKung Dex 消息包 -> $OUT_DIR"
colcon --log-base "$OUT_DIR/log" build \
    --base-paths "$SCRIPT_DIR/ros2_bridge_msgs_src/src" \
                 "$SCRIPT_DIR/dex_hand_msgs_src/src" \
    --build-base "$OUT_DIR/build" \
    --install-base "$OUT_DIR/install" \
    --merge-install \
    --packages-up-to ros2_bridge_msgs brainco_hand_msgs inspire_hand_msgs
