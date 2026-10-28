#!/bin/bash
# 安装天工行者 Pro 的消息包 bodyctrl_msgs（预编译 deb，x86_64）。
#
# bodyctrl_msgs 只有 tienkung_pro 使用，装入 /opt/ros/<distro> 后由 ROS2 自动
# 发现，不需要额外 source，也不影响 walker_s2 / tienkung_dex 的消息包
# （它们各自在 /opt/ubt_sim/ 下的独立工作空间里）。
#
# 使用：
#   bash /ubt_sim/teleoperation/msgs/tienkung_pro/install.sh
#
# arm64 没有对应 deb：用同目录 bodyctrl_msgs_src 源码 colcon 编译（做法见
# ubt_IL/docker/Dockerfile.arm64），或复用同一份源码自行建工作空间。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEB="$SCRIPT_DIR/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb"

if [ ! -f "$DEB" ]; then
    echo "[ERROR] 找不到 deb: $DEB" >&2
    exit 1
fi

if dpkg -s ros-humble-bodyctrl-msgs >/dev/null 2>&1; then
    echo "[INFO] bodyctrl_msgs 已安装，跳过"
    exit 0
fi

if [ "$(dpkg --print-architecture)" != "amd64" ]; then
    echo "[ERROR] 当前架构 $(dpkg --print-architecture) 不能用 amd64 deb，" >&2
    echo "        请改用 bodyctrl_msgs_src 源码编译（参考 ubt_IL/docker/Dockerfile.arm64）" >&2
    exit 1
fi

echo "[INFO] 安装 bodyctrl_msgs (tienkung_pro) ..."
dpkg -i "$DEB"
