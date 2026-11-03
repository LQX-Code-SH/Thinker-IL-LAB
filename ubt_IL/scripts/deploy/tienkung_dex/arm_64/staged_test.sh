#!/bin/bash
# 真机分步微小运动测试（头 / 手 / 臂），用于完整预备位之前的逐组验收。
#
#   bash staged_test.sh --step head
#   bash staged_test.sh --step hand
#   bash staged_test.sh --step arm
#   bash staged_test.sh --step all
#
# 每一步都是「基线 → 偏出 → 保持 → 回位」，幅度默认 0.05 rad（约 2.9°）/
# 手部 80 码（8%），并从反馈核对实测幅度与回位残差。
# 不写 /leg/cmd、不动腰。运行前确认 Bridge2 没在跑（两者都写 /arm/cmd）。
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

dex_source_ros
dex_check_msgs

if pgrep -f "ros2_deploy_bridge.py --config" >/dev/null 2>&1; then
    echo "[staged] WARN: 检测到 Bridge2 在跑，两者都写 /arm/cmd，先停掉它：" >&2
    echo "        pkill -f 'ros2_deploy_bridge.py --config'" >&2
    exit 1
fi

exec "$SYSPY" "$(dirname "${BASH_SOURCE[0]}")/staged_test.py" "$@"
