#!/bin/bash
# TienKung Dex 真机预备位（host 版）。
# 层 B：系统 python3 + ROS2 Jazzy 直接跑 scripts/deploy/tienkung_dex/robot_ready.py。
#
# 用法：
#   bash robot_ready_host.sh --dry-run          # 只打印四段目标，不连 ROS
#   bash robot_ready_host.sh --real             # 真机实发（最终目标 READY_POSE）
#   bash robot_ready_host.sh --real --hands right
#
# 注意：
#   * 不传 --real 时最终目标是仿真用的 {READY_POSE + READY_POSE_TARGET}，
#     真机会站成低 13 cm 的姿态 —— 真机请始终带 --real。
#   * 与 Bridge2 互斥（都写 /arm/cmd）；rollout 会自己拉起 Bridge2。
#   * 会下发 left/right 双臂 + 腰 + 头（不发腿）。人必须站在急停可达处。
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

dex_source_ros
dex_check_msgs

VARIANT="${VARIANT:-brainco}"     # 实物强脑；因时改 inspire

if ! printf '%s\n' "$@" | grep -q -- "--real"; then
    echo "[robot_ready] WARN: 未带 --real，将使用仿真目标（含重力前馈补偿，真机会低 13 cm）"
fi

# --variant 必须与实物一致：默认值（inspire）会把指位发到不存在的话题，
# 手臂照样动、但手型不变，且只在报告里给一行 WARN，很容易漏看。
EXTRA_ARGS=()
if ! printf '%s\n' "$@" | grep -q -- "--variant"; then
    EXTRA_ARGS+=(--variant "$VARIANT")
fi

exec "$SYSPY" "$PROJECT_ROOT/scripts/deploy/tienkung_dex/robot_ready.py" \
    --constants "$DEX_DIR/lerobot_robot_tienkung_dex/lerobot_robot_tienkung_dex/constants.py" \
    "${EXTRA_ARGS[@]}" "$@"
