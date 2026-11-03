#!/bin/bash
# TienKung Dex：已有 LeRobot 数据集 -> 修复 + 裁剪（无需源 HDF5）
#
# 与 convert.sh 同一套用法：CONFIG 描述输出特征表 + lerobot_mapping，
# 本脚本只是把配置和路径透传给 common/repair_trim_lerobot.py。
#
# 环境变量（及其默认值）：
#   CONFIG      修复配置 (默认: configs/tienkung_dex_real_pick_r13_1RGB.json)
#   SRC_ROOT    源数据集根目录 (默认: $PROJECT_ROOT/dataset/tienkung_dex_real_pick)
#   REPO_ID     源数据集 repo_id (默认: tienkung_dex_real_pick)
#   TGT_PATH    输出父目录 (默认: $PROJECT_ROOT/dataset)
#   NEW_REPO_ID 输出数据集名称 (默认: ${REPO_ID}_fixed)
#   EPISODES    要处理的 episode (默认: all；可写 0,2,5 或 0-4)
#   VCODEC      视频编码器 (默认: h264)
#
# 常用选项（透传）：
#   --dry-run     只打印裁剪/修复计划
#   --overwrite   覆盖已有输出
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

usage() {
    echo "用法: $0 [py-options ...]"
    echo ""
    echo "已有 LeRobot 数据集 -> 修复 + 裁剪（配置驱动，无需 HDF5）"
    echo ""
    echo "环境变量（及其默认值）："
    echo "  CONFIG      $(basename "${CONFIG:-$SCRIPT_DIR/configs/tienkung_dex_real_pick_r13_1RGB.json}")"
    echo "  SRC_ROOT    ${SRC_ROOT:-$PROJECT_ROOT/dataset/tienkung_dex_real_pick}"
    echo "  REPO_ID     ${REPO_ID:-tienkung_dex_real_pick}"
    echo "  TGT_PATH    ${TGT_PATH:-$PROJECT_ROOT/dataset}"
    echo "  NEW_REPO_ID \${REPO_ID}_fixed"
    echo "  EPISODES    all"
    echo "  VCODEC      h264"
    echo ""
    echo "示例："
    echo "  bash repair.sh --dry-run                       # 先看计划"
    echo "  bash repair.sh --overwrite                     # 按默认配置裁成右手 13 维 + 单 RGB"
    echo "  CONFIG=configs/xxx.json EPISODES=0 bash repair.sh --dry-run"
    exit 0
}

for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

CONFIG="${CONFIG:-$SCRIPT_DIR/configs/tienkung_dex_real_pick_r13_1RGB.json}"
SRC_ROOT="${SRC_ROOT:-$PROJECT_ROOT/dataset/tienkung_dex_real_pick}"
REPO_ID="${REPO_ID:-tienkung_dex_real_pick}"
TGT_PATH="${TGT_PATH:-$PROJECT_ROOT/dataset}"
NEW_REPO_ID="${NEW_REPO_ID:-${REPO_ID}_fixed}"
EPISODES="${EPISODES:-all}"
VCODEC="${VCODEC:-h264}"
PYTHON_SCRIPT="$SCRIPT_DIR/../common/repair_trim_lerobot.py"

[[ -f "$CONFIG" ]] || { echo "[repair] 错误：配置文件不存在: $CONFIG" >&2; exit 1; }
[[ -d "$SRC_ROOT" ]] || { echo "[repair] 错误：源数据集不存在: $SRC_ROOT" >&2; exit 1; }

echo "[repair] ========================================"
echo "[repair] LeRobot -> LeRobot 修复 + 裁剪"
echo "[repair] CONFIG      = $CONFIG"
echo "[repair] SRC_ROOT    = $SRC_ROOT"
echo "[repair] REPO_ID     = $REPO_ID"
echo "[repair] TGT_PATH    = $TGT_PATH"
echo "[repair] NEW_REPO_ID = $NEW_REPO_ID"
echo "[repair] EPISODES    = $EPISODES"
echo "[repair] VCODEC      = $VCODEC"
echo "[repair] ========================================"

python "$PYTHON_SCRIPT" \
  --config "$CONFIG" \
  --repo_id "$REPO_ID" \
  --root "$SRC_ROOT" \
  --new_repo_id "$NEW_REPO_ID" \
  --new_root "$TGT_PATH" \
  --episodes "$EPISODES" \
  --vcodec "$VCODEC" \
  "$@"
