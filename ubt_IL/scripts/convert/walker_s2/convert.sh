#!/bin/bash
# Walker S2 真机 HDF5 -> LeRobot v3.0 数据集转换
# 使用统一 hdf5_mapping 格式，宿主机 conda 或容器内均可运行。
#
# 环境变量覆盖默认参数，例如：
#   CONFIG=configs/walker_s2_real_10d_1RGBD.json bash convert.sh
#   SRC_ROOT=/ubt_IL/dataset/walker-s2-real-data/<single_episode> bash convert.sh
#
# 额外参数透传给 Python 转换器，例如：
#   bash convert.sh --overwrite        # 覆盖已有输出
#   bash convert.sh --save_one true    # 只转第一条 episode
#   bash convert.sh --fps auto         # 自动估 fps
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

usage() {
    echo "用法: $0 [py-options ...]"
    echo ""
    echo "Walker S2 真机 HDF5 -> LeRobot v3.0 数据集转换"
    echo "使用统一 hdf5_mapping 格式 + common/convert_to_lerobot.py"
    echo ""
    echo "环境变量（及其默认值）："
    echo "  SRC_ROOT    源数据目录 (默认: /ubt_IL/dataset/walker-s2-real-data)"
    echo "  TGT_PATH    输出目标目录 (默认: /ubt_IL/dataset)"
    echo "  CONFIG      JSON 配置文件 (默认: configs/walker_s2_real_19d_1RGBD.json)"
    echo "  REPO_ID     输出数据集名称 (默认: Walker_S2_real_19_1RGBD)"
    echo "  FPS         帧率 (默认: 13)"
    echo "  ROBOT_TYPE  机器人类型 (默认: walker_s2)"
    echo "  TASK_NAME   任务名称 (默认: walker_s2_real)"
    echo "  VCODEC      视频编码器 (默认: h264)"
    echo "  HDF5_REL_PATH HDF5 相对于 episode 目录的路径 (默认: hdf5/metadata_aligned.hdf5)"
    echo "  RESAMPLE_FPS 目标帧率，启用重采样 (默认: 空=不启用)"
    echo "  TIMESTAMP_HDF5_KEY 时间戳 HDF5 路径 (默认: 空=自动探测)"
    echo "  LABEL_ROOT    label.json 父目录 (默认: 空=不按标注分段)"
    echo ""
    echo "常用选项（透传）："
    echo "  --overwrite        覆盖已有输出数据集"
    echo "  --save_one true    只转换第一条 episode"
    echo "  --help, -h         显示此帮助信息"
    echo ""
    echo "示例："
    echo "  bash convert.sh"
    echo "  CONFIG=configs/walker_s2_real_10d_1RGBD.json bash convert.sh"
    echo "  bash convert.sh --overwrite --save_one true"
    echo "  RESAMPLE_FPS=30 bash convert.sh --overwrite"
    exit 0
}

for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

# === 配置（可环境变量覆盖）===
SRC_ROOT="${SRC_ROOT:-/ubt_IL/dataset/walker-s2-real-data}"
TGT_PATH="${TGT_PATH:-/ubt_IL/dataset}"
CONFIG="${CONFIG:-$SCRIPT_DIR/configs/walker_s2_real_19d_1RGBD.json}"
REPO_ID="${REPO_ID:-Walker_S2_real_19_1RGBD}"
FPS="${FPS:-13}"
ROBOT_TYPE="${ROBOT_TYPE:-walker_s2}"
TASK_NAME="${TASK_NAME:-walker_s2_real}"
VCODEC="${VCODEC:-h264}"
HDF5_REL_PATH="${HDF5_REL_PATH:-hdf5/metadata_aligned.hdf5}"
RESAMPLE_FPS="${RESAMPLE_FPS:-}"
TIMESTAMP_HDF5_KEY="${TIMESTAMP_HDF5_KEY:-}"
LABEL_ROOT="${LABEL_ROOT:-}"
PYTHON_SCRIPT="$SCRIPT_DIR/../common/convert_to_lerobot.py"

# === 校验 ===
[[ -f "$CONFIG" ]] || { echo "[convert] 错误：配置文件不存在: $CONFIG" >&2; exit 1; }
[[ -f "$PYTHON_SCRIPT" ]] || { echo "[convert] 错误：转换脚本不存在: $PYTHON_SCRIPT" >&2; exit 1; }
[[ -e "$SRC_ROOT" ]] || { echo "[convert] 错误：源数据不存在: $SRC_ROOT" >&2; exit 1; }

echo "[convert] ========================================"
echo "[convert] Walker S2 Real → LeRobot v3.0"
echo "[convert] (统一 hdf5_mapping + common converter)"
echo "[convert] ========================================"
echo "[convert] SRC_ROOT  = $SRC_ROOT"
echo "[convert] TGT_PATH  = $TGT_PATH"
echo "[convert] CONFIG    = $CONFIG"
echo "[convert] REPO_ID   = $REPO_ID"
echo "[convert] FPS       = $FPS"
echo "[convert] TASK_NAME = $TASK_NAME"
echo "[convert] VCODEC    = $VCODEC"
echo "[convert] ========================================"

python "$PYTHON_SCRIPT" \
  --config "$CONFIG" \
  --repo_id "$REPO_ID" \
  --src_root "$SRC_ROOT" \
  --tgt_path "$TGT_PATH" \
  --fps "$FPS" \
  --robot_type "$ROBOT_TYPE" \
  --task_name "$TASK_NAME" \
  --vcodec "$VCODEC" \
  --hdf5_rel_path "$HDF5_REL_PATH" \
  ${RESAMPLE_FPS:+--resample-fps "$RESAMPLE_FPS"} \
  ${TIMESTAMP_HDF5_KEY:+--timestamp-hdf5-key "$TIMESTAMP_HDF5_KEY"} \
  ${LABEL_ROOT:+--label-root "$LABEL_ROOT"} \
  "$@"
