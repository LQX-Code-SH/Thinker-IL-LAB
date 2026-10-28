#!/bin/bash
# TienKung Dex 仿真 HDF5 -> LeRobot v3.0 数据集转换
# 使用统一 hdf5_mapping 格式 + common/convert_to_lerobot.py
# 宿主机 conda 或容器内均可运行。
#
# 环境变量覆盖默认参数，例如：
#   CONFIG=configs/tienkung_dex_20d_1RGB.json bash convert.sh
#   SRC_ROOT=/ubt_sim/dataset/tienkung_dex bash convert.sh   # 容器内（ubt_sim 只读挂载）
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
    echo "TienKung Dex HDF5 -> LeRobot v3.0 数据集转换"
    echo "使用统一 hdf5_mapping 格式 + common/convert_to_lerobot.py"
    echo ""
    echo "环境变量（及其默认值）："
    echo "  SRC_ROOT    源数据目录 (默认: $PROJECT_ROOT/dataset/tienkung_dex)"
    echo "  TGT_PATH    输出目标目录 (默认: $PROJECT_ROOT/dataset)"
    echo "  CONFIG      JSON 配置文件 (默认: configs/tienkung_dex_20d_1RGB_inspire.json)"
    echo "  REPO_ID     输出数据集名称 (默认: tienkung_dex_sim_pick_apple)"
    echo "  FPS         帧率 (默认: 15)"
    echo "  ROBOT_TYPE  机器人类型 (默认: tienkung_dex)"
    echo "  TASK_NAME   任务名称 (默认: tienkung_dex_sim_pick_apple)"
    echo "  VCODEC      视频编码器 (默认: h264)"
    echo "  HDF5_REL_PATH HDF5 相对于 episode 目录的路径 (默认: trajectory.hdf5)"
    echo "  RESAMPLE_FPS 目标帧率，启用重采样 (默认: 空=不启用)"
    echo "  TIMESTAMP_HDF5_KEY 时间戳 HDF5 路径 (默认: 空=自动探测)"
    echo "  LABEL_ROOT    label.json 父目录 (默认: 空=不按标注分段)"
    echo "  TRIM_STATIONARY  1=开启静止帧 cap (默认: 空=关闭)"
    echo "  STATIONARY_DIAGNOSE  1=只统计静止分布不写盘 (默认: 空=关闭)"
    echo "  STATIONARY_KEY  判静止用的 compose 字段 (默认: action)"
    echo "  STATIONARY_WINDOW  位移窗口帧数 (默认: 空=自动 0.3*fps)"
    echo "  STATIONARY_THRESH  归一化静止阈值 (默认: 0.03)"
    echo "  STATIONARY_CAP  每段静止游程上限帧数 (默认: 8)"
    echo "  STATIONARY_MIN_RUN  最小静止游程 (默认: 3)"
    echo "  STATIONARY_RANGE_EPS  恒定维判定阈值 (默认: 1e-3)"
    echo ""
    echo "常用选项（透传）："
    echo "  --overwrite        覆盖已有输出数据集"
    echo "  --save_one true    只转换第一条 episode"
    echo "  --help, -h         显示此帮助信息"
    echo ""
    echo "示例："
    echo "  bash convert.sh"
    echo "  # 容器内直接读 ubt_sim 的采集产物（跑完 save_data.sh --batch 就在那里，"
    echo "  # ubt_sim 由 docker/run.sh 只读挂载到 /ubt_sim）；不给 SRC_ROOT 也会自动回退过去"
    echo "  SRC_ROOT=/ubt_sim/dataset/tienkung_dex bash convert.sh"
    echo "  bash convert.sh --overwrite --save_one true"
    echo "  STATIONARY_DIAGNOSE=1 bash convert.sh   # 先看静止分布"
    echo "  TRIM_STATIONARY=1 bash convert.sh --overwrite"
    exit 0
}

for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

# === 配置（可环境变量覆盖）===
SRC_ROOT_EXPLICIT="${SRC_ROOT:+1}"
CONFIG_EXPLICIT="${CONFIG:+1}"
SRC_ROOT="${SRC_ROOT:-$PROJECT_ROOT/dataset/tienkung_dex}"
TGT_PATH="${TGT_PATH:-$PROJECT_ROOT/dataset}"
CONFIG="${CONFIG:-$SCRIPT_DIR/configs/tienkung_dex_20d_1RGB_inspire.json}"
REPO_ID="${REPO_ID:-tienkung_dex_sim_pick_apple}"
FPS="${FPS:-15}"
ROBOT_TYPE="${ROBOT_TYPE:-tienkung_dex}"
TASK_NAME="${TASK_NAME:-tienkung_dex_sim_pick_apple}"
VCODEC="${VCODEC:-h264}"
HDF5_REL_PATH="${HDF5_REL_PATH:-trajectory.hdf5}"
RESAMPLE_FPS="${RESAMPLE_FPS:-}"
TIMESTAMP_HDF5_KEY="${TIMESTAMP_HDF5_KEY:-}"
LABEL_ROOT="${LABEL_ROOT:-}"
TRIM_STATIONARY="${TRIM_STATIONARY:-}"
STATIONARY_DIAGNOSE="${STATIONARY_DIAGNOSE:-}"
STATIONARY_KEY="${STATIONARY_KEY:-}"
STATIONARY_WINDOW="${STATIONARY_WINDOW:-}"
STATIONARY_THRESH="${STATIONARY_THRESH:-}"
STATIONARY_CAP="${STATIONARY_CAP:-}"
STATIONARY_MIN_RUN="${STATIONARY_MIN_RUN:-}"
STATIONARY_RANGE_EPS="${STATIONARY_RANGE_EPS:-}"
PYTHON_SCRIPT="$SCRIPT_DIR/../common/convert_to_lerobot.py"

# Dex 的数据默认由 ubt_sim 采集，落在 ubt_sim/dataset/tienkung_dex；
# 没搬到 ubt_IL/dataset 时自动回退过去，省掉一次手工拷贝。
if [[ ! -d "$SRC_ROOT" && -z "$SRC_ROOT_EXPLICIT" ]]; then
    UBT_SIM_SRC="$(cd "$PROJECT_ROOT/.." && pwd)/ubt_sim/dataset/tienkung_dex"
    if [[ -d "$UBT_SIM_SRC" ]]; then
        echo "[convert] 注意：$SRC_ROOT 不存在，回退到 $UBT_SIM_SRC"
        SRC_ROOT="$UBT_SIM_SRC"
    fi
fi

# 采集侧只落盘真机通道实际驱动的 13 个手部关节（URDF 里 5 个 mimic *_dip 不再记录，
# 与 ROS 侧 SetAngle/GetAngleAct 的 13 通道一一对应），配置里手部就是 13 维原样取用。
# 列数不是 13 说明数据来自旧版采集（18 列，含 5 个恒 0 的 dip），此时重排后的 25 维会
# 对不上配置的 shape=[20]，直接拦下比让转换器写出坏数据集好。
# CONFIG= 显式给出时不做探测。
if [[ -z "$CONFIG_EXPLICIT" ]]; then
    HAND_DIM="$(python - "$SRC_ROOT" "$HDF5_REL_PATH" <<'PY' 2>/dev/null || true
import os
import sys

import h5py

root, rel = sys.argv[1], sys.argv[2]
for name in sorted(os.listdir(root)):
    path = os.path.join(root, name, rel)
    if os.path.isfile(path):
        with h5py.File(path, "r") as f:
            print(f["hand_observations/hand_right_position"].shape[1])
        break
PY
)"
    case "$HAND_DIM" in
        13) echo "[convert] 检测到右手 13 维，使用配置文件 $(basename "$CONFIG")" ;;
        "") ;;  # 探测失败（源目录为空/没有 h5py）：留给下面的校验报错
        *)
            echo "[convert] 错误：右手是 ${HAND_DIM} 维，本配置要求 13 维" >&2
            echo "        旧版采集（18 列，含 5 个恒 0 的 *_dip）需要另写配置用 indices 剔除 dip 列，" >&2
            echo "        或重新采集；确认无误时用 CONFIG=<配置> 显式绕过此检查" >&2
            exit 1
            ;;
    esac
fi

# === 校验 ===
[[ -f "$CONFIG" ]] || { echo "[convert] 错误：配置文件不存在: $CONFIG" >&2; exit 1; }
[[ -f "$PYTHON_SCRIPT" ]] || { echo "[convert] 错误：转换脚本不存在: $PYTHON_SCRIPT" >&2; exit 1; }
[[ -e "$SRC_ROOT" ]] || { echo "[convert] 错误：源数据不存在: $SRC_ROOT" >&2; exit 1; }

echo "[convert] ========================================"
echo "[convert] TienKung Dex → LeRobot v3.0"
echo "[convert] (统一 hdf5_mapping + common converter)"
echo "[convert] ========================================"
echo "[convert] SRC_ROOT  = $SRC_ROOT"
echo "[convert] TGT_PATH  = $TGT_PATH"
echo "[convert] CONFIG    = $CONFIG"
echo "[convert] REPO_ID   = $REPO_ID"
echo "[convert] FPS       = $FPS"
echo "[convert] TASK_NAME = $TASK_NAME"
echo "[convert] VCODEC    = $VCODEC"
echo "[convert] HDF5_REL  = $HDF5_REL_PATH"
echo "[convert] ========================================"

# 布尔 flag：非空且非 0/false 时透传
TRIM_FLAG=""
{ [[ -n "$TRIM_STATIONARY" ]] && [[ "$TRIM_STATIONARY" != "0" ]] && [[ "$TRIM_STATIONARY" != "false" ]]; } && TRIM_FLAG="--trim-stationary"
DIAG_FLAG=""
{ [[ -n "$STATIONARY_DIAGNOSE" ]] && [[ "$STATIONARY_DIAGNOSE" != "0" ]] && [[ "$STATIONARY_DIAGNOSE" != "false" ]]; } && DIAG_FLAG="--stationary-diagnose"

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
  $TRIM_FLAG $DIAG_FLAG \
  ${STATIONARY_KEY:+--stationary-key "$STATIONARY_KEY"} \
  ${STATIONARY_WINDOW:+--stationary-window "$STATIONARY_WINDOW"} \
  ${STATIONARY_THRESH:+--stationary-thresh "$STATIONARY_THRESH"} \
  ${STATIONARY_CAP:+--stationary-cap "$STATIONARY_CAP"} \
  ${STATIONARY_MIN_RUN:+--stationary-min-run "$STATIONARY_MIN_RUN"} \
  ${STATIONARY_RANGE_EPS:+--stationary-range-eps "$STATIONARY_RANGE_EPS"} \
  "$@"
