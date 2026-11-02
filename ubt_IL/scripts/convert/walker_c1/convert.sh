#!/bin/bash
# Walker C1 仿真 HDF5 -> LeRobot v3.0 数据集转换
# 使用统一 hdf5_mapping 格式 + common/convert_to_lerobot.py
# 宿主机 conda 或容器内均可运行。
#
# 环境变量覆盖默认参数，例如：
#   CONFIG=configs/walker_c1_sim_26d_1RGB.json bash convert.sh
#   SRC_ROOT=/ubt_sim/dataset/walker_c1 bash convert.sh   # 容器内（ubt_sim 只读挂载）
#
# 额外参数透传给 Python 转换器，例如：
#   bash convert.sh --overwrite        # 覆盖已有输出
#   bash convert.sh --save_one true    # 只转第一条 episode
#   bash convert.sh --fps auto         # 自动估 fps
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# 优先使用仓库 vendored 的 lerobot 源码（容器内 editable 装的也是这份），
# 避免宿主机 conda 里旧版 lerobot 缺少 lerobot.configs.video 而报 ImportError。
if [[ -d "$PROJECT_ROOT/lerobot/src" ]]; then
    export PYTHONPATH="$PROJECT_ROOT/lerobot/src${PYTHONPATH:+:$PYTHONPATH}"
fi

usage() {
    echo "用法: $0 [py-options ...]"
    echo ""
    echo "Walker C1 HDF5 -> LeRobot v3.0 数据集转换（26 维：双臂 14 + 双手各 6）"
    echo "使用统一 hdf5_mapping 格式 + common/convert_to_lerobot.py"
    echo ""
    echo "环境变量（及其默认值）："
    echo "  SRC_ROOT    源数据目录 (默认: $PROJECT_ROOT/dataset/walker_c1，缺失回退 /ubt_sim/dataset/walker_c1)"
    echo "  TGT_PATH    输出目标目录 (默认: $PROJECT_ROOT/dataset)"
    echo "  DIM         策略维度：26（双臂+双手）| 13（仅右臂+右手，默认: 26）"
    echo "  CONFIG      JSON 配置文件 (默认: configs/walker_c1_sim_<DIM>d_1RGB.json)"
    echo "  REPO_ID     输出数据集名称 (默认: walker_c1_sim_<DIM>d_1RGB)"
    echo "  FPS         帧率 (默认: 15)"
    echo "  ROBOT_TYPE  机器人类型 (默认: walker_c1)"
    echo "  TASK_NAME   任务名称 (默认: walker_c1_sim_pick_place)"
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
    echo "  SYNC_ENCODING 1=逐集同步编码（默认 1；单次仅删除本集临时图片，避免大批量删除）"
    echo "  TAIL_TRIM   裁掉每集末尾 N 帧（默认: 10）"
    echo "              C1 采集末段包含「release → retreat → back to ready」快速复位动作，"
    echo "              不裁会让策略学到任务完成后猛甩手臂（置 0 可关闭）"
    echo ""
    echo "常用选项（透传）："
    echo "  --overwrite        覆盖已有输出数据集"
    echo "  --save_one true    只转换第一条 episode"
    echo "  --help, -h         显示此帮助信息"
    echo ""
    echo "示例："
    echo "  bash convert.sh"
    echo "  SRC_ROOT=/ubt_sim/dataset/walker_c1 bash convert.sh"
    echo "  bash convert.sh --overwrite --save_one true"
    exit 0
}

for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

# === 配置（可环境变量覆盖）===
SRC_ROOT_EXPLICIT="${SRC_ROOT:+1}"
CONFIG_EXPLICIT="${CONFIG:+1}"
SRC_ROOT="${SRC_ROOT:-$PROJECT_ROOT/dataset/walker_c1}"
TGT_PATH="${TGT_PATH:-$PROJECT_ROOT/dataset}"
DIM="${DIM:-26}"
case "$DIM" in
    26) DEFAULT_CONFIG="$SCRIPT_DIR/configs/walker_c1_sim_26d_1RGB.json"
        DEFAULT_REPO="walker_c1_sim_26d_1RGB" ;;
    13) DEFAULT_CONFIG="$SCRIPT_DIR/configs/walker_c1_sim_13d_1RGB.json"
        DEFAULT_REPO="walker_c1_sim_13d_1RGB" ;;
    *)  echo "[convert] 错误：DIM 只支持 26 或 13（当前: $DIM）" >&2; exit 1 ;;
esac
CONFIG="${CONFIG:-$DEFAULT_CONFIG}"
REPO_ID="${REPO_ID:-$DEFAULT_REPO}"
FPS="${FPS:-15}"
ROBOT_TYPE="${ROBOT_TYPE:-walker_c1}"
TASK_NAME="${TASK_NAME:-walker_c1_sim_pick_place}"
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
# C1 采集末段是快速复位动作（非任务动作），默认裁掉 10 帧（15 Hz 下约 0.67 s）
TAIL_TRIM="${TAIL_TRIM:-10}"
# 逐集同步编码：避免多集编码排队后一次性删除 >500 个临时图片文件
SYNC_ENCODING="${SYNC_ENCODING:-1}"
PYTHON_SCRIPT="$SCRIPT_DIR/../common/convert_to_lerobot.py"

# C1 数据由 ubt_sim 采集，落在 ubt_sim/dataset/walker_c1；
# 没搬到 ubt_IL/dataset 时自动回退过去，省掉一次手工拷贝。
if [[ ! -d "$SRC_ROOT" && -z "$SRC_ROOT_EXPLICIT" ]]; then
    UBT_SIM_SRC="$(cd "$PROJECT_ROOT/.." && pwd)/ubt_sim/dataset/walker_c1"
    if [[ -d "$UBT_SIM_SRC" ]]; then
        echo "[convert] 注意：$SRC_ROOT 不存在，回退到 $UBT_SIM_SRC"
        SRC_ROOT="$UBT_SIM_SRC"
    fi
fi

# 采集侧落盘：joint_observations/joint_position 31 维（臂14+头2+腰3+腿12）、
# hand_*_position 各 6 维。列数不符说明数据来自别的机型/旧版采集，直接拦下。
# CONFIG= 显式给出时不做探测。
if [[ -z "$CONFIG_EXPLICIT" ]]; then
    DIMS="$(python - "$SRC_ROOT" "$HDF5_REL_PATH" <<'PY' 2>/dev/null || true
import os
import sys

import h5py

root, rel = sys.argv[1], sys.argv[2]
for name in sorted(os.listdir(root)):
    path = os.path.join(root, name, rel)
    if os.path.isfile(path):
        with h5py.File(path, "r") as f:
            print(f["joint_observations/joint_position"].shape[1],
                  f["hand_observations/hand_right_position"].shape[1])
        break
PY
)"
    case "$DIMS" in
        "31 6") echo "[convert] 检测到 31 body / 6 手部通道，使用配置 $(basename "$CONFIG")" ;;
        "") ;;  # 探测失败（源目录为空/没有 h5py）：留给下面的校验报错
        *)
            echo "[convert] 错误：检测到 body/手部维度 = '${DIMS}'，本配置要求 '31 6'" >&2
            echo "        请确认数据来自 ubt_sim walker_c1 采集；确认无误时用 CONFIG=<配置> 显式绕过此检查" >&2
            exit 1
            ;;
    esac
fi

# === 校验 ===
[[ -f "$CONFIG" ]] || { echo "[convert] 错误：配置文件不存在: $CONFIG" >&2; exit 1; }
[[ -f "$PYTHON_SCRIPT" ]] || { echo "[convert] 错误：转换脚本不存在: $PYTHON_SCRIPT" >&2; exit 1; }
[[ -e "$SRC_ROOT" ]] || { echo "[convert] 错误：源数据不存在: $SRC_ROOT" >&2; exit 1; }

echo "[convert] ========================================"
echo "[convert] Walker C1 -> LeRobot v3.0"
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
echo "[convert] TAIL_TRIM = $TAIL_TRIM"
echo "[convert] DIM       = $DIM"
echo "[convert] SYNC_ENCODING = $SYNC_ENCODING"
echo "[convert] ========================================"

# 布尔 flag：非空且非 0/false 时透传
TRIM_FLAG=""
{ [[ -n "$TRIM_STATIONARY" ]] && [[ "$TRIM_STATIONARY" != "0" ]] && [[ "$TRIM_STATIONARY" != "false" ]]; } && TRIM_FLAG="--trim-stationary"
DIAG_FLAG=""
{ [[ -n "$STATIONARY_DIAGNOSE" ]] && [[ "$STATIONARY_DIAGNOSE" != "0" ]] && [[ "$STATIONARY_DIAGNOSE" != "false" ]]; } && DIAG_FLAG="--stationary-diagnose"
SYNC_FLAG=""
{ [[ "$SYNC_ENCODING" != "0" ]] && [[ "$SYNC_ENCODING" != "false" ]]; } && SYNC_FLAG="--sync-encoding"

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
  --tail-trim "$TAIL_TRIM" \
  $SYNC_FLAG \
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
