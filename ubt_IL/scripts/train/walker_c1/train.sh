#!/bin/bash
# Walker C1 模型训练（lerobot-train + config_path 方式）
# 容器内运行；默认 ACT / 26 维 / 1 路头部 RGB (240x320)。
#
# 环境变量（仅在显式设置时才覆盖 config 文件同名值）：
#   CONFIG            完整训练配置 JSON
#   DATASET_REPO_ID   数据集名（与转换时保持一致）
#   DATASET_ROOT      数据集根目录
#   OUTPUT_DIR        模型输出目录
#   STEPS             训练步数
#   SAVE_FREQ         checkpoint 保存间隔（步）
#   BATCH_SIZE        批量大小
#   SEED              随机种子
#   DEVICE            设备（cuda/cpu）
#   RESUME            续训开关（true/false）
#   WANDB_ENABLE      wandb 开关（true/false）
#   HF_HUB_OFFLINE    离线模式（默认 1）
#
# 示例：
#   bash train.sh
#   BATCH_SIZE=4 STEPS=80000 bash train.sh
#   bash train.sh --steps=2 --save_checkpoint=false   # smoke test
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

usage() {
    echo "用法: $0 [lerobot-train 参数 ...]"
    echo ""
    echo "Walker C1 训练（ACT / 26 维 = 双臂 14 + 双手各 6）"
    echo ""
    echo "环境变量（仅在显式设置时才覆盖 config 文件同名值）："
    echo "  CONFIG            完整训练配置 JSON"
    echo "                     (默认: $SCRIPT_DIR/configs/train_config_walker_c1_sim_act_26d_1RGB.json)"
    echo "  DATASET_REPO_ID   数据集名"
    echo "  DATASET_ROOT      数据集根目录"
    echo "  OUTPUT_DIR        模型输出目录"
    echo "  STEPS             训练步数"
    echo "  SAVE_FREQ         checkpoint 保存间隔（步）"
    echo "  BATCH_SIZE        批量大小"
    echo "  SEED              随机种子"
    echo "  DEVICE            设备（cuda/cpu）"
    echo "  RESUME            续训开关（true/false）"
    echo "  WANDB_ENABLE      wandb 开关（true/false）"
    echo "  HF_HUB_OFFLINE    离线模式（默认 1）"
    echo "  LEROBOT_DIR       lerobot 源码目录（默认 /ubt_IL/lerobot）"
    echo "  TRAIN_BIN         lerobot-train 可执行文件（默认 /lerobot/.venv/bin/lerobot-train）"
    echo ""
    echo "示例："
    echo "  bash train.sh"
    echo "  BATCH_SIZE=4 STEPS=80000 bash train.sh"
    echo "  bash train.sh --steps=2 --save_checkpoint=false"
    exit 0
}

for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

CONFIG="${CONFIG:-$SCRIPT_DIR/configs/train_config_walker_c1_sim_act_26d_1RGB.json}"
LEROBOT_DIR="${LEROBOT_DIR:-/ubt_IL/lerobot}"
TRAIN_BIN="${TRAIN_BIN:-/lerobot/.venv/bin/lerobot-train}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_HUB_OFFLINE

[[ -f "$CONFIG" ]] || { echo "[train] 错误：配置文件不存在: $CONFIG" >&2; exit 1; }
if [[ ! -x "$TRAIN_BIN" ]]; then
    echo "[train] 警告：$TRAIN_BIN 不可执行，回退到 PATH 中的 lerobot-train" >&2
    TRAIN_BIN="lerobot-train"
fi

OVERRIDES=()
[ -n "${DATASET_REPO_ID+x}" ] && OVERRIDES+=(--dataset.repo_id="$DATASET_REPO_ID")
[ -n "${DATASET_ROOT+x}" ] && OVERRIDES+=(--dataset.root="$DATASET_ROOT")
[ -n "${OUTPUT_DIR+x}" ] && OVERRIDES+=(--output_dir="$OUTPUT_DIR")
[ -n "${STEPS+x}" ] && OVERRIDES+=(--steps="$STEPS")
[ -n "${SAVE_FREQ+x}" ] && OVERRIDES+=(--save_freq="$SAVE_FREQ")
[ -n "${BATCH_SIZE+x}" ] && OVERRIDES+=(--batch_size="$BATCH_SIZE")
[ -n "${SEED+x}" ] && OVERRIDES+=(--seed="$SEED")
[ -n "${DEVICE+x}" ] && OVERRIDES+=(--policy.device="$DEVICE")
[ -n "${RESUME+x}" ] && OVERRIDES+=(--resume="$RESUME")
[ -n "${WANDB_ENABLE+x}" ] && OVERRIDES+=(--wandb.enable="$WANDB_ENABLE")

echo "[train] CONFIG = $CONFIG"
echo "[train] LEROBOT_DIR = $LEROBOT_DIR"
[ ${#OVERRIDES[@]} -gt 0 ] && echo "[train] 覆盖参数: ${OVERRIDES[*]}"

cd "$LEROBOT_DIR"

"$TRAIN_BIN" \
    --config_path="$CONFIG" \
    "${OVERRIDES[@]}" \
    "$@"
