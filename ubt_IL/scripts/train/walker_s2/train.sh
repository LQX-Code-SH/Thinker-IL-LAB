#!/bin/bash
# Walker S2 模型训练（lerobot-train + config_path 方式）
# 容器内运行：cd /ubt_IL/lerobot 后执行。
#
# 默认用仿真 ACT 配置；切 Pi0.5 / 真机配置用 CONFIG 环境变量，例如：
#   CONFIG=configs/train_config_walker_s2_sim_act_10_2RGB.json bash train.sh
#   CONFIG=$SCRIPT_DIR/configs/train_config_walker_s2_real_pi05_10d_2RGB.json bash train.sh
#
# 可选覆盖（不设则沿用 config 自带值，避免覆盖 Pi0.5 的 steps/lr 等）：
#   OUTPUT_DIR=... STEPS=... BATCH_SIZE=... DEVICE=cuda bash train.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

usage() {
    echo "用法: $0 [lerobot-train 参数 ...]"
    echo ""
    echo "Walker S2 训练（lerobot-train + config_path 方式）"
    echo ""
    echo "环境变量（仅在显式设置时才覆盖 config 文件同名值）："
    echo "  CONFIG            完整训练配置 JSON"
    echo "                     (默认: $SCRIPT_DIR/configs/train_config_walker_s2_sim_act_10_2RGB.json)"
    echo "  DATASET_REPO_ID   数据集名（训练时与转换时保持一致）"
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
    echo ""
    echo "示例："
    echo "  bash train.sh"
    echo "  CONFIG=configs/train_config_walker_s2_real_act_10d_2RGB.json bash train.sh"
    echo "  OUTPUT_DIR=/ubt_IL/model/xxx STEPS=80000 bash train.sh"
    exit 0
}

for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

CONFIG="${CONFIG:-$SCRIPT_DIR/configs/train_config_walker_s2_sim_act_10_2RGB.json}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_HUB_OFFLINE

[[ -f "$CONFIG" ]] || { echo "[train] 错误：配置文件不存在: $CONFIG" >&2; exit 1; }

# 仅在用户显式设置时才作为 CLI 覆盖传入，否则沿用 config 文件里的值。
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
[ ${#OVERRIDES[@]} -gt 0 ] && echo "[train] 覆盖参数: ${OVERRIDES[*]}"

cd /ubt_IL/lerobot

/lerobot/.venv/bin/lerobot-train \
    --config_path="$CONFIG" \
    "${OVERRIDES[@]}" \
    "$@"
