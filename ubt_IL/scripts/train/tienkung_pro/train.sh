#!/bin/bash
# 天工机器人 ACT 模型训练脚本
# 通过 --config_path 加载训练配置作为完整训练配置
# （含 input_features、网络结构、归一化、优化器等）；
# 仅在显式设置环境变量时才用 CLI 覆盖 config 文件中的对应字段，
# 未设置的沿用 config 自带值（draccus 合并时 CLI 优先）。
# 在 lerobot-tienkung 容器内运行
#
# 用法示例：
#   仿真 13D（默认配置即 train_config_tienkung_pro_sim_pick_place.json）：
#     bash train.sh
#   真机 26D：
#     CONFIG_PATH=.../configs/train_config_tienkung_pro_real_pick_place.json bash train.sh
#   覆盖常用参数：
#     OUTPUT_DIR=/ubt_IL/model/tienkung_sim_pick_place_act_v2 \
#     STEPS=80000 SAVE_FREQ=20000 BATCH_SIZE=16 bash train.sh
#   续训（CONFIG_PATH 指向 checkpoint 内的 train_config.json，
#   lerobot 会据其相对位置定位 model.safetensors 与 training_state）：
#     CONFIG_PATH=/ubt_IL/model/xxx/checkpoints/last/pretrained_model/train_config.json \
#     RESUME=true bash train.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

usage() {
    echo "用法: $0 [lerobot-train 参数 ...]"
    echo ""
    echo "天工 Pro ACT 训练（lerobot-train + config_path 方式）"
    echo ""
    echo "环境变量（仅在显式设置时才覆盖 config 文件同名值）："
    echo "  CONFIG_PATH      完整训练配置 JSON"
    echo "                   (默认: $SCRIPT_DIR/configs/train_config_tienkung_pro_sim_pick_place.json)"
    echo "  DATASET_REPO_ID  数据集名（训练时与转换时保持一致）"
    echo "  DATASET_ROOT     数据集根目录"
    echo "  OUTPUT_DIR       模型输出目录"
    echo "  STEPS            训练步数"
    echo "  SAVE_FREQ        checkpoint 保存间隔（步）"
    echo "  BATCH_SIZE       批量大小"
    echo "  SEED             随机种子"
    echo "  DEVICE           设备（cuda/cpu）"
    echo "  RESUME           续训开关（true/false）"
    echo "  WANDB_ENABLE     wandb 开关（true/false）"
    echo "  HF_HUB_OFFLINE   离线模式（默认 1）"
    echo ""
    echo "示例："
    echo "  bash train.sh"
    echo "  OUTPUT_DIR=/ubt_IL/model/xxx STEPS=80000 bash train.sh"
    echo "  CONFIG_PATH=.../checkpoints/last/pretrained_model/train_config.json RESUME=true bash train.sh"
    exit 0
}

for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

# === 配置 ===
CONFIG_PATH="${CONFIG_PATH:-$SCRIPT_DIR/configs/train_config_tienkung_pro_sim_pick_place.json}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_HUB_OFFLINE

[[ -f "$CONFIG_PATH" ]] || { echo "[train] 错误：配置文件不存在: $CONFIG_PATH" >&2; exit 1; }

# 仅在用户显式设置时才作为 CLI 覆盖传入，否则沿用 config 文件里的值。
# 注意：input_features 固定为 RGB 头部图像 + 状态（不含 head_depth），
# 以便与 rollout 部署（仅提供 head RGB 相机）保持一致。
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

echo "[train] CONFIG_PATH = $CONFIG_PATH"
[ ${#OVERRIDES[@]} -gt 0 ] && echo "[train] 覆盖参数: ${OVERRIDES[*]}"

cd /ubt_IL/lerobot

/lerobot/.venv/bin/lerobot-train \
    --config_path="$CONFIG_PATH" \
    "${OVERRIDES[@]}" \
    "$@"
