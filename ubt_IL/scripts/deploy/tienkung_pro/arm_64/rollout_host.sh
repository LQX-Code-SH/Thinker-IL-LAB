#!/bin/bash
# 天工机器人 host 版 rollout（去 Docker，conda env_vla + Python 3.12）
# 在宿主机直接跑 LeRobot 推理部署，不进容器。决策与依据见同目录 README.md。
#
# 用法:
#   bash rollout_host.sh                      # 用默认模型/配置
#   POLICY_PATH=... TASK=... bash rollout_host.sh
#   DISPLAY_CAM=false bash rollout_host.sh    # SSH 无 X 时关相机显示
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"   # -> .../ubt_IL
CONDA_BASE="${CONDA_BASE:-$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")}"
ENV_NAME="${ENV_NAME:-env_vla}"

# === 配置（均可环境变量覆盖） ===
# 已训练的 ACT checkpoint（last 软链 -> 100000）
POLICY_PATH="${POLICY_PATH:-$PROJECT_ROOT/model/tienkung_pick_up_act/checkpoints/last/pretrained_model}"
STRATEGY="${STRATEGY:-base}"
FPS="${FPS:-30}"
DURATION="${DURATION:-60}"
TASK="${TASK:-sim_pick_place}"
# 关节 DOF 配置名（取自 constants.JOINT_INDEX_ENUMS）：tienkung_26=全26；tienkung_13=右臂7+右手6。
# 须与 POLICY_PATH 指向模型的训练 DOF/关节顺序一致。
JOINT_CONFIG="${JOINT_CONFIG:-tienkung_26}"
ZMQ_HOST="${ZMQ_HOST:-127.0.0.1}"        # image_server 地址：本机/仿真=127.0.0.1；真机相机在机器人上则改其 IP
DISPLAY_CAM="${DISPLAY_CAM:-true}"        # SSH 无 DISPLAY 时设 false
BRIDGE_SCRIPT="$PROJECT_ROOT/tienkung/ros2_deploy_bridge.py"

# === 激活 env_vla (3.12) ===
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"

echo "[rollout] python      : $(which python)"
echo "[rollout] PROJECT_ROOT: $PROJECT_ROOT"
echo "[rollout] POLICY_PATH : $POLICY_PATH"
echo "[rollout] bridge      : $BRIDGE_SCRIPT"
echo "[rollout] ZMQ_HOST    : $ZMQ_HOST (image_server)"

cd "$PROJECT_ROOT/lerobot"

# 注意：--robot.bridge_script 覆盖插件默认的 /ubt_IL/... 容器路径
#       相机走 image_server（由 image_server_host.sh 在系统 python 上启动，ZMQ 5558）
lerobot-rollout \
    --strategy.type="$STRATEGY" \
    --policy.path="$POLICY_PATH" \
    --robot.type=tienkung \
    --robot.bridge_enabled=true \
    --robot.bridge_script="$BRIDGE_SCRIPT" \
    --robot.joint_config="$JOINT_CONFIG" \
    --robot.cameras="{head: {type: image_server, server_address: '${ZMQ_HOST}', port: 5558, offset_x: 0, width: 640, height: 360, fps: $FPS, display: ${DISPLAY_CAM}}}" \
    --task="$TASK" \
    --fps="$FPS" \
    --duration="$DURATION"
