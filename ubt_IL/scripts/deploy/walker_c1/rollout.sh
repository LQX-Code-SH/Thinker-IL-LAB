#!/bin/bash
# Walker C1 rollout（LeRobot → ZMQ → ros2_walker_c1_bridge → ROS2 → 真机/ubt_sim）
#
# 默认单集、15 fps、base strategy、维度预检 26 维。
#
# 常用环境变量：
#   POLICY_PATH        必填，pretrained_model 目录
#   ROBOT_MODEL        默认 walker_c1_26d（ROBOT_MODELS 注册表键）
#   ROBOT_CONFIG       默认 configs/walker_c1_26d.json
#   ROBOT_TYPE         默认 walker_c1
#   EPISODES           默认 1；>1 时默认每集重启进程并复位（PER_EPISODE_RESTART=1）
#   DURATION           每集最长秒数（默认 30）
#   FPS                控制环 FPS（默认 15）
#   TASK               任务描述
#   RESET_MODE         每集前复位方式：sim-scene | task | both | none（默认 sim-scene）
#   ALLOW_DIM_ONLY_POLICY  1=只校验维度不校验关节名（默认 1）
#   INFERENCE_TYPE     sync | act_async（默认 sync）
#   INFERENCE_HZ       act_async 重规划频率
#   EXECUTION_HORIZON  act_async chunk 截断步数
#   PREVIEW_CAMERA     1=同时开相机预览窗口
#   CAMERA_TOPIC       相机话题覆盖（真机：/sensor/camera/head_stereo_left/color/compressed）
#   PREVIEW_TOPIC      预览窗口用话题（默认跟随 CAMERA_TOPIC）
#
# 示例：
#   POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
#     bash rollout.sh
#   EPISODES=5 DURATION=20 RESET_MODE=both bash rollout.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

POLICY_PATH="${POLICY_PATH:-}"
ROBOT_MODEL="${ROBOT_MODEL:-walker_c1_26d}"
ROBOT_CONFIG="${ROBOT_CONFIG:-$SCRIPT_DIR/configs/walker_c1_26d.json}"
ROBOT_TYPE="${ROBOT_TYPE:-walker_c1}"
STRATEGY="${STRATEGY:-base}"
EPISODES="${EPISODES:-1}"
PER_EPISODE_RESTART="${PER_EPISODE_RESTART:-1}"
DURATION="${DURATION:-30}"
FPS="${FPS:-15}"
TASK="${TASK:-walker c1 pick place}"
RESET_MODE="${RESET_MODE:-sim-scene}"
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
INFERENCE_TYPE="${INFERENCE_TYPE:-sync}"
INFERENCE_HZ="${INFERENCE_HZ:-2}"
EXECUTION_HORIZON="${EXECUTION_HORIZON:-0}"
PREVIEW_CAMERA="${PREVIEW_CAMERA:-0}"
# 相机源：仿真 = /sensor/camera/head/color/raw（sensor_msgs/Image）；
# 真机 = /sensor/camera/head_stereo_left/color/compressed（CompressedImage，JPEG）。
# 消息类型由插件按话题名自动推断，这里只透传话题覆盖。
CAMERA_TOPIC="${CAMERA_TOPIC:-}"
PREVIEW_TOPIC="${PREVIEW_TOPIC:-${CAMERA_TOPIC:-/sensor/camera/head/color/raw}}"
if [[ -n "$CAMERA_TOPIC" ]]; then
    export WALKER_C1_CAMERA_TOPIC="$CAMERA_TOPIC"
fi
LEROBOT_DIR="${LEROBOT_DIR:-/ubt_IL/lerobot}"
if [[ ! -d "$LEROBOT_DIR" ]]; then
    LEROBOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)/lerobot"
fi
ROLLOUT_BIN="${ROLLOUT_BIN:-/lerobot/.venv/bin/lerobot-rollout}"
ROS_WS="${WALKER_C1_ROS_WS:-/ubt_IL/walker_s2/walker_sdk_ros2}"

usage() {
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
}
for arg in "$@"; do
    [[ "$arg" == "-h" || "$arg" == "--help" ]] && usage
done

[[ -n "$POLICY_PATH" ]] || { echo "[rollout] 错误：必须设置 POLICY_PATH" >&2; exit 1; }
[[ -f "$ROBOT_CONFIG" ]] || { echo "[rollout] 错误：ROBOT_CONFIG 不存在: $ROBOT_CONFIG" >&2; exit 1; }
if [[ ! -x "$ROLLOUT_BIN" ]]; then
    echo "[rollout] 警告：$ROLLOUT_BIN 不可执行，回退到 PATH 中的 lerobot-rollout" >&2
    ROLLOUT_BIN="lerobot-rollout"
fi

# ── 维度 / 关节名预检（policy config.json ↔ ROBOT_CONFIG.action_order）───────
if [[ -f "$POLICY_PATH/config.json" ]]; then
    PREFLIGHT_PY="$([[ -x /lerobot/.venv/bin/python ]] && echo /lerobot/.venv/bin/python || echo python3)"
    "$PREFLIGHT_PY" - "$ROBOT_CONFIG" "$POLICY_PATH/config.json" "$ALLOW_DIM_ONLY_POLICY" <<'PY'
import json, sys
from pathlib import Path

config_path, policy_path, dim_only = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
robot = json.loads(Path(config_path).read_text())
action_order = robot.get("action_order") or []
if not action_order:
    raise SystemExit(f"[ERROR] {config_path} has empty action_order")
expected_features = [f"{n}.pos" for n in action_order]

policy = json.loads(Path(policy_path).read_text())
shape = policy.get("output_features", {}).get("action", {}).get("shape")
if shape is None:
    shape = policy.get("policy", {}).get("output_features", {}).get("action", {}).get("shape")
if not shape:
    raise SystemExit("[ERROR] policy config.json has no output_features.action.shape")
action_dim = int(shape[0])
if action_dim != len(expected_features):
    raise SystemExit(
        f"[ERROR] Action dim mismatch: robot expects {len(expected_features)} "
        f"({Path(config_path).name}), policy has {action_dim}\n"
        f"        policy={policy_path}"
    )

names = None
for root in (policy, policy.get("policy", {})):
    if root.get("action_feature_names"):
        names = list(root["action_feature_names"])
        break
if names and not dim_only:
    if names != expected_features:
        mismatch = [(i, a, b) for i, (a, b) in enumerate(zip(names, expected_features)) if a != b]
        raise SystemExit(
            "[ERROR] Action feature names mismatch (first 5): "
            + repr(mismatch[:5])
            + "\n        ALLOW_DIM_ONLY_POLICY=1 可只校验维度"
        )
print(f"[preflight] OK: {len(expected_features)} DOF, "
      f"{'names verified' if names and not dim_only else 'dim only'}")
PY
else
    echo "[rollout] 提示：$POLICY_PATH/config.json 不存在，跳过维度预检" >&2
fi

# ── 相机预览（可选）───────────────────────────────────────────────────────────
PREVIEW_PID=""
if [[ "$PREVIEW_CAMERA" == "1" ]]; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    if [[ -f "$ROS_WS/install/setup.bash" ]]; then
        source "$ROS_WS/install/setup.bash"
    fi
    /usr/bin/python3 "$SCRIPT_DIR/preview_camera.py" --topic "$PREVIEW_TOPIC" &
    PREVIEW_PID=$!
fi
cleanup() {
    if [[ -n "$PREVIEW_PID" ]]; then kill "$PREVIEW_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT

# ── 复位（每集前）─────────────────────────────────────────────────────────────
reset_once() {
    [[ "$RESET_MODE" == "none" ]] && return 0
    bash "$SCRIPT_DIR/robot_ready.sh" --mode "$RESET_MODE" || {
        echo "[rollout] 警告：复位失败（$RESET_MODE），继续 rollout" >&2
    }
}

# ── 主循环 ────────────────────────────────────────────────────────────────────
EXTRA_ARGS=("$@")
ROBOT_CONFIG_ARG=()
[[ -n "$ROBOT_CONFIG" ]] && ROBOT_CONFIG_ARG=(--robot.robot_config_path="$ROBOT_CONFIG")

INFERENCE_ARGS=()
if [[ "$INFERENCE_TYPE" != "sync" ]]; then
    INFERENCE_ARGS+=(--inference.type="$INFERENCE_TYPE")
fi
if [[ "$INFERENCE_TYPE" == "act_async" ]]; then
    INFERENCE_ARGS+=(
        --inference.act_async.inference_hz="$INFERENCE_HZ"
        --inference.act_async.execution_horizon="$EXECUTION_HORIZON"
    )
fi

run_rollout() {
    # 注意：BaseStrategyConfig 不支持 --strategy.num_episodes（仅 DAgger 有），
    # 多集必须由外层循环 + 每集复位实现。
    cd "$LEROBOT_DIR"
    "$ROLLOUT_BIN" \
        --strategy.type="$STRATEGY" \
        --policy.path="$POLICY_PATH" \
        --robot.type="$ROBOT_TYPE" \
        "${ROBOT_CONFIG_ARG[@]}" \
        "${INFERENCE_ARGS[@]}" \
        --robot.joint_config="$ROBOT_MODEL" \
        --task="$TASK" \
        --fps="$FPS" \
        --duration="$DURATION" \
        "${EXTRA_ARGS[@]}"
}

echo "[rollout] ROBOT_TYPE=$ROBOT_TYPE ROBOT_MODEL=$ROBOT_MODEL"
echo "[rollout] POLICY_PATH=$POLICY_PATH"
echo "[rollout] EPISODES=$EPISODES DURATION=${DURATION}s FPS=$FPS RESET_MODE=$RESET_MODE"

for ((i = 1; i <= EPISODES; i++)); do
    [[ "$EPISODES" -gt 1 ]] && echo "[rollout] ===== episode $i/$EPISODES ====="
    reset_once
    run_rollout
    [[ "$PER_EPISODE_RESTART" != "1" && "$EPISODES" -gt 1 ]] && {
        echo "[rollout] PER_EPISODE_RESTART=0：base strategy 不支持多集，已只执行首集" >&2
        break
    }
done
