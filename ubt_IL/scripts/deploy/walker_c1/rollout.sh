#!/bin/bash
# Walker C1 rollout（LeRobot → ZMQ → ros2_walker_c1_bridge → ROS2 → 真机/ubt_sim）
#
# 默认按【真机】配置：WALKER_C1_REAL=1、walker_c1_13d、task 复位、act_async 1 Hz、
# half_sdk 门禁、真机头目相机。仿真请加 WALKER_C1_REAL=0（回到 26d / sim-scene / sync / 2 Hz）。
#
# 常用环境变量：
#   POLICY_PATH        必填，pretrained_model 目录
#   WALKER_C1_REAL     默认 1（真机）；0=仿真/离线
#   ROBOT_MODEL        默认 真机 walker_c1_13d / 仿真 walker_c1_26d
#   ROBOT_CONFIG       默认 configs/<ROBOT_MODEL>.json
#   ROBOT_TYPE         默认 walker_c1
#   EPISODES           默认 1；>1 时默认每集重启进程并复位（PER_EPISODE_RESTART=1）
#   DURATION           每集最长秒数（默认 30）
#   FPS                控制环 FPS（默认 15）
#   TASK               任务描述
#   RESET_MODE         复位方式：sim-scene | task | both | none（真机默认 task，仿真 sim-scene）
#   ALLOW_DIM_ONLY_POLICY  1=只校验维度不校验关节名（默认 1）
#   INFERENCE_TYPE     sync | act_async（默认真机 act_async / 仿真 sync）
#   INFERENCE_HZ       act_async 重规划频率（默认真机 1 / 仿真 2）
#   EXECUTION_HORIZON  act_async chunk 截断步数
#   PREVIEW_CAMERA     1=同时开相机预览窗口
#   CAMERA_TOPIC       相机话题覆盖（真机默认：/sensor/camera/head_stereo_left/color/raw）
#   PREVIEW_TOPIC      预览窗口用话题（默认跟随 CAMERA_TOPIC）
#   SDK_GUARD          1=强制校验（真机默认）| auto=检测到真机才校验 | 0=跳过
#   SDK_AUTO_SWITCH    1=不在 half_sdk 时自动切换（默认只提示并退出）
#
# 示例（最简，全部走默认）：
#   POLICY_PATH=/ubt_IL/model/walker_c1_real_13d_act_1RGB/checkpoints/last/pretrained_model \
#     bash rollout.sh
#   WALKER_C1_REAL=0 EPISODES=5 DURATION=20 RESET_MODE=both bash rollout.sh   # 仿真
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

POLICY_PATH="${POLICY_PATH:-}"

# ── 默认值：真机优先 ─────────────────────────────────────────────────────────
# WALKER_C1_REAL=1（默认）= 真机：13d 模型、task 复位、act_async/1Hz、half_sdk 门禁。
# 仿真请在命令前加 WALKER_C1_REAL=0（自动回到 26d / sim-scene / sync / 2Hz）。
WALKER_C1_REAL="${WALKER_C1_REAL:-1}"
SDK_GUARD="${SDK_GUARD:-$([[ "$WALKER_C1_REAL" == "1" ]] && echo 1 || echo auto)}"

if [[ -z "${ROBOT_MODEL:-}" ]]; then
    if [[ "$WALKER_C1_REAL" == "1" ]]; then
        ROBOT_MODEL="walker_c1_13d"
    else
        ROBOT_MODEL="walker_c1_26d"
    fi
fi
# ROBOT_CONFIG 默认按 ROBOT_MODEL 自动选（configs/<robot_model>.json）：
# 13d 模型配 26d 配置会在维度预检直接报 "expects 26, policy has 13"。
if [[ -z "${ROBOT_CONFIG:-}" ]]; then
    if [[ -f "$SCRIPT_DIR/configs/${ROBOT_MODEL}.json" ]]; then
        ROBOT_CONFIG="$SCRIPT_DIR/configs/${ROBOT_MODEL}.json"
    else
        ROBOT_CONFIG="$SCRIPT_DIR/configs/walker_c1_26d.json"
    fi
fi
ROBOT_TYPE="${ROBOT_TYPE:-walker_c1}"
STRATEGY="${STRATEGY:-base}"
EPISODES="${EPISODES:-1}"
PER_EPISODE_RESTART="${PER_EPISODE_RESTART:-1}"
DURATION="${DURATION:-30}"
FPS="${FPS:-15}"
TASK="${TASK:-walker c1 pick place}"
if [[ -z "${RESET_MODE:-}" ]]; then
    if [[ "$WALKER_C1_REAL" == "1" ]]; then RESET_MODE="task"; else RESET_MODE="sim-scene"; fi
fi
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
if [[ -z "${INFERENCE_TYPE:-}" ]]; then
    if [[ "$WALKER_C1_REAL" == "1" ]]; then INFERENCE_TYPE="act_async"; else INFERENCE_TYPE="sync"; fi
fi
if [[ -z "${INFERENCE_HZ:-}" ]]; then
    if [[ "$WALKER_C1_REAL" == "1" ]]; then INFERENCE_HZ="1"; else INFERENCE_HZ="2"; fi
fi
EXECUTION_HORIZON="${EXECUTION_HORIZON:-0}"
PREVIEW_CAMERA="${PREVIEW_CAMERA:-0}"
# 相机源：仿真 = /sensor/camera/head/color/raw（sensor_msgs/Image）；
# 真机 = /sensor/camera/head_stereo_left/color/raw（shm_msgs/Image6m，1920x1536）。
# 消息类型由插件按话题名自动推断，真机 Image6m 不是 sensor_msgs/Image，必须显式给类型。
CAMERA_TOPIC="${CAMERA_TOPIC:-}"
CAMERA_MSG_TYPE="${CAMERA_MSG_TYPE:-}"
# 真机（WALKER_C1_REAL=1）未显式指定时，默认走头目左目话题，避免误订阅仿真话题
if [[ -z "$CAMERA_TOPIC" && "${WALKER_C1_REAL:-0}" == "1" ]]; then
    CAMERA_TOPIC="/sensor/camera/head_stereo_left/color/raw"
    CAMERA_MSG_TYPE="${CAMERA_MSG_TYPE:-shm_msgs/Image6m}"
fi
PREVIEW_TOPIC="${PREVIEW_TOPIC:-${CAMERA_TOPIC:-/sensor/camera/head/color/raw}}"
if [[ -n "$CAMERA_TOPIC" ]]; then
    export WALKER_C1_CAMERA_TOPIC="$CAMERA_TOPIC"
fi
if [[ -n "$CAMERA_MSG_TYPE" ]]; then
    export WALKER_C1_CAMERA_MSG_TYPE="$CAMERA_MSG_TYPE"
fi
# 串行推理依赖单动作平滑：置 0 会退化为阶跃下发（顿挫），启动时显式告警
if [[ "$INFERENCE_TYPE" == "sync" && "${WALKER_C1_SINGLE_ACTION_SMOOTH_S:-0.15}" == "0" ]]; then
    echo "[rollout] 警告：串行推理必须开启平滑（WALKER_C1_SINGLE_ACTION_SMOOTH_S=0 会顿挫）；推荐改用 INFERENCE_TYPE=act_async" >&2
fi
echo "[rollout] CAMERA_TOPIC=${CAMERA_TOPIC:-<插件默认 /sensor/camera/head/color/raw>} (${CAMERA_MSG_TYPE:-auto})"
LEROBOT_DIR="${LEROBOT_DIR:-/ubt_IL/lerobot}"
if [[ ! -d "$LEROBOT_DIR" ]]; then
    LEROBOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)/lerobot"
fi
ROLLOUT_BIN="${ROLLOUT_BIN:-/lerobot/.venv/bin/lerobot-rollout}"
ROS_WS="${WALKER_C1_ROS_WS:-/ubt_IL/walker_s2/walker_sdk_ros2}"

usage() {
    sed -n '2,31p' "$0" | sed 's/^# \{0,1\}//'
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

# ── 真机门禁：必须处于 half_sdk（开发者模式 + 上半身 SDK 权限）───────────────
# shellcheck source=./sdk_guard.sh
source "$SCRIPT_DIR/sdk_guard.sh"
walker_c1_require_half_sdk "rollout" || exit 1

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

echo "[rollout] REAL=$WALKER_C1_REAL ROBOT_TYPE=$ROBOT_TYPE ROBOT_MODEL=$ROBOT_MODEL"
echo "[rollout] RESET_MODE=$RESET_MODE DURATION=$DURATION FPS=$FPS INFERENCE_TYPE=$INFERENCE_TYPE INFERENCE_HZ=$INFERENCE_HZ"
echo "[rollout] SDK_GUARD=$SDK_GUARD ROBOT_CONFIG=$ROBOT_CONFIG"
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
