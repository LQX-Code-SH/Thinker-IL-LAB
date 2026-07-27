#!/bin/bash
# Walker S2 部署（rollout）脚本
# 在 ubt_IL/lerobot 容器内运行。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/configs/walker"

POLICY_PATH="${POLICY_PATH:-}"
ROBOT_MODEL="${ROBOT_MODEL:-walker_s2_v4_hand_31d}"
ROBOT_CONFIG="${ROBOT_CONFIG:-$CONFIG_DIR/$ROBOT_MODEL.json}"
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-0}"
STRATEGY="${STRATEGY:-base}"
FPS="${FPS:-15}"
DURATION="${DURATION:-30}"
TASK="${TASK:-walker s2 rollout}"
PREVIEW_CAMERA="${PREVIEW_CAMERA:-1}"
PREVIEW_CAMERA_HOST="${PREVIEW_CAMERA_HOST:-127.0.0.1}"
PREVIEW_CAMERA_PORT="${PREVIEW_CAMERA_PORT:-5563}"
PREVIEW_CAMERA_NAME="${PREVIEW_CAMERA_NAME:-camera_head}"
PREVIEW_CAMERA_WIDTH="${PREVIEW_CAMERA_WIDTH:-0}"
PREVIEW_CAMERA_HEIGHT="${PREVIEW_CAMERA_HEIGHT:-0}"
PREVIEW_CAMERA_TIMEOUT_MS="${PREVIEW_CAMERA_TIMEOUT_MS:-5000}"
PREVIEW_CAMERA_PRINT_FPS="${PREVIEW_CAMERA_PRINT_FPS:-1}"
PREVIEW_CAMERA_WINDOW="${PREVIEW_CAMERA_WINDOW:-Walker camera}"
C1_APPLE_X="${C1_APPLE_X:-}"
C1_APPLE_Y="${C1_APPLE_Y:-}"
C1_APPLE_Z="${C1_APPLE_Z:-}"
C1_PLACEMENT_TOLERANCE="${C1_PLACEMENT_TOLERANCE:-0.02}"
C1_SETTLE_SIM_STEPS="${C1_SETTLE_SIM_STEPS:-10}"

if [ -z "$POLICY_PATH" ]; then
    echo "[ERROR] POLICY_PATH is required."
    echo "[INFO] Example:"
    echo "       ROBOT_MODEL=walker_s2_gripper_19d POLICY_PATH=/ubt_IL/model/<walker_policy>/checkpoints/last/pretrained_model bash /ubt_IL/scripts/deploy/rollout_walker.sh"
    exit 1
fi

if [ ! -f "$ROBOT_CONFIG" ]; then
    echo "[ERROR] ROBOT_CONFIG not found: $ROBOT_CONFIG"
    echo "[INFO] Set ROBOT_MODEL to one of the files under $CONFIG_DIR, or set ROBOT_CONFIG explicitly."
    exit 1
fi

if [ ! -f "$POLICY_PATH/config.json" ]; then
    echo "[ERROR] Policy config not found: $POLICY_PATH/config.json"
    echo "[INFO] Refusing to deploy without action-dimension preflight."
    exit 1
fi

/lerobot/.venv/bin/python - "$ROBOT_CONFIG" "$POLICY_PATH/config.json" "$ALLOW_DIM_ONLY_POLICY" <<'PY'
import json
import sys
from pathlib import Path

robot_config = Path(sys.argv[1])
policy_config = Path(sys.argv[2])
allow_dim_only = sys.argv[3] == "1"

with robot_config.open("r", encoding="utf-8") as f:
    robot = json.load(f)
with policy_config.open("r", encoding="utf-8") as f:
    policy = json.load(f)

action_order = robot.get("action_order")
if not isinstance(action_order, list) or not action_order:
    raise SystemExit(f"[ERROR] {robot_config} must contain non-empty action_order")
if any(not isinstance(name, str) or not name for name in action_order):
    raise SystemExit("[ERROR] action_order entries must be non-empty strings")
if any(name.endswith(".pos") for name in action_order):
    raise SystemExit("[ERROR] action_order should use real names without .pos; loader derives .pos features")

expected_features = [f"{name}.pos" for name in action_order]
expected_dim = len(expected_features)

shape = policy.get("output_features", {}).get("action", {}).get("shape")
if shape is None:
    shape = policy.get("policy", {}).get("output_features", {}).get("action", {}).get("shape")
if not shape:
    raise SystemExit("[ERROR] Could not find policy output action shape")
action_dim = int(shape[0])
if action_dim != expected_dim:
    raise SystemExit(
        f"[ERROR] Action dim mismatch: robot config expects {expected_dim}, policy has {action_dim}\n"
        f"        robot_config={robot_config}\n        policy_config={policy_config}"
    )

names = None
for root in (policy, policy.get("policy", {})):
    candidate = root.get("action_feature_names")
    if candidate:
        names = list(candidate)
        break
    output_action = root.get("output_features", {}).get("action", {})
    candidate = output_action.get("names")
    if isinstance(candidate, list):
        names = list(candidate)
        break

if names is not None:
    if names != expected_features:
        raise SystemExit(
            "[ERROR] Policy action names/order do not match robot config action_order.\n"
            f"        expected={expected_features}\n        policy={names}"
        )
elif not allow_dim_only:
    raise SystemExit(
        "[ERROR] Policy config has no action names; refusing dim-only deployment.\n"
        "        Set ALLOW_DIM_ONLY_POLICY=1 only if you verified the policy order matches robot action_order."
    )

print(f"[INFO] Walker robot model: {robot.get('robot_model', '?')}")
print(f"[INFO] Robot config: {robot_config}")
print(f"[INFO] Policy action dim: {action_dim}")
print(f"[INFO] Action order: {action_order}")
if names is None:
    print("[WARN] ALLOW_DIM_ONLY_POLICY=1: policy action names are unavailable; using robot config order by dimension only.")
PY

cd /ubt_IL/lerobot

PREVIEW_PID=""
cleanup_preview() {
    if [ -n "$PREVIEW_PID" ]; then
        kill "$PREVIEW_PID" 2>/dev/null || true
        wait "$PREVIEW_PID" 2>/dev/null || true
    fi
}

if [ "$PREVIEW_CAMERA" = "1" ]; then
    PREVIEW_CMD=(
        /lerobot/.venv/bin/python /ubt_IL/scripts/deploy/preview_walker_camera.py
        --host "$PREVIEW_CAMERA_HOST"
        --port "$PREVIEW_CAMERA_PORT"
        --camera "$PREVIEW_CAMERA_NAME"
        --width "$PREVIEW_CAMERA_WIDTH"
        --height "$PREVIEW_CAMERA_HEIGHT"
        --timeout-ms "$PREVIEW_CAMERA_TIMEOUT_MS"
        --window "$PREVIEW_CAMERA_WINDOW"
    )
    if [ "$PREVIEW_CAMERA_PRINT_FPS" = "1" ]; then
        PREVIEW_CMD+=(--print-fps)
    fi
    echo "[INFO] Starting Walker camera preview: ${PREVIEW_CAMERA_NAME} at ${PREVIEW_CAMERA_HOST}:${PREVIEW_CAMERA_PORT}"
    "${PREVIEW_CMD[@]}" &
    PREVIEW_PID=$!
    trap cleanup_preview EXIT INT TERM
fi

ROLLOUT_ARGS=(
    --strategy.type="$STRATEGY"
    --policy.path="$POLICY_PATH"
    --robot.type=walker
    --robot.robot_config_path="$ROBOT_CONFIG"
    --task="$TASK"
    --fps="$FPS"
    --duration="$DURATION"
)

if [ "$STRATEGY" = "walker_c1_once" ]; then
    if [ -z "$C1_APPLE_X" ] || [ -z "$C1_APPLE_Y" ] || [ -z "$C1_APPLE_Z" ]; then
        echo "[ERROR] walker_c1_once requires C1_APPLE_X, C1_APPLE_Y and C1_APPLE_Z."
        exit 1
    fi
    ROLLOUT_ARGS+=(
        --strategy.apple_x="$C1_APPLE_X"
        --strategy.apple_y="$C1_APPLE_Y"
        --strategy.apple_z="$C1_APPLE_Z"
        --strategy.placement_tolerance="$C1_PLACEMENT_TOLERANCE"
        --strategy.settle_sim_steps="$C1_SETTLE_SIM_STEPS"
    )
fi

/lerobot/.venv/bin/lerobot-rollout "${ROLLOUT_ARGS[@]}"
