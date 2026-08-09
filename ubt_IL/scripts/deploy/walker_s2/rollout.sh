#!/bin/bash
# Walker S2 部署（rollout）脚本
# 在 ubt_IL/lerobot 容器内运行。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

POLICY_PATH="${POLICY_PATH:-}"
# ROBOT_MODEL 统一入口：一个变量选定关节 DOF + 末端执行器 + 相机配置。
# 可用值见 walker constants.py 的 ROBOT_MODELS 注册表。
ROBOT_MODEL="${ROBOT_MODEL:-walker_s2_31d}"
# ROBOT_CONFIG 可选：指向自定义 JSON 覆盖文件（不使用 ROBOT_MODELS 默认参数时）。
ROBOT_CONFIG="${ROBOT_CONFIG:-}"
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
STRATEGY="${STRATEGY:-base}"
FPS="${FPS:-13}"
DURATION="${DURATION:-30}"
TASK="${TASK:-walker s2 rollout}"
PREVIEW_CAMERA="${PREVIEW_CAMERA:-1}"
PREVIEW_CAMERA_WIDTH="${PREVIEW_CAMERA_WIDTH:-0}"
PREVIEW_CAMERA_HEIGHT="${PREVIEW_CAMERA_HEIGHT:-0}"
PREVIEW_CAMERA_TIMEOUT="${PREVIEW_CAMERA_TIMEOUT:-10.0}"
PREVIEW_CAMERA_PRINT_FPS="${PREVIEW_CAMERA_PRINT_FPS:-1}"
PREVIEW_CAMERA_WINDOW="${PREVIEW_CAMERA_WINDOW:-Walker camera}"
RECORD_ACTIONS="${RECORD_ACTIONS:-1}"
RECORD_OUTPUT_DIR="${RECORD_OUTPUT_DIR:-/ubt_IL/scripts/deploy/output/walker_rollout_$(date +%Y%m%d_%H%M%S)}"

# 推理引擎类型：sync（默认，单动作）| act_async（异步产 chunk 整块推桥接，桥接做融合/插值/滤波）
INFERENCE_TYPE="${INFERENCE_TYPE:-sync}"
# act_async 参数（仅 INFERENCE_TYPE=act_async 时生效）
INFERENCE_HZ="${INFERENCE_HZ:-0.6}"          # 重规划频率（每 ~1/INFERENCE_HZ 秒推一块新 chunk）。
#   降频以扩大每块执行步数 = (1/INFERENCE_HZ)*fps，需 > chunk 静止前缀步数；否则只执行静止段、obs 不变陷入局部循环。
#   0.6Hz@15fps -> 每块执行 ~25 步（过 onset）。推理耗时>周期时不睡（实际由延迟兜底）。
EXECUTION_HORIZON="${EXECUTION_HORIZON:-0}"  # 截断 chunk 到前 N 步，0=不截断（推完整 chunk_size，如 10d 模型 50 步）
# 桥接 chunk 消费者参数（透传给 bridge 子进程，需 export）
BLEND_HORIZON="${BLEND_HORIZON:-10}"         # 动作块融合重叠点数：新 chunk 前缀与旧轨迹未执行后缀混合的步数。
#   需 < 每块执行步数(exec=Δt×fps)的【下限】而非中位：exec 随 inference_time 波动(14-36)，
#   取 10 < exec_min≈14，保证 纯新步数=exec-eff_blend≥4，快重规划时新预测仍能上纯(20 会让 exec 14 的块纯新为负)。
#   过渡 0.67s@15fps(smoothstep+rate limit 足够平滑)。实际 eff_blend=min(本值,leftover)。
export BLEND_HORIZON
export INFERENCE_HZ                     # 桥接读它算固定执行步 E=round(fps/INFERENCE_HZ)（receding horizon）

# ---- 桥接 300Hz 轨迹发布（chunk_consumer densify 后执行）----
BODY_PUBLISH_HZ="${BODY_PUBLISH_HZ:-300}"             # body 发布线程频率（densify 后执行频率，与 control_hz 一致）
BODY_V_MAX="${BODY_V_MAX:-2.0}"                       # rate_limit 单关节最大速度 rad/s
export BODY_PUBLISH_HZ
export BODY_V_MAX

# ── Validation ───────────────────────────────────────────────────────────────

if [ -z "$POLICY_PATH" ]; then
    echo "[ERROR] POLICY_PATH is required."
    echo "[INFO] Example:"
    echo "       ROBOT_MODEL=walker_s2_10d POLICY_PATH=/ubt_IL/model/<policy>/checkpoints/last/pretrained_model bash $0"
    exit 1
fi

if [ -n "$ROBOT_CONFIG" ] && [ ! -f "$ROBOT_CONFIG" ]; then
    echo "[ERROR] ROBOT_CONFIG not found: $ROBOT_CONFIG"
    exit 1
fi

echo "[INFO] ROBOT_MODEL=$ROBOT_MODEL"
if [ "$RECORD_ACTIONS" = "1" ]; then
    echo "[INFO] RECORD_ACTIONS=1 → output dir: $RECORD_OUTPUT_DIR"
    export RECORD_ACTIONS
    export RECORD_OUTPUT_DIR
fi

# ── Preflight: validate policy ↔ robot config dimension match ────────────────

if [ -f "$POLICY_PATH/config.json" ]; then
    if [ -n "$ROBOT_CONFIG" ]; then
        PREFLIGHT_SOURCE="$ROBOT_CONFIG"
        PREFLIGHT_MODE="config"
        echo "[INFO] Preflight: validating policy against ROBOT_CONFIG=$ROBOT_CONFIG"
    else
        PREFLIGHT_SOURCE="$ROBOT_MODEL"
        PREFLIGHT_MODE="model"
        echo "[INFO] Preflight: validating policy against ROBOT_MODEL=$ROBOT_MODEL"
    fi
    /lerobot/.venv/bin/python - "$PREFLIGHT_SOURCE" "$POLICY_PATH/config.json" "$ALLOW_DIM_ONLY_POLICY" "$PREFLIGHT_MODE" <<'PY'
import json, sys

source      = sys.argv[1]
pc_path     = sys.argv[2]
dim_only    = sys.argv[3] == "1"
source_type = sys.argv[4]

# ── Derive expected features ────────────────────────────────────────────
if source_type == "config":
    from pathlib import Path
    rc = Path(source)
    with rc.open("r", encoding="utf-8") as f:
        robot = json.load(f)
    action_order = robot.get("action_order")
    if not isinstance(action_order, list) or not action_order:
        raise SystemExit(f"[ERROR] {rc} must contain non-empty action_order")
    if any(not isinstance(n, str) or not n for n in action_order):
        raise SystemExit("[ERROR] action_order entries must be non-empty strings")
    expected_features = [f"{n}.pos" for n in action_order]
    source_label = str(rc)
else:
    from lerobot_robot_walker.constants import ROBOT_MODELS, joint_names_with_pos
    if source not in ROBOT_MODELS:
        raise SystemExit(
            f"[ERROR] {source!r} not in ROBOT_MODELS registry. "
            f"Available: {list(ROBOT_MODELS)}"
        )
    spec = ROBOT_MODELS[source]
    expected_features = joint_names_with_pos(spec["joint_order"])
    source_label = f"ROBOT_MODELS[{source}]"

expected_dim = len(expected_features)

# ── Read policy shape ───────────────────────────────────────────────────
with open(pc_path, "r", encoding="utf-8") as f:
    policy = json.load(f)

shape = policy.get("output_features", {}).get("action", {}).get("shape")
if shape is None:
    shape = policy.get("policy", {}).get("output_features", {}).get("action", {}).get("shape")
if not shape:
    raise SystemExit("[ERROR] Could not find policy output action shape in config.json")
action_dim = int(shape[0])
if action_dim != expected_dim:
    raise SystemExit(
        f"[ERROR] Action dim mismatch: {source_label} expects {expected_dim}, "
        f"policy has {action_dim}\n"
        f"        policy_config={pc_path}"
    )

# ── Validate action names ───────────────────────────────────────────────
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
            f"[ERROR] Policy action names/order do not match robot config.\n"
            f"        expected={expected_features}\n        policy={names}"
        )
elif not dim_only:
    raise SystemExit(
        f"[ERROR] Policy config has no action names; refusing dim-only deployment.\n"
        f"        Set ALLOW_DIM_ONLY_POLICY=1 only if you verified the policy "
        f"order matches {source_label}."
    )

print(f"[INFO] Robot config source : {source_label}")
print(f"[INFO] Policy action dim   : {action_dim}")
print(f"[INFO] Expected action dim : {expected_dim}")
print(f"[INFO] Action feature names: {expected_features}")
if names is None:
    print("[WARN] ALLOW_DIM_ONLY_POLICY=1: policy action names are unavailable; "
          "using robot config order by dimension only.")
PY
else
    echo "[WARN] Policy config.json not found at $POLICY_PATH/config.json; skipping dimension preflight."
fi

# ── Enter LeRobot workspace ──────────────────────────────────────────────────

cd /ubt_IL/lerobot || { echo "[ERROR] /ubt_IL/lerobot not found"; exit 1; }

# ── Preview camera ───────────────────────────────────────────────────────────

PREVIEW_PID=""

if [ "$PREVIEW_CAMERA" = "1" ]; then
    cleanup_preview() {
        if [ -n "$PREVIEW_PID" ]; then
            kill "$PREVIEW_PID" 2>/dev/null || true
            wait "$PREVIEW_PID" 2>/dev/null || true
        fi
    }
    trap cleanup_preview EXIT INT TERM

    PREVIEW_CMD=(
        /usr/bin/python3 "$SCRIPT_DIR/preview_camera.py"
        --robot "$ROBOT_MODEL"
        --width "$PREVIEW_CAMERA_WIDTH"
        --height "$PREVIEW_CAMERA_HEIGHT"
        --timeout "$PREVIEW_CAMERA_TIMEOUT"
        --window "$PREVIEW_CAMERA_WINDOW"
    )
    [ "$PREVIEW_CAMERA_PRINT_FPS" = "1" ] && PREVIEW_CMD+=(--print-fps)

    echo "[INFO] Starting camera preview for ROBOT_MODEL=$ROBOT_MODEL"
    "${PREVIEW_CMD[@]}" &
    PREVIEW_PID=$!
fi

# ── Rollout ──────────────────────────────────────────────────────────────────

ROBOT_CONFIG_ARG=()
if [ -n "$ROBOT_CONFIG" ]; then
    ROBOT_CONFIG_ARG=(--robot.robot_config_path="$ROBOT_CONFIG")
fi

# 推理引擎参数（sync 不传；act_async 传 type + 异步参数）
INFERENCE_ARGS=()
if [ "$INFERENCE_TYPE" != "sync" ]; then
    INFERENCE_ARGS+=(--inference.type="$INFERENCE_TYPE")
fi
if [ "$INFERENCE_TYPE" = "act_async" ]; then
    INFERENCE_ARGS+=(
        --inference.act_async.inference_hz="$INFERENCE_HZ"
        --inference.act_async.execution_horizon="$EXECUTION_HORIZON"
    )
    echo "[INFO] act_async: inference_hz=$INFERENCE_HZ execution_horizon=$EXECUTION_HORIZON (chunk -> bridge 融合/插值/滤波)"
fi

/lerobot/.venv/bin/lerobot-rollout \
    --strategy.type="$STRATEGY" \
    --policy.path="$POLICY_PATH" \
    --robot.type=walker \
    "${ROBOT_CONFIG_ARG[@]}" \
    "${INFERENCE_ARGS[@]}" \
    --robot.joint_config="$ROBOT_MODEL" \
    --task="$TASK" \
    --fps="$FPS" \
    --duration="$DURATION" \
    $EXTRA_ARGS
