#!/bin/bash
# Walker S2 推理服务器启动脚本（常驻预热 + ZMQ 指令/状态）。
# 在 ubt_IL/lerobot 容器内运行。后台 nohup 拉起 inference_server，一次性预热加载模型 +
# 连机器人，之后外部经 inference_client.sh 发 start/stop/home/status/load/shutdown 指令。
#
# 用法：
#   ROBOT_MODEL=walker_s2_10d \
#   POLICY_PATH=/ubt_IL/model/Pick_part_real_10d_2RGB_act/checkpoints/100000/pretrained_model \
#   INFERENCE_TYPE=act_async INFERENCE_HZ=2 FPS=15 \
#   bash /ubt_IL/scripts/deploy/walker_s2/inference_server.sh
#
# 与 rollout.sh 的区别：不跑单次 duration rollout，而是常驻服务；DURATION 无意义；PREVIEW_CAMERA
# 默认 0（服务器不开预览窗口以免争 CPU）；新增 SERVER_CMD_PORT/SERVER_STATUS_PORT。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

POLICY_PATH="${POLICY_PATH:-}"
# ROBOT_MODEL 统一入口：选定关节 DOF + 末端执行器 + 相机配置（见 walker constants.py ROBOT_MODELS）。
ROBOT_MODEL="${ROBOT_MODEL:-walker_s2_31d}"
ROBOT_CONFIG="${ROBOT_CONFIG:-}"
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
STRATEGY="${STRATEGY:-base}"
FPS="${FPS:-13}"
TASK="${TASK:-walker s2 inference server}"
PREVIEW_CAMERA="${PREVIEW_CAMERA:-0}"  # 服务器默认不开预览（留作 rollout.sh 用）

# 推理引擎类型：act_async（异步产 chunk 整块推桥接）| sync（单动作，无 pause/resume，start/stop 不适用）
INFERENCE_TYPE="${INFERENCE_TYPE:-act_async}"
INFERENCE_HZ="${INFERENCE_HZ:-0.6}"
EXECUTION_HORIZON="${EXECUTION_HORIZON:-0}"

# 桥接 chunk_consumer 参数（透传给桥接子进程：本脚本 export -> python 进程继承 -> 桥接读 os.environ）
BLEND_HORIZON="${BLEND_HORIZON:-10}"
BODY_PUBLISH_HZ="${BODY_PUBLISH_HZ:-300}"
BODY_V_MAX="${BODY_V_MAX:-2.0}"
export BLEND_HORIZON
export BODY_PUBLISH_HZ
export BODY_V_MAX
export INFERENCE_HZ  # 引擎经 CLI 取；此处 export 保持与 rollout.sh 一致

# 服务器 ZMQ 端口（避开桥接 5561/5562/5563）
SERVER_CMD_PORT="${SERVER_CMD_PORT:-5570}"
SERVER_STATUS_PORT="${SERVER_STATUS_PORT:-5571}"

# 运行时文件
PID_FILE="${PID_FILE:-/tmp/walker_inference_server.pid}"
LOG_FILE="${LOG_FILE:-/tmp/walker_inference_server.log}"

# 判断 PID 是否真的是在跑的 inference_server 进程。
# 仅 kill -0 不够：僵尸进程(defunct)仍占 PID 表 -> kill -0 成功 -> 误判；
# PID 复用也会误判。这里额外查 /proc/PID/cmdline 是否含 inference_server：
# zombie 的 cmdline 为空、无关进程的 cmdline 不含 -> 都判为陈旧。
_pid_is_running_server() {
    local pid="$1"
    [ -n "$pid" ] || return 1
    kill -0 "$pid" 2>/dev/null || return 1
    grep -qa "lerobot.scripts.inference_server" "/proc/$pid/cmdline" 2>/dev/null
}

# ── 校验 ──────────────────────────────────────────────────────────────────────

if [ -z "$POLICY_PATH" ]; then
    echo "[ERROR] POLICY_PATH is required."
    echo "[INFO] Example:"
    echo "       ROBOT_MODEL=walker_s2_10d POLICY_PATH=/ubt_IL/model/<policy>/checkpoints/last/pretrained_model \\"
    echo "       INFERENCE_TYPE=act_async INFERENCE_HZ=2 FPS=15 bash $0"
    exit 1
fi

if [ -n "$ROBOT_CONFIG" ] && [ ! -f "$ROBOT_CONFIG" ]; then
    echo "[ERROR] ROBOT_CONFIG not found: $ROBOT_CONFIG"
    exit 1
fi

echo "[INFO] ROBOT_MODEL=$ROBOT_MODEL  INFERENCE_TYPE=$INFERENCE_TYPE  FPS=$FPS"
echo "[INFO] POLICY_PATH=$POLICY_PATH"

# ── 单例检查 ──────────────────────────────────────────────────────────────────

if [ -f "$PID_FILE" ]; then
    OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
    if _pid_is_running_server "$OLD_PID"; then
        echo "[ERROR] Inference server already running (PID $OLD_PID)."
        echo "        To stop: bash $SCRIPT_DIR/inference_client.sh shutdown"
        echo "        Or kill $OLD_PID and remove $PID_FILE."
        exit 1
    else
        echo "[INFO] Stale PID file (PID $OLD_PID not a running server); removing."
        rm -f "$PID_FILE"
    fi
fi

# ── Preflight: policy ↔ robot config 维度校验（复用 rollout.sh 同款）──────────

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

print(f"[INFO] Policy action dim   : {action_dim}")
print(f"[INFO] Expected action dim : {expected_dim}")
PY
else
    echo "[WARN] Policy config.json not found at $POLICY_PATH/config.json; skipping dimension preflight."
fi

# ── 进入 LeRobot 工作区 ───────────────────────────────────────────────────────

cd /ubt_IL/lerobot || { echo "[ERROR] /ubt_IL/lerobot not found"; exit 1; }

# ── 构造 CLI 参数（镜像 rollout.sh）──────────────────────────────────────────

ROBOT_CONFIG_ARG=()
if [ -n "$ROBOT_CONFIG" ]; then
    ROBOT_CONFIG_ARG=(--robot.robot_config_path="$ROBOT_CONFIG")
fi

INFERENCE_ARGS=()
if [ "$INFERENCE_TYPE" != "sync" ]; then
    INFERENCE_ARGS+=(--inference.type="$INFERENCE_TYPE")
fi
if [ "$INFERENCE_TYPE" = "act_async" ]; then
    INFERENCE_ARGS+=(
        --inference.act_async.inference_hz="$INFERENCE_HZ"
        --inference.act_async.execution_horizon="$EXECUTION_HORIZON"
    )
    echo "[INFO] act_async: inference_hz=$INFERENCE_HZ execution_horizon=$EXECUTION_HORIZON"
fi

# ── 后台拉起服务 ──────────────────────────────────────────────────────────────

LAUNCH_CMD=(
    /lerobot/.venv/bin/python -m lerobot.scripts.inference_server
    --strategy.type="$STRATEGY"
    --policy.path="$POLICY_PATH"
    --robot.type=walker
    "${ROBOT_CONFIG_ARG[@]}"
    "${INFERENCE_ARGS[@]}"
    --robot.joint_config="$ROBOT_MODEL"
    --task="$TASK"
    --fps="$FPS"
    --server.cmd_port="$SERVER_CMD_PORT"
    --server.status_port="$SERVER_STATUS_PORT"
)

echo "[INFO] Launching inference server (background, preheating)..."
echo "[INFO] log: $LOG_FILE"
# 把 PID_FILE 路径传给服务进程，使其优雅退出时自删 PID 文件（atexit）。
# SIGKILL/崩溃时 PID 文件残留 -> 由上方 _pid_is_running_server 守卫兜底。
WALKER_INFERENCE_PID_FILE="$PID_FILE" nohup "${LAUNCH_CMD[@]}" > "$LOG_FILE" 2>&1 &
SERVER_PID=$!
echo "$SERVER_PID" > "$PID_FILE"

echo "[OK] Inference server started (PID $SERVER_PID)."
echo "     Preheating model + connecting robot... (tail -f $LOG_FILE)"
echo "     cmd  port: tcp://*:$SERVER_CMD_PORT  (REP: start|stop|home|status|load|shutdown)"
echo "     stat port: tcp://*:$SERVER_STATUS_PORT (PUB: state stream)"
echo "[INFO] Poll status until READY:"
echo "       bash $SCRIPT_DIR/inference_client.sh status"
echo "       bash $SCRIPT_DIR/inference_client.sh --watch"
