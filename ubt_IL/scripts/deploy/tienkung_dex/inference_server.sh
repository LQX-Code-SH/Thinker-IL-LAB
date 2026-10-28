#!/bin/bash
# TienKung Dex 推理服务器启动脚本（常驻预热 + ZMQ 指令/状态）。
# 在 lerobot-tienkung 容器内运行。后台 nohup 拉起 inference_server，一次性预热加载模型 +
# 连机器人，之后外部经 inference_client.sh 发 start/stop/home/status/shutdown 指令。
#
# 与 rollout.sh 的区别：不跑单次 duration rollout，而是常驻服务；DURATION 无意义。
# 跑策略之前同样要先 robot_ready.sh 把机器人摆到预备位（服务只负责 start/stop）。
#
# 用法：
#   POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
#   JOINT_CONFIG=tienkung_dex_20 FPS=15 \
#   bash /ubt_IL/scripts/deploy/tienkung_dex/inference_server.sh
#
# 注意：`load`（运行中热切换模型）在 Dex 上不可用——服务端重建机器人的代码路径写死了
# `--robot.type=walker` 且不带 `--robot.cameras`，换模型请用 shutdown + 重新 launch。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

POLICY_PATH="${POLICY_PATH:-}"
JOINT_CONFIG="${JOINT_CONFIG:-tienkung_dex_20}"
HAND_VARIANT="${HAND_VARIANT:-}"
STRATEGY="${STRATEGY:-base}"
# 控制环频率 = 数据集帧率（15 Hz）
FPS="${FPS:-15}"
TASK="${TASK:-tienkung dex inference server}"
ZMQ_HOST="${ZMQ_HOST:-127.0.0.1}"
# 相机端口：仿真 5758（ubt_sim 直发 JPEG）／真机 5558（Bridge2 转码）
CAMERA_PORT="${CAMERA_PORT:-5758}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-360}"
CAMERA_DISPLAY="${CAMERA_DISPLAY:-false}"   # 常驻服务默认不开预览窗口（争 CPU）
IMAGE_ENABLED="${IMAGE_ENABLED:-0}"
# 单步相对目标限幅 rad（插件侧安全层）。留空=不限制；真机常驻服务同样建议 0.01
MAX_RELATIVE_TARGET="${MAX_RELATIVE_TARGET:-}"
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"

# 推理引擎：Dex 桥接只消费单步指令，没有 chunk 消费者 -> 只支持 sync
INFERENCE_TYPE="${INFERENCE_TYPE:-sync}"

# 服务器 ZMQ 端口（避开桥接 5559/5560 与图像 5758/5558）
SERVER_CMD_PORT="${SERVER_CMD_PORT:-5570}"
SERVER_STATUS_PORT="${SERVER_STATUS_PORT:-5571}"

# 运行时文件
PID_FILE="${PID_FILE:-/tmp/tienkung_dex_inference_server.pid}"
LOG_FILE="${LOG_FILE:-/tmp/tienkung_dex_inference_server.log}"

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
    echo "[ERROR] POLICY_PATH 必填。"
    echo "[INFO] 示例："
    echo "       POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \\"
    echo "       JOINT_CONFIG=tienkung_dex_20 FPS=15 bash $0"
    exit 1
fi

if [ "$INFERENCE_TYPE" != "sync" ]; then
    echo "[ERROR] tienkung_dex 只支持 INFERENCE_TYPE=sync（当前: $INFERENCE_TYPE）。"
    echo "        Dex 桥接只解析单步 {\"right_arm\":[7],\"right_hand\":[N]} 指令，没有 chunk 消费者。"
    exit 1
fi

echo "[INFO] JOINT_CONFIG=$JOINT_CONFIG  HAND_VARIANT=${HAND_VARIANT:-<跟随 joint_config>}  FPS=$FPS"
echo "[INFO] POLICY_PATH=$POLICY_PATH"
echo "[INFO] 相机: $ZMQ_HOST:$CAMERA_PORT  image_enabled=$IMAGE_ENABLED"
[ -n "$MAX_RELATIVE_TARGET" ] && echo "[INFO] 单步相对限幅 max_relative_target=$MAX_RELATIVE_TARGET rad"

# ── 单例检查 ──────────────────────────────────────────────────────────────────

if [ -f "$PID_FILE" ]; then
    OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
    if _pid_is_running_server "$OLD_PID"; then
        echo "[ERROR] 推理服务已在运行（PID $OLD_PID）。"
        echo "        停止: bash $SCRIPT_DIR/inference_client.sh shutdown"
        echo "        或 kill $OLD_PID 并删除 $PID_FILE。"
        exit 1
    else
        echo "[INFO] 陈旧 PID 文件（PID $OLD_PID 不是运行中的服务），删除。"
        rm -f "$PID_FILE"
    fi
fi

# ── 预检：policy 维度 ↔ 关节配置（与 rollout.sh 同一套）────────────────────────

if [ -f "$POLICY_PATH/config.json" ]; then
    echo "[INFO] Preflight: 校验 policy 与 JOINT_CONFIG=$JOINT_CONFIG 的维度"
    /lerobot/.venv/bin/python - "$JOINT_CONFIG" "$POLICY_PATH/config.json" "$ALLOW_DIM_ONLY_POLICY" <<'PY'
import json, sys

joint_config = sys.argv[1]
pc_path      = sys.argv[2]
dim_only     = sys.argv[3] == "1"

from lerobot_robot_tienkung_dex.constants import JOINT_INDEX_ENUMS, joint_names_with_pos

if joint_config not in JOINT_INDEX_ENUMS:
    raise SystemExit(
        f"[ERROR] {joint_config!r} 未注册。可用: {list(JOINT_INDEX_ENUMS)}"
    )
expected_features = joint_names_with_pos(JOINT_INDEX_ENUMS[joint_config])
expected_dim = len(expected_features)

with open(pc_path, "r", encoding="utf-8") as f:
    policy = json.load(f)

shape = policy.get("output_features", {}).get("action", {}).get("shape")
if shape is None:
    shape = policy.get("policy", {}).get("output_features", {}).get("action", {}).get("shape")
if not shape:
    raise SystemExit("[ERROR] policy config.json 里找不到 output action shape")
action_dim = int(shape[0])
if action_dim != expected_dim:
    raise SystemExit(
        f"[ERROR] 动作维度不匹配: JOINT_CONFIG={joint_config} 期望 {expected_dim} 维，"
        f"policy 是 {action_dim} 维\n        policy_config={pc_path}"
    )

names = None
for root in (policy, policy.get("policy", {})):
    candidate = root.get("action_feature_names")
    if candidate:
        names = list(candidate)
        break
    candidate = root.get("output_features", {}).get("action", {}).get("names")
    if isinstance(candidate, list):
        names = list(candidate)
        break

if names is not None and names != expected_features:
    raise SystemExit(
        "[ERROR] policy 的动作名/顺序与关节配置不一致。\n"
        f"        期望={expected_features}\n        policy={names}"
    )
if names is None and not dim_only:
    raise SystemExit(
        "[ERROR] policy 未携带动作名，拒绝仅按维度部署。\n"
        f"        若已确认 policy 的顺序与 {joint_config} 一致，设 ALLOW_DIM_ONLY_POLICY=1。"
    )

print(f"[INFO] 动作维度: policy={action_dim} / {joint_config}={expected_dim}")
PY
else
    echo "[WARN] 找不到 $POLICY_PATH/config.json，跳过维度预检。"
fi

# ── 进入 LeRobot 工作区 ───────────────────────────────────────────────────────

cd /ubt_IL/lerobot || { echo "[ERROR] /ubt_IL/lerobot 不存在"; exit 1; }

# ── 构造 CLI 参数（镜像 rollout.sh）──────────────────────────────────────────

ROBOT_ARGS=(--robot.type=tienkung_dex --robot.bridge_enabled=true
            --robot.joint_config="$JOINT_CONFIG")
[ -n "$HAND_VARIANT" ] && ROBOT_ARGS+=(--robot.hand_variant="$HAND_VARIANT")
[ "$IMAGE_ENABLED" = "1" ] && ROBOT_ARGS+=(--robot.image_enabled=true)
[ -n "$MAX_RELATIVE_TARGET" ] && ROBOT_ARGS+=(--robot.max_relative_target="$MAX_RELATIVE_TARGET")

# ── 后台拉起服务 ──────────────────────────────────────────────────────────────

LAUNCH_CMD=(
    /lerobot/.venv/bin/python -m lerobot.scripts.inference_server
    --strategy.type="$STRATEGY"
    --policy.path="$POLICY_PATH"
    "${ROBOT_ARGS[@]}"
    --robot.cameras="{head: {type: dex_camera, server_address: '${ZMQ_HOST}', port: ${CAMERA_PORT}, offset_x: 0, width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: $FPS, display: $CAMERA_DISPLAY}}"
    --task="$TASK"
    --fps="$FPS"
    --server.cmd_port="$SERVER_CMD_PORT"
    --server.status_port="$SERVER_STATUS_PORT"
)

echo "[INFO] 拉起推理服务（后台，预热中）..."
echo "[INFO] log: $LOG_FILE"
# 把 PID_FILE 路径传给服务进程，使其优雅退出时自删 PID 文件（atexit）。
# SIGKILL/崩溃时 PID 文件残留 -> 由上方 _pid_is_running_server 守卫兜底。
# 环境变量名沿用服务端读取的 WALKER_INFERENCE_PID_FILE（上游写死的名字，不改它）。
WALKER_INFERENCE_PID_FILE="$PID_FILE" nohup "${LAUNCH_CMD[@]}" > "$LOG_FILE" 2>&1 &
SERVER_PID=$!
echo "$SERVER_PID" > "$PID_FILE"

echo "[OK] 推理服务已启动（PID $SERVER_PID）。"
echo "     正在预热模型并连机器人...（tail -f $LOG_FILE）"
echo "     cmd  port: tcp://*:$SERVER_CMD_PORT  （REP: start|stop|home|status|shutdown）"
echo "     stat port: tcp://*:$SERVER_STATUS_PORT （PUB: 状态流）"
echo "[INFO] 轮询到 READY："
echo "       bash $SCRIPT_DIR/inference_client.sh status"
echo "       bash $SCRIPT_DIR/inference_client.sh watch"
