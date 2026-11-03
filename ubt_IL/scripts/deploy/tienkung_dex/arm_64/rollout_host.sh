#!/bin/bash
# TienKung Dex 真机 rollout（host 版，Thor / Jazzy / 不进容器）。
#
# 前置：
#   1) bash setup_env.sh 已完成
#   2) 机器人已到预备位：bash robot_ready_host.sh --real
#      （robot_ready 与 Bridge2 都写 /arm/cmd，二者不能同时跑；先 ready 再 rollout）
#   3) 遥控器已切到话题控制模式（否则 /arm/cmd 无人消费，机器人不动）
#
# 用法：
#   POLICY_PATH=/abs/path/pretrained_model bash rollout_host.sh
#   DURATION=10 MAX_RELATIVE_TARGET=0.01 bash rollout_host.sh      # 首轮建议
#   DISPLAY_CAM=true bash rollout_host.sh                          # 本机有 X 时弹窗预览
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

# ── 配置（全部可环境变量覆盖）───────────────────────────────────────────────
POLICY_PATH="${POLICY_PATH:-}"                       # 必填
JOINT_CONFIG="${JOINT_CONFIG:-tienkung_dex_brainco_13}"   # 实物强脑：右臂7 + 右手6
# 因时手 r13 数据集改用：JOINT_CONFIG=tienkung_dex_inspire_13（自动 pico6 展开）
HAND_VARIANT="${HAND_VARIANT:-}"                     # 留空跟随 joint_config
HAND_ACTION_MODE="${HAND_ACTION_MODE:-}"             # 留空 = 由 JOINT_CONFIG 自动推导
STRATEGY="${STRATEGY:-base}"
FPS="${FPS:-15}"                                     # = 数据集帧率
DURATION="${DURATION:-60}"
TASK="${TASK:-pick apple}"
ZMQ_HOST="${ZMQ_HOST:-127.0.0.1}"
CAMERA_PORT="${CAMERA_PORT:-5558}"                   # 真机由 Bridge2 转码后发这里
IMAGE_ENABLED="${IMAGE_ENABLED:-1}"
IMAGE_COMPRESSED="${IMAGE_COMPRESSED:-1}"            # 走 .../image_raw/compressed，帧率高一倍
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-360}"
DISPLAY_CAM="${DISPLAY_CAM:-false}"                  # SSH 无 X 保持 false
MAX_RELATIVE_TARGET="${MAX_RELATIVE_TARGET:-}"       # 首轮 0.01；确认后置空
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
INFERENCE_TYPE="${INFERENCE_TYPE:-sync}"             # Dex 桥接只消费单步动作

# ── 校验 ────────────────────────────────────────────────────────────────────
[ -n "$POLICY_PATH" ] || {
    echo "[rollout] ERROR: POLICY_PATH 必填（checkpoint 的 pretrained_model 目录）" >&2
    echo "[rollout]   例: POLICY_PATH=$PROJECT_ROOT/model/tienkung_dex_brainco_pick_act/checkpoints/last/pretrained_model" >&2
    exit 1
}
[ -e "$POLICY_PATH" ] || { echo "[rollout] ERROR: POLICY_PATH 不存在: $POLICY_PATH" >&2; exit 1; }
[ -f "$POLICY_PATH/config.json" ] || echo "[rollout] WARN: 缺 $POLICY_PATH/config.json，跳过维度预检"
[ "$INFERENCE_TYPE" = "sync" ] || {
    echo "[rollout] ERROR: tienkung_dex 只支持 INFERENCE_TYPE=sync（当前 $INFERENCE_TYPE）" >&2
    echo "[rollout]        async/rtc 推整块 chunk，Dex 桥接只解析单步，消息会被静默丢弃" >&2
    exit 1
}
[ -x "$VENV/bin/lerobot-rollout" ] || { echo "[rollout] ERROR: venv 未就绪，先跑 setup_env.sh" >&2; exit 1; }

# Bridge2 是子进程，靠父进程 env 继承 ROS（bash -lc 不会 source .bashrc）
dex_source_ros
dex_strip_shim          # 层 A 不需要 IDE shim；ROS 的 PYTHONPATH 保留给 Bridge2
dex_check_msgs

echo "[rollout] POLICY_PATH : $POLICY_PATH"
echo "[rollout] JOINT_CONFIG: $JOINT_CONFIG  (hand=${HAND_VARIANT:-<跟随>})"
echo "[rollout] FPS=$FPS  DURATION=$DURATION  CMD=∠$ZMQ_HOST:$CAMERA_PORT"
echo "[rollout] 相机: ${CAMERA_WIDTH}x${CAMERA_HEIGHT}  display=$DISPLAY_CAM  image_enabled=$IMAGE_ENABLED"
[ -n "$MAX_RELATIVE_TARGET" ] && echo "[rollout] 单步限幅 max_relative_target=$MAX_RELATIVE_TARGET rad"

# ── 维度预检：policy ↔ JOINT_CONFIG（名称比对对 ACT 不触发，只比维度）────────
if [ -f "$POLICY_PATH/config.json" ]; then
    echo "[rollout] 预检 policy 维度 ..."
    "$VENV/bin/python" - "$JOINT_CONFIG" "$POLICY_PATH/config.json" "$ALLOW_DIM_ONLY_POLICY" <<'PY'
import json, sys
joint_config, pc_path, dim_only = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
from lerobot_robot_tienkung_dex.constants import JOINT_INDEX_ENUMS, joint_names_with_pos

if joint_config not in JOINT_INDEX_ENUMS:
    raise SystemExit(f"[rollout] ERROR: {joint_config!r} 未注册，可用 {list(JOINT_INDEX_ENUMS)}")
expected = joint_names_with_pos(JOINT_INDEX_ENUMS[joint_config])

policy = json.load(open(pc_path, encoding="utf-8"))
shape = (policy.get("output_features") or policy.get("policy", {}).get("output_features") or {}) \
    .get("action", {}).get("shape")
if not shape:
    raise SystemExit("[rollout] ERROR: policy config.json 里找不到 output action shape")
action_dim = int(shape[0])
if action_dim != len(expected):
    raise SystemExit(
        f"[rollout] ERROR: 动作维度不匹配: JOINT_CONFIG={joint_config} 期望 {len(expected)}，policy {action_dim}"
    )
print(f"[rollout] 维度校验通过: {action_dim} 维")
PY
fi

# ── 组装参数 ────────────────────────────────────────────────────────────────
ROBOT_ARGS=(
    --robot.type=tienkung_dex
    --robot.bridge_enabled=true
    --robot.bridge_script="$BRIDGE_SCRIPT"        # 覆盖插件里写死的 /ubt_IL/...
    --robot.joint_config="$JOINT_CONFIG"
    --robot.image_enabled="${IMAGE_ENABLED}"
    --robot.image_port="${CAMERA_PORT}"
    --robot.image_width="${CAMERA_WIDTH}"         # Bridge2 编码前缩放（720p→360p）
    --robot.image_height="${CAMERA_HEIGHT}"
    --robot.image_compressed="${IMAGE_COMPRESSED}"
)
[ -n "$HAND_VARIANT" ] && ROBOT_ARGS+=(--robot.hand_variant="$HAND_VARIANT")
[ -n "$HAND_ACTION_MODE" ] && ROBOT_ARGS+=(--robot.hand_action_mode="$HAND_ACTION_MODE")
[ -n "$MAX_RELATIVE_TARGET" ] && ROBOT_ARGS+=(--robot.max_relative_target="$MAX_RELATIVE_TARGET")

cd "$PROJECT_ROOT/lerobot"

exec "$VENV/bin/lerobot-rollout" \
    --strategy.type="$STRATEGY" \
    --policy.path="$POLICY_PATH" \
    "${ROBOT_ARGS[@]}" \
    --robot.cameras="{head: {type: dex_camera, server_address: '${ZMQ_HOST}', port: ${CAMERA_PORT}, offset_x: 0, width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${FPS}, display: ${DISPLAY_CAM}}}" \
    --task="$TASK" \
    --fps="$FPS" \
    --duration="$DURATION"
