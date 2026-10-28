#!/bin/bash
# TienKung Dex 部署（rollout）脚本
# 在 lerobot-tienkung 容器内运行。
#
# 前置条件：
#   1) 机器人已在预备位（bash robot_ready.sh [--real]）
#   2) 仿真：ROS_DOMAIN_ID=146 的仿真已起，其 AsyncJpegCameraSender 在 5758 发 JPEG
#      真机：Bridge2 带 --enable-image，把 /ob_camera_head/color/image_raw 转到 5558
#
# 用法：
#   POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
#   JOINT_CONFIG=tienkung_dex_20 FPS=15 DURATION=60 \
#   bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
#
#   真机：
#   ROS_DOMAIN_ID=0 POLICY_PATH=... CAMERA_PORT=5558 IMAGE_ENABLED=1 \
#   bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

POLICY_PATH="${POLICY_PATH:-}"
# 关节 DOF 配置：tienkung_dex_20（右臂7+右手13，因时）/
#               tienkung_dex_brainco_13（右臂7+右手6，强脑）
JOINT_CONFIG="${JOINT_CONFIG:-tienkung_dex_20}"
# 手型，留空表示跟随 JOINT_CONFIG（推荐：两者必须一致，插件会校验）
HAND_VARIANT="${HAND_VARIANT:-}"
STRATEGY="${STRATEGY:-base}"
# 控制环频率 = 数据集帧率。数据是 15 Hz 采的，控制环跟着 15 Hz 才对得上时间轴。
FPS="${FPS:-15}"
DURATION="${DURATION:-60}"
TASK="${TASK:-pick apple}"
ZMQ_HOST="${ZMQ_HOST:-127.0.0.1}"
# 相机端口：仿真 5758（ubt_sim 直发 JPEG）／真机 5558（Bridge2 转码）
CAMERA_PORT="${CAMERA_PORT:-5758}"
CAMERA_DISPLAY="${CAMERA_DISPLAY:-true}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-360}"
# 真机要让 Bridge2 把 ROS Image 编成 JPEG（仿真不需要，相机直连 5758）
IMAGE_ENABLED="${IMAGE_ENABLED:-0}"
# 单步相对目标限幅 rad（插件侧安全层）。留空=不限制；真机首轮建议 0.01
MAX_RELATIVE_TARGET="${MAX_RELATIVE_TARGET:-}"
ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
# 推理引擎：Dex 桥接只消费单步 {"right_arm":[7],"right_hand":[N]}，没有 chunk 消费者
INFERENCE_TYPE="${INFERENCE_TYPE:-sync}"

# ── 校验 ──────────────────────────────────────────────────────────────────────

if [ -z "$POLICY_PATH" ]; then
    echo "[ERROR] POLICY_PATH 必填。"
    echo "[INFO] 示例："
    echo "       POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \\"
    echo "       JOINT_CONFIG=tienkung_dex_20 bash $0"
    exit 1
fi

if [ "$INFERENCE_TYPE" != "sync" ]; then
    echo "[ERROR] tienkung_dex 只支持 INFERENCE_TYPE=sync（当前: $INFERENCE_TYPE）。"
    echo "        act_async / rtc 会把整块 chunk 推给桥接，而 Dex 桥接只解析单步指令，"
    echo "        消息没有 right_arm/right_hand 字段会被静默丢弃（机器人不动）。"
    exit 1
fi

echo "[INFO] JOINT_CONFIG=$JOINT_CONFIG  HAND_VARIANT=${HAND_VARIANT:-<跟随 joint_config>}"
echo "[INFO] POLICY_PATH=$POLICY_PATH"
echo "[INFO] FPS=$FPS  DURATION=$DURATION  ZMQ_HOST=$ZMQ_HOST  相机端口=$CAMERA_PORT"
[ -n "$MAX_RELATIVE_TARGET" ] && echo "[INFO] 单步相对限幅 max_relative_target=$MAX_RELATIVE_TARGET rad"

# ── 预检：policy 维度 ↔ 关节配置（与 walker_s2/rollout.sh 同一套做法）────────────
#
# 名字比对对 ACT **不会触发**：ACT 的 pretrained_model/config.json 里
# output_features.action 只有 {"type":"ACTION","shape":[N]}，没有 names
# （PolicyFeature 本身就没有 names 字段）。所以对 ACT 实际只剩维度比对，
# ALLOW_DIM_ONLY_POLICY 默认 1。顺序的真正保险是：
#   TienKungDex20JointIndex 的成员顺序 == 转换配置里的 names == 数据集顺序，
# 这条一致性在插件侧由枚举定义保证，Bridge2 按关节名寻址、不按位。
# （枚举成员带 .pos 后缀、转换配置的 names 不带，比对时去掉后缀才是逐字相等。）

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

# 名字比对：目前只有带 action_feature_names 的策略类型会命中（ACT 不命中）
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

if names is not None:
    if names != expected_features:
        raise SystemExit(
            "[ERROR] policy 的动作名/顺序与关节配置不一致。\n"
            f"        期望={expected_features}\n        policy={names}"
        )
elif not dim_only:
    raise SystemExit(
        "[ERROR] policy 未携带动作名，拒绝仅按维度部署。\n"
        f"        若已确认 policy 的顺序与 {joint_config} 一致，设 ALLOW_DIM_ONLY_POLICY=1。"
    )

print(f"[INFO] JOINT_CONFIG 动作维度 : {expected_dim}")
print(f"[INFO] Policy 动作维度       : {action_dim}")
print(f"[INFO] 动作顺序              : {expected_features}")
if names is None:
    print("[WARN] ALLOW_DIM_ONLY_POLICY=1: policy 无动作名，仅按维度校验。")
PY
else
    echo "[WARN] 找不到 $POLICY_PATH/config.json，跳过维度预检。"
fi

# ── 进入 LeRobot 工作区 ───────────────────────────────────────────────────────

cd /ubt_IL/lerobot || { echo "[ERROR] /ubt_IL/lerobot 不存在"; exit 1; }

# ── Rollout ──────────────────────────────────────────────────────────────────

ROBOT_ARGS=(--robot.type=tienkung_dex --robot.bridge_enabled=true
            --robot.joint_config="$JOINT_CONFIG")
[ -n "$HAND_VARIANT" ] && ROBOT_ARGS+=(--robot.hand_variant="$HAND_VARIANT")
[ "$IMAGE_ENABLED" = "1" ] && ROBOT_ARGS+=(--robot.image_enabled=true)
[ -n "$MAX_RELATIVE_TARGET" ] && ROBOT_ARGS+=(--robot.max_relative_target="$MAX_RELATIVE_TARGET")

/lerobot/.venv/bin/lerobot-rollout \
    --strategy.type="$STRATEGY" \
    --policy.path="$POLICY_PATH" \
    "${ROBOT_ARGS[@]}" \
    --robot.cameras="{head: {type: dex_camera, server_address: '${ZMQ_HOST}', port: ${CAMERA_PORT}, offset_x: 0, width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: $FPS, display: $CAMERA_DISPLAY}}" \
    --task="$TASK" \
    --fps="$FPS" \
    --duration="$DURATION" \
    $EXTRA_ARGS
