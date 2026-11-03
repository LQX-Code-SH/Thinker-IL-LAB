#!/bin/bash
# 只读冒烟：验证 Bridge2 ↔ 真机 DDS ↔ 头相机 三条链路，不下发任何控制指令。
# 适合在预备位/rollout 之前跑，也可作为"今天机器还通不通"的体检。
#
# 用法：
#   bash bridge_smoke.sh             # 8 秒采样
#   SMOKE_SECONDS=15 bash bridge_smoke.sh
#
# 判据（脚本会自己给结论）：
#   status ≥ 30 Hz、image ≥ 10 fps、estop=false、无消息包缺失
#   （状态在桥接侧限频到 60Hz：/robot_state 是 200Hz，全量转发会把执行线程占满）
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

DUR="${SMOKE_SECONDS:-8}"   # 注意别用 SECONDS：那是 bash 内建（shell 已运行秒数）
LOG="${LOG:-/tmp/dex_bridge_smoke.log}"

dex_source_ros
dex_check_msgs

echo "[smoke] 启动 Bridge2（只订阅，不发布控制指令）..."
"$SYSPY" "$BRIDGE_SCRIPT" \
    --config "{\"hand_variant\":\"brainco\",\"image_width\":${CAMERA_WIDTH:-640},\"image_height\":${CAMERA_HEIGHT:-360},\"image_compressed\":${IMAGE_COMPRESSED:-true}}" \
    --enable-image > "$LOG" 2>&1 &
BRIDGE_PID=$!
trap 'kill -TERM $BRIDGE_PID 2>/dev/null || true' EXIT
sleep 4

echo "[smoke] 采样 ${DUR}s：ZMQ 5560(状态) / 5558(图像) ..."
"$SYSPY" - "$DUR" <<'PY'
import json, sys, time
import zmq

dur = float(sys.argv[1])
ctx = zmq.Context()
sub_status = ctx.socket(zmq.SUB)
sub_status.connect("tcp://127.0.0.1:5560")
sub_status.setsockopt_string(zmq.SUBSCRIBE, "")

sub_img = ctx.socket(zmq.SUB)
sub_img.connect("tcp://127.0.0.1:5558")
sub_img.setsockopt_string(zmq.SUBSCRIBE, "")

poller = zmq.Poller()
poller.register(sub_status, zmq.POLLIN)
poller.register(sub_img, zmq.POLLIN)

n_status = n_img = 0
last = None
shape = None
t0 = time.time()
while time.time() - t0 < dur:
    for sock, _ in poller.poll(500):
        if sock is sub_status:
            n_status += 1
            last = json.loads(sub_status.recv())
        else:
            n_img += 1
            raw = sub_img.recv()
            if shape is None:
                import cv2, numpy as np
                fr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                shape = None if fr is None else fr.shape

elapsed = time.time() - t0
hz_s = n_status / elapsed
fps_i = n_img / elapsed
print(f"[smoke] status {n_status} msgs  ({hz_s:.1f} Hz)")
print(f"[smoke] image  {n_img} frames ({fps_i:.1f} fps)  shape={shape}")
if last:
    arm = last.get("right_arm") or []
    hand = last.get("right_hand") or []
    print(f"[smoke] right_arm[{len(arm)}] first3={[round(v,3) for v in arm[:3]]}")
    print(f"[smoke] right_hand[{len(hand)}] = {[round(v,3) for v in hand]}   estop={last.get('estop')}")

ok = hz_s >= 30 and fps_i >= 10 and last is not None and last.get("estop") is False
print("[smoke] => " + ("通路 OK" if ok else "未达标，看日志: " + __import__("os").environ.get("LOG", "/tmp/dex_bridge_smoke.log")))
sys.exit(0 if ok else 2)
PY
RC=$?

kill -TERM "$BRIDGE_PID" 2>/dev/null || true
wait "$BRIDGE_PID" 2>/dev/null || true
echo "[smoke] Bridge2 日志尾部："; tail -6 "$LOG"
exit $RC
