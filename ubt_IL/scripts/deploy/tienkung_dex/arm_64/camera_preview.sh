#!/bin/bash
# 头相机预览：起 Bridge2（只订阅，不发控制指令）→ 显示 ZMQ 上的画面。
#
#   bash camera_preview.sh              # 实时窗口（需 DISPLAY，按 q 退出）
#   bash camera_preview.sh --snapshot   # 只存一张 PNG（/tmp/dex_camera_preview.png）
#   bash camera_preview.sh --size raw   # 看原始 1280x720（不缩放）
#   bash camera_preview.sh --port 5558  # 连已有的桥接，不再自己起
#
# 画面左上角叠加 fps 与分辨率，用来确认"策略看到的就是这个画面"。
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

SNAPSHOT=0
SIZE="policy"
PORT="${CAMERA_PORT:-5558}"
START_BRIDGE=1
while [ $# -gt 0 ]; do
    case "$1" in
        --snapshot) SNAPSHOT=1 ;;
        --size)     SIZE="$2"; shift ;;
        --port)     PORT="$2"; START_BRIDGE=0; shift ;;
        -h|--help)  sed -n '2,10p' "$0"; exit 0 ;;
        *) echo "[preview] 未知参数: $1" >&2; exit 2 ;;
    esac
    shift
done

LOG="${LOG:-/tmp/dex_camera_preview_bridge.log}"
OUT="${OUT:-/tmp/dex_camera_preview.png}"

if [ "$SIZE" = "raw" ]; then W=0; H=0; else W="${CAMERA_WIDTH:-640}"; H="${CAMERA_HEIGHT:-360}"; fi

if [ "$START_BRIDGE" = "1" ]; then
    pkill -f "ros2_deploy_bridge.py --config" 2>/dev/null || true
    sleep 0.5
    dex_source_ros >/dev/null
    "$SYSPY" "$BRIDGE_SCRIPT" \
        --config "{\"hand_variant\":\"brainco\",\"image_width\":${W},\"image_height\":${H},\"image_compressed\":${IMAGE_COMPRESSED:-true}}" \
        --enable-image > "$LOG" 2>&1 &
    BRIDGE_PID=$!
    trap 'kill -TERM $BRIDGE_PID 2>/dev/null || true' EXIT
    echo "[preview] Bridge2 已起 (pid $BRIDGE_PID)，日志 $LOG"
    sleep 4
fi

dex_strip_shim
"$VENV/bin/python" - "$PORT" "$SNAPSHOT" "$OUT" <<'PY'
import sys
import time

import cv2
import numpy as np
import zmq

port = int(sys.argv[1])
snapshot = sys.argv[2] == "1"
out = sys.argv[3]

ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.connect(f"tcp://127.0.0.1:{port}")
s.setsockopt_string(zmq.SUBSCRIBE, "")
s.setsockopt(zmq.RCVHWM, 1)
s.setsockopt(zmq.RCVTIMEO, 5000)

if snapshot:
    buf = s.recv()
    frame = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise SystemExit("[preview] JPEG 解码失败")
    cv2.imwrite(out, frame)
    print(f"[preview] 已保存 {out}  shape={frame.shape}")
    ctx.destroy()
    raise SystemExit(0)

n = 0
t0 = time.time()
while True:
    try:
        buf = s.recv()
    except zmq.Again:
        print("[preview] 5s 没收到帧：桥接是否在跑？端口对不对？")
        break
    frame = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        continue
    n += 1
    el = max(1e-3, time.time() - t0)
    if n % 5 == 0:
        fps = n / el
        cv2.putText(frame, f"{fps:.1f} fps  {frame.shape[1]}x{frame.shape[0]}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.imshow("dex head camera (q to quit)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
ctx.destroy()
print(f"[preview] 共 {n} 帧，平均 {n / max(1e-3, time.time() - t0):.1f} fps")
PY
