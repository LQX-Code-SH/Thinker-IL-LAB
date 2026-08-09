#!/bin/bash
# realsense_wrist_camera — one-command start (runs in background by default)
#
# Usage:
#   bash start.sh                 # background start
#   bash start.sh --fg            # foreground (Ctrl+C to stop)
#   bash start.sh --stop           # stop running background service
#   bash start.sh --status         # check if service is running

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_FILE="${PKG_DIR}/configs/wrist_cameras.json"
PID_FILE="/tmp/realsense_wrist_camera.pid"
LOG_FILE="/tmp/realsense_wrist_camera.log"

# --stop: kill running service
if [ "${1:-}" = "--stop" ]; then
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if kill -0 "$PID" 2>/dev/null; then
            echo "[start_realsense_wrist_camera] Stopping (PID=$PID)..."
            kill "$PID"
            rm -f "$PID_FILE"
            echo "[start_realsense_wrist_camera] Stopped."
        else
            echo "[start_realsense_wrist_camera] PID $PID is not running. Removing stale PID file."
            rm -f "$PID_FILE"
        fi
    else
        echo "[start_realsense_wrist_camera] No PID file found (service not running)."
    fi
    exit 0
fi

# --status: check if running
if [ "${1:-}" = "--status" ]; then
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if kill -0 "$PID" 2>/dev/null; then
            echo "[start_realsense_wrist_camera] Running (PID=$PID)."
            exit 0
        fi
    fi
    echo "[start_realsense_wrist_camera] Not running."
    exit 1
fi

echo "[start_realsense_wrist_camera] Setting up environment..."

# ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[start_realsense_wrist_camera] WARNING: /opt/ros/humble/setup.bash not found"
fi

# Walker ROS2 messages
WALKER_WS="/ubt_IL/walker/walker_sdk_ros2"
if [ -f "${WALKER_WS}/install/setup.bash" ]; then
    source "${WALKER_WS}/install/setup.bash"
fi

# Docker 环境自洽：docker exec 新会话不继承 entrypoint 的 export，这里显式补齐。
# ROS_DOMAIN_ID 优先用已有值，否则取容器 env 的 DOMAIN_ID（run.sh -e 传入），否则 0。
# FastRTPS 禁用共享内存传输（Docker 内必需，即使 --network=host）。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${DOMAIN_ID:-0}}"
if [ -f /opt/fastdds_no_shm.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_no_shm.xml
fi

# Ensure --user install CLI scripts are on PATH
if [ -d "$HOME/.local/bin" ]; then
    export PATH="$HOME/.local/bin:$PATH"
fi

# Separate --fg from real camera arguments (--stop/--status already handled above)
FG_MODE=0
REAL_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--fg" ]; then
        FG_MODE=1
    else
        REAL_ARGS+=("$arg")
    fi
done

# Build command
CMD=(/usr/bin/python3 -m realsense_wrist_camera)
if [ ${#REAL_ARGS[@]} -gt 0 ]; then
    CMD+=("${REAL_ARGS[@]}")
elif [ -f "$CONFIG_FILE" ]; then
    echo "[start_realsense_wrist_camera] Using config: ${CONFIG_FILE}"
    CMD+=(--config "$CONFIG_FILE")
else
    echo "[start_realsense_wrist_camera] No config found, auto-discovering..."
    CMD+=(--discover)
fi

# Foreground mode
if [ "$FG_MODE" = "1" ]; then
    echo "[start_realsense_wrist_camera] Starting in foreground (Ctrl+C to stop)..."
    exec "${CMD[@]}"
fi

# Background mode (default)
echo "[start_realsense_wrist_camera] Starting in background..."
nohup "${CMD[@]}" > "$LOG_FILE" 2>&1 &
PID=$!
echo "$PID" > "$PID_FILE"

sleep 1.5
if kill -0 "$PID" 2>/dev/null; then
    echo "[start_realsense_wrist_camera] Running (PID=$PID)."
    echo "[start_realsense_wrist_camera] Log:  $LOG_FILE"
    echo "[start_realsense_wrist_camera] Stop:  bash $0 --stop"
else
    echo "[start_realsense_wrist_camera] FAILED to start. Check log: $LOG_FILE"
    rm -f "$PID_FILE"
    exit 1
fi
