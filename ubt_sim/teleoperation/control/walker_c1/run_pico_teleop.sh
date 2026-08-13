#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SDK_SETUP="${WALKER_SDK_SETUP:-/opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash}"
PYTHON_BIN="${PICO_PYTHON:-/usr/bin/python3}"
LEGACY_PICO_LIB_DIR="$(cd "$SCRIPT_DIR/../../../../../xgmr_tmp/pico/pico_teleop/deps" 2>/dev/null && pwd || true)"

if [[ -z "${PICO_SDK_LIB_DIR:-}" ]]; then
    for candidate in /opt/pico-runtime /opt/pico-sdk "$LEGACY_PICO_LIB_DIR"; do
        if [[ -n "$candidate" && -f "$candidate/libPXREARobotSDK.so" ]]; then
            PICO_SDK_LIB_DIR="$candidate"
            break
        fi
    done
fi
PICO_SDK_LIB_DIR="${PICO_SDK_LIB_DIR:-}"
PICO_SDK_PYTHON_DIR="${PICO_SDK_PYTHON_DIR:-}"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
    exit 2
fi
if [[ ! -f "$SDK_SETUP" ]]; then
    echo "[ERROR] Walker SDK message setup not found: $SDK_SETUP" >&2
    exit 2
fi

set +u
source "$ROS_SETUP"
source "$SDK_SETUP"
set -u

if [[ -n "$PICO_SDK_LIB_DIR" && -f "$PICO_SDK_LIB_DIR/libPXREARobotSDK.so" ]]; then
    export LD_LIBRARY_PATH="$PICO_SDK_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi
if [[ -z "$PICO_SDK_PYTHON_DIR" && -d "$PICO_SDK_LIB_DIR/python" ]]; then
    PICO_SDK_PYTHON_DIR="$PICO_SDK_LIB_DIR/python"
fi
if [[ -n "$PICO_SDK_PYTHON_DIR" ]]; then
    export PYTHONPATH="$PICO_SDK_PYTHON_DIR:${PYTHONPATH:-}"
fi

"$PYTHON_BIN" -c "import ikpy, numpy, rclpy; from mc_task_msgs.msg import RobotCommand; from mc_state_msgs.msg import RobotState"
exec "$PYTHON_BIN" "$SCRIPT_DIR/pico_teleop.py" "$@"
