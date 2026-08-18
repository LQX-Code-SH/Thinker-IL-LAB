#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WALKER_SDK_SETUP="${WALKER_SDK_SETUP:-/opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash}"
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

if [[ -n "$PICO_SDK_LIB_DIR" && -f "$PICO_SDK_LIB_DIR/libPXREARobotSDK.so" ]]; then
    export LD_LIBRARY_PATH="$PICO_SDK_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi
if [[ -z "$PICO_SDK_PYTHON_DIR" && -d "$PICO_SDK_LIB_DIR/python" ]]; then
    PICO_SDK_PYTHON_DIR="$PICO_SDK_LIB_DIR/python"
fi
if [[ -n "$PICO_SDK_PYTHON_DIR" ]]; then
    export PYTHONPATH="$PICO_SDK_PYTHON_DIR:${PYTHONPATH:-}"
fi

mode="preview"
expect_mode_value=false
for argument in "$@"; do
    if [[ "$expect_mode_value" == true ]]; then
        mode="$argument"
        expect_mode_value=false
    elif [[ "$argument" == "--mode" ]]; then
        expect_mode_value=true
    elif [[ "$argument" == --mode=* ]]; then
        mode="${argument#--mode=}"
    fi
done

if [[ "$mode" == "sim" ]]; then
    if [[ ! -f "$ROS_SETUP" || ! -f "$WALKER_SDK_SETUP" ]]; then
        echo "[ERROR] simulation mode requires ROS and Walker message setup inside the container" >&2
        exit 2
    fi
    set +u
    source "$ROS_SETUP"
    source "$WALKER_SDK_SETUP"
    set -u
fi

"$PYTHON_BIN" -c "import numpy, xrobotoolkit_sdk"
exec "$PYTHON_BIN" -B "$SCRIPT_DIR/pico_hand_live.py" "$@"
