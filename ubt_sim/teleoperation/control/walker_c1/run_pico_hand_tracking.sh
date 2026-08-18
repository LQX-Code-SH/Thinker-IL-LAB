#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
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

for argument in "$@"; do
    if [[ "$argument" == "--ros-markers" ]]; then
        if [[ ! -f "$ROS_SETUP" ]]; then
            echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
            exit 2
        fi
        set +u
        source "$ROS_SETUP"
        set -u
        break
    fi
done

"$PYTHON_BIN" -c "import numpy, xrobotoolkit_sdk"
exec "$PYTHON_BIN" "$SCRIPT_DIR/pico_hand_debug.py" "$@"
