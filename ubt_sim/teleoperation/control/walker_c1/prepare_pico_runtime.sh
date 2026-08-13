#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PICO_PYTHON:-/usr/bin/python3}"
SOURCE_DIR="${1:-/opt/pico-sdk}"
RUNTIME_DIR="${2:-/opt/pico-runtime}"

if [[ "$($PYTHON_BIN -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" != "3.10" ]]; then
    echo "[ERROR] XRoboToolkit requires CPython 3.10: $PYTHON_BIN" >&2
    exit 2
fi
if [[ ! -f "$SOURCE_DIR/libPXREARobotSDK.so" ]]; then
    echo "[ERROR] Missing $SOURCE_DIR/libPXREARobotSDK.so" >&2
    exit 2
fi

shopt -s nullglob
wheels=("$SOURCE_DIR"/xrobotoolkit_sdk-*-cp310-*.whl)
if (( ${#wheels[@]} != 1 )); then
    echo "[ERROR] Expected exactly one CPython 3.10 xrobotoolkit_sdk wheel in $SOURCE_DIR" >&2
    exit 2
fi

install -d "$RUNTIME_DIR/python"
install -m 755 "$SOURCE_DIR/libPXREARobotSDK.so" "$RUNTIME_DIR/libPXREARobotSDK.so"
"$PYTHON_BIN" -c \
    'import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])' \
    "${wheels[0]}" "$RUNTIME_DIR/python"

export LD_LIBRARY_PATH="$RUNTIME_DIR:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$RUNTIME_DIR/python:${PYTHONPATH:-}"
"$PYTHON_BIN" -c \
    'import xrobotoolkit_sdk; print("[INFO] PICO SDK import OK:", xrobotoolkit_sdk.__file__)'

echo "[INFO] PICO runtime ready: $RUNTIME_DIR"
