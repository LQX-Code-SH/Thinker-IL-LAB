#!/bin/bash
# Walker C1 rollout wrapper. Each episode runs a fresh LeRobot base rollout,
# matching the S2/Tienkung process lifecycle without modifying LeRobot core.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-146}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/ubt_IL/docker/fastdds_no_shm.xml}"
export ROS2CLI_DISABLE_DAEMON=1
export ROBOT_MODEL="walker_c1_26d"
export ROBOT_CONFIG="${ROBOT_CONFIG:-$SCRIPT_DIR/configs/walker/walker_c1_26d.json}"
export ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
export STRATEGY="base"
export FPS="${FPS:-30}"
export DURATION="${DURATION:-30}"
export TASK="${TASK:-walker c1 pick apple and place on plate}"
export PREVIEW_CAMERA="${PREVIEW_CAMERA:-0}"

EPISODES="${EPISODES:-1}"
EVAL_SEED="${EVAL_SEED:-1000}"
RANDOMIZE_APPLE="${RANDOMIZE_APPLE:-1}"
APPLE_CENTER_X="${APPLE_CENTER_X:-8.17}"
APPLE_CENTER_Y="${APPLE_CENTER_Y:-5.86}"
APPLE_HALF_EXTENT="${APPLE_HALF_EXTENT:-0.025}"
APPLE_Z="${APPLE_Z:-0.9421}"
PLATE_CENTER_X="${PLATE_CENTER_X:-8.19}"
PLATE_CENTER_Y="${PLATE_CENTER_Y:-6.083}"
SUCCESS_RADIUS="${SUCCESS_RADIUS:-0.12}"

usage() {
    cat <<'EOF'
Usage:
  bash rollout_walker_c1.sh
  bash rollout_walker_c1.sh --episodes N [options]

The default is one episode. Multiple episodes use an outer process loop:
each episode places the apple, starts a fresh LeRobot base rollout, reloads
the policy/Bridge2/camera, then checks the final apple position.

Options:
  --episodes N         Number of independently loaded rollouts (default: 1).
  --duration SECONDS   Maximum control time per rollout (default: 30).
  --seed N             Random apple-position seed (default: 1000).
  --fixed-apple        Place at the collection-region center.
  --randomize-apple    Randomize in the configured 5 cm x 5 cm square (default).
  --success-radius M   Apple-to-plate XY success radius (default: 0.12).
  -h, --help           Show this help.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --episodes)
            [ "$#" -ge 2 ] || { echo "[ERROR] --episodes requires a value" >&2; exit 2; }
            EPISODES="$2"
            shift 2
            ;;
        --duration)
            [ "$#" -ge 2 ] || { echo "[ERROR] --duration requires a value" >&2; exit 2; }
            DURATION="$2"
            export DURATION
            shift 2
            ;;
        --seed)
            [ "$#" -ge 2 ] || { echo "[ERROR] --seed requires a value" >&2; exit 2; }
            EVAL_SEED="$2"
            shift 2
            ;;
        --fixed-apple)
            RANDOMIZE_APPLE=0
            shift
            ;;
        --randomize-apple)
            RANDOMIZE_APPLE=1
            shift
            ;;
        --success-radius)
            [ "$#" -ge 2 ] || { echo "[ERROR] --success-radius requires a value" >&2; exit 2; }
            SUCCESS_RADIUS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ "$EPISODES" =~ ^[1-9][0-9]*$ ]] || {
    echo "[ERROR] episodes must be a positive integer" >&2
    exit 2
}
[[ "$EVAL_SEED" =~ ^-?[0-9]+$ ]] || {
    echo "[ERROR] seed must be an integer" >&2
    exit 2
}
[[ "$RANDOMIZE_APPLE" = "0" || "$RANDOMIZE_APPLE" = "1" ]] || {
    echo "[ERROR] RANDOMIZE_APPLE must be 0 or 1" >&2
    exit 2
}
[ -n "${POLICY_PATH:-}" ] || {
    echo "[ERROR] POLICY_PATH is required" >&2
    exit 2
}

set +u
source /opt/ros/humble/setup.bash
source /ubt_IL/walker/walker_sdk_ros2/install/setup.bash
set -u

read_object_position() {
    local state
    state="$(
        timeout 6 ros2 topic echo --once --no-daemon --field data \
            /sim/object_state std_msgs/msg/String 2>/dev/null | sed -n '1p'
    )" || return 1
    [ -n "$state" ] || return 1
    /lerobot/.venv/bin/python -c '
import json, sys
xyz = json.loads(sys.argv[1]).get("object_pos_w")
if not xyz or len(xyz) < 3:
    raise SystemExit(1)
print(*(float(v) for v in xyz[:3]))
' "$state"
}

apple_target() {
    /lerobot/.venv/bin/python -c '
import random, sys
seed, episode = int(sys.argv[1]), int(sys.argv[2])
cx, cy, half = map(float, sys.argv[3:6])
rng = random.Random(seed + episode)
if sys.argv[6] == "1":
    x = cx + rng.uniform(-half, half)
    y = cy + rng.uniform(-half, half)
else:
    x, y = cx, cy
print(f"{x:.9f} {y:.9f}")
' "$EVAL_SEED" "$1" "$APPLE_CENTER_X" "$APPLE_CENTER_Y" \
        "$APPLE_HALF_EXTENT" "$RANDOMIZE_APPLE"
}

place_apple() {
    local target_x="$1"
    local target_y="$2"
    local attempt observed_x observed_y observed_z

    for attempt in 1 2 3; do
        if ! timeout 10 ros2 topic pub --once \
            /sim/cmd_set_object_pose geometry_msgs/msg/Point \
            "{x: $target_x, y: $target_y, z: $APPLE_Z}" >/dev/null 2>&1; then
            echo "[WARN] apple publish attempt $attempt failed" >&2
            continue
        fi
        sleep 0.5
        if read -r observed_x observed_y observed_z < <(read_object_position); then
            if /lerobot/.venv/bin/python -c '
import math, sys
x, y, z, tx, ty = map(float, sys.argv[1:])
raise SystemExit(0 if z > 0.9 and math.hypot(x - tx, y - ty) <= 0.12 else 1)
' "$observed_x" "$observed_y" "$observed_z" "$target_x" "$target_y"; then
                printf '[INFO] apple ready: target=[%.3f, %.3f, %.4f], observed=[%.3f, %.3f, %.4f]\n' \
                    "$target_x" "$target_y" "$APPLE_Z" \
                    "$observed_x" "$observed_y" "$observed_z"
                return 0
            fi
        fi
        echo "[WARN] apple placement attempt $attempt was not observed at the target" >&2
    done
    return 1
}

episode_success() {
    local observed_x observed_y observed_z
    read -r observed_x observed_y observed_z < <(read_object_position) || return 2
    /lerobot/.venv/bin/python -c '
import math, sys
x, y, z, px, py, radius = map(float, sys.argv[1:])
distance = math.hypot(x - px, y - py)
print(f"[INFO] final apple=[{x:.3f}, {y:.3f}, {z:.4f}], plate_dist={distance:.3f}")
raise SystemExit(0 if z > 0.9 and distance <= radius else 1)
' "$observed_x" "$observed_y" "$observed_z" \
        "$PLATE_CENTER_X" "$PLATE_CENTER_Y" "$SUCCESS_RADIUS"
}

successes=0
completed=0

for ((episode = 1; episode <= EPISODES; episode++)); do
    echo "[INFO] === independently loaded rollout $episode/$EPISODES ==="
    read -r apple_x apple_y < <(apple_target "$episode")

    if ! place_apple "$apple_x" "$apple_y"; then
        echo "[ERROR] simulator did not place the apple; stopping before policy startup" >&2
        exit 1
    fi

    if ! bash "$SCRIPT_DIR/rollout_walker.sh"; then
        echo "[ERROR] rollout process exited with an error; stopping evaluation" >&2
        exit 1
    fi

    completed=$((completed + 1))
    if episode_success; then
        successes=$((successes + 1))
        echo "[INFO] episode $episode SUCCESS"
    else
        success_check_rc=$?
        if [ "$success_check_rc" -eq 2 ]; then
            echo "[ERROR] could not read final object state; stopping evaluation" >&2
            exit 1
        fi
        echo "[INFO] episode $episode FAILURE"
    fi
    rate="$(
        /lerobot/.venv/bin/python -c \
            'import sys; print(f"{100*int(sys.argv[1])/int(sys.argv[2]):.1f}")' \
            "$successes" "$completed"
    )"
    echo "[INFO] running summary: $successes/$completed success ($rate%)"
done

rate="$(
    /lerobot/.venv/bin/python -c \
        'import sys; print(f"{100*int(sys.argv[1])/int(sys.argv[2]):.1f}")' \
        "$successes" "$completed"
)"
echo "[INFO] Summary: $successes/$completed success ($rate%)"
