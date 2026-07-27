#!/bin/bash
# Walker C1 rollout wrapper. Each episode runs a fresh LeRobot rollout process;
# its C1 strategy places the apple only after policy/robot/camera setup.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-146}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/ubt_IL/docker/fastdds_no_shm.xml}"
export ROS2CLI_DISABLE_DAEMON=1
export ROBOT_MODEL="walker_c1_26d"
export ROBOT_CONFIG="${ROBOT_CONFIG:-$SCRIPT_DIR/configs/walker/walker_c1_26d.json}"
export ALLOW_DIM_ONLY_POLICY="${ALLOW_DIM_ONLY_POLICY:-1}"
export STRATEGY="walker_c1_once"
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
APPLE_PLACEMENT_TOLERANCE="${APPLE_PLACEMENT_TOLERANCE:-0.02}"
ROBOT_READY_TOLERANCE="${ROBOT_READY_TOLERANCE:-0.08}"
ROBOT_STOP_VELOCITY="${ROBOT_STOP_VELOCITY:-0.10}"
ROBOT_STABLE_SAMPLES="${ROBOT_STABLE_SAMPLES:-3}"
ROBOT_READY_TIMEOUT="${ROBOT_READY_TIMEOUT:-20}"
PLATE_CENTER_X="${PLATE_CENTER_X:-8.19}"
PLATE_CENTER_Y="${PLATE_CENTER_Y:-6.083}"
SUCCESS_RADIUS="${SUCCESS_RADIUS:-0.12}"

usage() {
    cat <<'EOF'
Usage:
  bash rollout_walker_c1.sh
  bash rollout_walker_c1.sh --episodes N [options]

The default is one episode. Multiple episodes use an outer process loop.
Each process reloads the policy/Bridge2/camera, places the apple immediately
before control starts, then checks the final apple position.

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

read_object_sample() {
    local state
    state="$(
        timeout 6 ros2 topic echo --once --no-daemon --field data \
            /sim/object_state std_msgs/msg/String 2>/dev/null | sed -n '1p'
    )" || return 1
    [ -n "$state" ] || return 1
    /lerobot/.venv/bin/python -c '
import json, sys
data = json.loads(sys.argv[1])
xyz = data.get("object_pos_w")
if not xyz or len(xyz) < 3:
    raise SystemExit(1)
names = data.get("joint_names") or []
velocity = data.get("joint_vel_probe") or []
upper_body_velocity = [
    abs(float(value))
    for name, value in zip(names, velocity)
    if name.startswith((
        "L_shoulder_", "R_shoulder_",
        "L_elbow_", "R_elbow_",
        "L_wrist_", "R_wrist_",
        "head_", "waist_",
    ))
]
if not upper_body_velocity:
    raise SystemExit(1)
print(
    *(float(v) for v in xyz[:3]),
    int(data.get("sim_step", -1)),
    max(upper_body_velocity),
)
' "$state"
}

read_object_position() {
    local observed_x observed_y observed_z _sim_step _max_velocity
    read -r observed_x observed_y observed_z _sim_step _max_velocity \
        < <(read_object_sample) || return 1
    printf '%s %s %s\n' "$observed_x" "$observed_y" "$observed_z"
}

read_robot_ready_error() {
    local state
    state="$(
        timeout 6 ros2 topic echo --once --no-daemon \
            /mc/sdk/robot_state mc_state_msgs/msg/RobotState 2>/dev/null
    )" || return 1
    [ -n "$state" ] || return 1
    /lerobot/.venv/bin/python -c '
import json, sys, yaml

state = next(yaml.safe_load_all(sys.argv[1]))
joint_state = state.get("joint_states") or {}
position = dict(zip(joint_state.get("name") or [], joint_state.get("position") or []))
with open(sys.argv[2], encoding="utf-8") as stream:
    ready = dict(json.load(stream)["body"]["home"])
ready.update({
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.50,
    "waist_yaw_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "waist_roll_joint": 0.0,
})
missing = [name for name in ready if name not in position]
if missing:
    raise SystemExit(1)
print(max(abs(float(position[name]) - target) for name, target in ready.items()))
' "$state" "$ROBOT_CONFIG"
}

wait_robot_ready_and_stopped() {
    local deadline=$((SECONDS + ROBOT_READY_TIMEOUT))
    local stable_count=0
    local last_step=-1
    local observed_x observed_y observed_z sim_step max_velocity ready_error

    echo "[INFO] waiting for the robot to be ready and stationary before apple placement ..."
    while [ "$SECONDS" -lt "$deadline" ]; do
        if ! read -r observed_x observed_y observed_z sim_step max_velocity \
            < <(read_object_sample); then
            continue
        fi
        if [ "$sim_step" -le "$last_step" ]; then
            continue
        fi
        last_step="$sim_step"
        if /lerobot/.venv/bin/python -c '
import sys
raise SystemExit(0 if float(sys.argv[1]) <= float(sys.argv[2]) else 1)
' "$max_velocity" "$ROBOT_STOP_VELOCITY"; then
            stable_count=$((stable_count + 1))
        else
            stable_count=0
        fi
        if [ "$stable_count" -lt "$ROBOT_STABLE_SAMPLES" ]; then
            continue
        fi
        if ready_error="$(read_robot_ready_error)" &&
            /lerobot/.venv/bin/python -c '
import sys
raise SystemExit(0 if float(sys.argv[1]) <= float(sys.argv[2]) else 1)
' "$ready_error" "$ROBOT_READY_TOLERANCE"; then
            printf '[INFO] robot ready: max_joint_error=%.4f rad, max_velocity=%.4f rad/s\n' \
                "$ready_error" "$max_velocity"
            return 0
        fi
        stable_count=0
    done
    echo "[ERROR] robot did not reach a stationary ready pose before timeout" >&2
    return 1
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

    if ! wait_robot_ready_and_stopped; then
        echo "[ERROR] stopping before moving the apple" >&2
        exit 1
    fi

    if ! C1_APPLE_X="$apple_x" \
        C1_APPLE_Y="$apple_y" \
        C1_APPLE_Z="$APPLE_Z" \
        C1_PLACEMENT_TOLERANCE="$APPLE_PLACEMENT_TOLERANCE" \
        bash "$SCRIPT_DIR/rollout_walker.sh"; then
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
