#!/bin/bash
# Walker C1 online-IK pick-place batch runner.
#
# Produces N successful recorded fixed-position pick-place trajectories by
# restarting the sim stack before each fresh attempt. This keeps the retry policy from the
# replay runner while executing the non-replay controller:
#   reset.py ready pose -> read /sim/object_state -> IK plan -> grasp -> plate -> ready.
#
# Usage:
#   docker exec walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_online_ik_batch.sh <N> [max_attempts_per_episode]

set -o pipefail
N="${1:-1}"
MAX_ATTEMPTS="${2:-5}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="/tmp/c1_online_ik_batch_$(date +%s)"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
export ROS_DOMAIN_ID=146

TOTAL_ATTEMPTS=0
SUCCESS_COUNT=0
EPISODE_ATTEMPT_COUNTS=()

cleanup() {
    trap - EXIT INT TERM
    pkill -9 -f '[p]ick_place_controller.py' 2>/dev/null || true
    pkill -9 -f '[s]im_runner|[w]alker_c1_ros2|[z]mq_image' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

restart_stack() {
    pkill -9 -f '[p]ick_place_controller.py' 2>/dev/null || true
    pkill -9 -f '[s]im_runner|[w]alker_c1_ros2|[z]mq_image' 2>/dev/null
    sleep 3
    ros2 daemon stop >/dev/null 2>&1
    ros2 daemon start >/dev/null 2>&1
    cd "$PROJECT_DIR"
    UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 ROS_DOMAIN_ID=146 \
        nohup bash scripts/start_sim.sh --headless --device cpu --step_hz 30 \
        > "$LOG_DIR/stack_${TOTAL_ATTEMPTS}.log" 2>&1 &
    for attempt in $(seq 1 45); do
        if /usr/bin/python3 -c "
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
node = Node('boot_probe_$$')
got = {}
node.create_subscription(String, '/sim/object_state', lambda m: got.update(ok=True), 10)
end = time.time() + 6
while time.time() < end and not got:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if got else 1)
" 2>/dev/null; then
            return 0
        fi
        sleep 10
    done
    return 1
}

for ep in $(seq 1 "$N"); do
    EP_SUCCESS=0
    for try in $(seq 1 "$MAX_ATTEMPTS"); do
        TOTAL_ATTEMPTS=$((TOTAL_ATTEMPTS + 1))
        echo "=================================================="
        echo "[BATCH] Target episode $ep/$N, attempt $try/$MAX_ATTEMPTS (global attempt $TOTAL_ATTEMPTS): restarting stack..."
        echo "=================================================="

        if ! restart_stack; then
            echo "[BATCH] Stack failed to boot, retrying..."
            continue
        fi

        echo "[BATCH] Stack ready, running online IK controller..."
        OUT=$(timeout 600 /usr/bin/python3 \
            "$PROJECT_DIR/teleoperation/control/walker_c1/pick_place_controller.py" \
            --episodes 1 --max-grasp-attempts 1 --save \
            2>&1 | tee "$LOG_DIR/episode_${ep}_try${try}.log")

        if echo "$OUT" | grep -q "SUCCESS" && echo "$OUT" | grep -q "trajectory saved:"; then
            echo "[BATCH] Target episode $ep: SUCCESS on attempt $try"
            EP_SUCCESS=1
            EPISODE_ATTEMPT_COUNTS+=("$try")
            SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
            break
        else
            echo "[BATCH] Target episode $ep: attempt $try FAILED, will restart and retry"
        fi
    done
    if [ "$EP_SUCCESS" -ne 1 ]; then
        echo "[BATCH] Target episode $ep: FAILED after $MAX_ATTEMPTS attempts"
        EPISODE_ATTEMPT_COUNTS+=("EXHAUSTED")
    fi
done

echo "=================================================="
echo "[BATCH] Summary: $SUCCESS_COUNT/$N target episodes achieved, $TOTAL_ATTEMPTS total sim attempts"
for i in "${!EPISODE_ATTEMPT_COUNTS[@]}"; do
    echo "  target episode $((i+1)): attempts-to-success = ${EPISODE_ATTEMPT_COUNTS[$i]}"
done
echo "[BATCH] Logs: $LOG_DIR"
