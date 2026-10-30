#!/bin/bash
# UBT Sim unified launcher — auto-detects robot type from UBT_SIM_TASK.
#
# Usage:
#   bash scripts/start_sim.sh                          # default: Tienkung Pro
#   UBT_SIM_TASK=UBTSim-WalkerS2-PartSorting-v0 bash scripts/start_sim.sh
#   UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 bash scripts/start_sim.sh
#   UBT_SIM_NO_BRIDGE=1 bash scripts/start_sim.sh       # skip ZMQ bridge
#   UBT_SIM_LOAD_ONLY=1 bash scripts/start_sim.sh        # scene preview only
#
# 注意：Walker C1 与 Walker S2 的 ZMQ 端口区间相同（5655-5658），二者不能同时运行。
#
# 话题与真机一致，仿真/真机只靠 ROS_DOMAIN_ID 区分（仿真默认 146，真机 0）。

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# --- Resolve defaults ---
UBT_SIM_TASK="${UBT_SIM_TASK:-UBTSim-TienkungPro-Parlor-v0}"
UBT_SIM_NUM_ENVS="${UBT_SIM_NUM_ENVS:-1}"

# Detect robot from task name
if [[ "$UBT_SIM_TASK" == *"WalkerC1"* ]]; then
    ROBOT="walker_c1"
elif [[ "$UBT_SIM_TASK" == *"WalkerS2"* ]]; then
    ROBOT="walker_s2"
elif [[ "$UBT_SIM_TASK" == *"TienkungDex"* ]]; then
    ROBOT="tienkung_dex"
else
    ROBOT="tienkung_pro"
fi

# Hand variant for TienKung Dex task names
DEX_VARIANT="inspire"
if [[ "$UBT_SIM_TASK" == *"Brainco"* ]]; then
    DEX_VARIANT="brainco"
fi
BRIDGE_PID=""

cleanup() {
    if [ -n "$BRIDGE_PID" ]; then
        echo "[INFO] Stopping ${ROBOT} ROS2-ZMQ bridge (PID=$BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null
        for i in 1 2 3; do
            if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        if kill -0 "$BRIDGE_PID" 2>/dev/null; then
            kill -9 "$BRIDGE_PID" 2>/dev/null
        fi
        wait "$BRIDGE_PID" 2>/dev/null
    fi
    exit 0
}
trap cleanup EXIT INT TERM

# --- Auto-start ROS2-ZMQ bridge (unless disabled) ---
if [ -z "${UBT_SIM_NO_BRIDGE:-}" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        # ROS 2 setup.bash references unset vars (AMENT_TRACE_SETUP_FILES);
        # temporarily disable -u while sourcing.
        set +u
        source /opt/ros/humble/setup.bash
        set -u
        # 仿真默认 146，真机默认 0——话题完全一样，只靠 domain 分流
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-146}"

        if [ "$ROBOT" = "walker_c1" ]; then
            # Walker C1 使用独立的消息工作空间（与 Walker S2 的同名包隔离）
            if [ -f /opt/ubt_sim/walker_c1_msgs/install/setup.bash ]; then
                set +u
                source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
                set -u
            else
                echo "[ERROR] Walker C1 桥接需要独立消息包: /opt/ubt_sim/walker_c1_msgs/install/setup.bash"
                echo "[INFO] 容器内先执行: cd /ubt_sim/docker && bash run.sh init"
                exit 1
            fi
            # RMW 按镜像实际安装的实现选择：本仓库镜像只装了 cyclonedds
            # （若手动 export RMW_IMPLEMENTATION 则沿用）
            if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
                if [ -f /opt/ros/humble/lib/librmw_fastrtps_cpp.so ]; then
                    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
                else
                    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
                fi
            fi
            BRIDGE_CFG="${UBT_SIM_WALKER_C1_BRIDGE_CONFIG:-$PROJECT_DIR/teleoperation/bridges/walker_c1/walker_c1_bridge_config.yaml}"
            /usr/bin/python3 "$PROJECT_DIR/teleoperation/bridges/walker_c1/walker_c1_ros2_zmq_bridge.py" \
                --config "$BRIDGE_CFG" &
        elif [ "$ROBOT" = "walker_s2" ]; then
            # Walker S2: SDK message packages are required
            if [ -f /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash ]; then
                set +u
                source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
                set -u
            else
                echo "[ERROR] Walker SDK ROS2 messages not built: /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash"
                echo "[INFO] Run inside container: cd /ubt_sim/docker && bash run.sh init"
                exit 1
            fi
            export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
            BRIDGE_CFG="${UBT_SIM_WALKER_S2_BRIDGE_CONFIG:-$PROJECT_DIR/teleoperation/bridges/walker_s2/walker_s2_bridge_config.yaml}"
            /usr/bin/python3 "$PROJECT_DIR/teleoperation/bridges/walker_s2/walker_s2_ros2_zmq_bridge.py" \
                --config "$BRIDGE_CFG" &
        elif [ "$ROBOT" = "tienkung_dex" ]; then
            # 真机同款消息包（ros2_bridge_msgs + 灵巧手），run.sh init 编译
            if [ -f /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash ]; then
                set +u
                source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash
                set -u
            else
                echo "[ERROR] TienKung Dex 桥接需要消息包: /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash"
                echo "[INFO] 容器内先执行: cd /ubt_sim/docker && bash run.sh init"
                exit 1
            fi
            BRIDGE_CFG="${UBT_SIM_TIENKUNG_DEX_BRIDGE_CONFIG:-$PROJECT_DIR/teleoperation/bridges/tienkung_dex/bridge_config.yaml}"
            /usr/bin/python3 "$PROJECT_DIR/teleoperation/bridges/tienkung_dex/tienkung_dex_ros2_zmq_bridge.py" \
                --config "$BRIDGE_CFG" --variant "$DEX_VARIANT" &
        else
            /usr/bin/python3 "$PROJECT_DIR/teleoperation/bridges/tienkung_pro/tienkung_pro_ros2_zmq_bridge.py" &
        fi

        BRIDGE_PID=$!
        echo "[INFO] ${ROBOT} ROS2-ZMQ bridge started (PID=$BRIDGE_PID)"
        sleep 2
    else
        echo "[WARN] ROS2 not found in container. Skipping bridge. Set UBT_SIM_NO_BRIDGE=1 to suppress."
    fi
fi

# --- Build extra args ---
EXTRA_ARGS=()
if [ -n "${UBT_SIM_LOAD_ONLY:-}" ]; then
    EXTRA_ARGS+=(--load_only)
    if [ -z "${UBT_SIM_LOAD_ONLY_KEEP_DEVICE:-}" ]; then
        if [ "$ROBOT" = "walker_s2" ] || [ "$ROBOT" = "walker_c1" ]; then
            EXTRA_ARGS+=(--device "${UBT_SIM_LOAD_ONLY_DEVICE:-${UBT_SIM_WALKER_S2_DEVICE:-cuda:0}}")
        else
            EXTRA_ARGS+=(--device "${UBT_SIM_LOAD_ONLY_DEVICE:-cpu}")
        fi
    fi
fi

# --- Launch simulation ---
/isaac-sim/python.sh "$SCRIPT_DIR/sim_runner.py" \
    --task "$UBT_SIM_TASK" \
    --enable_cameras \
    --num_envs "$UBT_SIM_NUM_ENVS" \
    "${EXTRA_ARGS[@]}" \
    "$@"
