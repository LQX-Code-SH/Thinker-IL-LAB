#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

# Create Xauthority if it doesn't exist
XAUTH="${HOME}/.Xauthority"
if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"
fi

case "${1:-}" in
    build)
        echo "[INFO] Building image: $IMAGE (from $BASE_IMAGE)"
        docker build \
            --build-arg BASE_IMAGE="$BASE_IMAGE" \
            -t "$IMAGE" \
            -f "$SCRIPT_DIR/Dockerfile" \
            "$PROJECT_DIR"
        echo "[INFO] Image built: $IMAGE"
        ;;
    start)
        # Check if container already exists
        if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            # Container exists, just start it
            if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
                echo "[WARN] Container '$CONTAINER_NAME' is already running."
            else
                echo "[INFO] Starting existing container '$CONTAINER_NAME'..."
                xhost + 2>/dev/null || true
                docker start "$CONTAINER_NAME"
                echo "[INFO] Container '$CONTAINER_NAME' started."
            fi
        else
            # Create new container
            echo "[INFO] Creating container '$CONTAINER_NAME'..."
            mkdir -p "$CACHE_DIR"/{kit,glcache,ov,pip,computecache,data,logs,documents}

            xhost + 2>/dev/null || true

            docker run -d --name "$CONTAINER_NAME" \
                --gpus all \
                --network host \
                -v "${PROJECT_DIR}:/ubt_sim" \
                -v "${CACHE_DIR}/kit:/isaac-sim/kit/cache:rw" \
                -v "${CACHE_DIR}/glcache:/root/.cache/nvidia/GLCache:rw" \
                -v "${CACHE_DIR}/ov:/root/.cache/ov:rw" \
                -v "${CACHE_DIR}/pip:/root/.cache/pip:rw" \
                -v "${CACHE_DIR}/computecache:/root/.nv/ComputeCache:rw" \
                -v "${CACHE_DIR}/data:/root/.local/share/ov/data:rw" \
                -v "${CACHE_DIR}/logs:/root/.nvidia-omniverse/logs:rw" \
                -v "${CACHE_DIR}/documents:/root/Documents:rw" \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "${XAUTH}:/root/.Xauthority:rw" \
                -e "DISPLAY=${DISPLAY}" \
                -e "ACCEPT_EULA=Y" \
                -e "PRIVACY_CONSENT=Y" \
                -e "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE" \
                -w /ubt_sim \
                --privileged \
                "$IMAGE" tail -f /dev/null

            echo "[INFO] Container '$CONTAINER_NAME' created and started."
        fi

        echo ""
        echo "Next steps:"
        echo "  Enter container:  bash run.sh bash"
        echo "  Initialize env:   bash run.sh init"
        echo "  Check env:        bash run.sh check"
        ;;
    stop)
        # Stop bridge first if running
        BRIDGE_PID=$(docker exec "$CONTAINER_NAME" pgrep -f "tienkung_pro_ros2_zmq_bridge" 2>/dev/null || true)
        if [ -n "$BRIDGE_PID" ]; then
            echo "[INFO] Stopping bridge process..."
            docker exec "$CONTAINER_NAME" bash -c "kill $BRIDGE_PID 2>/dev/null" || true
        fi
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[INFO] Stopping container '$CONTAINER_NAME'..."
            docker stop "$CONTAINER_NAME"
            echo "[INFO] Container stopped."
        else
            echo "[WARN] Container '$CONTAINER_NAME' is not running."
        fi
        ;;
    restart)
        if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' does not exist!"
            echo "[INFO] Create it first: bash run.sh start"
            exit 1
        fi
        echo "[INFO] Restarting container '$CONTAINER_NAME'..."
        xhost + 2>/dev/null || true
        docker restart "$CONTAINER_NAME"
        echo "[INFO] Container restarted."
        ;;
    bash)
        if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' does not exist!"
            echo "[INFO] Create it first: bash run.sh start"
            exit 1
        fi
        if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' is not running!"
            echo "[INFO] Start it first: bash run.sh start"
            exit 1
        fi
        docker exec -it "$CONTAINER_NAME" bash -c "\
            source /opt/ros/humble/setup.bash 2>/dev/null || true; \
            source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash 2>/dev/null || true; \
            export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; \
            export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE; \
            bash"
        ;;
    rm)
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[INFO] Stopping running container..."
            docker stop "$CONTAINER_NAME" >/dev/null
        fi
        if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
            echo "[INFO] Container '$CONTAINER_NAME' removed."
        else
            echo "[WARN] Container '$CONTAINER_NAME' does not exist."
        fi
        ;;
    init)
        if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' is not running!"
            echo "[INFO] Start it first: bash run.sh start"
            exit 1
        fi

        # Install ubt_sim for Isaac Sim Python 3.11
        echo "[INFO] Installing ubt_sim and dependencies (Isaac Sim Python 3.11)..."
        docker exec "$CONTAINER_NAME" bash -c "\
            cd /ubt_sim/source && \
            /isaac-sim/python.sh -m pip install --upgrade pip $PIP_MIRROR && \
            /isaac-sim/python.sh -m pip install -e . $PIP_MIRROR && \
            /isaac-sim/python.sh -m pip install pyzmq 'numpy<2' 'packaging==24.2' $PIP_MIRROR"

        echo "[INFO] Fixing torch packaging symlink..."
        docker exec "$CONTAINER_NAME" bash -c "\
            STRUCTURES='/isaac-sim/exts/omni.isaac.ml_archive/pip_prebundle/torch/_vendor/packaging/_structures.py'; \
            if [ ! -f \"\$STRUCTURES\" ] || [ -L \"\$STRUCTURES\" ]; then \
                rm -f \"\$STRUCTURES\"; \
                mkdir -p \"\$(dirname \"\$STRUCTURES\")\"; \
                cp /isaac-sim/kit/python/lib/python3.11/site-packages/packaging/_structures.py \"\$STRUCTURES\"; \
                echo '[FIX] Restored torch/_vendor/packaging/_structures.py'; \
            fi"

        # ---- 按机型安装/编译消息包（teleoperation/msgs/<机型>/）----
        # 每台机器人只碰自己的目录与独立工作空间；某个机型失败不中断其它机型，
        # 全部跑完后统一报错。只想补一台时，直接单跑对应的脚本即可。
        MSGS_FAILED=""

        echo "[INFO] Installing TienKung Pro message packages (bodyctrl_msgs)..."
        if ! docker exec "$CONTAINER_NAME" \
            bash /ubt_sim/teleoperation/msgs/tienkung_pro/install.sh; then
            MSGS_FAILED="$MSGS_FAILED tienkung_pro"
        fi

        echo "[INFO] Building Walker S2 message packages..."
        if ! docker exec "$CONTAINER_NAME" \
            bash /ubt_sim/teleoperation/msgs/walker_s2/build.sh; then
            MSGS_FAILED="$MSGS_FAILED walker_s2"
        fi

        echo "[INFO] Building TienKung Dex message packages..."
        if ! docker exec "$CONTAINER_NAME" \
            bash /ubt_sim/teleoperation/msgs/tienkung_dex/build.sh; then
            MSGS_FAILED="$MSGS_FAILED tienkung_dex"
        fi

        # Build C++ image bridge
        echo "[INFO] Building C++ image bridge..."
        docker exec "$CONTAINER_NAME" bash -c "\
            source /opt/ros/humble/setup.bash && \
            cd /ubt_sim/teleoperation/bridges && \
            bash build_cpp_bridge.sh 2>/dev/null || echo '[WARN] C++ image bridge build failed (non-critical)'"

        if [ -n "$MSGS_FAILED" ]; then
            echo "[ERROR] 以下机型的消息包安装/编译失败:$MSGS_FAILED"
            echo "[INFO] 可单独重跑: bash /ubt_sim/teleoperation/msgs/<机型>/{build,install}.sh"
            exit 1
        fi

        echo "[INFO] Environment initialized."
        ;;
    bridge-start)
        if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' is not running!"
            exit 1
        fi
        # Check if bridge is already running
        BRIDGE_PID=$(docker exec "$CONTAINER_NAME" pgrep -f "tienkung_pro_ros2_zmq_bridge" 2>/dev/null || true)
        if [ -n "$BRIDGE_PID" ]; then
            echo "[WARN] Bridge already running (PID=$BRIDGE_PID)"
            exit 0
        fi
        echo "[INFO] Starting ROS2-ZMQ bridge..."
        docker exec -d "$CONTAINER_NAME" bash -c "\
            source /opt/ros/humble/setup.bash && \
            export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && \
            export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE && \
            /usr/bin/python3 $BRIDGE_SCRIPT"
        sleep 1
        BRIDGE_PID=$(docker exec "$CONTAINER_NAME" pgrep -f "tienkung_pro_ros2_zmq_bridge" 2>/dev/null || true)
        if [ -n "$BRIDGE_PID" ]; then
            echo "[INFO] Bridge started (PID=$BRIDGE_PID)"
        else
            echo "[ERROR] Bridge failed to start"
            exit 1
        fi
        ;;
    bridge-stop)
        BRIDGE_PID=$(docker exec "$CONTAINER_NAME" pgrep -f "tienkung_pro_ros2_zmq_bridge" 2>/dev/null || true)
        if [ -z "$BRIDGE_PID" ]; then
            echo "[WARN] Bridge is not running"
            exit 0
        fi
        echo "[INFO] Stopping bridge (PID=$BRIDGE_PID)..."
        docker exec "$CONTAINER_NAME" bash -c "kill $BRIDGE_PID 2>/dev/null"
        sleep 1
        # Force kill if still running
        BRIDGE_PID=$(docker exec "$CONTAINER_NAME" pgrep -f "tienkung_pro_ros2_zmq_bridge" 2>/dev/null || true)
        if [ -n "$BRIDGE_PID" ]; then
            echo "[WARN] Force killing bridge..."
            docker exec "$CONTAINER_NAME" bash -c "kill -9 $BRIDGE_PID 2>/dev/null"
        fi
        echo "[INFO] Bridge stopped."
        ;;
    check)
        if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[ERROR] Container '$CONTAINER_NAME' is not running!"
            exit 1
        fi
        echo "=========================================="
        echo "  UBT Sim Environment Check"
        echo "=========================================="
        echo ""

        ERRORS=0
        WARNINGS=0

        # Project mount
        if docker exec "$CONTAINER_NAME" test -d /ubt_sim/source; then
            echo "[OK] Project mounted: /ubt_sim"
        else
            echo "[FAIL] Project NOT mounted!"
            ((ERRORS++))
        fi

        # Assets
        if docker exec "$CONTAINER_NAME" test -d /ubt_sim/assets; then
            echo "[OK] Assets directory: /ubt_sim/assets"
        else
            echo "[FAIL] Assets directory missing!"
            ((ERRORS++))
        fi

        # Isaac Sim Python
        if docker exec "$CONTAINER_NAME" test -x /isaac-sim/python.sh; then
            VER=$(docker exec "$CONTAINER_NAME" /isaac-sim/python.sh --version 2>&1 | head -1)
            echo "[OK] Isaac Sim Python: $VER"
        else
            echo "[FAIL] Isaac Sim Python not found!"
            ((ERRORS++))
        fi

        # ubt_sim package
        if docker exec "$CONTAINER_NAME" /isaac-sim/python.sh -c "import ubt_sim" 2>/dev/null; then
            echo "[OK] ubt_sim package: installed"
        else
            echo "[FAIL] ubt_sim package: NOT installed (run: bash run.sh init)"
            ((ERRORS++))
        fi

        # pyzmq
        if docker exec "$CONTAINER_NAME" /isaac-sim/python.sh -c "import zmq" 2>/dev/null; then
            echo "[OK] pyzmq: installed"
        else
            echo "[FAIL] pyzmq: NOT installed (run: bash run.sh init)"
            ((ERRORS++))
        fi

        # numpy version
        NUMPY_VER=$(docker exec "$CONTAINER_NAME" /isaac-sim/python.sh -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "N/A")
        if [ "$NUMPY_VER" != "N/A" ]; then
            MAJOR=$(echo "$NUMPY_VER" | cut -d. -f1)
            if [ "$MAJOR" -lt 2 ]; then
                echo "[OK] numpy: $NUMPY_VER (< 2)"
            else
                echo "[FAIL] numpy: $NUMPY_VER (must be < 2)"
                ((ERRORS++))
            fi
        else
            echo "[FAIL] numpy: NOT installed"
            ((ERRORS++))
        fi

        # ROS2
        if docker exec "$CONTAINER_NAME" test -f /opt/ros/humble/setup.bash 2>/dev/null; then
            echo "[OK] ROS2 Humble: installed"
        else
            echo "[FAIL] ROS2 Humble: NOT installed"
            ((ERRORS++))
        fi

        # bodyctrl_msgs
        if docker exec "$CONTAINER_NAME" dpkg -s ros-humble-bodyctrl-msgs >/dev/null 2>&1; then
            echo "[OK] bodyctrl_msgs: installed"
        else
            echo "[WARN] bodyctrl_msgs: NOT installed (run: bash run.sh init)"
            ((WARNINGS++))
        fi

        # Walker S2 ROS2 SDK messages
        if docker exec "$CONTAINER_NAME" test -f /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash 2>/dev/null; then
            echo "[OK] Walker SDK ROS2 messages: install setup found"
        else
            echo "[FAIL] Walker SDK ROS2 messages: NOT built (run: bash run.sh init)"
            ((ERRORS++))
        fi
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash && for pkg in rosa_msgs shm_msgs mc_task_msgs mc_state_msgs ecat_task_msgs; do ros2 pkg list | grep -qx \"\$pkg\" || exit 1; done" 2>/dev/null; then
            echo "[OK] Walker SDK ROS2 packages: available"
        else
            echo "[FAIL] Walker SDK ROS2 packages: unavailable (run: bash run.sh init)"
            ((ERRORS++))
        fi
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash && /usr/bin/python3 -c 'from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand; from mc_state_msgs.msg import RobotState; from ecat_task_msgs.msg import GripCmd, GripStatus'" 2>/dev/null; then
            echo "[OK] Walker SDK Python message imports: available"
        else
            echo "[FAIL] Walker SDK Python message imports: unavailable (run: bash run.sh init)"
            ((ERRORS++))
        fi

        # TienKung Dex 真机同款消息包（桥接与控制脚本都用它们）
        if docker exec "$CONTAINER_NAME" test -f /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash 2>/dev/null; then
            echo "[OK] TienKung Dex real-robot messages: install setup found"
        else
            echo "[WARN] TienKung Dex real-robot messages: NOT built (run: bash run.sh init)"
            ((WARNINGS++))
        fi
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash && /usr/bin/python3 -c 'from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, RobotState, MotorStatus; from brainco_hand_msgs.msg import SetMotorMulti; from inspire_hand_msgs.msg import SetAngle, GetAngleAct'" 2>/dev/null; then
            echo "[OK] TienKung Dex real-robot message imports: available"
        else
            echo "[WARN] TienKung Dex real-robot message imports: unavailable (run: bash run.sh init)"
            ((WARNINGS++))
        fi

        # GPU
        if docker exec "$CONTAINER_NAME" nvidia-smi >/dev/null 2>&1; then
            GPU=$(docker exec "$CONTAINER_NAME" nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
            echo "[OK] GPU: $GPU"
        else
            echo "[FAIL] GPU: not detected"
            ((ERRORS++))
        fi

        # X11
        if docker exec "$CONTAINER_NAME" bash -c 'ls /tmp/.X11-unix/X* 2>/dev/null | head -1 | grep -q .' 2>/dev/null; then
            XDISPLAY=$(docker exec "$CONTAINER_NAME" bash -c 'ls /tmp/.X11-unix/X* 2>/dev/null | head -1 | sed "s|.*/X||"')
            echo "[OK] X11: available (DISPLAY=:$XDISPLAY)"
        else
            echo "[WARN] X11: not available (headless mode only)"
            ((WARNINGS++))
        fi

        # Network
        NET=$(docker inspect --format='{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME" 2>/dev/null || echo "unknown")
        if [ "$NET" == "host" ]; then
            echo "[OK] Network: host mode"
        else
            echo "[FAIL] Network: $NET (expected host mode for ZMQ)"
            ((ERRORS++))
        fi

        echo ""
        echo "=========================================="
        if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
            echo "  All checks passed!"
        elif [ $ERRORS -eq 0 ]; then
            echo "  Checks passed with $WARNINGS warning(s)"
        else
            echo "  $ERRORS error(s), $WARNINGS warning(s)"
            exit 1
        fi
        echo "=========================================="
        ;;
    *)
        echo "Usage: $0 {build|start|stop|restart|bash|rm|init|check|bridge-start|bridge-stop}"
        echo ""
        echo "Commands:"
        echo "  build         Build the Docker image with ROS2 + Isaac Sim"
        echo "  start         Create and/or start the container"
        echo "  stop          Stop the container (and bridge)"
        echo "  restart       Restart the container"
        echo "  bash          Enter the container shell (with ROS2 env)"
        echo "  rm            Remove the container"
        echo "  init          Install all dependencies inside container"
        echo "  check         Verify container environment"
        echo "  bridge-start  Start ROS2-ZMQ bridge inside container"
        echo "  bridge-stop   Stop ROS2-ZMQ bridge inside container"
        exit 1
        ;;
esac
