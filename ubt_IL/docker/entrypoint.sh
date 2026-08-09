#!/bin/bash
set -e

# Source ROS2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# 天工 bodyctrl_msgs（arm64 构建时源码编译产物；x86 由 deb 装入 /opt/ros/humble）
if [ -f /opt/bodyctrl_msgs_ws/install/setup.bash ]; then
    source /opt/bodyctrl_msgs_ws/install/setup.bash
fi

# Fast-DDS: disable shared memory transport (required for Docker, even with --network=host)
# Without this, ros2 topic list works but ros2 topic echo / subscribe fails
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_no_shm.xml

build_walker_ros2_msgs() {
    local walker_ws="/ubt_IL/walker/walker_sdk_ros2"
    local walker_msgs="shm_msgs mc_state_msgs mc_task_msgs emb_task_msgs sys_task_msgs rosa_msgs ecat_task_msgs"

    if [ ! -d "$walker_ws" ]; then
        return 0
    fi

    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    local ros_pythonpath="$PYTHONPATH"
    if [ -f "$walker_ws/install/setup.bash" ]; then
        source "$walker_ws/install/setup.bash"
    fi

    if /usr/bin/python3 - <<'PY' >/dev/null 2>&1
from mc_state_msgs.msg import RobotState
from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
from shm_msgs.msg import Image2m
for msg_type in (RobotState, JointCmd, JointCommand, RobotCommand, Image2m):
    msg_type.__class__.__import_type_support__()
PY
    then
        echo "[entrypoint] Walker ROS2 messages already available."
        return 0
    fi

    echo "[entrypoint] Building Walker ROS2 message packages..."
    (
        cd "$walker_ws"
        # A failed/stale CMake cache can keep /lerobot/.venv/bin/python3 as the
        # rosidl generator even after PATH is fixed, so rebuild messages cleanly.
        rm -rf build install log
        unset VIRTUAL_ENV PYTHONHOME
        export PYTHONPATH="$ros_pythonpath"
        export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
        export Python3_EXECUTABLE=/usr/bin/python3
        export PYTHON_EXECUTABLE=/usr/bin/python3
        export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        local python_multiarch
        python_multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
        if [ -z "$python_multiarch" ]; then
            python_multiarch="$(uname -m | sed 's/aarch64/aarch64-linux-gnu/;s/x86_64/x86_64-linux-gnu/')"
        fi
        local python_soabi
        python_soabi="$(/usr/bin/python3 - <<'PY'
import sysconfig
print(sysconfig.get_config_var('SOABI'))
PY
)"
        /usr/bin/colcon build --packages-select $walker_msgs \
            --cmake-args \
                -DPython3_EXECUTABLE=/usr/bin/python3 \
                -DPYTHON_EXECUTABLE=/usr/bin/python3 \
                -DPYTHON_LIBRARY=/usr/lib/${python_multiarch}/libpython3.10.so \
                -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
                -DPYTHON_SOABI=${python_soabi}
    ) || {
        echo "[entrypoint] WARNING: Walker ROS2 message build failed"
        return 0
    }

    if [ -f "$walker_ws/install/setup.bash" ]; then
        source "$walker_ws/install/setup.bash"
    fi
}

# 防御性安全网：确保 LeRobot venv 用的是源码编译的 Jetson sm_87 torch。
# uv pip install -e .（lerobot/plugins）不升级已满足约束的依赖，理论上不会覆盖；
# 但若解析意外把通用 cu128 wheel（无 sm_87）装进来，此处用镜像层 wheel 强制装回。
reinstall_jetson_torch() {
    [ -d /opt/jetson-wheels ] || return 0
    if python -c "import torch; alc=torch.cuda.get_arch_list(); assert any('8.7' in str(a) for a in alc)" 2>/dev/null; then
        echo "[entrypoint] Jetson torch (sm_87) active: $(python -c 'import torch;print(torch.__version__)' 2>/dev/null)"
        return 0
    fi
    echo "[entrypoint] Reinstalling source-built Jetson torch/torchvision (sm_87)..."
    uv pip install --no-deps --force-reinstall \
        /opt/jetson-wheels/torch-*.whl /opt/jetson-wheels/torchvision-*.whl \
        || echo "[entrypoint] WARNING: Jetson torch reinstall failed"
}

# ROS_DOMAIN_ID: 默认 0 (真机)，可通过 DOMAIN_ID 环境变量覆盖
if [ -n "$DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID="$DOMAIN_ID"
else
    export ROS_DOMAIN_ID=0
fi

# Ensure HuggingFace cache directory exists (inside /ubt_IL mount, always writable)
if [ -n "$HF_HOME" ]; then
    mkdir -p "$HF_HOME" 2>/dev/null || true
fi

# Ensure torch hub cache directory exists (TORCH_HOME points into /ubt_IL mount)
if [ -n "$TORCH_HOME" ]; then
    mkdir -p "$TORCH_HOME" 2>/dev/null || true
fi

# 运行时安装（如果挂载了项目目录）
# 挂载路径为 /ubt_IL，避免覆盖基础镜像的 /lerobot/.venv/
if [ -d "/ubt_IL" ]; then
    # Activate base image venv for subsequent uv commands
    export VIRTUAL_ENV=/lerobot/.venv
    export PATH="/lerobot/.venv/bin:$PATH"

    # Install lerobot from source (editable) if not already
    if ! python -c "import lerobot; assert '/ubt_IL/lerobot/' in lerobot.__file__" 2>/dev/null; then
        echo "[entrypoint] Installing lerobot from /ubt_IL/lerobot (editable)..."
        uv pip install "numpy<2" || true
        cd /ubt_IL/lerobot && uv pip install -e . || echo "[entrypoint] WARNING: lerobot install failed"
    fi

    # Build/source Walker ROS2 messages before installing the Python plugin.
    build_walker_ros2_msgs

    # Install TienKung plugin (editable)
    if [ -d "/ubt_IL/tienkung/lerobot_robot_tienkung" ]; then
        echo "[entrypoint] Installing lerobot-robot-tienkung plugin..."
        uv pip install -e /ubt_IL/tienkung/lerobot_robot_tienkung || echo "[entrypoint] WARNING: tienkung plugin install failed"
    fi

    # Install Walker plugin (editable)
    if [ -d "/ubt_IL/walker/lerobot_robot_walker" ]; then
        echo "[entrypoint] Installing lerobot-robot-walker plugin..."
        uv pip install -e /ubt_IL/walker/lerobot_robot_walker || echo "[entrypoint] WARNING: walker plugin install failed"
    fi

    # RealSense D405 wrist camera service (optional, on-demand install + start).
    # arm64 镜像构建期已预装驱动+插件,此块仅作 x86/手动 fallback。
    # Set INSTALL_REALSENSE_WRIST_CAMERA=1 to auto-install at container startup.
    # Set REALSENSE_WRIST_CAMERA_CONFIG=/path/to/config.json to auto-start the service.
    if [ "${INSTALL_REALSENSE_WRIST_CAMERA:-0}" = "1" ]; then
        if [ -f "/ubt_IL/walker/realsense_wrist_camera/scripts/install.sh" ]; then
            echo "[entrypoint] Installing realsense_wrist_camera..."
            bash /ubt_IL/walker/realsense_wrist_camera/scripts/install.sh || \
                echo "[entrypoint] WARNING: realsense_wrist_camera install failed"
        fi
        export PATH="$HOME/.local/bin:$PATH"
        if [ -n "${REALSENSE_WRIST_CAMERA_CONFIG:-}" ]; then
            echo "[entrypoint] Starting realsense-wrist-camera with config: ${REALSENSE_WRIST_CAMERA_CONFIG}"
            /usr/bin/python3 -m realsense_wrist_camera \
                --config "$REALSENSE_WRIST_CAMERA_CONFIG" &
        else
            echo "[entrypoint] Starting realsense-wrist-camera with auto-discover..."
            /usr/bin/python3 -m realsense_wrist_camera \
                --discover &
        fi
    fi

    # 安全网：editable 安装后确认/恢复 sm_87 torch（见函数注释）。
    reinstall_jetson_torch

    # Replace headless OpenCV with GUI version (MUST be after lerobot install)
    # lerobot's dependencies pull in opencv-python-headless + numpy>=2, so we fix it last.
    # Check via pip list (not import) because numpy mismatch may crash cv2 import.
    if uv pip list 2>/dev/null | grep -q "opencv-python-headless"; then
        echo "[entrypoint] Replacing opencv-python-headless with opencv-python (GUI support)..."
        uv pip uninstall opencv-python-headless opencv-python -y 2>/dev/null || true
        uv pip install "opencv-python<4.10" "numpy<2" || echo "[entrypoint] WARNING: opencv upgrade failed"
    fi
fi

exec "$@"
