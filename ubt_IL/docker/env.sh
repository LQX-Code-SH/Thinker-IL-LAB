#!/bin/bash
# Docker environment variables for LeRobot TienKung container
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCH="${ARCH:-$(uname -m)}"
CONTAINER_NAME="${CONTAINER_NAME:-lerobot-tienkung}"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PLUGIN_DIR="${PROJECT_ROOT}/tienkung_pro"
HF_HOME="/ubt_IL/.cache/huggingface"
# torchvision/torch.hub 缓存目录：基镜像默认指向 /data/models/torch（容器内不存在且无权创建），
# 改到 bind mount 路径，使 ResNet 等 pretrained 权重可下载并持久化。
TORCH_HOME="/ubt_IL/.cache/torch"
PIP_MIRROR="-i https://pypi.tuna.tsinghua.edu.cn/simple"
UV_INDEX_URL="https://pypi.tuna.tsinghua.edu.cn/simple"
DOMAIN_ID="${DOMAIN_ID:-0}"

# ── 平台识别：Jetson Thor (JetPack 7 / L4T r38 / sm_110) vs Orin (JetPack 6 / r36 / sm_87) ──
# 两者 aarch64 相同但 GPU 用户态栈、Ubuntu 版本、ROS 发行版都不同，走不同 Dockerfile。
IS_THOR=0
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    if grep -qE "R38" /etc/nv_tegra_release 2>/dev/null; then
        IS_THOR=1
    elif nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | grep -qi "thor"; then
        IS_THOR=1
    fi
fi

# DDS 参数（docker run 额外参数）：Thor 默认与机器人主控栈共用 CycloneDDS + mgbe2_0
DEFAULT_DDS_ENV_ARGS=()
DEFAULT_DDS_MOUNT_ARGS=()

case "$ARCH" in
    aarch64|arm64)
        if [ "$IS_THOR" = "1" ]; then
            # Thor：基镜像用机器人自带的 Humble 工作空间镜像，torch 从 astron 运行镜像移植 sm_110
            DEFAULT_IMAGE="lerobot-tienkung:humble-thor"
            DEFAULT_DOCKERFILE="$SCRIPT_DIR/Dockerfile.thor"
            DEFAULT_BASE_IMAGE="glcr.rd.ubtrobot.com/t800/common/ros2_ws:humble-dyn-v0.0.192"
            DEFAULT_TORCH_SRC_IMAGE="glcr.rd.ubtrobot.com/runtime/astron/astron-integration:c3_vision-v0.2.8"
            DEFAULT_DOCKER_GPU_ARGS="--runtime nvidia"
            # C1 真机只接 GM(S)L 头相机，无 RealSense 腕部相机
            DEFAULT_ENABLE_WRIST_CAMERA=0
            DEFAULT_RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
            DEFAULT_CYCLONEDDS_URI="file:///opt/cyclonedds_thor.xml"
            DEFAULT_DDS_ENV_ARGS=(
                -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-$DEFAULT_RMW_IMPLEMENTATION}"
                -e "CYCLONEDDS_URI=${CYCLONEDDS_URI:-$DEFAULT_CYCLONEDDS_URI}"
            )
            # 把配置挂进容器：改网卡名/participant 上限后重启容器即生效，不必重建镜像
            DEFAULT_DDS_MOUNT_ARGS=(
                -v "$SCRIPT_DIR/cyclonedds_thor.xml:/opt/cyclonedds_thor.xml:ro"
            )
        else
            DEFAULT_IMAGE="lerobot-tienkung:humble-arm64"
            DEFAULT_DOCKERFILE="$SCRIPT_DIR/Dockerfile.arm64"
            DEFAULT_BASE_IMAGE="swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64"
            DEFAULT_TORCH_SRC_IMAGE=""
            DEFAULT_DOCKER_GPU_ARGS="--runtime nvidia"
            # 真机直连 D405 腕部相机，默认自启动（驱动+插件已在镜像构建期预装）
            DEFAULT_ENABLE_WRIST_CAMERA=1
        fi
        ;;
    x86_64|amd64)
        DEFAULT_IMAGE="lerobot-tienkung:humble"
        DEFAULT_DOCKERFILE="$SCRIPT_DIR/Dockerfile"
        DEFAULT_BASE_IMAGE="swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/huggingface/lerobot-gpu:latest"
        DEFAULT_TORCH_SRC_IMAGE=""
        DEFAULT_DOCKER_GPU_ARGS="--gpus all"
        # x86 工作站在机器人外部运行，无腕部相机，默认不自启动（镜像也未装驱动）
        DEFAULT_ENABLE_WRIST_CAMERA=0
        ;;
    *)
        DEFAULT_IMAGE="lerobot-tienkung:humble-${ARCH}"
        DEFAULT_DOCKERFILE="$SCRIPT_DIR/Dockerfile"
        DEFAULT_BASE_IMAGE="swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/huggingface/lerobot-gpu:latest"
        DEFAULT_TORCH_SRC_IMAGE=""
        DEFAULT_DOCKER_GPU_ARGS="--gpus all"
        DEFAULT_ENABLE_WRIST_CAMERA=0
        ;;
esac

IMAGE="${IMAGE:-$DEFAULT_IMAGE}"
DOCKERFILE="${DOCKERFILE:-$DEFAULT_DOCKERFILE}"
BASE_IMAGE="${BASE_IMAGE:-$DEFAULT_BASE_IMAGE}"
TORCH_SRC_IMAGE="${TORCH_SRC_IMAGE:-${DEFAULT_TORCH_SRC_IMAGE:-}}"
if [ -z "${DOCKER_GPU_ARGS+x}" ]; then
    DOCKER_GPU_ARGS="$DEFAULT_DOCKER_GPU_ARGS"
fi
if [ -z "${DDS_ENV_ARGS+x}" ]; then
    DDS_ENV_ARGS=("${DEFAULT_DDS_ENV_ARGS[@]}")
fi
if [ -z "${DDS_MOUNT_ARGS+x}" ]; then
    DDS_MOUNT_ARGS=("${DEFAULT_DDS_MOUNT_ARGS[@]}")
fi
