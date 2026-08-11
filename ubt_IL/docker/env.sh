#!/bin/bash
# Docker environment variables for LeRobot TienKung container
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCH="${ARCH:-$(uname -m)}"
CONTAINER_NAME="${CONTAINER_NAME:-lerobot-tienkung}"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PLUGIN_DIR="${PROJECT_ROOT}/tienkung"
HF_HOME="/ubt_IL/.cache/huggingface"
# torchvision/torch.hub 缓存目录：基镜像默认指向 /data/models/torch（容器内不存在且无权创建），
# 改到 bind mount 路径，使 ResNet 等 pretrained 权重可下载并持久化。
TORCH_HOME="/ubt_IL/.cache/torch"
PIP_MIRROR="-i https://pypi.tuna.tsinghua.edu.cn/simple"
UV_INDEX_URL="https://pypi.tuna.tsinghua.edu.cn/simple"
DOMAIN_ID="${DOMAIN_ID:-0}"

case "$ARCH" in
    aarch64|arm64)
        DEFAULT_IMAGE="lerobot-tienkung:humble-arm64"
        DEFAULT_DOCKERFILE="$SCRIPT_DIR/Dockerfile.arm64"
        DEFAULT_BASE_IMAGE="swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64"
        DEFAULT_DOCKER_GPU_ARGS="--runtime nvidia"
        # 真机直连 D405 腕部相机，默认自启动（驱动+插件已在镜像构建期预装）
        DEFAULT_ENABLE_WRIST_CAMERA=1
        ;;
    x86_64|amd64)
        DEFAULT_IMAGE="lerobot-tienkung:humble"
        DEFAULT_DOCKERFILE="$SCRIPT_DIR/Dockerfile"
        DEFAULT_BASE_IMAGE="swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/huggingface/lerobot-gpu:latest"
        DEFAULT_DOCKER_GPU_ARGS="--gpus all"
        # x86 工作站在机器人外部运行，无腕部相机，默认不自启动（镜像也未装驱动）
        DEFAULT_ENABLE_WRIST_CAMERA=0
        ;;
    *)
        DEFAULT_IMAGE="lerobot-tienkung:humble-${ARCH}"
        DEFAULT_DOCKERFILE="$SCRIPT_DIR/Dockerfile"
        DEFAULT_BASE_IMAGE="swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/huggingface/lerobot-gpu:latest"
        DEFAULT_DOCKER_GPU_ARGS="--gpus all"
        DEFAULT_ENABLE_WRIST_CAMERA=0
        ;;
esac

IMAGE="${IMAGE:-$DEFAULT_IMAGE}"
DOCKERFILE="${DOCKERFILE:-$DEFAULT_DOCKERFILE}"
BASE_IMAGE="${BASE_IMAGE:-$DEFAULT_BASE_IMAGE}"
if [ -z "${DOCKER_GPU_ARGS+x}" ]; then
    DOCKER_GPU_ARGS="$DEFAULT_DOCKER_GPU_ARGS"
fi
