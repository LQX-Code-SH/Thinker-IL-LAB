# Walker C1 真机（NVIDIA Thor / JetPack 7 / L4T r38）专用镜像。
#
# 与 Dockerfile.arm64（Orin / JetPack 6 / r36）的关键差异：
#   1. 基镜像换成机器人自带的 ROS2 工作空间镜像：Ubuntu 22.04 + ROS2 Humble +
#      colcon + cv_bridge，实测在本机（Thor / Ubuntu 24.04 宿主）运行正常。
#      不用 dustynv/l4t-pytorch:r36.4.0 —— 那是 r36 BSP 用户态，与 Thor 的
#      r38/openrm 驱动栈不匹配，且 Docker Hub 在本机不可达。
#   2. torch 从 astron-integration 镜像多阶段 COPY：2.9.0+cu130，arch_list 含
#      sm_110（Thor CC 11.0）。仓库自带的 sm_87 wheel（Orin）在 Thor 上会报
#      "no kernel image is available"。
#   3. 默认 RMW 换成 CycloneDDS：机器人主控栈就是 rmw_cyclonedds_cpp +
#      CYCLONEDDS_URI（限定 mgbe2_0），同 RMW 才能稳定互通（FastDDS 配置保留，
#      走 RMW_IMPLEMENTATION=rmw_fastrtps_cpp 覆盖）。
#   4. numpy2 环境：Thor 的 torch 2.9 支持 numpy 2，LeRobot 0.5.2 要求
#      numpy>=2,<2.3，故不再像 Orin 那样把 numpy 钉到 <2（LEROBOT_ALLOW_NUMPY2=1
#      让 entrypoint.sh 跳过该处理）。
#
# 构建（docker/ 目录下）：
#   bash run.sh build          # env.sh 检测到 L4T r38 时自动选用本文件
# 或显式：
#   DOCKERFILE=./Dockerfile.thor IMAGE=lerobot-tienkung:humble-thor bash run.sh build

ARG BASE_IMAGE=glcr.rd.ubtrobot.com/t800/common/ros2_ws:humble-dyn-v0.0.192
ARG TORCH_SRC_IMAGE=glcr.rd.ubtrobot.com/runtime/astron/astron-integration:c3_vision-v0.2.8

# ── 阶段 1：从 astron 运行镜像取走 sm_110 torch 栈（CUDA 13.0 运行库随 nvidia pip 包）──
FROM ${TORCH_SRC_IMAGE} AS torchsrc

# ── 阶段 2：Humble 基镜像 + python3.12 venv ────────────────────────────────────
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
USER root
ENV DEBIAN_FRONTEND=noninteractive
# 基镜像 TORCH_HOME 若指向不可写路径，会让 torchvision 预训练权重下载失败
ENV TORCH_HOME=/ubt_IL/.cache/torch
# Thor：torch 2.9 支持 numpy 2，LeRobot 0.5.2 要求 numpy>=2 —— 不做 numpy<2 钉版
ENV LEROBOT_ALLOW_NUMPY2=1
# 与机器人主控栈保持同一 RMW（可用 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp 覆盖）
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1) python3.12（LeRobot 0.5.x 要求 >=3.12，jammy 只有 3.10）+ CycloneDDS
# 注：不用 `add-apt-repository ppa:deadsnakes/ppa` —— 它会去请求 Launchpad API 并在
# 本机网络下长时间挂起；改为直接取 PPA 签名公钥 + 写 sources.list（实测 4s 完成）。
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xBA6932366A755776" \
       | gpg --dearmor -o /usr/share/keyrings/deadsnakes.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/deadsnakes.gpg] https://ppa.launchpadcontent.net/deadsnakes/ppa/ubuntu jammy main" \
       > /etc/apt/sources.list.d/deadsnakes.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        python3.12 python3.12-dev python3.12-venv \
        build-essential \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-image-transport ros-humble-cv-bridge ros-humble-message-filters \
        libyaml-cpp-dev libxkbcommon-x11-0 libvulkan1 \
    && rm -rf /var/lib/apt/lists/*

# 2) 系统 python3.10 依赖（Bridge2 / camera relay / preview_camera 都用它）
RUN /usr/bin/python3 -m pip install --no-cache-dir --retries 5 --timeout 60 \
        -i https://pypi.tuna.tsinghua.edu.cn/simple \
        pyzmq h5py \
    && ln -sf /usr/bin/python3 /usr/local/bin/rospython

# 3) 容器用户归一：基镜像的 ubt 即 uid 1000，改名为 user_lerobot（uid/gid 不变）。
#    /ubt_IL 是 bind mount（宿主 walker:1000），entrypoint 要在其内写 colcon install
#    与 *.egg-info，uid 不一致会 permission denied；先建用户再建 venv，venv 内文件
#    天然属主正确，省掉对 4.4G 文件做 chown -R。
RUN groupmod -n user_lerobot ubt \
    && usermod -l user_lerobot -d /home/user_lerobot -m -s /bin/bash ubt \
    && (usermod -aG video,audio,render,dialout user_lerobot || true) \
    && mkdir -p /lerobot /ubt_IL /opt/tienkung_dex_msgs_ws \
    && chown user_lerobot:user_lerobot /lerobot /ubt_IL /opt/tienkung_dex_msgs_ws

COPY docker/entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

# 4) LeRobot venv（3.12）+ uv（以 user_lerobot 身份创建）
USER user_lerobot
RUN /usr/bin/python3.12 -m venv /lerobot/.venv \
    && /lerobot/.venv/bin/python -m pip install --upgrade pip \
        -i https://pypi.tuna.tsinghua.edu.cn/simple \
    && /lerobot/.venv/bin/python -m pip install uv \
        -i https://pypi.tuna.tsinghua.edu.cn/simple

# 5) 移植 Thor sm_110 torch / torchvision（含 nvidia cu13 运行库，约 4.4G）
#    --chown：直接落到 user_lerobot 名下，运行时 uv/pip 才能写 venv
COPY --chown=user_lerobot:user_lerobot --from=torchsrc /usr/local/lib/python3.12/dist-packages/ /lerobot/.venv/lib/python3.12/site-packages/
RUN rm -f /lerobot/.venv/lib/python3.12/site-packages/a1_coverage.pth \
    && /lerobot/.venv/bin/python -c "\
import torch, torchvision; \
print('[build] torch', torch.__version__, 'cuda', torch.version.cuda); \
print('[build] arch_list', torch.cuda.get_arch_list()); \
print('[build] torchvision', torchvision.__version__)"

# 6) LeRobot [dataset] extra：rollout 模块 import 时硬性 require datasets/av
RUN /lerobot/.venv/bin/python -m pip install --no-cache-dir \
        -i https://pypi.tuna.tsinghua.edu.cn/simple \
        "datasets>=4.7.0,<5.0.0" "pandas>=2.0.0,<3.0.0" \
        "pyarrow>=21.0.0,<30.0.0" "av>=15.0.0,<16.0" \
        "matplotlib>=3.7.0,<3.10.0"

# 7) DDS 配置：CycloneDDS 与机器人栈同构（网卡 mgbe2_0）+ FastDDS 兜底（禁 shm）
COPY docker/cyclonedds_thor.xml /opt/cyclonedds_thor.xml
COPY docker/fastdds_no_shm.xml /opt/fastdds_no_shm.xml
ENV CYCLONEDDS_URI=file:///opt/cyclonedds_thor.xml

USER user_lerobot
WORKDIR /ubt_IL

ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["bash"]
