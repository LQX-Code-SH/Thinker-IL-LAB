ARG BASE_IMAGE=swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/huggingface/lerobot-gpu:latest
FROM ${BASE_IMAGE}

USER root
ENV DEBIAN_FRONTEND=noninteractive
# 覆盖基镜像 TORCH_HOME=/data/models/torch（容器内 /data 不存在且无权创建），
# 改到 bind mount 路径，使 torchvision ResNet 等 pretrained 权重可下载并持久化。
ENV TORCH_HOME=/ubt_IL/.cache/torch

# 清华 TUNA apt 源
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
    && sed -i 's|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humble 源 + 安装
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common curl \
    && add-apt-repository -y universe \
    && curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       | tee /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-builtin-interfaces \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    build-essential cmake python3-colcon-common-extensions python3-empy \
    libyaml-cpp-dev \
    libxkbcommon-x11-0 libvulkan1 mesa-vulkan-drivers \
    libgtk-3-0 libcanberra-gtk3-module \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# bodyctrl_msgs (预编译 deb)
COPY docker/ros2_msgs/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb /tmp/
RUN dpkg -i /tmp/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb \
    && rm -f /tmp/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb

# tienkung_dex 消息包（ros2_bridge_msgs + inspire_hand_msgs + brainco_hand_msgs）：
# 源码复制自 ubt_sim/teleoperation/msgs/tienkung_dex/，构建上下文是 ubt_IL/。
# 编译脚本自己 source /opt/ros/humble 与 colcon，产物进独立工作空间 /opt/tienkung_dex_msgs_ws。
COPY tienkung_dex/ros2_dex_msgs /opt/tienkung_dex_msgs_src
RUN bash /opt/tienkung_dex_msgs_src/build.sh \
    && chown -R user_lerobot:user_lerobot /opt/tienkung_dex_msgs_ws

# 系统 Python 依赖 + rospython 别名
RUN /usr/bin/python3 -m pip install --upgrade pip \
    && /usr/bin/python3 -m pip install --break-system-packages --no-cache-dir \
    --retries 5 --timeout 60 \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    pyzmq "numpy<2" "opencv-python<4.10" ikpy h5py pandas pyarrow \
    && ln -sf /usr/bin/python3 /usr/local/bin/rospython

# Fast-DDS config: disable shared memory (fixes ROS2 data delivery inside Docker)
COPY docker/fastdds_no_shm.xml /opt/fastdds_no_shm.xml

# Bridge2 is bind-mounted via /ubt_IL/tienkung_pro/ros2_deploy_bridge.py (no COPY needed)
COPY docker/entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

USER user_lerobot
WORKDIR /ubt_IL

# LeRobot [dataset] extra 运行时依赖：rollout 模块在 import 时硬性 require
# datasets/av（lerobot.datasets 录制回放视频用）。lerobot 源码是 bind-mount，
# entrypoint 只做 editable 安装不含此 extra，故构建时预装这 4 个独立 PyPI 包。
# 基镜像 lerobot-gpu:latest 的 /lerobot/.venv 可能不含 pip（:latest drift），ensurepip 补齐。
RUN /lerobot/.venv/bin/python -m ensurepip --upgrade \
    && /lerobot/.venv/bin/python -m pip install --no-cache-dir \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    "datasets>=4.7.0,<5.0.0" "pandas>=2.0.0,<3.0.0" \
    "pyarrow>=21.0.0,<30.0.0" "av>=15.0.0,<16.0.0"

ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["bash"]
