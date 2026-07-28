ARG BASE_IMAGE=nvcr.nju.edu.cn/nvidia/isaac-lab:2.2.0
FROM ${BASE_IMAGE}

# ---- ROS2 Humble (清华大学镜像源) ----
RUN set -eux; \
    . /etc/os-release; \
    printf '%s\n' \
      "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME} main restricted universe multiverse" \
      "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME}-updates main restricted universe multiverse" \
      "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME}-backports main restricted universe multiverse" \
      "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME}-security main restricted universe multiverse" \
      > /etc/apt/sources.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg; \
    curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu ${UBUNTU_CODENAME} main" \
      > /etc/apt/sources.list.d/ros2.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-image-transport \
      ros-humble-image-transport-plugins \
      ros-humble-cv-bridge \
      ros-humble-message-filters \
      ros-humble-rqt-image-view \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-ament-cmake \
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-default-runtime \
      ros-humble-std-msgs \
      ros-humble-geometry-msgs \
      ros-humble-sensor-msgs \
      python3-colcon-common-extensions; \
    rm -rf /var/lib/apt/lists/*

# ---- ROS2 子进程所需 Python 3.10 包 ----
# python3-opencv 用 apt 安装（带 GTK GUI 支持，pip 的 manylinux wheel 不带 GUI）
# 基础镜像预装了 sury/php PPA 的新版 libbrotli1(1.1.0)，与 Jammy 的 libbrotli-dev(依赖 1.0.9) 冲突
# 需清掉冲突源 + 强制降级已安装的 libbrotli1 回 Jammy 版本
RUN rm -rf /etc/apt/sources.list.d/* && \
    apt-get update && \
    apt-get install -y --allow-downgrades --no-install-recommends \
    libbrotli1/jammy \
    libfreetype6-dev python3-pip python3-opencv libgtk-3-dev && \
    rm -rf /var/lib/apt/lists/* && \
    /usr/bin/python3 -m pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
    "numpy<2" pyzmq h5py ikpy "pin<3" pyyaml empy lark protobuf

# ---- C++ 图像桥接编译依赖 ----
RUN apt-get update && apt-get install -y --no-install-recommends g++ make libzmq3-dev && \
    rm -rf /var/lib/apt/lists/*

# ---- Walker S2 ROS2 SDK message packages ----
COPY teleoperation/msgs/walker_sdk_ros2_msgs_src/src /opt/ubt_sim/walker_sdk_ros2_msgs/src
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon --log-base /opt/ubt_sim/walker_sdk_ros2_msgs/log build \
      --base-paths /opt/ubt_sim/walker_sdk_ros2_msgs/src \
      --build-base /opt/ubt_sim/walker_sdk_ros2_msgs/build \
      --install-base /opt/ubt_sim/walker_sdk_ros2_msgs/install \
      --merge-install \
      --packages-up-to rosa_msgs shm_msgs mc_task_msgs mc_state_msgs ecat_task_msgs"
