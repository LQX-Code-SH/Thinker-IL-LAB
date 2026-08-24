# Docker 容器

本目录提供 **LeRobot + ROS2 Humble** 容器化环境，面向天工行者（Walker TienKung）与 Walker S2 EDU 探索者真机模仿学习的训练与部署。支持两种架构：x86_64 工作站与 Jetson/aarch64 真机，由 `env.sh` 依据宿主机架构自动选择对应 Dockerfile 与默认参数。

## 目录文件

| 文件 | 作用 |
| --- | --- |
| `Dockerfile` | x86_64 镜像构建（基镜像 `huggingface/lerobot-gpu:latest`，自带 `/lerobot/.venv`） |
| `Dockerfile.arm64` | Jetson aarch64 镜像构建（基镜像 `dustynv/l4t-pytorch:r36.4.0-linuxarm64`，自建 Python 3.12 venv） |
| `run.sh` | CLI 编排器：`build / start / stop / restart / bash / rm / check` |
| `entrypoint.sh` | 容器启动时幂等执行：editable 安装 lerobot 与插件、编译 Walker ROS2 msg、修复 opencv/numpy、校验 sm_87 torch |
| `env.sh` | 按 `uname -m` 选默认镜像 / Dockerfile / GPU 参数 |
| `fastdds_no_shm.xml` | 禁用 FastDDS 共享内存传输，解决 Docker 内 ROS2 订阅失败 |
| `ros2_msgs/` | 预编译 `ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb`（x86）+ `bodyctrl_msgs_src/` 源码（arm64 构建时编译） |
| `torch-2.7.1...aarch64.whl` / `torchvision-0.22.1...aarch64.whl` | Jetson sm_87 源码编译 wheel，随镜像分发（约 261 MB） |

项目代码以 bind mount 挂入容器 `/ubt_IL`，lerobot 与各插件在每次启动时以 editable 模式安装，无需在镜像内 COPY 源码。

## 架构自动选择

`env.sh` 依据 `uname -m` 选择默认值：

| Host arch | Dockerfile | 默认镜像 | 默认 GPU 参数 |
| --- | --- | --- | --- |
| `x86_64` / `amd64` | `Dockerfile` | `lerobot-tienkung:humble` | `--gpus all` |
| `aarch64` / `arm64` | `Dockerfile.arm64` | `lerobot-tienkung:humble-arm64` | `--runtime nvidia` |

## 构建与运行命令

```bash
cd ubt_IL/docker

bash run.sh build     # 构建镜像（首次数分钟）
bash run.sh start     # 幂等启动：已运行则提示，已停止则 start，不存在则 run
bash run.sh check     # 体检：挂载 / lerobot / 插件 / ROS2 msg / GPU / 网络
bash run.sh bash      # 进入容器 shell（自动 source ROS2 与 walker workspace）
bash run.sh stop      # 停止容器
bash run.sh restart   # 重启容器（stop + start）
bash run.sh rm        # 停止并删除容器
```

`start` 会后台跟踪容器日志，轮询等待 entrypoint 把 lerobot、tienkung/walker 插件与 Walker ROS2 消息全部安装/编译完成且可 import 后才返回；若 entrypoint 装完仍 import 失败，将直接报错退出，不放过半成品状态。容器以 `--network=host --shm-size=16g` 运行，并转发 X11 用于 GUI。

## 环境变量覆盖

默认值不匹配宿主机时，用环境变量覆盖：

```bash
# 指定其他基镜像
BASE_IMAGE=nvcr.io/nvidia/l4t-pytorch:<tag> bash run.sh build

# 指定其他 Dockerfile 或输出镜像名
DOCKERFILE=./Dockerfile.arm64 IMAGE=my-lerobot:arm64 bash run.sh build

# Jetson 较新 NVIDIA Container Toolkit 可改用 --gpus all
DOCKER_GPU_ARGS="--gpus all" bash run.sh start

# 调试时关闭显式 GPU 参数
DOCKER_GPU_ARGS="" bash run.sh start

# 修改 ROS2 域 ID（默认 0，真机用）
DOMAIN_ID=1 bash run.sh start

# 修改容器名
CONTAINER_NAME=my-container bash run.sh start
```

ARM64 镜像也可先手动拉取并打标签，避免每次构建重新拉取：

```bash
sudo docker pull swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64
sudo docker tag \
  swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64 \
  docker.io/dustynv/l4t-pytorch:r36.4.0
BASE_IMAGE=docker.io/dustynv/l4t-pytorch:r36.4.0 bash run.sh build
```

## 注意事项

1. **架构强绑定，不能单机并存双架构**。arm64 镜像并非通用 arm64，而是 Jetson 专用：基镜像 `dustynv/l4t-pytorch` 面向 JetPack 6、torch wheel 为 Orin sm_87 (CC 8.7) 源码编译、GPU 参数依赖 Jetson 版 `--runtime nvidia`。x86 宿主机只能构建并运行 x86 容器；arm64 容器必须在 Jetson 上构建与运行。跨架构用 buildx + QEMU 理论可行，但 colcon 编译 ROS2 msg 极慢，且产出的 aarch64 二进制无法在 x86 运行。

2. **bodyctrl_msgs 两架构均可用，安装方式不同**。x86 用预编译 deb（`ros2_msgs/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb`，含 x86_64 原生 type-support，`dpkg` 装入 `/opt/ros/humble`）；arm64 无对应 deb，改由 `Dockerfile.arm64` 构建时从 `ros2_msgs/bodyctrl_msgs_src` 源码 colcon 编译到 `/opt/bodyctrl_msgs_ws`，`entrypoint.sh` 与 `run.sh bash` 启动时 source。源码备份自 `ubt_sim/teleoperation/msgs/body_crtl_msgs_src`（包名 `bodyctrl_msgs`，与 deb 同名同版本）。Walker S2 EDU 不依赖 bodyctrl_msgs（其消息由 `entrypoint.sh` 从 `/ubt_IL/walker/walker_sdk_ros2` 源码编译 8 个包）。

3. **FastDDS 必须禁用共享内存**。容器内即使 `--network=host`，共享内存传输仍会导致 `ros2 topic list` 可用但 `echo`/`subscribe` 失败。`fastdds_no_shm.xml` 白名单含 `127.0.0.1` 与 Walker S2 EDU 直连网段 `192.168.11.3`；改网段时需同步更新此文件。

4. **TORCH_HOME 已重定向**。基镜像默认 `TORCH_HOME=/data/models/torch`，容器内 `/data` 不存在且无权创建，已改到 bind mount 路径 `/ubt_IL/.cache/torch`，使 torchvision ResNet 等 pretrained 权重可下载并持久化。Dockerfile、`env.sh`、`entrypoint.sh` 三处协同设置。

5. **entrypoint 每次启动做运行时安装**。lerobot、`lerobot_robot_tienkung`、`lerobot_robot_walker` 均以 editable 安装，源码改完重启容器即生效。entrypoint 还会：把 lerobot 依赖拉入的 `opencv-python-headless` 换成 GUI 版 `opencv-python` 并钉死 `numpy<2`（须在 lerobot 安装之后，否则被覆盖）；对 Jetson 防御性校验 `torch.cuda.get_arch_list()` 含 `8.7`，否则强制装回 sm_87 wheel。

6. **Jetson torch wheel 随镜像分发**。通用 `download.pytorch.org` cu128 wheel 未编译 SM_87，会报 `no kernel image is available`。本目录两个 wheel 在本机源码编译（CUDA 12.6, cuDNN 9.4, cp312，耗时约 3.5 h），版本满足 LeRobot 约束（torch `>=2.7,<2.12`；torchvision `>=0.22,<0.27`），COPY 进 `/opt/jetson-wheels` 避免每台 Jetson 重编。两文件约占本目录 99% 体积。

7. **LeRobot `[dataset]` extra 已预装**。lerobot 源码是 bind-mount，entrypoint 只做 editable 安装不含此 extra，但 `rollout` 模块 import 时硬性 require `datasets`/`av` 等，故两份 Dockerfile 在构建期预装 `datasets pandas pyarrow av` 四个独立 PyPI 包。
