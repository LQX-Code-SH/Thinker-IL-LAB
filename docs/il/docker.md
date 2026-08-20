# 模仿学习平台：容器构建与使用

`ubt_IL` 模仿学习平台运行在 **LeRobot + ROS2 Humble** 容器内。不同机器人的容器构建流程完全一致，统一在本页维护；各机型页直接进入机型专属步骤。

宿主机项目挂载于容器 `/ubt_IL`，后续转换/训练/评估命令均在容器内执行。

## 构建并进入容器（宿主机）

```bash
cd ubt_IL/docker
bash run.sh build      # 构建容器镜像（首次；自动按平台选择 Dockerfile）
bash run.sh start      # 启动容器
bash run.sh bash       # 进入容器 shell
```

## 架构自动选择

镜像构建脚本按宿主机 CPU 架构自动选择 Dockerfile：

| 宿主机架构 | Dockerfile | 说明 |
|------------|-----------|------|
| `x86_64` / `amd64` | `Dockerfile` | x86 工作站训练/仿真部署 |
| `aarch64` / `arm64` | `Dockerfile.arm64` | Jetson 真机 Vision 板部署（arm 镜像须在 Jetson 上构建） |


## 常用 run.sh 命令

```text
build    构建镜像
start    幂等启动：已运行则提示，已停止则 start，不存在则 run
check    体检：挂载 / lerobot / 插件 / ROS2 msg / GPU / 网络
bash     进入容器 shell（自动 source ROS2 与 walker workspace）
stop     停止容器
restart  重启容器（stop + start）
rm       停止并删除容器
```

## 容器内 Python 环境

- 容器内双 Python 环境：ROS 相关使用 `/usr/bin/python3`，LeRobot 相关脚本使用 `/lerobot/.venv/bin/python`（默认）。
- x86 镜像基于 `huggingface/lerobot-gpu:latest`，自带 `/lerobot/.venv`；ROS 相关使用 `/usr/bin/python3`，LeRobot 相关脚本使用 `/lerobot/.venv/bin/python`（默认）。
- 容器以 `--network=host --shm-size=16g` 运行，并转发 X11 用于 GUI。
- 通信链路：LeRobot（Python 3.12）-ZMQ `5559`/`5560`-> Bridge2（系统 Python 3.10）-ROS2 DDS-> 机器人/仿真器；相机推流走 ZMQ `5558`。
- walker真机部署时容器在**机器人 Vision 板（Jetson）**上构建，构建脚本自动识别主板类型构建 arm 容器。

> 镜像与 `entrypoint.sh` 的完整说明（Dockerfile 细节、环境变量覆盖、FastDDS 共享内存、Jetson torch wheel 等）见 [容器环境（ubt_IL/docker）](../common/docker.md)。

## 下一步

- [平台概览](index.md) - 返回平台介绍页
- [天工行者](../tienkung/convert-train.md) - 天工机器人训练流程 
- [Walker S2 EDU 探索者](../walker-s2/convert-train.md) - Walker S2 EDU 探索者 机器人训练流程

