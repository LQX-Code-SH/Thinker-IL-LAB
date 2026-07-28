# ubt_IL — UBTECH 模仿学习训练部署平台

基于 LeRobot 的天工与 Walker S2 机器人模仿学习训练与部署平台。

## 架构

两层桥接架构，LeRobot 通过 ZMQ 与 Bridge2 通信，Bridge2 通过 ROS2 DDS 与机器人硬件通信：

```
TienKung:
LeRobot (3.12) ──ZMQ 5559/5560──► Bridge2 (3.10) ──ROS2 DDS──► 天工硬件
                │
                └──ZMQ 5558──► ImageServer (独立进程) ──► 相机硬件

Walker S2:
LeRobot (3.12) ──ZMQ 5561/5562──► Walker Bridge2 (3.10) ──ROS2 DDS──► Walker S2
                │
                └──ZMQ 5563──► CameraRelay (独立进程) ──ROS2 DDS──► shm_msgs Camera
```

- **TienKung Bridge2** (`tienkung/ros2_deploy_bridge.py`): ZMQ 5559(action)/5560(state) <-> ROS2 DDS
- **TienKung ImageServer** (`scripts/deploy/tienkung_pro/image_server.py`): 相机硬件 → ZMQ 5558 (JPEG) → ImageServerCamera
- **Walker Bridge2** (`walker/ros2_walker_bridge.py`): ZMQ 5561(action)/5562(state) <-> ROS2 DDS（body + hand/gripper）
- **Walker CameraRelay** (`walker/walker_camera_relay.py`): ZMQ 5563(image) ← ROS2 DDS shm_msgs camera topics

## 目录结构

| 目录 | 内容 | 运行环境 | 说明 |
|------|------|----------|------|
| `lerobot/` | LeRobot 源码（最小副本） | 容器内 | 上游 HuggingFace LeRobot，脱离 git |
| `tienkung/` | 天工插件 | 容器内 | 独立 pip 包，注册为 `tienkung` 类型 |
| `walker/` | Walker S2 插件 + Bridge + ROS2 SDK | 容器内 | 独立 pip 包，注册为 `walker` / `walker_camera` 类型 |
| `docker/` | 容器构建与管理 | 宿主机 | `env.sh` + `run.sh` 统一管理 |
| `dataset/` | 转换后 LeRobot 数据集 | — | HDF5→LeRobot 格式 |
| `model/` | 训练/测试模型权重 | — | ACT 策略 checkpoint |
| `scripts/convert/` | 数据转换脚本 + 配置(按机型) | 宿主机 conda | HDF5 → LeRobot 格式转换 |
| `scripts/train/` | 训练配置 + 脚本(按机型) | 容器内 | train.sh, train_config_*.json |
| `scripts/deploy/` | 部署、回放、复位、相机(按机型) | 容器内 | rollout.sh, replay.py, reset.py, image_server.py |

## Quick Commands

```bash
# 构建镜像 + 启动容器（自动启动 Bridge2）
cd docker
bash run.sh build && bash run.sh start

# 进入容器
bash run.sh bash

# 环境健康检查
bash run.sh check

# 停止容器
bash run.sh stop

# 数据转换（宿主机 conda 环境）
bash scripts/convert/tienkung_pro/convert.sh

# 训练（容器内）
bash /ubt_IL/scripts/train/tienkung_pro/train.sh

# 部署（容器内）
bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh

# 机器人复位（容器内）
source /opt/ros/humble/setup.bash
python3 /ubt_IL/scripts/deploy/tienkung_pro/reset.py
```

## 容器内路径

| 路径 | 说明 |
|------|------|
| `/ubt_IL/lerobot` | LeRobot 源码（bind mount） |
| `/ubt_IL/tienkung/` | 天工插件（bind mount） |
| `/ubt_IL/walker/` | Walker S2 插件、Bridge2、ROS2 SDK/messages（bind mount） |
| `/ubt_IL/dataset/` | 数据集（bind mount） |
| `/ubt_IL/model/` | 模型权重（bind mount） |
| `/ubt_IL/scripts/` | 脚本（bind mount） |
| `/lerobot/.venv/` | 基础镜像 venv（不动） |
| `/ubt_IL/tienkung/ros2_deploy_bridge.py` | Bridge2 脚本（bind-mount 自动可用） |

## 26-Dim State/Action Vector

```
[0-6] 左臂(7) | [7-12] 左手(6) | [13-19] 右臂(7) | [20-25] 右手(6)
```

## 31-Dim Walker S2 State/Action Vector

Walker S2 真机插件默认 31 维：

```
[0-6] 左臂(7) | [7-13] 右臂(7) | [14-15] 头部(2) | [16] 腰部(1) | [17-23] 左手 V4(7) | [24-30] 右手 V4(7)
```

Walker ZMQ 消息: `{"left_arm":[7], "right_arm":[7], "head":[2], "waist":[1], "left_hand":[7], "right_hand":[7], "ts":...}`

⚠️ 当前 `scripts/train/walker_s2/configs/train_config_walker_s2_sim.json` / `model/Walker_S2_sim_act` 是 19 维仿真配置；P0 部署迁移不包含 19→31 维动作适配，不能默认直接用于 Walker 31 维真机 rollout。

## Critical Constraints

- **ROS2 Domain ID = 0**（真机），可通过 `DOMAIN_ID` 环境变量覆盖
- **系统 Python 3.10**（`/usr/bin/python3`），ROS2 Bridge2 使用
- **LeRobot Python 3.12**（`/lerobot/.venv/bin/python`），训练/推理使用
- **numpy<2**（cv_bridge 兼容性），**opencv-python-headless<4.10**
- **天工插件与 LeRobot 源码相互独立**：插件通过 `@register_subclass` 注册，无需修改上游代码
- **Walker 插件与 LeRobot 源码相互独立**：`lerobot_robot_walker` 通过第三方插件机制注册；ROS2 messages 由容器 entrypoint/run.sh 在 `/ubt_IL/walker/walker_sdk_ros2` 中构建
- **FastDDS no-SHM 白名单**：当前包含 `127.0.0.1`、`192.168.41.99`、`192.168.11.99`，按实际真机网段调整
- **Walker 维度约束**：P0 仅迁移 31 维 Walker 真机接口，不实现 19 维仿真模型到 31 维真机动作的适配

## 嵌套文档

- [tienkung/CLAUDE.md](tienkung/CLAUDE.md) — 天工插件技术架构（ZMQ 端口、ROS2 话题、ACT 配置、灵巧手逻辑、插件注册）
- [tienkung/README.md](tienkung/README.md) — 部署操作指南（快速开始、训练、部署、验证）
- [walker/CLAUDE.md](walker/CLAUDE.md) — Walker S2 插件技术架构（ZMQ 端口、ROS2 话题、31 维向量、V4 手控制、插件注册）
