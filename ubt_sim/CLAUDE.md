# ubt_sim

Thinker 机器人仿真平台，基于 Isaac Lab。

## 快速开始

```bash
# 构建并启动容器
cd docker
bash run.sh build && bash run.sh start && bash run.sh init && bash run.sh check

# 启动仿真（根据 UBT_SIM_TASK 自动检测机器人）
bash scripts/start_sim.sh
# Walker S2 EDU：
UBT_SIM_TASK=UBTSim-WalkerS2-PartSorting-v0 bash scripts/start_sim.sh

# 天工行者数据采集（同一容器内，用系统 Python 3.10）
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0
/usr/bin/python3 teleoperation/control/tienkung_pro/pick_place_save_data.py
```

## 架构

单容器架构，ROS2 Humble（Py 3.10）与 Isaac Sim（Py 3.11）共存：

```
┌─────────────────────────────────────────────────────────┐
│ Docker Container (host network)                         │
│                                                         │
│  Isaac Sim (Py 3.11)                                    │
│  └─ scripts/sim_runner.py (自动检测天工行者 / Walker S2 EDU) │
│       │                                                 │
│       │  ZMQ PUB/SUB (127.0.0.1)                        │
│       │  ├─ cmd port (5555 / 5655)                      │
│       │  ├─ status port (5556 / 5656)                   │
│       │  ├─ image port (5557 / 5657)                    │
│       │  └─ jpeg port (N/A  / 5658)                     │
│       ▼                                                 │
│  ROS2 Bridge (Py 3.10, 子进程)                          │
│  ├─ bridges/tienkung_pro/tienkung_pro_ros2_zmq_bridge.py (天工行者) │
│  └─ bridges/walker_s2/walker_s2_ros2_zmq_bridge.py         │
│       │                                                 │
│       │  ROS2 DDS (domain 146 仿真 / 0 真机)            │
│       ▼                                                 │
│  控制脚本 (Py 3.10)                                     │
│  ├─ teleoperation/control/tienkung_pro/  (天工行者)         │
│  └─ teleoperation/control/walker_s2/    (Walker S2 EDU)        │
└─────────────────────────────────────────────────────────┘
```

- `source/`（Isaac Sim Py 3.11）和 `teleoperation/`（ROS2 Py 3.10）运行在不同 Python 环境，**零交叉导入**，仅通过 ZMQ 通信
- Walker S2 EDU 探索者桥接使用独立的 ZMQ 端口组（默认 5655-5658），天工行者（Walker TienKung）使用 5555-5557
- 天工行者桥接集成 C++ 图像桥接（`zmq_image_bridge`），支持 ZMQ 原始图像 → ROS2 Image 消息的高效转换
- `start_sim.sh` 自动启动桥接子进程，`UBT_SIM_NO_BRIDGE=1` 可跳过
- 真机部署：同一容器切换 `ROS_DOMAIN_ID=0` 即可

## 目录结构

- `config/` — YAML 任务/场景配置
- `source/ubt_sim/` — Python pip 包
- `assets/` — 3D 模型文件（USD, URDF, 贴图）
- `teleoperation/` — 遥操作脚本（ROS2 桥接、控制、诊断）
- `docker/` — Docker 容器配置
  - `isaac_sim/` — 仿真 + ROS2 统一容器（含 Dockerfile）
- `scripts/` — 启动脚本（`start_sim.sh` + `sim_runner.py`，自动检测机器人类型）

## 扩展

### 添加新场景
1. 放置 3D 文件到 `assets/scenes/<name>/`
2. 创建 `config/<task>.yaml`

### 添加新机器人
1. 放置 3D 文件到 `assets/robots/<name>/`
2. 创建 `source/.../devices/<name>/`（config.py + controller.py + action_process.py）
3. 创建任务 `source/.../task/<task>/`
4. 创建 `config/<task>.yaml`

## 关键约束

- ROS2 Domain ID = 0（真机模式），146（仿真模式）
- Isaac Sim 用 `/isaac-sim/python.sh` (3.11)，ROS2 用 `/usr/bin/python3` (3.10)，不可混用
- numpy < 2（cv_bridge 兼容性）
- Walker S2 EDU 当前采用 **GPU 渲染 + CPU PhysX**：`start_sim.sh` / `sim_runner.py` 默认 AppLauncher/render device 为 `cuda:0`，但 Isaac Lab physics/env device 为 `cpu`。这是为了规避 Isaac Sim 5.0 + Isaac Lab 2.2 在 Walker S2 EDU articulation 初始化时 `get_dof_velocities()` 触发的 PhysX GPU tensor device mismatch（`getVelocities: expected device 0, received device -1`）。不要默认改回 GPU physics；如需实验可显式设置 `UBT_SIM_WALKER_S2_PHYSICS_DEVICE=cuda:0`。
