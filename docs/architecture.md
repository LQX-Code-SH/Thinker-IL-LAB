# 架构总览

## 双子项目

| 子项目 | 定位 | 技术栈 | 运行方式 |
|--------|------|--------|----------|
| `ubt_sim` | 机器人仿真平台：仿真验证、遥操作、数据采集 | Isaac Sim 5.0 + Isaac Lab 2.2 + ROS2 | Docker 容器（Isaac Sim Python 3.11 + 系统 Python 3.10） |
| `ubt_IL` | 模仿学习平台：数据转换、训练、评估、部署 | LeRobot（vendored 子模块）+ ROS2 | Docker 容器（Python 3.12 venv + 系统 Python 3.10） |

## 数据流

```
┌──────────────────────────────────────────────────────────────────────────┐
│                            ubt_IL 模仿学习平台                              │
│                数据转换 ──► 模型训练 ──► 离线评估 ─► 推理部署                 │
└───────────▲───────────────┬──────────────────────▲──────────────┬────────┘
            │               │④仿真部署              │              │⑤真机部署
            │①数据采集       ▼ （ROS桥接）           │②真机采集      │（ROS桥接）
   ┌────────┴────────────────┐        ┌─────────────┴──────────────▼──────┐
   │      ubt_sim 仿真平台    │        │    真机（天工 Pro / Walker S2）     │
   │     （Isaac Sim 容器）   │        │     ROS_DOMAIN_ID=0 直连网段       │
   └─────────────────────────┘        └───────────────────────────────────┘
```

- **① 数据采集（仿真）**：`ubt_sim` 内置脚本任务（抓取 / 分拣）批量采集 15Hz HDF5 数据，仅任务成功才落盘。
- **② 数据采集（真机）**：使用 [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) 遥操数采平台采集，可导出 HDF5 或 LeRobot 数据集。
- **③ 转换 / 训练 / 评估**：在 `ubt_IL` 容器内完成，HDF5 统一转换为 LeRobot v3 数据集后训练 ACT / Pi0.5 策略，再做离线 MSE 评估。
- **④ 仿真部署**：模型经 ROS2-ZMQ 桥接回注 Isaac Sim，与真机部署话题一致，用于真机部署前验证。
- **⑤ 真机部署**：`rollout.sh` 推理部署，或 Walker S2 推理服务器常驻预热部署。

## ubt_sim 仿真平台架构

三进程协同（**Python 版本不可混用**）：

```
Isaac Sim (Py3.11)  ──ZMQ──►  ROS2-ZMQ Bridge (Py3.10)  ──ROS2──►  遥操作/采集脚本
```

- **仿真** `/isaac-sim/python.sh`：加载 Gym 任务（天工 Pro 客厅 / Walker S2 仓库），图像直发 ZMQ 端口。
- **桥接** `/usr/bin/python3`：ZMQ↔ROS2 双向转发，`start_sim.sh` 自动拉起。
- **采集** `/usr/bin/python3`：发布指令、录制 15Hz HDF5。

通过 ROS2-ZMQ 桥接，仿真以**与真机一致的 ROS2 话题**对外发布状态与图像，实现 sim-to-real 一致的遥操作与数据采集。

## ubt_IL 模仿学习平台架构

通信链路（以天工为例）：

```
LeRobot (Python 3.12 venv)
   │  ZMQ 5559/5560
   ▼
Bridge2 (系统 Python 3.10 + ROS2)
   │  ROS2 DDS
   ▼
机器人 / 仿真器
   ▲
   └── 相机推流走 ZMQ 5558（ImageServer）
```

容器内双 Python 环境：ROS 相关用 `/usr/bin/python3`，LeRobot 相关用 `/lerobot/.venv/bin/python`。

## ZMQ 端口一览

| 端口 | 模块 | 用途 |
|------|------|------|
| 5555 | 天工仿真桥接 | 控制指令（bridge PUB） |
| 5556 | 天工仿真桥接 | 机器人状态（bridge SUB） |
| 5557 | 天工仿真桥接 | 相机原图像（C++ 桥接处理） |
| 5558 | 仿真 / ImageServer | JPEG 相机直连（仿真发布；真机由 `image_server.py` 提供） |
| 5559 / 5560 | ubt_IL ↔ Bridge2 | 动作指令 / 机器人状态 |
| 5655 | Walker S2 仿真桥接 | 控制指令（bridge PUB） |
| 5656 | Walker S2 仿真桥接 | 机器人状态（bridge SUB） |
| 5657 | Walker S2 仿真 | 四路相机 RGB（采集直连） |
| 5658 | Walker S2 仿真 | JPEG 图像 |
| 5561 / 5562 / 5563 | Walker S2 真机 Bridge2 | 指令 / 状态通道 |
| 5570 | Walker S2 推理服务器 | ZMQ REP 指令端口 |
| 5571 | Walker S2 推理服务器 | ZMQ PUB 状态端口 |

## 项目结构

```
Thinker-IL-LAB/
├── ubt_sim/                    # 仿真平台
│   ├── assets/                 # 3D 模型（USD/URDF/贴图，Git LFS）
│   ├── config/                 # YAML 任务/场景配置
│   ├── docker/                 # Docker 容器配置
│   ├── scripts/                # 仿真启动脚本
│   ├── source/ubt_sim/         # Python pip 包（Isaac Sim 侧，Py3.11）
│   └── teleoperation/          # 机器人控制脚本（ROS2 侧）
│       ├── bridges/            # ROS2-ZMQ 桥接
│       ├── control/            # 机器人控制 + 数据采集
│       ├── image/              # 图像传输
│       ├── msgs/               # ROS2 自定义消息
│       └── tools/              # 诊断/回放工具
└── ubt_IL/                     # 模仿学习平台
    ├── dataset/                # 数据集与转换输出
    ├── docker/                 # Docker 容器配置
    ├── lerobot/                # vendored lerobot 源码（子模块）
    ├── model/                  # 训练输出与预训练权重
    ├── scripts/
    │   ├── convert/            # HDF5 -> LeRobot 数据转换
    │   ├── train/              # 训练启动脚本
    │   ├── deploy/             # 部署脚本（reset / rollout / replay / 推理服务器）
    │   └── eval/               # 离线评估
    ├── tienkung/               # 天工 lerobot 插件
    └── walker/                 # Walker S2 插件 + ROS2 SDK/messages + Bridge2
```
