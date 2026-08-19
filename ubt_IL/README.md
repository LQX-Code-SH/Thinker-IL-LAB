# UBTECH-IL-LAB

天工 Pro 与 Walker S2 机器人模仿学习平台，基于 [LeRobot](https://github.com/huggingface/lerobot)，覆盖数据转换、模型训练、真机/仿真部署全流程。

## 支持的机器人

| 机器人 | DOF | 夹爪 | 手部 | 插件目录 |
|--------|-----|------|------|----------|
| TienKung Pro | 50 | - | 12×2 (五指灵巧手) | `tienkung/lerobot_robot_tienkung` |
| Walker S2 | 34 | 2×2 (PGC 两指) | - | `walker/lerobot_robot_walker` |

> 技术架构、端口定义、ROS2 话题、向量格式等细节参见 [CLAUDE.md](./CLAUDE.md)。


### 测试案例

| 天工 Pro 仿真 | 天工 Pro 真机 |
|---|---|
| <video src="docs/assets/tienkung仿真部署效果.mp4" controls muted width="360"></video> | <video src="docs/assets/tienkung真机部署效果.mp4" controls muted width="360"></video> |

| Walker S2 仿真 | Walker S2 真机 |
|---|---|
| <video src="docs/assets/walker仿真部署效果.mp4" controls muted width="360"></video> | <video src="docs/assets/walker真机部署效果.mp4" controls muted width="360"></video> |


## 快速开始
请根据目标机器人查看对应快速使用文档（含环境搭建、仿真启动、数据采集、相机测试、故障排查等完整步骤）：
- [Tienkung Pro 使用说明](docs/tienkung_pro/getting-started.md)（[仿真工作流](docs/tienkung_pro/sim-workflow.md) / [真机工作流](docs/tienkung_pro/real-workflow.md)）
- [Walker S2 使用说明](docs/walker_s2/getting-started.md)（[仿真工作流](docs/walker_s2/sim-workflow.md) / [真机工作流](docs/walker_s2/real-workflow.md) / [推理服务](docs/walker_s2/inference-server.md)）
- [数据转换详细说明](scripts/convert/README.md)

## 工作流

一句话流程：HDF5 采集数据 -> 转换为 LeRobot 数据集 -> 容器内训练 -> `rollout.sh` 部署。




## 项目结构

```
ubt_IL/
├── dataset/               # 数据集与转换输出
├── docker/                # Docker 容器配置（run.sh / env.sh / fastdds 配置）
├── docs/                  # 各机器人使用文档
├── lerobot/               # vendored lerobot 源码
├── model/                 # 训练输出与预训练权重
├── scripts/
│   ├── convert/           # HDF5 -> LeRobot 数据转换
│   ├── train/             # 训练启动脚本
│   ├── deploy/            # 部署脚本（reset / rollout / replay）
│   └── eval/              # 离线评估
├── tienkung/              # 天工 lerobot 插件
└── walker/                # Walker S2 插件 + ROS2 SDK/messages + Bridge2
```
