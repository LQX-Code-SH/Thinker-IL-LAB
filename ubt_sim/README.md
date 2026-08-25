# Thinker-Sim-Lab

Thinker 机器人仿真平台，基于 NVIDIA Isaac Sim 5.0 + Isaac Lab 2.2 支持多款机器人 ROS2 仿真控制，用于真机部署前的验证测试、模仿学习仿真数据采集等功能。

![天工行者无疆仿真界面预览](docs/assets/tienkung-pro仿真界面预览.png)

![Walker S2 EDU 探索者 仿真界面预览](docs/assets/walker-s2仿真界面预览.png)

## 支持的机器人

| 机器人 | DOF | 夹爪 | 手部 | USD 路径 |
|--------|-----|------|------|----------|
| Walker TienKung Pro | 50 | — | 12×2 (五指灵巧手) | `assets/robots/tienkung_pro/tienkung_pro_v2.usd` |
| Walker S2 EDU 探索者 | 34 | 2×2 (PGC 两指) | — | `assets/robots/walker_s2/s2_v1.usd` |

## 当前支持的任务

| 任务 ID | 机器人 | 场景 | 说明 |
|---------|--------|------|------|
| `UBTSim-TienkungPro-Parlor-v0` | 天工行者无疆 | 客厅 | 抓苹果遥操作 + 数据采集（默认任务） |
| `UBTSim-WalkerS2-PartSorting-v0` | Walker S2 EDU 探索者 | 零件分拣 | 零件分拣：4 个零件 + 收纳箱（默认任务） |
| `UBTSim-WalkerS2-PickPart-v0` | Walker S2 EDU 探索者 | 零件分拣 | 单零件抓取：桌面只保留单个零件，配合 `pick_part.py` 采集 |

任务通过 YAML 配置文件定义（`config/*.yaml`），Python 任务类从 YAML 自动加载场景、机器人、相机和仿真参数。任务/机器人由 `UBT_SIM_TASK` 环境变量选择（默认天工行者无疆 客厅任务）。

### 架构

三进程协同（Python 版本不可混用）：

```
Isaac Sim (Py3.11)  ──ZMQ──►  ROS2-ZMQ Bridge (Py3.10)  ──ROS2──►  遥操作/采集脚本
```

- **仿真** `/isaac-sim/python.sh`：加载 Gym 任务（天工行者无疆 客厅 / Walker S2 EDU 探索者 仓库），JPEG/RGB 直发 ZMQ 端口。
- **桥接** `/usr/bin/python3`：ZMQ↔ROS2 双向转发，`start_sim.sh` 自动拉起。
- **采集** `/usr/bin/python3`：发布指令、录制 15Hz HDF5。

> 仿真用 Isaac Sim Python 3.11；桥接/采集用系统 Python 3.10（rclpy 所在）。

## Git Clone 注意事项

本项目使用 Git LFS 管理 3D 模型等大文件（`*.usd`、`*.urdf`、`*.exr`、`*.png`）。未安装 Git LFS 或下载不完整时，这些文件仅为指针文件，仿真无法启动。

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone <仓库地址>  # 先克隆代码，跳过 LFS
git lfs install                          # 安装 LFS（仅需一次）
git clone <仓库地址>                      # 克隆（LFS 文件自动下载）
git lfs pull                             # 补全未下载的 LFS 文件
```

## 快速使用

请根据目标机器人查看对应快速使用文档（含环境搭建、仿真启动、数据采集、相机测试、故障排查等完整步骤）：

- [Walker TienKung Pro 仿真使用说明](docs/tienkung_pro/getting-started.md)
- [Walker S2 EDU 探索者 仿真使用说明](docs/walker_s2/getting-started.md)

一句话流程：构建启动容器 → 容器内动仿真→ 运行对应采集脚本录制 HDF5 数据。

## 项目结构

```
ubt_sim/
├── assets/                # 3D 模型（USD, URDF, 贴图）
│   ├── robots/
│   └── scenes/
├── config/                # YAML 任务/场景配置
│   ├── tienkung_pro/
│   └── walker_s2/
├── dataset/               # 采集数据保存目录
├── docker/                # Docker 容器配置
├── docs/                  # 详细文档（各机器人快速使用说明）
├── scripts/               # 仿真启动脚本
├── shell/                 # Isaac Sim 运行时软链接
├── source/ubt_sim/        # Python pip 包（Py 3.11，Isaac Sim 侧）
│   ├── devices/           # 机器人配置 + 遥操作设备
│   ├── env/               # 数字孪生环境 + MDP
│   ├── task/              # Gym 任务注册
│   └── utils/             # 工具函数
└── teleoperation/         # 机器人控制脚本（ROS2 侧）
    ├── bridges/           # ROS2-ZMQ 桥接
    ├── control/           # 机器人控制 + 数据采集
    ├── image/             # 图像传输
    ├── msgs/              # ROS2 自定义消息
    └── tools/             # 诊断工具
```

## License

Apache-2.0
