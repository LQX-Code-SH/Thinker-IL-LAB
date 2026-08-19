# UBTECH-IL-LAB

优必选EDU版机器人模仿学习工具链

📖 在线文档站点：<https://lqx-code-sh.github.io/UBTECH-IL-LAB/>

## 项目简介

本项目基于 [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) 与 [LeRobot](https://github.com/huggingface/lerobot) 框架开发，为天工（TienKung）及 Walker S2 机器人提供完整的模仿学习工具链，涵盖以下核心能力：

| 能力              | 说明                                          | 状态      |
| ----------------- | --------------------------------------------- | --------- |
| 🌐 ROS 仿真环境   | 高逼真度 Isaac Sim 仿真，支持遥操作与数据采集 | ✅ 已完成 |
| 🎮 遥控操作       | 键盘/空间鼠标等设备遥操作，仿真与真机统一接口 | 🚧 开发中 |
| 📦 数据采集与转换 | HDF5 / LeRobot 格式数据采集，格式转换与清洗   | ✅ 已完成 |
| 🧠 模型训练       | 基于 LeRobot 的模仿学习策略训练               | ✅ 已完成 |
| 🤖 真机部署       | 模型推理与真机控制部署                        | ✅ 已完成 |
| 🤖 支持机型      | TienKung Pro / Walker S2(已支持)        | ✅ 已发布 |
| 🤖 其他机型       | Walker C1 / TienKung 3.0        | 🚧 待发布 |

## 效果展示

### 仿真平台预览

| TienKung Pro | Walker S2 |
|-------------|-----------|
| <img src="ubt_sim/docs/assets/tienkung-pro仿真界面预览.png" alt="TienKung Pro 仿真界面预览" width="460"> | <img src="ubt_sim/docs/assets/walker-s2仿真界面预览.png" alt="Walker S2 仿真界面预览" width="460"> |

### 测试案例

| 天工 Pro 仿真 | 天工 Pro 真机 |
|---|---|
| <video src="ubt_IL/docs/assets/tienkung仿真部署效果.mp4" controls muted width="460"></video> | <video src="ubt_IL/docs/assets/tienkung真机部署效果.mp4" controls muted width="460"></video> |

| Walker S2 仿真 | Walker S2 真机 |
|---|---|
| <video src="ubt_IL/docs/assets/walker仿真部署效果.mp4" controls muted width="460"></video> | <video src="ubt_IL/docs/assets/walker真机部署效果.mp4" controls muted width="460"></video> |

## 整体架构

仓库包含 `ubt_sim`（仿真平台）与 `ubt_IL`（模仿学习平台）两个子项目，关系与数据流：

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                 ubt_IL 模仿学习平台                                      │
│                    数据转换 ──► 模型训练 ──► 离线评估 ─► 推理部署                           │
└───────────▲───────────────┬───────────────────────────▲───────────────┬────────────────┘
            │               │④仿真部署                   │               │⑤真机部署
            │①数据采集       ▼ （ROS桥接）                │②真机采集       │（ROS桥接）
   ┌────────┴────────────────────────┐        ┌─────────┴───────────────▼───────────┐
   │         ubt_sim 仿真平台         │        │     真机（天工 Pro / Walker S2）      │
   │        （Isaac Sim 容器）        │        │     ROS_DOMAIN_ID=0 直连网段          │
   └─────────────────────────────────┘        └─────────────────────────────────────┘
```


## 代码获取

克隆仓库后，需先拉取 LFS 大文件（USD 模型、贴图等）并初始化子模块（`lerobot`）。

```bash
# （可选）配置代理：访问 GitHub 较慢时设置，端口按本地代理调整
git config --global http.proxy  http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
export GIT_LFS_PROXY="http://127.0.0.1:7897"   # LFS 走代理

# 1. 克隆仓库（先跳过 LFS 和子模块）
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/LQX-Code-SH/UBTECH-IL-LAB.git

# 2. 拉取 LFS 大文件
git lfs pull

# 3. 初始化并拉取子模块（lerobot）
git submodule update --init
```

### 环境要求

| 依赖 | 要求 | 说明 |
|------|------|------|
| 操作系统 | Linux（Ubuntu 22.04 推荐） | 两个子项目均以容器方式运行 |
| GPU | NVIDIA GPU（3090+） | Isaac Sim 渲染与模型训练/推理 |
| Docker | 已安装 [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) | 容器内使用 GPU |
| Git LFS | 已安装 | USD 模型、贴图等大文件管理 |
| 磁盘空间 | 充足（建议 100GB+ 可用） | LFS 大文件、数据集、checkpoint 占用较大 |

## 快速开始

本仓库分为机器人仿真平台与模仿学习平台两个子项目，涵盖机器人操作模仿学习全流程，具体使用流程请查看下列文档教程：

- [ubt_sim 仿真平台使用说明](ubt_sim/README.md) - 机器人仿真平台（Isaac Sim）：仿真数据采集、ROS仿真验证、模仿学习仿真部署
- [ubt_IL 模仿学习平台使用说明](ubt_IL/README.md) - 模仿学习平台（LeRobot）：数据转换、数据可视化、模型训练、离线评估、真机/仿真部署

### 更多文档

| 主题 | 文档 |
|------|------|
| 仿真快速上手（天工 Pro） | [ubt_sim/docs/tienkung_pro/getting-started.md](ubt_sim/docs/tienkung_pro/getting-started.md) |
| 仿真快速上手（Walker S2） | [ubt_sim/docs/walker_s2/getting-started.md](ubt_sim/docs/walker_s2/getting-started.md) |
| 训练部署快速上手（天工 Pro） | [ubt_IL/docs/tienkung_pro/getting-started.md](ubt_IL/docs/tienkung_pro/getting-started.md) |
| 训练部署快速上手（Walker S2） | [ubt_IL/docs/walker_s2/getting-started.md](ubt_IL/docs/walker_s2/getting-started.md) |
| 模仿学习仿真工作流（天工 Pro / Walker S2） | [sim-workflow.md](ubt_IL/docs/tienkung_pro/sim-workflow.md) / [sim-workflow.md](ubt_IL/docs/walker_s2/sim-workflow.md) |
| 模仿学习真机工作流（天工 Pro / Walker S2） | [real-workflow.md](ubt_IL/docs/tienkung_pro/real-workflow.md) / [real-workflow.md](ubt_IL/docs/walker_s2/real-workflow.md) |
| 数据转换详细说明 | [ubt_IL/scripts/convert/README.md](ubt_IL/scripts/convert/README.md) |
| ARM 板（Jetson Orin）真机部署 | [ubt_IL/scripts/deploy/tienkung_pro/arm_64/README.md](ubt_IL/scripts/deploy/tienkung_pro/arm_64/README.md) |

## 致谢

本项目站在以下开源项目的肩膀上，谨致谢忱：

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) - 高保真机器人仿真环境
- [HuggingFace LeRobot](https://github.com/huggingface/lerobot) - 模仿学习框架（ACT 策略、数据格式与训练/推理工具链）
- [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) - 优必选遥操数采平台
- [优必选开源中心](https://thinkercosmos.ubtrobot.com/#/open-source-center?ref=header-nav) - 优必选开源项目集合

感谢以上社区与所有贡献者的卓越工作。如本项目对您有帮助，欢迎 Star ⭐ 支持。

## License

Apache-2.0
