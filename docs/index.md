# UBTECH-IL-LAB

优必选 EDU 版机器人模仿学习工具链，为 **天工 Pro（TienKung Pro）** 与 **Walker S2** 机器人提供从仿真数据采集到真机部署的完整模仿学习解决方案。

基于 [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) 与 [HuggingFace LeRobot](https://github.com/huggingface/lerobot) 框架构建。

[:fontawesome-brands-github: **GitHub 仓库** · LQX-Code-SH/UBTECH-IL-LAB](https://github.com/LQX-Code-SH/UBTECH-IL-LAB){ .md-button .md-button--primary }
[快速开始 :material-arrow-right:](getting-started.md){ .md-button }

## 核心能力

| 能力 | 说明 | 状态 |
|------|------|------|
| :material-robot-industrial: ROS 仿真环境 | 高逼真度 Isaac Sim 仿真，支持遥操作与数据采集 | :material-check-circle: 已完成 |
| :material-gamepad-variant: 遥控操作 | 键盘/空间鼠标等设备遥操作，仿真与真机统一接口 | :material-progress-clock: 开发中 |
| :material-database: 数据采集与转换 | HDF5 / LeRobot 格式数据采集，格式转换与清洗 | :material-check-circle: 已完成 |
| :material-brain: 模型训练 | 基于 LeRobot 的模仿学习策略训练（ACT / Pi0.5） | :material-check-circle: 已完成 |
| :material-robot: 真机部署 | 模型推理与真机控制部署（含推理服务器常驻预热） | :material-check-circle: 已完成 |
| :material-robot: 支持机型 | TienKung Pro / Walker S2 | :material-check-circle: 已发布 |
| :material-robot-outline: 其他机型 | Walker C1 / TienKung 3.0 | :material-progress-clock: 待发布 |

## 效果展示

### 仿真平台预览

| TienKung Pro | Walker S2 |
|-------------|-----------|
| ![TienKung Pro 仿真界面预览](assets/tienkung-pro仿真界面预览.png) | ![Walker S2 仿真界面预览](assets/walker-s2仿真界面预览.png) |

### 部署效果

<div class="grid video-grid" markdown>
<div markdown>

**天工 Pro 仿真**

<video src="assets/tienkung仿真部署效果.mp4" controls muted width="100%"></video>

</div>
<div markdown>

**天工 Pro 真机**

<video src="assets/tienkung真机部署效果.mp4" controls muted width="100%"></video>

</div>
<div markdown>

**Walker S2 仿真**

<video src="assets/walker仿真部署效果.mp4" controls muted width="100%"></video>

</div>
<div markdown>

**Walker S2 真机**

<video src="assets/walker真机部署效果.mp4" controls muted width="100%"></video>

</div>
</div>

## 整体架构

仓库包含 `ubt_sim`（仿真平台）与 `ubt_IL`（模仿学习平台）两个子项目，关系与数据流：

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

详细说明见 [架构总览](architecture.md)。

## 从这里开始

- [快速开始](getting-started.md) - 克隆代码、环境要求、全流程一览
- [:material-robot: 天工 Pro](tienkung/index.md) - 仿真 / 转换 / 训练 / 仿真与真机部署 / ARM 板端部署
- [:material-robot-outline: Walker S2](walker-s2/index.md) - 仿真 / 转换 / 训练 / 部署 / 推理服务器
- [通用参考](common/data-conversion.md) - 数据转换详解、数据回放、HuggingFace 数据/模型管理

## 致谢

本项目站在以下开源项目的肩膀上，谨致谢忱：

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) - 高保真机器人仿真环境
- [HuggingFace LeRobot](https://github.com/huggingface/lerobot) - 模仿学习框架（ACT 策略、数据格式与训练/推理工具链）
- [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) - 优必选遥操数采平台
- [Thinker cosmos开源社区](https://thinkercosmos.ubtrobot.com/#/open-source-center?ref=header-nav) - 优必选开源中心

## License

Apache-2.0
