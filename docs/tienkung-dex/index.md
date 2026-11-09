# 天工行者 Dex（Walker TienKung Dex）

天工行者 Dex（Walker TienKung Dex）为天工行者系列的单臂灵巧操作机型：右臂 7 DOF + 右手灵巧手，支持 **因时 Inspire RH5DG2**（67 revolute）与 **强脑 BrainCo Revo2**（53 revolute）两种手型。策略只控制右臂与右手，左臂/腰/头/腿由 `robot_ready.sh` 摆到预备位后保持不动。

![天工行者 Dex 仿真界面预览](../assets/tienkung-dex仿真界面预览.png)

## 机型规格

| 项目 | 规格 |
|------|------|
| 构型 | 右臂 7 DOF + 右手灵巧手（因时 13 通道 / 强脑 6 电机） |
| 灵巧手 | 因时 Inspire RH5DG2（67 revolute）/ 强脑 BrainCo Revo2（53 revolute） |
| 常用训练维度 | 20D（右臂 7 + 右手 13，因时）/ 13D（右臂 7 + 右手 6，强脑或因时 r13） |
| 不参与策略的关节 | 左臂 / 腰 / 头 / 腿（数据里全程恒定，摆到预备位后保持） |
| LeRobot 插件 | `ubt_IL/tienkung_dex/lerobot_robot_tienkung_dex`（注册名 `tienkung_dex`，相机类型 `dex_camera`） |
| USD 模型 | `ubt_sim/assets/robots/tienkung_dex/usd/tienkung_dex_inspire.usd` / `tienkung_dex_brainco.usd` |
| 相机 | 头部单目 RGB：`/ob_camera_head/color/image_raw`（真机/仿真同族） |
| 训练分辨率 | 仿真 / 强脑真机 640×360 RGB；因时 r13 真机配置为 1280×720 |
| ROS 域 | 默认 `ROS_DOMAIN_ID=0`（仿真与真机同域，隔离时两边都设 `146`） |

> ✅ 真机已验证（2026-11）：因时 Inspire 与强脑 BrainCo 两款手型的真机训练、部署闭环均已跑通。已验证数据集 `tienkung_dex_real_pick_apple_brainco_13d`（32 集 / 16956 帧 / 30 fps，13D = 右臂 7 + 强脑手 6）。

## 动作空间（三种 `JOINT_CONFIG`）

| `JOINT_CONFIG` | 维度 | 手 | 策略侧手量纲 |
|---|---|---|---|
| `tienkung_dex_20` | 20D = 右臂 7 + 右手 13 | 因时 Inspire | 13 维关节角 rad |
| `tienkung_dex_brainco_13` | 13D = 右臂 7 + 右手 6 | 强脑 BrainCo | 6 维闭合度 0~1 |
| `tienkung_dex_inspire_13` | 13D = 右臂 7 + 右手 6 | 因时 Inspire | 6 维归一化 0~1，Bridge2 展开成 13 个角码 |

## 仿真任务场景

| 任务 ID | 手部 | 关节数 | 配置 |
|---------|------|--------|------|
| `UBTSim-TienkungDex-Parlor-v0` | Inspire RH5DG2 | 67 | `config/tienkung_dex/parlor.yaml` |
| `UBTSim-TienkungDex-Brainco-Parlor-v0` | BrainCo Revo2 | 53 | `config/tienkung_dex/brainco_parlor.yaml` |

## 文档线

平台入口（通用架构与命令见平台概览页）：

- [仿真平台 · 天工行者 Dex](sim-setup.md) - 容器构建、启动仿真、批量采集、相机测试、数据回放
- [模仿学习平台 · 天工行者 Dex](convert-train.md) - HDF5 -> LeRobot 转换、ACT 训练、离线评估、部署参数

按使用顺序：

1. [仿真全流程](sim-workflow.md) - 端到端：仿真采集 -> 转换 -> 训练 -> 评估 -> 回注仿真部署
2. [真机全流程](real-workflow.md) - 端到端：真机采集 -> 转换 -> 训练 -> 评估 -> 真机部署
3. [机器人本体部署](arm-deploy.md) - Thor / Jazzy 板端无容器部署（venv + 系统 Python 双栈）

## 部署效果

**仿真部署**

<video src="../assets/tienkung-dex仿真部署效果.mp4" controls muted width="100%"></video>

**真机部署**

<video src="../assets/tienkung-dex真机部署效果.mp4" controls muted width="100%"></video>
