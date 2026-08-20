# Walker S2 EDU 探索者

Walker S2 EDU 探索者 人形机器人，34 DOF，双手为 PGC 两指夹爪（可选 V4 灵巧手）。

![Walker S2 EDU 探索者 仿真界面预览](../assets/walker-s2仿真界面预览.png)

## 机型规格

| 项目 | 规格 |
|------|------|
| DOF | 34（17 body + 双手） |
| 末端执行器 | 2×2 PGC 两指夹爪（可选 V4 灵巧手 7DOF×2） |
| 常用训练维度 | 10D（右臂 7 + 头 2 + 右夹爪 1）/ 19D（17 body + 2 夹爪）/ 31D（17 body + V4 双手 14） |
| LeRobot 插件 | `ubt_IL/walker/lerobot_robot_walker` |
| USD 模型 | `ubt_sim/assets/robots/walker_s2/s2_v1.usd` |
| 相机 | 四路：头部双目 stereo_left/right + 腕部 wrist_left/right |

> 单臂/单夹爪采集数据中恒定维度会引发训练归一化炸裂（QUANTILES NaN/Inf），真机数据建议降维到 10D（右臂+头+右夹爪）训练。

## 仿真任务场景

| 任务 ID | 场景 | 说明 |
|---------|------|------|
| `UBTSim-WalkerS2-PartSorting-v0` | 零件分拣 | 4 个零件 + 收纳箱（默认任务） |
| `UBTSim-WalkerS2-PickPart-v0` | 零件分拣 | 单零件抓取：桌面只保留单个零件，配合 `pick_part.py` 采集 |

## 文档线

平台入口（通用架构与命令见平台概览页）：

- [仿真平台 · Walker S2 EDU 探索者](sim-setup.md) - 容器构建、启动仿真、批量采集、机器人控制器工具
- [模仿学习平台 · Walker S2 EDU 探索者](convert-train.md) - HDF5 -> LeRobot 转换、ACT / Pi0.5 训练、离线评估

按使用顺序：

1. [仿真全流程](sim-workflow.md) - 端到端：仿真采集 -> 转换 -> 训练 -> 评估 -> 回注仿真部署
2. [真机全流程](real-workflow.md) - 端到端：真机采集 -> 转换 -> 训练 -> 评估 -> 真机部署
3. [真机部署](deploy.md) - rollout 部署详解、安全预检、ROBOT_MODELS 注册表、在线评估
4. [推理服务器](inference-server.md) - 常驻预热、ZMQ 指令、远程拉起、热切换模型

## 部署效果

**仿真部署**

<video src="../assets/walker仿真部署效果.mp4" controls muted width="100%"></video>

**真机部署**

<video src="../assets/walker真机部署效果.mp4" controls muted width="100%"></video>
