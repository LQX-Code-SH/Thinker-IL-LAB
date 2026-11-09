# Walker C1 EDU 共创者

Walker C1 EDU 共创者 人形机器人，全机 53 DOF，上肢 41 关节可控。双臂各 7 DOF，双手为 11 关节灵巧手（SDK 侧压缩为 6 个通道），头部单目 RGB 相机。

![Walker C1 EDU 共创者 仿真界面预览](../assets/walker-c1仿真界面预览.png)

## 机型规格

| 项目 | 规格 |
|------|------|
| DOF | 53（双臂 14 + 双手 11×2 + 头 2 + 腰 3 + 腿 12） |
| 末端执行器 | 2×灵巧手（每手 11 关节；SDK 6 通道：`thumb_swing` / `thumb_mcp` / `index_mcp` / `middle_mcp` / `ring_mcp` / `little_mcp`） |
| 常用训练维度 | 26D（左臂 7 + 右臂 7 + 左手 6 + 右手 6）/ 13D（右臂 7 + 右手 6，单臂任务） |
| 不参与策略的关节 | 头 2 / 腰 3 / 腿 12（保持机器人当前位；仿真锚定 HOME_POSE） |
| LeRobot 插件 | `ubt_IL/walker_c1/lerobot_robot_walker_c1`（注册名 `walker_c1`） |
| USD 模型 | `ubt_sim/assets/robots/walker_c1/usd/walker_c1_with_hands.usd` |
| 相机 | 头部单目 RGB：仿真 `/sensor/camera/head/color/raw`，真机 `/sensor/camera/head_stereo_left/color/raw` |
| 训练分辨率 | 仿真 320×240；真机 640×512（5:4） |
| ROS 域 | 默认 `ROS_DOMAIN_ID=0`（仿真与真机同域，靠域 ID 隔离） |

> 真机已验证（2026-11）：13D（右臂 7 + 右手 6）、30 fps、头相机 640×512，数据集 `walker_c1_real_13d_1RGB`（34 集 / 11406 帧）→ 模型 `walker_c1_real_13d_act_1RGB`，真机部署闭环已跑通。

## 仿真任务场景

| 任务 ID | 场景 | 说明 |
|---------|------|------|
| `UBTSim-WalkerC1-Parlor-v0` | 客厅抓放 | 苹果抓取 + 放置，在线 IK 采集（默认任务） |

C1 与 Walker S2 的仿真 ZMQ 端口同为 5655-5658，**同一时间只能运行一个机型**。

## 文档线

平台入口（通用架构与命令见平台概览页）：

- [仿真平台 · Walker C1 EDU 共创者](sim-setup.md) - 容器构建、启动仿真、批量采集、复位与数据回放
- [模仿学习平台 · Walker C1 EDU 共创者](convert-train.md) - HDF5 -> LeRobot 转换、ACT 训练、离线评估、部署参数

按使用顺序：

1. [仿真全流程](sim-workflow.md) - 端到端：仿真采集 -> 转换 -> 训练 -> 评估 -> 回注仿真部署
2. [真机全流程](real-workflow.md) - 端到端：half_sdk 前置 -> 真机采集 -> 转换 -> 训练 -> 真机部署
3. [真机部署](deploy.md) - half_sdk 门禁、摆位/回 home、ROBOT_MODELS 注册表、安全预检
4. [推理服务器](inference-server.md) - 常驻预热、ZMQ 指令、远程拉起、热切换模型
5. [技术架构](architecture.md) - 26D/13D 契约、ZMQ 端口、ROS2 话题、手部映射

## 部署效果

**仿真部署**

<video src="../assets/walker-c1仿真部署效果.mp4" controls muted width="100%"></video>

**真机部署**

<video src="../assets/walker-c1真机部署效果.mp4" controls muted width="100%"></video>
