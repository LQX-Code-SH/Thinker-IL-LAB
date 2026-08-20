# Walker C1 真机 VLA 开发交接

> 面向接手开发人员的执行版，更新于 2026-08-20。
> 当前分支：`walker_c1`；已知代码基线：`73c4a18`。
> 仿真部分可以详细看README_C1_LEROBOT.md和PICO_README.md。

## 1. 项目目标

把下面这条真机链路做完整：

```text
Thinker Studio 采集 C1 真机示教数据
→ 转换为 LeRobot Dataset
→ 训练 Diffusion / VLA 模型
→ 离线和影子模式验证
→ 部署到 C1 真机完成苹果抓取
```

当前仿真链路基本可用，真机链路还没有闭环。不要把仓库中的 C1 仿真 rollout 脚本直接用于真机。

## 2. 当前做到哪里了

### 已有

- C1 Isaac Sim 场景、自动抓放和 PICO 遥操作。
- C1 仿真 HDF5 转 LeRobot，支持：
  - 26 维：双臂 14 维 + 双手逻辑量 12 维。
  - 36 维：双臂 14 维 + 双手物理关节 22 维。
- C1 26 维 Diffusion 训练配置。
- C1 仿真推理和苹果位置随机化。
- Walker 通用 LeRobot Robot、ROS 2/ZMQ Bridge2 和相机接入。
- C1 真机 ROS 只读检查说明。
- S2 的 Thinker Studio 数据转换器和 Pi0.5 配置，可作为实现参考。

### 缺少

- 一份经过分析的 Thinker Studio C1 原始数据样本。
- C1 Thinker Studio 专用检查器和转换器。
- C1 真机专用 robot config。
- C1 真机 rollout 脚本和影子模式。
- 首帧平滑、状态/图像超时、动作 watchdog 等真机安全机制。
- C1 真机训练和部署验证。
- C1 语言条件 Pi0.5/VLA 配置。

当前 C1 正式训练配置是 **Diffusion Policy，不是真正使用语言的 VLA**。命令行中出现 task 字符串，不代表模型使用了语言输入。

## 3. 先看这些文件

| 文件 | 内容 |
|---|---|
| [C1_REAL_ROBOT_READONLY_CHECK.md](C1_REAL_ROBOT_READONLY_CHECK.md) | 第一次连接真机的只读检查 |
| [README_C1_LEROBOT.md](README_C1_LEROBOT.md) | C1 仿真采集、转换、训练和推理 |
| [C1_joint_map.md](C1_joint_map.md) | C1 关节名、顺序和限位 |
| `ubt_IL/scripts/convert/convert_walker_real_to_lerobot_v3.py` | S2 Thinker Studio 转换参考 |
| `ubt_IL/scripts/convert/configs/Walker_C1_26_1RGB.json` | C1 26 维数据定义 |
| `ubt_IL/scripts/deploy/train_config_walker_c1_diffusion.json` | C1 Diffusion 训练参数 |
| `ubt_IL/scripts/deploy/train_config_walker_s2_real_pi05.json` | S2 Pi0.5/VLA 参考配置 |
| `ubt_IL/scripts/deploy/configs/walker/walker_c1_26d.json` | 当前 C1 配置，主要用于仿真 |
| `ubt_IL/scripts/deploy/rollout_walker_c1.sh` | C1 仿真专用 rollout |
| `ubt_IL/walker/ros2_walker_bridge.py` | ROS 2/ZMQ 控制桥 |

## 4. 推荐开发顺序

### 第一步：确认真机 ROS 接口

按照 [C1_REAL_ROBOT_READONLY_CHECK.md](C1_REAL_ROBOT_READONLY_CHECK.md) 操作，只订阅，不发布。

必须保存以下信息：

- `/mc/sdk/robot_state` 的真实消息和完整关节名。
- 左右手状态和命令 topic。
- 头部相机 topic、消息类型、分辨率、encoding 和实际 FPS。
- 各 topic 的 QoS。
- 真机和开发机 IP、`ROS_DOMAIN_ID`。

重点未决项：代码使用 `elbow_pitch_joint`，部分 SDK 材料中出现过 `elbow_roll_joint`。必须以目标真机消息中的名字为准。

验收：开发机能够持续接收真机状态和图像，不启动任何控制程序。

### 第二步：分析 Thinker Studio 数据

先拿三条 C1 原始 episode：

1. 双臂小幅运动、双手开合的标定数据。
2. 固定位置苹果抓取成功数据。
3. 苹果随机位置数据或失败数据。

不要一开始批量转换。先确认：

- HDF5 内部完整路径和 JSON 数据结构。
- 身体 state、身体 command、左右手 state/command 分别记录在哪里。
- 所有字段的关节名、顺序和单位。
- MP4 路径、尺寸、颜色和真实帧率。
- 相机、状态和命令时间戳是否同一时钟。
- 命令和关节反馈之间的延迟。

现有 S2 转换器读取 Thinker Studio 的 `metadata_aligned.hdf5` 和对齐 MP4，但其身体 action 使用观测位置、手部使用夹爪命令，而且只支持 S2 夹爪。不能直接用于 C1。

建议新增：

```text
ubt_IL/scripts/convert/inspect_thinker_c1.py
ubt_IL/scripts/convert/convert_walker_c1_real_to_lerobot_v3.py
ubt_IL/scripts/convert/configs/Walker_C1_real_26_1RGB.json
```

检查器至少输出：字段树、关节名、数值范围、各模态 FPS、掉帧、时间差、视频信息和命令—反馈延迟。

验收：明确写下 `observation.state[t]` 和 `action[t]` 分别来自哪个原始字段，不能用“暂时按数组顺序”或“先复制 state”代替。

### 第三步：转换真机数据

首版建议使用 26 维：

```text
左臂 7 + 右臂 7 + 左逻辑手 6 + 右逻辑手 6
```

转换器要求：

- 按关节名映射，不依赖原始数组顺序。
- 缺失、重复或未知关节默认报错。
- 手部物理量到 6 维逻辑量的映射有明确标定。
- 图像、state 和 action 按时间戳同步。
- 输出 LeRobot feature names。
- 输出每条 episode 的质量报告和首末帧预览。
- 转换完成后重新读取数据，检查 shape、NaN、视频和帧数。

数据目标仍然是 30 Hz。但必须统计真实时间戳：如果实际只有 12.5 或 15 Hz，不能只把数据集 metadata 写成 30 Hz。需要修采集或显式重采样。

固定位置和随机位置都要采集：

- 固定位置用于建立基线和排查系统问题。
- 随机位置用于泛化，必须记录随机区域或实际坐标。
- 训练集和验证集按 episode/位置划分，不能按帧随机划分。

验收：随机播放至少 10 条转换后的 episode，人工确认图像、state 和 action 同步，左右臂、左右手、方向和单位正确。

### 第四步：先训练 Diffusion 基线

当前 C1 配置：

```text
ubt_IL/scripts/deploy/train_config_walker_c1_diffusion.json
```

关键参数：

| 参数 | 值 | 含义 |
|---|---:|---|
| `n_obs_steps` | 2 | 使用最近两个观测 |
| `horizon` | 16 | 每次预测 16 个候选动作 |
| `n_action_steps` | 8 | 每次实际执行 8 步后重新推理 |
| `drop_n_last_frames` | 7 | 避免训练窗口越过 episode 末尾 |
| `num_train_timesteps` | 100 | DDPM 训练噪声步数 |
| `num_inference_steps` | 10 | 一次推理的去噪步数 |
| `batch_size` | 8 | 训练 batch |
| `steps` | 50000 | 正式训练步数 |

按 30 Hz 计算：预测 16 步覆盖约 0.53 秒，执行 8 步约 0.27 秒。`n_action_steps=8` 和 `num_inference_steps=10` 是两件事。

新建真机训练配置，不要覆盖仿真配置：

```text
ubt_IL/scripts/deploy/train_config_walker_c1_real_diffusion.json
```

主要修改数据路径、图像 shape、输出目录和数据增强。配置完成后的命令形式：

```bash
cd /ubt_IL
/lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/deploy/train_config_walker_c1_real_diffusion.json
```

该真机配置目前尚未创建，不能直接执行上面的命令。

训练验收：

- smoke training 能保存和重载 checkpoint。
- 固定输入推理无 NaN/Inf。
- 输出严格为 26 维，顺序与数据集一致。
- 输出在关节限位内，单步变化合理。
- 有验证集预测动作与标注动作的可视化，而不只是 loss。

### 第五步：补真机安全部署

不要修改仿真配置来兼容真机。新建：

```text
ubt_IL/scripts/deploy/configs/walker/walker_c1_26d_real.json
ubt_IL/scripts/deploy/rollout_walker_c1_real.sh
```

真机 config 必须：

- 删除或设空 `/sim/object_state`、`/sim/cmd_set_object_pose`。
- 使用真机实际关节名和相机配置。
- 启用非空的 `max_relative_target`，初值需保守并由现场验证。
- 增加状态、图像和动作超时配置。
- 明确 disconnect 是保持、回 home 还是停止发布。

当前实现的已知风险：

- Bridge2 第一个身体目标从目标本身开始插值，没有从实测当前姿态平滑过去。
- 当前 C1 配置的 `max_relative_target` 是 `null`。
- `disable_torque_on_disconnect=true` 实际会发送 home，不是真正断力矩。
- 缺少策略/网络中断后的 stale-action watchdog。
- 相机在 Bridge2 回调内做 JPEG 和 base64，历史吞吐可能只有 10–13 Hz。

需要实现：

1. 首帧从最新实测关节位置平滑到目标。
2. state/image freshness 检查。
3. 动作超时 watchdog。
4. 关节限位、单周期增量和速度限制。
5. 影子模式：读取实时数据并计算动作，但绝不发布命令。
6. 显式真机确认，例如 `CONFIRM_REAL_ROBOT=YES`。
7. 默认短时运行，并记录观测、预测、发布值和停止原因。

验收顺序：

```text
只读 ROS
→ 离线 checkpoint 推理
→ 真机实时影子推理
→ 单关节极小动作
→ 双臂/手部低速短时测试
→ 固定位置苹果单次任务
→ 固定位置重复统计
→ 苹果随机位置测试
```

真机现场必须有急停人员。任何图像/状态超时、NaN、越限、方向错误、跳变或振荡都立即停止。

### 第六步：再做 Pi0.5/VLA

参考：

```text
ubt_IL/scripts/deploy/train_config_walker_s2_real_pi05.json
```

C1 建议先做 26 维，因为参考模型的 `max_state_dim` 和 `max_action_dim` 是 32；36 维不能直接套用。

注意：参考 Pi0.5 配置的 action chunk 是 50。30 Hz 下相当于约 1.67 秒动作，第一次真机运行太长。可以保留训练 chunk，但部署时减少每次真正执行的步数并频繁重规划。

VLA 验收除了动作指标，还要确认：

- task text 确实写入数据集。
- tokenizer 和模型真的读取了 task text。
- 改变指令会产生合理、可解释的策略变化。
- 在相同数据划分和真机协议下与 Diffusion 基线比较。

## 5. 仿真和真机部署差别

策略 API 可以基本复用，但系统层差别很大：

| 仿真 | 真机 |
|---|---|
| Domain 默认 146 | 通常使用 Domain 0 |
| 可读取苹果真值并自动随机化 | 需要人工摆放或外部定位 |
| 可自动 reset | 需要安全回位和场景确认 |
| 相机和时钟较稳定 | 有网络、编码、掉帧和延迟 |
| 关节名由仿真模型确定 | 必须读取目标真机确认 |
| 失败后可以直接重启 | 需要急停、watchdog、限位和现场监护 |
| 可用仿真真值判定成功 | 需要视觉、传感器或人工标注 |

所以结论是：**模型调用差别不大，真机集成和安全工作量很大。**

## 6. 第一周建议任务

按优先级完成：

1. 阅读第 3 节列出的文件。
2. 完成一次真机只读 ROS 检查并保存结果。
3. 获取一条 Thinker Studio C1 标定 episode。
4. 写数据结构/FPS/时间戳检查器。
5. 冻结 26 维 state/action 数据契约。
6. 开始 C1 真机转换器，不接真机控制。

不要把第一周花在调大模型或直接跑真机上。数据动作语义和真机安全底座没有确认前，模型效果没有可信度。

## 7. 完成标准

满足以下条件才可以写“C1 真机 VLA 工具链完成”：

- Thinker Studio 原始数据可重复转换为 LeRobot Dataset。
- 有自动质量报告，证明关节映射、频率和同步正确。
- Diffusion 真机基线有固定位置和随机位置统计。
- Pi0.5/VLA 确实使用语言输入并通过同一协议验证。
- 仿真和真机配置、启动脚本完全分离。
- 影子模式、首帧平滑、限位、超时和 watchdog 经过测试。
- 断网、程序退出和急停的行为得到硬件团队确认。
- 实验日志能追踪代码、数据、配置、checkpoint 和停止原因。

## 8. 需要尽快确认

- Thinker Studio 版本、工程文件和导出步骤。
- 一条 C1 原始数据及字段说明。
- 身体和灵巧手的真实 command 字段。
- 真机 IP、ROS setup 和软件/固件版本。
- 真机关节名和灵巧手 6 维映射。
- 相机 topic、类型、encoding、分辨率和 FPS。
- 厂家认可的 hold、home、停止发布和 torque-off 行为。
- 急停流程、允许速度和安全工作区。
- Pi0.5 基础权重、权限和训练机器位置。

其中任何一项不明确，都应先做只读验证或向硬件团队确认，不要通过真机动作猜测。
