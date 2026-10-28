# 天工行者 Dex V3 仿真数据「采集 → 转换 → 训练 → 评估 → 部署」工作流

> 机型：**TienKung Dex（天工行者 Dex V3）** — 右臂 7 + 右手 13（因时 Inspire）
> 仿真任务：`UBTSim-TienkungDex-Parlor-v0`（强脑手用 `UBTSim-TienkungDex-Brainco-Parlor-v0`）
> 机器人：`--robot.type=tienkung_dex`，`ROS_DOMAIN_ID=146`

## 目录

- [0. 工作流总览](#0-工作流总览)
- [1. 前置：仿真数据采集（ubt_sim）](#1-前置仿真数据采集ubt_sim)
- [2. 启动容器（ubt_IL）](#2-启动容器ubt_il)
- [3. 数据转换（HDF5 → LeRobot）](#3-数据转换hdf5--lerobot)
- [4. 模型训练](#4-模型训练)
- [5. 策略评估（离线 MSE）](#5-策略评估离线-mse)
- [6. 模型部署（仿真）](#6-模型部署仿真)
- [7. 常见问题](#7-常见问题)

## 0. 工作流总览

```
ubt_sim 采集 (trajectory.hdf5)
      │  右臂 7 + 右手 13（其余维度恒定，丢弃）
      ▼
scripts/convert/tienkung_dex/convert.sh  ──► dataset/tienkung_dex_sim_pick_apple (30 集 / 20 维 / 15 fps)
      ▼
scripts/train/tienkung_dex/train.sh      ──► model/tienkung_dex_sim_pick_apple_act
      ▼
scripts/eval/eval_policy.py              ──► 离线 MSE + 逐维曲线
      ▼
robot_ready.sh → rollout.sh              ──► 回注仿真，判采集成败
```

两个必须记住的约束：

1. **动作空间只有 20 维**。左臂 / 腰 / 头 / 腿在数据里 range ≤ 0.007 rad，属于死维，进数据集会
   让 QUANTILES 归一化炸裂（`NaN`/`Inf` 直接毁训练）。当前转换配置已经把它们排除。
2. **部署前必须先把机器人摆到预备位**（`robot_ready.sh`）。预备位是**绝对**目标，不是回零，
   也不是"相对当帧读数"。

## 1. 前置：仿真数据采集（ubt_sim）

```bash
# ① 在 ubt_sim 容器内启动仿真（任务名含 TienkungDex 即自动识别机型；
#    start_sim.sh 会顺手拉起 ROS2-ZMQ 桥接，那是 ubt_sim 自己的 5755-5758，与 IL 侧的
#    5559/5560 无关。不要加 UBT_SIM_NO_BRIDGE=1——IL 的 /arm/cmd 就是靠它转给仿真进程的）
cd ubt_sim
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh

# ② 数据采集（脚本自己 source ROS 环境，默认 ROS_DOMAIN_ID=146）
bash teleoperation/control/tienkung_dex/save_data.sh                    # 手动 Enter 控制
bash teleoperation/control/tienkung_dex/save_data.sh --batch 20         # 批量采集 20 集
```

产物：`ubt_sim/dataset/tienkung_dex/<epoch>/trajectory.hdf5`，attrs 带关节名 / fps / 手型 / task。

> 已有 30 集采集结果时可直接跳到第 3 节。

## 2. 启动容器（ubt_IL）

```bash
cd ubt_IL/docker
bash run.sh build      # 首次
bash run.sh start
bash run.sh check      # 应看到 "tienkung_dex plugin / bridge / ROS2 msgs" 三行 [OK]
bash run.sh bash
```

## 3. 数据转换（HDF5 → LeRobot）

### 3.1 转换命令

```bash
cd /ubt_IL

# 先转 1 集验证链路
SRC_ROOT=/ubt_sim/dataset/tienkung_dex \
bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh --save_one true --overwrite

# 再全量
SRC_ROOT=/ubt_sim/dataset/tienkung_dex \
bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh --overwrite
```

默认参数：`CONFIG=configs/tienkung_dex_20d_1RGB.json`、`REPO_ID=tienkung_dex_sim_pick_apple`、
`FPS=15`、`HDF5_REL_PATH=trajectory.hdf5`、`ROBOT_TYPE=tienkung_dex`。
产物在 `/ubt_IL/dataset/tienkung_dex_sim_pick_apple/`。

状态 / 动作的取数：

| 字段 | HDF5 来源 | 列 |
|------|-----------|-----|
| `observation.state` | `joint_observations/joint_position` + `hand_observations/hand_right_position`（**实测值**） | `[24..30]` + `[0,1,2,4,5,6,8,9,10,12,13,15,16]` |
| `action` | `action_observations/joint_position` + `action_observations/hand_right_position`（**指令值**） | 同上 |

> **别把两者互换**：`action_*` 是录制时的指令，`*_observations` 是实测。手部 action 领先
> state 最多 1.31 rad，这正是部署时 `get_observation()` 的语义。

剔除的 5 维是五个 `*_dip_joint`（URDF 里是 mimic，数据里恒 0）。

### 3.2 数据可视化

```bash
# 在容器内
lerobot-dataset-viz --repo-id tienkung_dex_sim_pick_apple --root /ubt_IL/dataset/tienkung_dex_sim_pick_apple
```

检查点：画面朝向是否正常、手部抓放与视频是否同步、20 维曲线里手部维度有没有全程平直（平直说明列切错了）。

### 3.3 静止帧（`TRIM_STATIONARY`）

**Dex 默认不开**。实测 30 集里约 24% 相邻帧完全相同，成因是 15 Hz 录制跑在约 11 Hz 的仿真主循环上，
重复散布在整段运动中；静止裁剪只裁「长的静止 run」，既解决不了重复帧，还会切碎连续动作。

确实需要时：

```bash
STATIONARY_DIAGNOSE=1 bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh   # 先看分布
TRIM_STATIONARY=1 bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh --overwrite
```

## 4. 模型训练

```bash
# 默认配置：30 集 / 20 维 / 30000 步 / 每 5000 存 checkpoint
bash /ubt_IL/scripts/train/tienkung_dex/train.sh

# 覆盖常用参数
OUTPUT_DIR=/ubt_IL/model/tienkung_dex_sim_pick_apple_act_v2 \
STEPS=50000 SAVE_FREQ=10000 BATCH_SIZE=16 bash /ubt_IL/scripts/train/tienkung_dex/train.sh

# 断点续训（CONFIG_PATH 指向 checkpoint 内的 train_config.json）
CONFIG_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model/train_config.json \
RESUME=true bash /ubt_IL/scripts/train/tienkung_dex/train.sh
```

| 环境变量 | 默认 |
|----------|------|
| `CONFIG_PATH` | `configs/train_config_tienkung_dex_sim_pick_apple.json` |
| `OUTPUT_DIR` | `/ubt_IL/model/tienkung_dex_sim_pick_apple_act` |
| `STEPS` / `SAVE_FREQ` | `30000` / `5000` |
| `BATCH_SIZE` | 配置文件里的值（16） |
| `RESUME` | `false` |
| `WANDB_ENABLE` | `false` |

产物：`checkpoints/<step>/pretrained_model`（含 `config.json` + `model.safetensors`）+ `checkpoints/last`。

> 30 集约 16k 帧，batch 16 下 3 万步 ≈ 30 epoch，已足够；再往上过拟合风险明显。
> `save_freq` 每 5k 存一个，可用离线 MSE + 仿真回注择优。

## 5. 策略评估（离线 MSE）

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/tienkung_dex_sim_pick_apple \
  --episodes 5 --inference-freq 1 --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_tienkung_dex_sim_pick_apple_act \
  --output /ubt_IL/scripts/eval/output/eval_tienkung_dex_sim_pick_apple_act/results.json \
  --device cuda
```

`--inference-freq 1` 表示逐帧推理，最接近部署时的行为；默认值会按 `n_action_steps` 排队。

检查点：20 维逐维曲线里**手部维度不应全程平直**，否则大概率是转换列切错或模型没学到手部。

## 6. 模型部署（仿真）

```bash
# ① ubt_sim 容器内：启动仿真（另开终端）
cd ubt_sim
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh

# ② 推理容器内：回预备位（四段到位 + 手型），先看相机画面确认视角正确
export ROS_DOMAIN_ID=146
bash /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.sh

# ③ 跑策略
POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_20 FPS=15 DURATION=60 \
bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh

# ④ 复位场景，准备下一集
ROS_DOMAIN_ID=146 /usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_dex/reset.py
```

常驻推理服务（预热一次，之后秒级 start/stop）：

```bash
POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_20 FPS=15 \
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_server.sh

bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh status
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh start
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh stop
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh shutdown
```

判成功：仿真 `/sim/task_completed` 的误差 < 0.15 m，5 集里至少 3 集成功。

### 控制频率

数据集是 15 Hz，仿真主循环约 11 Hz（RTF≈0.11）。`FPS` 先按 15 跑，同时观察 rollout 打印的
实际下发频率与 `/robot_state` 回传频率；如果实际远低于 15，同一串动作在仿真时间轴上会被拉长
（机械臂显得"慢半拍"），此时再调 `FPS`，**数据集保持 15 Hz 不动**。

## 7. 常见问题

| 现象 | 排查 |
|------|------|
| `robot_ready.sh` 报「Bridge2 正在运行」 | `robot_ready.py` 与 Bridge2 都写 `/arm/cmd`，二者不能同时跑；先停 rollout（`--force` 可跳过，慎用） |
| 仿真里手臂不动 | `ROS_DOMAIN_ID` 是否为 146；`/robot_state` 是否在回传（`ros2 topic hz /robot_state`） |
| 相机无画面 | 5758 是否有 JPEG 流；`dex_camera` 类型名是否写对（不是 `image_server`） |
| 动作维度预检失败 | `JOINT_CONFIG` 与训练时不一致（20 ↔ 13） |
| 手部动作整体错位 | `HAND_VARIANT` 与实物不符；Bridge2 启动日志里会打印「右手通道 -> 关节」表 |
| 部署后手臂一步跳很远 | `MAX_RELATIVE_TARGET` 留空时插件不限幅；仿真调试建议设 `0.05` |
| 训练 OOM | 降 `BATCH_SIZE`；GPU 显存被仿真占满时先关仿真 |
