# Walker C1 快速开始

Walker C1（Astron）双臂 + 双手灵巧手的模仿学习链路：HDF5 采集 → LeRobot 数据集 →
ACT 训练 → 离线评估 → 仿真/真机部署。

- **策略维度**：26（左臂 7 + 右臂 7 + 左手 6 + 右手 6）或 13（右臂 7 + 右手 6，左臂/左手保持 home）
- **不控制**：头 2 / 腰 3 / 腿 12（机器人自身保持当前位；仿真锚定 HOME_POSE）
- **相机**：头部单目 RGB，训练分辨率 320×240
  - 仿真：`/sensor/camera/head/color/raw`（`sensor_msgs/Image`）
  - 真机：`/sensor/camera/head_stereo_left/color/compressed`（`sensor_msgs/CompressedImage`）
- **手部**：真机 SDK 6 个压缩通道（`thumb_swing / thumb_mcp / index_mcp / middle_mcp / ring_mcp / little_mcp`）

## 目录

1. [克隆与数据](#1-克隆与数据)
2. [构建环境](#2-构建环境)
3. [数据转换](#3-数据转换)
4. [模型训练](#4-模型训练)
5. [模型评估](#5-模型评估)
6. [模型部署](#6-模型部署)
7. [常见问题](#7-常见问题)

## 1. 克隆与数据

```bash
git clone <repo> UBTECH-IL-LAB
cd UBTECH-IL-LAB
```

仿真数据由 `ubt_sim` 采集，落在 `ubt_sim/dataset/walker_c1/<epoch秒>/trajectory.hdf5`。
`ubt_IL` 容器会把 `ubt_sim/` 只读挂载到 `/ubt_sim`，转换时可直接读取，无需拷贝。

单集 HDF5 结构（与 tienkung_pro/dex 同族）：

| 路径 | 形状 | 含义 |
|---|---|---|
| `joint_observations/joint_position` | (T, 31) | 身体关节：L 臂 0-6 / R 臂 7-13 / 头 14-15 / 腰 16-18 / 腿 19-30 |
| `hand_observations/hand_left_position` | (T, 6) | 左手 SDK 6 通道 |
| `hand_observations/hand_right_position` | (T, 6) | 右手 SDK 6 通道 |
| `action_observations/*` | 同上 | 最近一次下发指令 |
| `camera_observations/color_images/camera_head` | (T,) | PNG 字节流，640×480 |
| `timestamp` | (T,) | 采集时刻 |

## 2. 构建环境

```bash
cd ubt_IL/docker
bash run.sh build && bash run.sh start && bash run.sh check
bash run.sh bash
```

容器启动时 `entrypoint.sh` 会自动：

- 安装 `lerobot`（editable）
- 编译/加载 `mc_task_msgs` / `mc_state_msgs`（复用 `walker_s2/walker_sdk_ros2` 工作空间）
- 安装 `lerobot_robot_walker_c1`（editable）

也可手动安装：

```bash
python /ubt_IL/walker_c1/init_walker_c1.py
# 输出 Robot type: walker_c1 / Camera type: walker_c1_camera
```

## 3. 数据转换

```bash
# 容器内；默认读 ubt_IL/dataset/walker_c1，缺失自动回退 /ubt_sim/dataset/walker_c1
bash /ubt_IL/scripts/convert/walker_c1/convert.sh

# 先验证一集
bash /ubt_IL/scripts/convert/walker_c1/convert.sh --save_one true --overwrite
```

产物：`/ubt_IL/dataset/walker_c1_sim_26d_1RGB`（26 维、1 路 240×320、15 fps）。
单臂任务可用 13 维：`DIM=13 bash .../convert.sh` → `walker_c1_sim_13d_1RGB`。
检查 `meta/info.json`：`observation.state`/`action` 的 `shape` 为 `[26]`，
`observation.images.camera_head` 为 `[3,240,320]`。

## 4. 模型训练

```bash
# smoke
bash /ubt_IL/scripts/train/walker_c1/train.sh --steps=2 --save_checkpoint=false

# 常规训练
bash /ubt_IL/scripts/train/walker_c1/train.sh
```

默认 ACT、`steps=30000`、`batch_size=8`，输出
`/ubt_IL/model/walker_c1_sim_26d_act_1RGB`。详见
[scripts/train/walker_c1/README.md](../../scripts/train/walker_c1/README.md)。

## 5. 模型评估

```bash
python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/walker_c1_sim_26d_1RGB \
  --episodes 5 --inference-freq 15 --plot \
  --output /ubt_IL/scripts/eval/output/eval_walker_c1_sim_26d_act/results.json
```

13 维 ACT（30k 步）在 30 集仿真数据上的离线评估结果：

![walker_c1 离线评估曲线](../assets/walker_c1模型离线评估曲线.png)

`--plot` 会把逐关节 GT vs Pred 曲线写到 `--output` 同目录的 `plots/` 下
（每个 episode 一张，30 集共 30 张）。

## 6. 模型部署

```bash
# 仿真：先起 ubt_sim，再在容器内
export ROS_DOMAIN_ID=0
python3 /ubt_IL/scripts/deploy/walker_c1/preview_camera.py --once --save-frame /tmp/c1.jpg
bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh

POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh

# 真机（Thor 本体）：相机走压缩话题，DDS 走 CycloneDDS（run.sh 已注入）
python3 /ubt_IL/scripts/deploy/walker_c1/preview_camera.py --once --save-frame /tmp/c1_real.jpg \
  --topic /sensor/camera/head_stereo_left/color/compressed

POLICY_PATH=/path/to/pretrained_model DURATION=10 RESET_MODE=task \
CAMERA_TOPIC=/sensor/camera/head_stereo_left/color/compressed \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh
```

真机、常驻推理服务与参数细节见：

- [sim-workflow.md](sim-workflow.md)
- [real-workflow.md](real-workflow.md)
- [inference-server.md](inference-server.md)
- [architecture.md](architecture.md)（26 维契约 / 端口 / 话题 / hand 映射）

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| `joint_config 'walker_c1_26d' not registered` | 插件未安装：`python /ubt_IL/walker_c1/init_walker_c1.py` |
| 转换报「维度 = '31 6' 要求不符」 | 数据不是 ubt_sim walker_c1 采集；确认来源或 `CONFIG=` 显式指定 |
| 训练报相机 key 不存在 | 转换配置的 `observation.images.camera_head` 未被写入：检查 `meta/info.json` |
| rollout 预检维度不符 | 策略不是 26 维；确认训练用的是 `walker_c1_sim_26d_1RGB` |
| 相机超时 | 话题名/`ROS_DOMAIN_ID`/QoS（必须 BEST_EFFORT）；真机话题是 `head_stereo_left/color/compressed` |
| `torch.cuda.is_available()` False | 容器缺 `NVIDIA_VISIBLE_DEVICES`/`NVIDIA_DRIVER_CAPABILITIES`；用 `run.sh start` 启动 |
| 左手完全不动 | 当前采集批次左手全程 ≈0（任务只用右手），属于数据特性 |

> 现有 `ubt_sim/dataset/walker_c1` 约 11 集 × 100 帧，仅够 smoke 与流程验证；
> 正式训练需补采（建议 ≥ 50 集）。
