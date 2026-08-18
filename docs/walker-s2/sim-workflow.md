# Walker S2：仿真工作流（端到端）

> 适用：**Isaac Sim 仿真采集 -> 转换 -> 训练 -> 评估 -> 回注仿真部署** 完整闭环。

## 0. 工作流总览

```
仿真数据采集(hdf5) -> 数据转换 -> 模型训练(ACT/Pi0.5) -> 离线 MSE 评估 -> 仿真部署
```

## 1. 前置：仿真数据采集

数据来源为 Isaac Sim 仿真器采集（`ubt_sim` 项目），完整步骤见 [仿真环境与数据采集](sim-setup.md)。

```bash
# ① 启动仿真环境与任务
cd ubt_sim/docker
bash run.sh bash
UBT_SIM_TASK=UBTSim-WalkerS2-PickPart-v0 bash /ubt_sim/scripts/start_sim.sh

# ② 单次采集（--save 落盘 + --reset-scene 重置场景 + --robot-init 初始化位姿）
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/pick_part.py --save --reset-scene --robot-init

# ③ 批量采集（默认 400 次）
bash /ubt_sim/teleoperation/control/walker_s2/save_data.sh
```

- 输出目录：`/ubt_sim/dataset/walker_s2/<时间戳>/trajectory.hdf5`（15Hz 采样）
- 采集完成后，将 HDF5 数据拷贝到 `/ubt_IL/dataset/Walker-s2-pick-part-sim/` 方便容器挂载数据。

## 2. 启动容器（ubt_IL）

```bash
cd ubt_IL/docker
bash run.sh build      # 首次
bash run.sh start
bash run.sh bash       # 进入容器，后续转换/训练/评估命令均在容器内执行
```

## 3. 数据转换（HDF5 -> LeRobot）

```bash
CONFIG=/ubt_IL/scripts/convert/walker_s2/configs/walker_s2_sim_10d_2RGB.json \
SRC_ROOT=/ubt_IL/dataset/Walker-s2-pick-part-sim \
HDF5_REL_PATH=trajectory.hdf5 \
TGT_PATH=/ubt_IL/dataset \
REPO_ID=Walker_S2_sim_10_2RGB \
TASK_NAME=walker_s2_sim_pick_part \
bash /ubt_IL/scripts/convert/walker_s2/convert.sh

# 静止帧裁剪版本（去除抓取等待期的静止段，避免推理陷入局部静止）
TRIM_STATIONARY=1 \
CONFIG=/ubt_IL/scripts/convert/walker_s2/configs/walker_s2_sim_10d_2RGB.json \
SRC_ROOT=/ubt_IL/dataset/Walker-s2-pick-part-sim \
HDF5_REL_PATH=trajectory.hdf5 \
TGT_PATH=/ubt_IL/dataset \
REPO_ID=Walker_S2_sim_10_2RGB \
TASK_NAME=walker_s2_sim_pick_part \
bash /ubt_IL/scripts/convert/walker_s2/convert.sh
```

配置选择、完整环境变量与可视化检查见 [数据转换与训练 §2-3](convert-train.md#2-数据转换hdf5---lerobot)。

![walker 仿真数据预览](../assets/walker仿真数据预览.jpg)

## 4. 模型训练

```bash
# 使用默认配置训练（仿真 ACT 10D 2RGB）
bash /ubt_IL/scripts/train/walker_s2/train.sh

# 覆盖常用参数
CONFIG=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_act_10_2RGB.json \
OUTPUT_DIR=/ubt_IL/model/Walker_S2_sim_10_2RGB_act \
STEPS=50000 SAVE_FREQ=5000 BATCH_SIZE=8 \
bash /ubt_IL/scripts/train/walker_s2/train.sh
```

ACT / Pi0.5 完整训练配方见 [数据转换与训练 §4-5](convert-train.md#4-模型训练act)。

## 5. 策略评估（离线 MSE）

```bash
cd /ubt_IL/lerobot
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/Walker_S2_sim_10_2RGB \
  --episodes 10 \
  --inference-freq 1 \
  --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_Walker_S2_sim \
  --output /ubt_IL/scripts/eval/output/eval_Walker_S2_sim/results.json \
  --device cuda
```

![walker 仿真离线评估](../assets/walker仿真离线评估.png)

参数说明见 [数据转换与训练 §6](convert-train.md#6-策略评估离线-mse)。

## 6. 模型部署（回注仿真）

仿真部署与真机共用 `rollout.sh` / 推理服务器，差异在于：

- 机器人控制经 **ROS2-ZMQ Bridge** 回注 Isaac Sim，`ZMQ_HOST=127.0.0.1`（本地桥接）；
- 相机来自仿真（四路 RGB 或所选配置相机），无需真机相机；
- 部署前先做**安全预检**（action 维度匹配），通过后才允许运动。

```bash
# 1. 启动仿真--宿主机进仿真容器
cd ubt_sim/docker
bash run.sh start
bash run.sh bash
UBT_SIM_TASK=UBTSim-WalkerS2-PickPart-v0 bash /ubt_sim/scripts/start_sim.sh      # 启动模型对应的仿真任务

# 2. 启动推理容器--宿主机
cd ubt_IL/docker
bash run.sh start
bash run.sh bash

# 3. 初始化动作（抬起手臂到桌面上）--推理容器内
bash /ubt_IL/scripts/deploy/walker_s2/robot_ready.sh

# 4. 部署仿真模型异步推理（以 020000 checkpoint 为例）
ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/020000/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=2 \
FPS=13 DURATION=60 \
ZMQ_HOST=127.0.0.1 \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh

# （可选）验证相机通路，可指定相机话题/机器人配置
/usr/bin/python3 scripts/deploy/walker_s2/preview_camera.py --robot walker_s2_10d
```

部署效果：

<video controls muted loop width="100%" src="../assets/walker仿真部署效果.mp4"></video>

完整环境变量、ROBOT_MODELS 注册表与安全预检说明见 [真机部署](deploy.md)。

### 推理服务器（常驻预热/外部调用）

`rollout.sh` 每次冷启动（policy 加载 + 桥接拉起）耗时长。推理服务器**常驻**预热，外部推理客户端可通过 ZMQ 指令使用推理服务，用于切换 VLA 操作任务和其他任务（如导航、搬箱）：

```bash
cd /ubt_IL/scripts/deploy/walker_s2
ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/last/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=2 FPS=13 \
bash inference_server.sh

bash inference_client.sh status    # 查询状态到 READY
bash inference_client.sh start     # 开始推理
bash inference_client.sh stop      # 停止推理
```

详细使用说明见 [推理服务器](inference-server.md)。

## 7. 常见问题

| 问题 | 原因 / 解决 |
|------|-------------|
| 转换报"配置不存在" | `convert.sh` 默认 `CONFIG` 在仓库中未提供；请显式指定 `configs/` 下实际存在的配置 |
| 数据集帧率与预期不符 | 采集 15Hz，转换默认 `FPS=13`；如需保留原频率用 `FPS=auto`，或 `RESAMPLE_FPS` 重采样 |
| 部署时报维度不匹配 | 安全预检拒绝：确认 `ROBOT_MODEL` 与策略维度对应（10D->`walker_s2_10d`，19D->`walker_s2_19d`，31D->`walker_s2_31d`）；无 action names 的策略需 `ALLOW_DIM_ONLY_POLICY=1` |
| 推理服务器 `start` 报错 | `start/stop` 依赖 act_async 的 pause/resume；`sync` 引擎无此语义，需 `INFERENCE_TYPE=act_async` |
| 仿真相机与真机相机尺寸不同 | 仿真 RGB 为 [3,240,320]（10D）或 [3,480,640]（33D），真机为 [256,320] 等；模型输入需与训练数据一致，勿混用 |
| 重复拉起推理服务器失败 | 单例 pid 文件 `/tmp/walker_inference_server.pid` 存在；`shutdown` 正常退出或手动清理后再拉 |
