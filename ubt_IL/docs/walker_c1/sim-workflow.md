# Walker C1 仿真工作流

```
ubt_sim 采集 HDF5 ──► convert.sh ──► LeRobot 数据集 ──► lerobot-train ──► checkpoint
                                                                 │
                                                                 ├─► eval_policy.py（离线）
                                                                 └─► rollout.sh（仿真闭环）
```

## 0. 总览

| 阶段 | 命令 | 产物 |
|---|---|---|
| 仿真采集 | `ubt_sim/teleoperation/control/walker_c1/save_data.sh` | `ubt_sim/dataset/walker_c1/<epoch>/trajectory.hdf5` |
| 转换 | `scripts/convert/walker_c1/convert.sh` | `ubt_IL/dataset/walker_c1_sim_{26d,13d}_1RGB` |
| 训练 | `scripts/train/walker_c1/train.sh` | `ubt_IL/model/walker_c1_sim_26d_act_1RGB` |
| 评估 | `scripts/eval/eval_policy.py` | `scripts/eval/output/.../results.json` |
| 部署 | `scripts/deploy/walker_c1/rollout.sh` | 仿真机械臂动作 |

## 1. 仿真采集（ubt_sim）

```bash
# ubt_sim 容器内（另开终端）
UBT_SIM_ROBOT=walker_c1 bash run.sh bash
bash scripts/start_c1_pick_place_sim.sh        # 或 UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 bash scripts/start_sim.sh

# 采集：默认 1 集；--batch 20 批量变位置
bash teleoperation/control/walker_c1/save_data.sh --batch 20
```

仿真窗口 `R` = 场景复位，`Space` = 请求保存当前 episode。
落盘路径 `ubt_sim/dataset/walker_c1/<epoch秒>/trajectory.hdf5`，15 Hz，单路 `camera_head` 640×480 PNG。

> 采集前确认 `/mc/sdk/robot_state` 有数据（约 48 Hz）、手部增益 10/1 + effort 2 N·m。

## 2. 启动 ubt_IL 容器

```bash
cd ubt_IL/docker
bash run.sh start && bash run.sh check
bash run.sh bash
```

`/ubt_sim` 为只读挂载；容器里 `ls /ubt_sim` 报不存在说明容器是旧挂载建的：
`bash run.sh rm && bash run.sh start`。

## 3. 数据转换

```bash
# 先转一集验证
bash /ubt_IL/scripts/convert/walker_c1/convert.sh --save_one true --overwrite

# 全量（26 维）
bash /ubt_IL/scripts/convert/walker_c1/convert.sh --overwrite

# 13 维（右臂+右手，单臂任务）
DIM=13 bash /ubt_IL/scripts/convert/walker_c1/convert.sh
```

常用覆盖：`REPO_ID`、`TASK_NAME`、`FPS`、`SRC_ROOT`、`TGT_PATH`。
执行 `bash convert.sh --help` 查看全部。

校验：

```bash
python - <<'PY'
import json
info = json.load(open("/ubt_IL/dataset/walker_c1_sim_26d_1RGB/meta/info.json"))
print(info["total_episodes"], info["features"]["observation.state"]["shape"],
      info["features"]["observation.images.camera_head"]["shape"])
PY
```

期望输出：`11 [26] [3, 240, 320]`（集数取决于采集量）。

可视化：

```bash
python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id walker_c1_sim_26d_1RGB --root /ubt_IL/dataset/walker_c1_sim_26d_1RGB
```

## 4. 训练

```bash
bash /ubt_IL/scripts/train/walker_c1/train.sh --steps=2 --save_checkpoint=false   # smoke
bash /ubt_IL/scripts/train/walker_c1/train.sh                                     # 正式
```

关键配置（`configs/train_config_walker_c1_sim_act_26d_1RGB.json`）：

| 项 | 值 |
|---|---|
| 策略 | ACT，`chunk_size=n_action_steps=100` |
| 输入 | `observation.images.camera_head [3,240,320]` + `observation.state [26]` |
| 输出 | `action [26]` |
| 训练 | `batch_size=8`、`steps=30000`、`save_freq=5000` |

## 5. 评估

```bash
python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/walker_c1_sim_26d_1RGB \
  --episodes 5 --inference-freq 15 --plot \
  --output /ubt_IL/scripts/eval/output/eval_walker_c1_sim_26d_act/results.json
```

`--inference-freq 15` 模拟 15 Hz 部署节奏；`--plot` 输出逐关节 GT vs Pred 曲线。
也可先做单帧推理自检：

```bash
python /ubt_IL/scripts/deploy/walker_c1/infer_check.py \
  --checkpoint /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --dataset /ubt_IL/dataset/walker_c1_sim_26d_1RGB --frame 0
```

## 6. 仿真部署（闭环验证）

```bash
# ① 确认仿真已在跑（ubt_sim 容器），ROS 域一致
export ROS_DOMAIN_ID=0        # ubt_sim 默认 0；隔离时用 146 且两边一致

# ② 相机
python3 /ubt_IL/scripts/deploy/walker_c1/preview_camera.py --once --save-frame /tmp/c1.jpg

# ③ 复位 + 预备位（三段式）
bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh

# ④ rollout
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh

# ⑤ 多集（每集复位 + 重启进程；base strategy 不支持 --strategy.num_episodes）
EPISODES=5 DURATION=20 RESET_MODE=both \
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh
```

端口：C1 插件 5661/5662/5663；ubt_sim C1 桥接 5655-5658；可同时运行。

## 7. FAQ

| 现象 | 处理 |
|---|---|
| 转换「Found 0 episodes」 | `SRC_ROOT` 下没有 `*/trajectory.hdf5`；确认 `HDF5_REL_PATH` |
| `meta/info.json` 里图像是 640×480 | 配置被改过；应为 `image_size: [320,240]` |
| 训练 loss 不降 | 数据太少（11 集）；先补采或减少 `dim_model` |
| rollout 维度预检失败 | policy 不是 26 维；或 `ROBOT_CONFIG` 指向了别的机型配置 |
| 头/腰/腿缓慢下沉 | 仿真侧未下发关节应保持 HOME_POSE；确认没有外部节点在发这些关节 |
| 相机不开图 | `preview_camera.py` 的 `--topic`；`ROS_DOMAIN_ID`；QoS BEST_EFFORT |
| 动作很慢/很卡 | 提高 `FPS`；`INFERENCE_TYPE=act_async INFERENCE_HZ=2` |
