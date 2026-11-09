# Walker C1 EDU 共创者 训练

ACT 策略，支持两种维度，1 路头部 RGB 240×320；头 / 腰 / 腿不进模型，部署时由机器人自身保持：

| 配置 | 维度 | 组成 | 适用 |
|---|---|---|---|
| `train_config_walker_c1_sim_act_26d_1RGB.json` | 26 | 左臂 7 + 右臂 7 + 左手 6 + 右手 6 | 双臂任务 |
| `train_config_walker_c1_sim_act_13d_1RGB.json` | 13 | 右臂 7 + 右手 6 | 单臂任务（左臂/左手部署时保持 home） |
| `train_config_walker_c1_real_act_13d_1RGB.json` | 13 | 右臂 7 + 右手 6 | 真机数据集 `walker_c1_real_13d_1RGB`（34 集 / 11406 帧） |

## 真机数据训练（13 维）

```bash
# 容器内
CONFIG=/ubt_IL/scripts/train/walker_c1/configs/train_config_walker_c1_real_act_13d_1RGB.json \
  bash /ubt_IL/scripts/train/walker_c1/train.sh
```

- 数据集：`/ubt_IL/dataset/walker_c1_real_13d_1RGB`（LeRobot v3.0，30fps，头部相机 512×640）
- 模型输入图像统一 resize 到 `240×320`，`use_amp=true`
- 输出：`/ubt_IL/model/walker_c1_real_13d_act_1RGB`，steps=30000，save_freq=10000

## 首次训练

```bash
# 容器内
bash /ubt_IL/scripts/train/walker_c1/train.sh
```

默认读取 `configs/train_config_walker_c1_sim_act_26d_1RGB.json`：
`dataset.repo_id=walker_c1_sim_26d_1RGB`、`steps=30000`、`batch_size=8`、
输出 `/ubt_IL/model/walker_c1_sim_26d_act_1RGB`。

## 13 维（单臂）训练

```bash
CONFIG=configs/train_config_walker_c1_sim_act_13d_1RGB.json \
  bash /ubt_IL/scripts/train/walker_c1/train.sh
```

数据集需先用 13 维配置转换：`DIM=13 bash scripts/convert/walker_c1/convert.sh`。

## Smoke test

```bash
bash /ubt_IL/scripts/train/walker_c1/train.sh --steps=2 --save_checkpoint=false
```

## 常用覆盖

```bash
BATCH_SIZE=4 STEPS=80000 OUTPUT_DIR=/ubt_IL/model/c1_exp2 bash train.sh
DEVICE=cpu bash train.sh --steps=2 --save_checkpoint=false
DATASET_ROOT=/ubt_IL/dataset/walker_c1_sim_26d_1RGB bash train.sh
```

## 续训

```bash
bash train.sh --resume=true --config_path=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model/train_config.json
```

## 调参建议

| 场景 | 参数 |
|---|---|
| 过拟合小数据集（现有 ~11 集） | `--batch_size=4 --steps=10000`，或 `--policy.use_vae=true` |
| 图像域差异大（仿真→真机） | `--dataset.image_transforms.enable=true` |
| 显存不足 | `--batch_size=2 --policy.dim_model=256 --policy.n_encoder_layers=3` |
| 学习率过大导致抖动 | `--policy.optimizer_lr=5e-05` |

## 配置要点

- `policy.input_features`：`observation.images.camera_head` `[3,240,320]` + `observation.state` `[26]`
- `policy.output_features.action`：`[26]`，顺序必须与
  `scripts/deploy/walker_c1/configs/walker_c1_26d.json` 的 `action_order` 完全一致
- `chunk_size = n_action_steps = 100`，部署时由 bridge 融合/加密到 300 Hz

## 数据量提醒

`ubt_sim/dataset/walker_c1` 当前只有 ~11 集 × ~100 帧，仅够 smoke 与流程验证；
正式训练需补采（建议 ≥ 50 集同任务、涵盖左右手动作）。
