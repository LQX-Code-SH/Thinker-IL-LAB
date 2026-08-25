# Walker S2 EDU 探索者 训练

容器内训练命令，在 `cd /ubt_IL/lerobot` 后执行。

> 便捷封装：`bash /ubt_IL/scripts/train/walker_s2/train.sh`（默认仿真 ACT 配置；`CONFIG` 切换 Pi0.5/真机配置，`STEPS`/`OUTPUT_DIR`/`BATCH_SIZE` 可选覆盖，不设则沿用 config 值）。下方为完整 `lerobot-train` 命令参考。

## Walker S2 EDU 探索者 仿真 ACT 训练

### 首次训练

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_act_10_2RGB.json
```

配置文件：[train_config_walker_s2_sim_act_10_2RGB.json](configs/train_config_walker_s2_sim_act_10_2RGB.json)

### Smoke Test（训练前快速验证）

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_act_10_2RGB.json \
  --steps=2 \
  --save_checkpoint=false \
  --output_dir=/ubt_IL/model/Walker_S2_sim_act_smoke
```

### 继续训练

训练结束后效果不够好，在已有 checkpoint 基础上继续训练更多步数：

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=100000
```

> **注意**：`--config_path` 必须指向 checkpoint 内保存的 `train_config.json`，不是原始的配置文件。
> `checkpoints/last` 是指向最近 checkpoint 的软链接。

### 继续训练 + 调参

```bash
cd /ubt_IL/lerobot

# 更多步数 + 开启图像增强
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=150000 \
  --dataset.image_transforms.enable=true

# 降低学习率微调
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=100000 \
  --optimizer.lr=5e-06
```

### 关键配置说明

| 字段 | Walker S2 EDU 探索者 仿真 |
|------|---------------|
| camera key | `observation.images.camera_head` |
| 图像 shape | `[3, 360, 640]` |
| state shape | `[19]` |
| action shape | `[19]` |
| 数据集路径 | `/ubt_IL/dataset/Walker_S2_sim` |
| 模型输出 | `/ubt_IL/model/Walker_S2_sim_act` |
