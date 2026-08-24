# Walker S2 Edu 探索者 训练

容器内训练命令，在 `cd /ubt_IL/lerobot` 后执行。

> 便捷封装：`bash /ubt_IL/scripts/train/walker_s2/train.sh`（默认仿真 ACT 配置；`CONFIG` 切换 Pi0.5/真机配置，`STEPS`/`OUTPUT_DIR`/`BATCH_SIZE` 可选覆盖，不设则沿用 config 值）。下方为完整 `lerobot-train` 命令参考。

## Walker S2 Edu 探索者 仿真 ACT 训练

### 首次训练

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim.json
```

配置文件：[train_config_walker_s2_sim.json](configs/train_config_walker_s2_sim.json)

### Smoke Test（训练前快速验证）

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim.json \
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

| 字段 | Walker S2 Edu 探索者 仿真 |
|------|---------------|
| camera key | `observation.images.camera_head` |
| 图像 shape | `[3, 360, 640]` |
| state shape | `[19]` |
| action shape | `[19]` |
| 数据集路径 | `/ubt_IL/dataset/Walker_S2_sim` |
| 模型输出 | `/ubt_IL/model/Walker_S2_sim_act` |

---

## Walker S2 Edu 探索者 仿真 Pi0.5 VLA 训练

Pi0.5 是 Physical Intelligence 的 ~4B 参数视觉-语言-动作（VLA）模型，基于 PaliGemma-2B 视觉语言骨干 + Gemma-300M 动作专家，使用 flow matching 生成动作。

> **硬件要求**：需要 ≥24GB 显存的 GPU（如 RTX 4090）。bf16 精度下约需 16-20GB 显存（开 gradient checkpointing）。
> **预训练模型**：Pi0.5 必须从 `lerobot/pi05_base` 预训练权重初始化，无法从头训练。首次使用前需下载。

### 首次训练

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_pi05.json
```

配置文件：[train_config_walker_s2_sim_pi05.json](configs/train_config_walker_s2_sim_pi05.json)

### Smoke Test（训练前快速验证）

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_pi05.json \
  --steps=2 \
  --save_checkpoint=false \
  --output_dir=/ubt_IL/model/Walker_S2_sim_pi05_smoke
```

### 继续训练

```bash
cd /ubt_IL/lerobot

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=10000
```

### 继续训练 + 调参

```bash
cd /ubt_IL/lerobot

# 更多步数 + 解冻 vision encoder
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=10000 \
  --policy.freeze_vision_encoder=false

# 降低学习率微调
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=10000 \
  --policy.optimizer_lr=5e-06

# 仅训练 action expert（冻结 VLM）
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=5000 \
  --policy.train_expert_only=true
```

### 关键配置说明

| 字段 | Walker S2 Edu 探索者 Pi0.5 | 说明 |
|------|-----------------|------|
| policy type | `pi05` | Pi0.5 VLA 策略 |
| pretrained_path | `lerobot/pi05_base` | 必须从预训练模型初始化 |
| dtype | `bfloat16` | 减少 50% 显存 |
| camera key | `observation.images.camera_head` | |
| 图像 resize | `[224, 224]` | Pi0.5 固定输入尺寸 |
| state shape | `[19]` -> pad 到 32 | Pi0.5 自动 padding |
| action shape | `[19]` -> pad 到 32 | Pi0.5 自动 padding |
| chunk_size | 50 | Pi0.5 默认 action horizon |
| n_action_steps | 50 | 每次推理执行步数 |
| normalization | VISUAL=IDENTITY, STATE/ACTION=QUANTILES | Pi0.5 默认，需数据有 q01/q99 |
| gradient_checkpointing | true | 4B 模型显存优化必需 |
| batch_size | 4 | 4B 模型显存限制 |
| steps | 5000 | 微调步数 |
| optimizer_lr | 2.5e-5 | Pi0.5 推荐 peak LR，cosine decay 到 2.5e-6 |
| 数据集路径 | `/ubt_IL/dataset/Walker_S2_sim` | |
| 模型输出 | `/ubt_IL/model/Walker_S2_sim_pi05` | |

### vs ACT 配置差异

| 配置项 | ACT | Pi0.5 |
|--------|-----|-------|
| 模型规模 | ~30M | ~4B |
| 架构 | ResNet18 + Transformer | PaliGemma-2B + Gemma-300M |
| 归一化 | MEAN_STD (全部) | QUANTILES (state/action) + IDENTITY (visual) |
| 图像尺寸 | 360×640 (原始) | resize 到 224×224 |
| 动作预测 | VAE + absolute | Flow matching + optional relative |
| 语言输入 | 无 | task 描述 ("walker_s2_sim_pick") |
| batch_size | 8 | 4 |
| 训练步数 | 50,000 | 5,000 |
| optimizer | AdamW (1e-5, β=(0.9,0.999)) | AdamW (2.5e-5, β=(0.9,0.95)) |
| scheduler | 无 | Cosine decay + warmup |
