# Walker S2 EDU: Imitation Learning Platform

> Code: `ubt_IL/scripts/{convert,train,eval,deploy}` (run inside the container)
> Prerequisite: repository cloned and submodules initialized as described in [Getting Started](../getting-started.md); HDF5 data ready ([simulation collection](sim-setup.md) or real-robot collection)

This page documents the shared conversion/training/evaluation/deployment workflow and configuration parameters for Walker S2 EDU. Data-source-specific commands are embedded in the corresponding workflow pages.

## 1. Environment (ubt_IL container)

See [Imitation Learning Platform · Container Setup](../il/docker.md) for `build` / `start` / `bash`. Run all remaining commands inside the container.

- Two Python environments: use `/usr/bin/python3` for ROS-related code and `/lerobot/.venv/bin/python` (default) for LeRobot scripts.
- For real-robot deployment, the container is built on the robot **Vision board (Jetson)**; the build script detects the board and builds an ARM container.

## 2. Data conversion (HDF5 -> LeRobot)

Script: `/ubt_IL/scripts/convert/walker_s2/convert.sh` (core converter `ubt_IL/scripts/convert/common/convert_to_lerobot.py`). It converts raw HDF5 into a LeRobot v3 dataset.

**Conversion command** (values for each variable are described in the config and environment tables below):

```bash
CONFIG=<conversion config JSON> \
SRC_ROOT=<HDF5 source directory> \
TGT_PATH=<LeRobot output root> \
REPO_ID=<output dataset name> \
TASK_NAME=<task name> \
bash /ubt_IL/scripts/convert/walker_s2/convert.sh
```

**Examples** (including a stationary-trimmed variant):

- Simulation data: [Simulation workflow · Data conversion](sim-workflow.md#3-data-conversion)
- Real-robot data: [Real-robot workflow · Data conversion](real-workflow.md#3-data-conversion)

Conversion configs live in `/ubt_IL/scripts/convert/walker_s2/configs/` and include field filtering and mapping. Choose or adapt one for your training needs:

| Config | Dimension | Scenario |
|----------|------|------|
| `walker_s2_sim_10d_2RGB.json` | 10D (right arm 7 + head 2 + right gripper 1) | Simulation (**recommended**) |
| `walker_s2_sim_33d_4RGB.json` | 33D (with depth) | Simulation |
| `walker_s2_real_10d_2RGB.json` | 10D | Real robot (**recommended**, avoids normalization blow-up from constant dimensions) |

!!! warning "convert.sh default config is not in the repository"
    `convert.sh` defaults to `CONFIG=configs/walker_s2_real_19d_1RGBD.json`, which **does not exist** in the repository. Always specify an existing config from the table above.

**Common environment variables** (defaults can be omitted; see `bash convert.sh -h` and [Data Conversion](../common/data-conversion.md)):

| Variable | Description |
|------|------|
| `SRC_ROOT` | HDF5 root (default `/ubt_IL/dataset/walker-s2-real-data`; change it to the simulation data directory for sim data) |
| `TGT_PATH` | LeRobot output root (default `/ubt_IL/dataset`) |
| `CONFIG` | Conversion config JSON (see above; **must be specified explicitly**) |
| `REPO_ID` | Output dataset name (keep it consistent with training) |
| `TASK_NAME` | Task name (written to each frame's `task` field and used as the language condition) |
| `FPS` | Target frame rate (default `13` sim / `15` real; `auto` uses the source HDF5 rate) |
| `ROBOT_TYPE` | Robot type (default `walker_s2`, written to metadata) |
| `HDF5_REL_PATH` | HDF5 path relative to each episode directory (default `hdf5/metadata_aligned.hdf5`; simulation data uses `trajectory.hdf5`) |
| `VCODEC` | Video codec (default `h264`; `jpeg` / `png` also supported) |
| `TRIM_STATIONARY` | `1` enables stationary-frame trimming (see below) |
| `RESAMPLE_FPS` | Target frame rate; enables resampling when set (disabled by default) |

!!! tip "Stationary-frame trimming"
    Stationary frames can trap the policy in a local stationary loop. Prefix the conversion command with `TRIM_STATIONARY=1 \` to trim them (threshold/window/cap settings are described in [Data Conversion](../common/data-conversion.md)).

> 19D real-robot data: the repository only provides a 10D real conversion config. For 19D, write a config based on the joint order in `walker_s2_gripper_19d.json`.

**Output**: a LeRobot v3 dataset under `/ubt_IL/dataset/<REPO_ID>/` (`meta/`, `chunk-000/` data parquet, `videos/`).

## 3. Data visualization

After conversion, inspect dataset quality with `lerobot-dataset-viz`. Checking data before training is important to avoid training blindly:

**Visualization command**
```bash
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id <REPO_ID> \
  --episode-index 0 \
  --root /ubt_IL/dataset/<REPO_ID>
```

> **Note**: `--root` must point to the dataset directory containing `meta/` (the `repo_id` directory itself), not its parent. `HF_HUB_OFFLINE=1` disables access to the HuggingFace Hub.

**Examples**: [Simulation workflow · Visualization](sim-workflow.md#32-data-visualization) / [Real-robot workflow · Visualization](real-workflow.md#32-data-visualization).

> Check especially: image resolution, video frame rate, state/action dimension and order, grip range, and empty episodes. Simulation RGB is [3,240,320] (10D) or [3,480,640] (33D); real is [256,320] etc. Model input must match the training data — do not mix them.

## 4. Policy training (ACT)

Training script: `/ubt_IL/scripts/train/walker_s2/train.sh`; configs in `/ubt_IL/scripts/train/walker_s2/configs/`.

**Training command** (optional overrides are listed below; only explicitly set values override the config):

```bash
CONFIG=<training config JSON> \
DATASET_REPO_ID=<dataset repo_id> \
OUTPUT_DIR=<model output directory> \
STEPS=<training steps> SAVE_FREQ=<save interval> BATCH_SIZE=<batch size> \
bash /ubt_IL/scripts/train/walker_s2/train.sh
```

**Examples**:

- Simulation training: [Simulation workflow · Training](sim-workflow.md#4-model-training)
- Real-robot training: [Real-robot workflow · Training](real-workflow.md#4-model-training)

**Without train.sh** (call `lerobot-train` directly from `/ubt_IL/lerobot`):

```bash
# Smoke test (quick pre-training check, 2 steps, no checkpoint)
cd /ubt_IL/lerobot
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_act_10_2RGB.json \
  --steps=2 --save_checkpoint=false \
  --output_dir=/ubt_IL/model/Walker_S2_sim_act_smoke

# Continue training + tuning
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=150000 \
  --dataset.image_transforms.enable=true      # enable image augmentation

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=100000 \
  --optimizer.lr=5e-06                        # lower the learning rate for fine-tuning
```

**Common overrides** (only applied when the environment variable is **explicitly set**; `HF_HUB_OFFLINE=1` is enabled by default):

| Variable | Description |
|------|------|
| `CONFIG` | Training config JSON (default `configs/train_config_walker_s2_sim_act_10_2RGB.json`); ⚠️ defaults to the **simulation config**, so real training must specify a real config (e.g. `train_config_pick_part_real_10d_2RGB.json`) |
| `DATASET_REPO_ID` / `DATASET_ROOT` | Dataset name / root (default `Walker_S2_sim_10_2RGB` / `/ubt_IL/dataset`; must match the conversion `REPO_ID`) |
| `OUTPUT_DIR` | Model output directory (default `/ubt_IL/model/Walker_S2_sim_10_2RGB_act`; checkpoints under `checkpoints/<step>/pretrained_model/`) |
| `STEPS` | Training steps (default `50000`) |
| `SAVE_FREQ` | Checkpoint save interval (default `5000`) |
| `BATCH_SIZE` | Batch size (default `8`) |
| `SEED` | Random seed (default `1000`) |
| `DEVICE` | Training device (default `cuda`) |
| `RESUME` | Resume training (default `false`; `true` resumes from the checkpoint) |
| `WANDB_ENABLE` | Enable wandb logging (default `false`) |

## 5. Policy training (Pi0.5 VLA)

Pi0.5 is Physical Intelligence's ~4B-parameter vision-language-action (VLA) model, built on a PaliGemma-2B vision-language backbone plus a Gemma-300M action expert, using flow matching to generate actions.

> **Hardware**: requires a GPU with ≥24 GB VRAM (e.g. RTX 4090); in bf16 it needs about 16-20 GB with gradient checkpointing.
> **Pretrained weights**: Pi0.5 must be initialized from `lerobot/pi05_base`; it cannot be trained from scratch. Download the weights before first use.

```bash
cd /ubt_IL/lerobot

# First training run
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_pi05.json

# Continue training + tuning
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true --steps=10000 \
  --policy.freeze_vision_encoder=false       # unfreeze the vision encoder

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true --steps=5000 \
  --policy.train_expert_only=true            # train only the action expert (freeze the VLM)
```

Key differences from ACT:

| Item | ACT | Pi0.5 |
|--------|-----|-------|
| Model size | ~30M | ~4B |
| Architecture | ResNet18 + Transformer | PaliGemma-2B + Gemma-300M |
| Normalization | MEAN_STD (all) | QUANTILES (state/action) + IDENTITY (visual) |
| Image size | Original (360×640 etc.) | Resize to 224×224 |
| Action prediction | VAE + absolute | Flow matching |
| Language input | None | Task description (requires a `task` field in the data) |
| batch_size | 8 | 4 |
| Training steps | 50,000 | 5,000 |
| Optimizer | AdamW (1e-5) | AdamW (2.5e-5, cosine decay + warmup) |

## 6. Policy evaluation (offline MSE)

Script (shared with TienKung): `/ubt_IL/scripts/eval/eval_policy.py`. It runs offline inference over a LeRobot dataset, computes the per-frame MSE between **predicted and ground-truth actions**, and can plot per-episode comparisons. Quantify policy quality before deployment.

**Evaluation command**:

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path <pretrained_model directory (config.json + model.safetensors)> \
  --dataset-path <LeRobot dataset directory containing meta/info.json> \
  --episodes <number of episodes> \
  --inference-freq <infer every N steps> \
  --plot --plot-dir <plot output directory> \
  --output <result JSON path> \
  --device cuda
```

**Examples**:

- Simulation model: [Simulation workflow · Evaluation](sim-workflow.md#5-policy-evaluation)
- Real-robot model: [Real-robot workflow · Evaluation](real-workflow.md#5-policy-evaluation)

| Argument | Description |
|------|------|
| `--policy-path` | `pretrained_model` directory (containing `config.json` + `model.safetensors`) |
| `--dataset-path` | LeRobot dataset root (containing `meta/info.json`) |
| `--episodes` | Number of episodes to evaluate (default: all) |
| `--inference-freq` | Run inference every N steps and reuse the predicted chunk in between (emulates real deployment; `1` = every step) |
| `--plot` / `--plot-dir` | Per-episode prediction-vs-ground-truth plots and output directory |
| `--output` | Result JSON path |
| `--device` | `cuda` or `cpu` (default `cuda`, falls back automatically) |

Once evaluation passes, continue with [§7 Model deployment](#7-model-deployment).

## 7. Model deployment

Script: `/ubt_IL/scripts/deploy/walker_s2/rollout.sh`. It loads a checkpoint and sends actions to Isaac Sim or the real robot through the ROS2-ZMQ bridge. A **safety precheck** (action dimension match) runs before deployment and must pass before motion is allowed. Helper tools: `robot_ready.sh` (initialize/home), `preview_camera.py` (camera path check), `inference_server.sh` (warm-started inference service).

**Deployment command** (optional variables are listed below; unset values use script defaults):

```bash
POLICY_PATH=<checkpoint pretrained_model directory> \
ROBOT_MODEL=<robot DOF config> \
INFERENCE_TYPE=<inference engine> \
ZMQ_HOST=<simulation bridge/robot address> \
TASK=<task description> \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

**Deployment guides**:

- Simulation deployment (feedback into the simulator; recommended first): [Simulation workflow · Model deployment](sim-workflow.md#6-model-deployment-sim)
- Real-robot deployment (ROBOT_MODELS registry and safety precheck): [Real-robot workflow · Model deployment](real-workflow.md#6-model-deployment-real) · [Real-robot deployment](deploy.md)
- Inference server (warm start/external calls): [Inference server](inference-server.md)

**Common environment variables**:

| Variable | Description |
|------|------|
| `POLICY_PATH` | **Required, no default**; checkpoint directory (containing `config.json`) |
| `ROBOT_MODEL` | ROBOT_MODELS registry key (default `walker_s2_31d`; `walker_s2_10d` = right arm + head + right gripper 10D, `walker_s2_19d` = 17 body + 2 grippers); **must match the training DOF** |
| `INFERENCE_TYPE` | Inference engine: `sync` (default) / `act_async` (asynchronous replanning with `INFERENCE_HZ`) |
| `INFERENCE_HZ` | act_async replanning rate (default `0.6`) |
| `STRATEGY` | Rollout strategy (default `base` = autonomous) |
| `ZMQ_HOST` | Bridge address: `127.0.0.1` for simulation feedback (default); use the robot-side IP for cross-machine real deployment |
| `FPS` | Control rate (default `13`, aligned with training fps) |
| `DURATION` | Run time (seconds, default `30`) |
| `TASK` | Task description |
| `PREVIEW_CAMERA` / `RECORD_ACTIONS` | Camera preview window / rollout action recording switches |

See [Real-robot deployment](deploy.md) for all variables, the ROBOT_MODELS registry, and safety prechecks.

## FAQ

| Symptom | Fix |
|------|------|
| Conversion reports "config does not exist" | The `convert.sh` default `CONFIG` is not in the repository; specify an existing config from `configs/` |
| Real-robot 19D data cannot be converted | The repository only provides a 10D real config; for 19D, write one based on the joint order in `walker_s2_gripper_19d.json` |
| Dataset frame rate differs from expected | Collection runs at 15 Hz and conversion defaults to `FPS=13` (real config uses 15); use `FPS=auto` to keep the source rate or `RESAMPLE_FPS` to resample |
| Normalization explodes (QUANTILES NaN/Inf) | Caused by 9 constant dimensions in 19D real data; convert with the 10D config (right arm + head + right gripper) to drop dead dimensions |
| Long stationary segments; the policy stops during deployment | Add `TRIM_STATIONARY=1` during conversion |
| Occasional non-convergence | Insufficient data; collect more episodes; confirm `image_transforms` is enabled |
