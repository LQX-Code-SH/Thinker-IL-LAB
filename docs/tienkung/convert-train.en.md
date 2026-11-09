# Walker TienKung Pro: Imitation Learning Platform

> Code: `ubt_IL/scripts/{convert,train,eval,deploy}` (run inside the container)
> Prerequisite: repository cloned and submodules initialized as described in [Getting Started](../getting-started.md); HDF5 data ready ([simulation collection](sim-setup.md) or real-robot collection)

This page documents the shared conversion/training/evaluation/deployment workflow and configuration parameters for Walker TienKung Pro. Data-source-specific commands are embedded in the corresponding workflow pages.

## 1. Environment (ubt_IL container)

See [Imitation Learning Platform · Container Setup](../il/docker.md) for `build` / `start` / `bash`. Run all remaining commands inside the container.

## 2. Data conversion (HDF5 -> LeRobot)

Script: `/ubt_IL/scripts/convert/tienkung_pro/convert.sh`. It converts raw HDF5 into a LeRobot v3 dataset at `/ubt_IL/dataset/<REPO_ID>/`.

**Conversion command** (values for each variable are described in the config and environment tables below):

```bash
CONFIG=<conversion config JSON> \
SRC_ROOT=<HDF5 source directory> \
TGT_PATH=<LeRobot output root> \
REPO_ID=<output dataset name> \
TASK_NAME=<task name> \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh
```

**Examples**:

- Simulation data: [Simulation workflow · Data conversion](sim-workflow.md#3-data-conversion)
- Real-robot data: [Real-robot workflow · Data conversion](real-workflow.md#3-data-conversion)

Conversion configs live in `/ubt_IL/scripts/convert/tienkung_pro/configs/` and include field filtering and mapping. Choose or adapt one for your training needs:

| Config | Dimension | Scenario |
|----------|------|------|
| `Tien_Kung_13_1RGB_sim.json` | 13D (right arm 7 + right hand 6) | Simulation (**recommended**, avoids left-arm dead dimensions) |
| `tienkung_pro_26d_1RGB.json` | 26D, both arms | Simulation (dual-arm scenario) |
| `tienkung_pro_26d_1RGB_real.json` | 26D, both arms, RGB only | Real robot (real HDF5 has no depth stream) |
| `tienkung_pro_13d_1RGB.json` | 13D | Real robot |

**Common environment variables** (defaults can be omitted; see `bash convert.sh -h` and [Data Conversion](../common/data-conversion.md)):

| Variable | Description |
|------|------|
| `SRC_ROOT` | HDF5 root (one subdirectory per episode, each containing `trajectory.hdf5`; default `/ubt_IL/dataset/tienkung_pro`) |
| `TGT_PATH` | LeRobot output root (default `/ubt_IL/dataset`) |
| `CONFIG` | Conversion config JSON (see the table above) |
| `REPO_ID` | Output dataset name (keep it consistent with training) |
| `TASK_NAME` | Task name (written to each frame's `task` field and used as the language condition during training) |
| `FPS` | Target frame rate (default `15`) |
| `ROBOT_TYPE` | Robot type (default `tienkung`) |
| `VCODEC` | Video codec (default `h264`) |
| `HDF5_REL_PATH` | HDF5 path relative to each episode directory (default `trajectory.hdf5`) |
| `TRIM_STATIONARY` | `1` enables stationary-frame trimming (see below) |
| `STATIONARY_DIAGNOSE` | `1` only reports the stationary distribution without writing data (for threshold calibration) |
| `RESAMPLE_FPS` | Target frame rate; enables resampling when set (disabled by default) |

!!! tip "Stationary-frame trimming"
    Stationary frames can trap the policy in a local stationary loop. Set `TRIM_STATIONARY=1` to trim stationary runs longer than the cap (default 8 frames). Prefix the conversion command with `TRIM_STATIONARY=1 \`.

**Output**: a LeRobot v3 dataset under `/ubt_IL/dataset/<REPO_ID>/` (`meta/`, `chunk-000/` data parquet, `videos/`).

## 3. Data visualization

After conversion, inspect dataset quality with `lerobot-dataset-viz`. Checking data before training is important to avoid training blindly:

```bash
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id <REPO_ID> \
  --episode-index 0 \
  --root /ubt_IL/dataset/<REPO_ID>
```

> **Note**: `--root` must point to the dataset directory containing `meta/` (the `repo_id` directory itself), not its parent. `HF_HUB_OFFLINE=1` disables access to the HuggingFace Hub.

**Examples**: [Simulation workflow · Visualization](sim-workflow.md#32-data-visualization) / [Real-robot workflow · Visualization](real-workflow.md#32-data-visualization).

## 4. Policy training

Training script: `/ubt_IL/scripts/train/tienkung_pro/train.sh`; configs in `/ubt_IL/scripts/train/tienkung_pro/configs/`.

**Training command** (optional overrides are listed below; unset values come from the config file):

```bash
CONFIG_PATH=<training config JSON> \
OUTPUT_DIR=<model output directory> \
STEPS=<training steps> SAVE_FREQ=<save interval> BATCH_SIZE=<batch size> \
bash /ubt_IL/scripts/train/tienkung_pro/train.sh
```

**Examples**:

- Simulation training: [Simulation workflow · Training](sim-workflow.md#4-model-training)
- Real-robot training: [Real-robot workflow · Training](real-workflow.md#4-model-training)

**Without train.sh** (call `lerobot-train` directly from `/ubt_IL/lerobot`):

```bash
cd /ubt_IL/lerobot
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/tienkung_pro/configs/train_config_tienkung_pro_sim_pick_place.json \
  --output_dir=/ubt_IL/model/tienkung_sim_pick_place_act \
  --steps=80000 --save_freq=20000 --batch_size=8
```

**Common overrides** (unset values come from the config file):

| Variable | Description |
|------|------|
| `CONFIG_PATH` | Training config JSON; ⚠️ `train.sh` defaults to the **simulation config**, so real-robot training must specify the real config explicitly |
| `DATASET_REPO_ID` / `DATASET_ROOT` | Dataset name / root (must match the conversion `REPO_ID`; `DATASET_ROOT` must be the dataset directory containing `meta/`) |
| `OUTPUT_DIR` | Model output directory (checkpoints under `checkpoints/last/pretrained_model/`) |
| `STEPS` | Training steps |
| `SAVE_FREQ` | Checkpoint save interval (steps) |
| `BATCH_SIZE` | Batch size |
| `RESUME` | Resume training (`true` makes `CONFIG_PATH` point to `train_config.json` inside the checkpoint) |

## 5. Policy evaluation (offline MSE)

Script: `/ubt_IL/scripts/eval/eval_policy.py`. It runs offline inference over a LeRobot dataset, computes the per-frame MSE between **predicted and ground-truth actions**, and can plot per-episode comparisons. Quantify policy quality before deployment.

**Evaluation command**:

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path <ACT checkpoint pretrained_model directory> \
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
| `--policy-path` | Trained ACT checkpoint (the `pretrained_model` directory) |
| `--dataset-path` | LeRobot dataset root (containing `meta/info.json`) |
| `--episodes` | Number of episodes to evaluate (default: all) |
| `--inference-freq` | Run inference every N steps to emulate the deployment chunk queue; defaults to `policy.n_action_steps`; `1` means every step |
| `--plot` / `--plot-dir` | Per-episode prediction-vs-ground-truth plots (PNG) |
| `--output` | Result JSON (summary + per-episode MSE) |
| `--device` | `cuda` (default; falls back to `cpu` when unavailable) |

Once evaluation passes, continue with [§6 Model deployment](#6-model-deployment).

## 6. Model deployment

Script: `/ubt_IL/scripts/deploy/tienkung_pro/rollout.sh`. It loads a trained ACT checkpoint, runs inference at the control rate, and sends actions to the simulator or real robot over ZMQ. Helper tools: `reset.py` (initialize/reset pose), `replay.py` (replay dataset actions for validation), `image_client.py` (camera path validation).

**Deployment command** (optional variables are listed below; unset values use script defaults):

```bash
POLICY_PATH=<ACT checkpoint pretrained_model directory> \
JOINT_CONFIG=<joint DOF config> \
ZMQ_HOST=<simulator/robot address> \
TASK=<task description> \
bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh
```

**Examples**:

- Simulation deployment (feedback into the simulator; recommended first): [Simulation workflow · Model deployment](sim-workflow.md#6-sim-deployment)
- Real-robot deployment (option A remote container / option B Jetson board / custom DOF): [Real-robot workflow · Model deployment](real-workflow.md#6-real-robot-deployment)

**Common environment variables**:

| Variable | Description |
|------|------|
| `POLICY_PATH` | Trained ACT checkpoint (`checkpoints/<step>/pretrained_model`) |
| `JOINT_CONFIG` | Joint DOF config: `tienkung_26` = full 26, `tienkung_13` = right arm 7 + right hand 6; custom configs are supported and **must match the training DOF** |
| `STRATEGY` | Execution strategy (`base` = autonomous execution) |
| `ZMQ_HOST` | Target address: simulation `127.0.0.1` (default); hardware must override it (e.g. `192.168.41.2`) |
| `FPS` | Control rate (aligned with training fps, default `15`) |
| `DURATION` | Run time (seconds, default `60`) |
| `TASK` | Task description |

## FAQ

| Symptom | Fix |
|------|------|
| Normalization explodes during training (QUANTILES NaN/Inf) | Constant dimensions in the data (e.g. a locked left arm in 26D); convert with the 13D config to drop dead dimensions |
| Trajectory contains long stationary segments; the policy stops during deployment | Add `TRIM_STATIONARY=1` during conversion |
| Training raises `FileExistsError` | `OUTPUT_DIR` already exists with `RESUME=false`; use a new directory or delete the old one |
| Real-robot conversion reports a missing depth stream | Real HDF5 has no depth; use `tienkung_pro_26d_1RGB_real.json` (RGB only) |
| Action dimensions are wrong after conversion | The real `master` gripper is mapped to 6D via invert/repeat/pad; confirm the real config was used, not the sim config |
| Occasional non-convergence | Insufficient data; collect more episodes and confirm `image_transforms` is enabled for generalization |
