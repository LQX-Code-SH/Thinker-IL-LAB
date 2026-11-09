# Walker TienKung Dex Real-Robot Workflow: Collect -> Convert -> Train -> Evaluate -> Deploy

> Robot: **Walker TienKung Dex**
> ✅ **Verified on hardware (Nov 2026)**: the full loop works with BrainCo 13D (right arm 7 + right hand 6, 30 fps) real-robot data.
> The steps are the same for both hands; only the hand variant / `JOINT_CONFIG` / conversion config differ.

## Contents

- [0. Workflow overview](#0-workflow-overview)
- [1. Prerequisite: real-robot data collection](#1-prerequisite-real-robot-data-collection)
- [2. Start the container (ubt_IL)](#2-start-the-container)
- [3. Data conversion (HDF5 -> LeRobot)](#3-data-conversion)
- [4. Policy training](#4-model-training)
- [5. Policy evaluation (offline MSE)](#5-policy-evaluation)
- [6. Model deployment (real robot)](#6-model-deployment-real-robot)
- [7. FAQ](#7-faq)

<a id="0-workflow-overview"></a>

## 0. Workflow overview

```
Real-robot collection (trajectory.hdf5)
      │  right arm 7 + right hand 13 (or BrainCo 6)
      ▼
scripts/convert/tienkung_dex/convert.sh  ──► dataset/tienkung_dex_real_<task>
      ▼
scripts/train/tienkung_dex/train.sh      ──► model/tienkung_dex_real_<task>_act
      ▼
scripts/eval/eval_policy.py
      ▼
robot_ready.sh --real → rollout.sh        ──► real-robot execution
```

Training and deployment are essentially identical to simulation, using the same scripts with slightly different dataset configuration.

<a id="1-prerequisite-real-robot-data-collection"></a>

## 1. Prerequisite: real-robot data collection

Data is currently collected with the [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) teleoperation platform (see the official documentation), which can export LeRobot v3.0 datasets directly. You can also place collected real-robot HDF5 data under `/ubt_IL/dataset/hdf5/` (or any `SRC_ROOT`) and proceed to conversion.

**Verified real-robot dataset**: `tienkung_dex_real_pick_apple_brainco_13d` (32 episodes / 16,956 frames / 30 fps; both `observation.state` and `action` are 13D = right arm 7 + BrainCo hand 6; head camera `observation.images.head` 640×360 h264). The first sim2real validation reused a simulation-trained policy; once enough real data was available (≥30 episodes), training switched to real data directly. To mix simulation and real data, collect more first, then merge and convert.

<a id="2-start-the-container"></a>

## 2. Start the container (ubt_IL)

```bash
cd ubt_IL/docker
bash run.sh build      # first time (an x86 workstation build is sufficient)
bash run.sh start
bash run.sh check
bash run.sh bash
```

> To run the policy directly on the robot (Thor) without a container, see [On-robot deployment](arm-deploy.md).

<a id="3-data-conversion"></a>

## 3. Data conversion (HDF5 -> LeRobot)

**Verified path: BrainCo 13D real-robot data** (config `configs/tienkung_dex_13d_1RGB_brainco_real.json`; 13D = right arm 7 + BrainCo hand 6 closure values 0~1; head camera `observation.images.head` 640×360, monocular RGB):

```bash
# Real-robot BrainCo 13D conversion (real HDF5 has no depth; this is expected)
CONFIG=/ubt_IL/scripts/convert/tienkung_dex/configs/tienkung_dex_13d_1RGB_brainco_real.json \
SRC_ROOT=/ubt_IL/dataset/hdf5/tienkung_dex_real \
REPO_ID=tienkung_dex_real_pick_apple_brainco_13d \
TASK_NAME=pick_the_apple \
FPS=30 \
bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh --overwrite
```

> `FPS=30` matches the real recording rate (the verified dataset is 30 fps). `TRIM_STATIONARY` **can be enabled as needed** for real data: the recording rate matches the main loop and there are no simulation-style duplicate frames spread across the motion, so trimming long stationary ready-pose segments at the start and end is beneficial.

Other hand variants:

| Hand / dataset | Conversion config | Dimension |
|---|---|---|
| Inspire r13 (pico6, 6D 0~1) | `tienkung_dex_real_pick_r13_1RGB.json` | 13D |
| Inspire 20D (13D rad) | `tienkung_dex_20d_1RGB_inspire.json` | 20D |
| BrainCo (6D 0~1) | `tienkung_dex_13d_1RGB_brainco_real.json` | 13D (verified) |

Visualize after conversion:

```bash
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id tienkung_dex_real_pick_apple_brainco_13d \
  --episode-index 0 \
  --root /ubt_IL/dataset/tienkung_dex_real_pick_apple_brainco_13d
```

<a id="4-model-training"></a>

## 4. Policy training

**Verified path: BrainCo 13D real-robot training** (config `train_config_tienkung_dex_real_pick_apple_brainco_13d.json`, dataset `tienkung_dex_real_pick_apple_brainco_13d`, output `tienkung_dex_real_pick_apple_brainco_13d_act`):

```bash
CONFIG_PATH=/ubt_IL/scripts/train/tienkung_dex/configs/train_config_tienkung_dex_real_pick_apple_brainco_13d.json \
OUTPUT_DIR=/ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act \
bash /ubt_IL/scripts/train/tienkung_dex/train.sh
```

> `train.sh` defaults `CONFIG_PATH` to the **simulation config**; real-robot training must override it explicitly. `train_config_tienkung_dex_real_pick_apple.json` is a copy of the simulation config that only changes `dataset.repo_id` / `dataset.root` / `policy.repo_id` / `output_dir` / `job_name`; if the real `REPO_ID` differs from the conversion, update those fields. When real lighting differs strongly from simulation, add `image_transforms` (color jitter / random affine) by referring to `train_config_tienkung_pro_real_pick_place.json`.

Everything else is identical to simulation (see [Simulation workflow · Training](sim-workflow.md#4-model-training)).

<a id="5-policy-evaluation"></a>

## 5. Policy evaluation (offline MSE)

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/tienkung_dex_real_pick_apple_brainco_13d \
  --episodes 5 --inference-freq 1 --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_tienkung_dex_real_pick_apple_brainco_13d_act \
  --output /ubt_IL/scripts/eval/output/eval_tienkung_dex_real_pick_apple_brainco_13d_act/results.json \
  --device cuda
```

With small real datasets, inspect per-dimension curves for plausibility (hand jitter, wrist overshoot) instead of relying only on the total MSE.

<a id="6-model-deployment-real-robot"></a>

## 6. Model deployment (real robot)

### Prerequisites

1. **Enter topic-control mode**: use the remote to switch to the half-body policy (toggle right while standing).
2. **Legs are owned by motion control**: `robot_ready.py` never publishes to `/leg/cmd`; Bridge2 publishes only `/arm/cmd` and hand commands.
3. **Hand variant matches**: `--variant inspire|brainco` must match the physical hand (`HAND_VARIANT` / `JOINT_CONFIG` must follow).
4. **Emergency stop available**: Bridge2 monitors `is_estop` on `/power/board/key_status` and stops publishing when true.
5. **Inspire r13 uses 6→13 expansion**: `JOINT_CONFIG=tienkung_dex_inspire_13` automatically derives `HAND_ACTION_MODE=pico6` — the policy outputs a **6D normalized 0~1** `right_hand` (0=open, 1=closed), and Bridge2 expands it into 13 0.1° codes with `pico6_to_inspire13()`; state echoes the same 6D command (open loop). Passing 13D is rejected as a dimension mismatch; passing rad moves the fingers barely at all.

### First run: ready pose only

```bash
export ROS_DOMAIN_ID=0
bash /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.sh --real
```

Hardware uses `READY_POSE` (**not** the simulation `READY_POSE_TARGET`, which is a gravity feed-forward compensation that makes command equal actual; using it on hardware leaves the robot about 13 cm lower).

The `READY_POSE` was validated against the simulation view during real-robot integration. With arms hanging naturally, the hands are at 0.80 m, below the table (0.987 m), so the motion must be staged in four segments (`READY_CLEARANCE_POSE → READY_FOREARM_POSE → READY_UPPER_VERTICAL_POSE → final`, 5 s each by default).

### Deployment: short duration and small limits first

**Verified path: BrainCo 13D real-robot deployment** (`JOINT_CONFIG=tienkung_dex_brainco_13`, policy 13D = right arm 7 + BrainCo hand 6 values 0~1; `FPS` matches the dataset, 30):

```bash
# Bridge2 is started by rollout.sh (--robot.bridge_enabled=true); do not start it manually.
# Any leftover dex bridge process on the same host is killed before startup.
ROS_DOMAIN_ID=0 \
POLICY_PATH=/ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_brainco_13 \
CAMERA_PORT=5558 IMAGE_ENABLED=1 \
FPS=30 \
DURATION=10 MAX_RELATIVE_TARGET=0.01 \
bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
```

Demo:

<video src="../../assets/tienkung-dex真机部署效果.mp4" controls muted width="100%"></video>

### Hand representations

| Mode | `JOINT_CONFIG` | Policy-side `right_hand` | Conversion |
|---|---|---|---|
| BrainCo 13D | `tienkung_dex_brainco_13` | 6D closure 0~1 | Action `code = round(1 + v*999)`; state `v = (code-1)/999` |
| Inspire 20D | `tienkung_dex_20` | 13D joint angles in rad | Bridge2 converts each channel with `inspire_rad_to_code` to 0.1° codes |
| Inspire r13 | `tienkung_dex_inspire_13` | 6D normalized 0~1 | `pico6_to_inspire13()` expands to 13 codes |

> When bypassing the plugin and feeding `right_hand` directly to Bridge2: `brainco` / `pico6` must be 0~1; `rad13` must be 13D rad. Passing 0~1 to `rad13` is interpreted as rad and the fingers barely move.

### On-robot deployment (Thor / Jazzy, no container)

The path above runs the policy in an x86 workstation container while the robot only executes. To run the policy directly on the robot's ARM board (NVIDIA Thor, Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12), use the host scripts in [On-robot deployment](arm-deploy.md), with no Docker. Data conversion and training are still recommended on a workstation/container; the robot only performs deployment.

### Warm-started inference service

```bash
ROS_DOMAIN_ID=0 \
POLICY_PATH=/ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_brainco_13 CAMERA_PORT=5558 IMAGE_ENABLED=1 FPS=30 \
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_server.sh

bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh status   # wait for READY
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh start
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh stop
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh shutdown
```

> Dex **does not support `load` (model hot-swap)**: the server's robot reconstruction path hard-codes `--robot.type=walker` without `--robot.cameras`, which rebuilds a camera-less robot for Dex. To change models, `shutdown` and `launch` again.

<a id="7-faq"></a>

## 7. FAQ

| Symptom | Check |
|------|------|
| Topics are visible but no data arrives | Cross-distro type hash mismatch: compare `ros2 topic info -v` on both sides; or fall back to running Bridge2 on the compute host |
| `/arm/cmd` is published but the robot does not move | Is topic-control mode active (remote half-body/full-body policy + toggle up)? Is `is_estop` true on `/power/board/key_status`? |
| Arm motion is too large / starts too abruptly | Is `MAX_RELATIVE_TARGET` set? Is `DURATION` still the short first-run value? |
| Pose is lower than expected | The simulation `READY_POSE_TARGET` was used; hardware should use `robot_ready.sh --real` (`READY_POSE`) |
| Hand variant is clearly wrong | `HAND_VARIANT` / `JOINT_CONFIG` does not match the physical hand (Inspire 13 channels ↔ BrainCo 6 motors) |
| Inspire hand does not move / log reports `right_hand` dimension mismatch | r13 models need `JOINT_CONFIG=tienkung_dex_inspire_13` (pico6, 6D actions 0~1); only the older 20D model uses `tienkung_dex_20` (rad13, 13D rad actions) |
| Thumb abduction differs from the ready pose | Channel 10 of the pico6 expansion is determined by `thumb_roll` (all zeros → code 1750); the first policy frame overrides the ready-pose code 850 from `robot_ready` |
| No camera image | Does Bridge2 run with `image_enabled=true`? Is the port 5558? Does `/ob_camera_head/color/image_raw` publish? |
| Bridge2 exits immediately | "Channel table mismatch" means the plugin and Bridge2 constant copies have drifted; "message package missing" means `/opt/tienkung_dex_msgs_ws` was not sourced |
| Robot still moves slightly after emergency stop | Bridge2 only stops publishing new commands; previously sent targets remain in the drives. Press the physical emergency stop |
