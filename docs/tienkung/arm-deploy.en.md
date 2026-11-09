# Walker TienKung Pro: ARM Board Deployment (Jetson AGX Orin)

> In-depth reference for the self-contained bundle at `ubt_IL/scripts/deploy/tienkung_pro/arm_64/`: runtime architecture, environment build, on-board training, and troubleshooting.
> Runtime: Jetson AGX Orin 8GB.
> The project supports both ARM containers and Conda, but on-board memory is limited; a lightweight Conda environment is recommended.
> Step-by-step instructions: [Real-robot workflow · Option B: on-board deployment](real-workflow.md#6b-jetson-agx-orin).

- Target: Jetson AGX Orin (TienKung Orin1 control board `192.168.41.2`; JetPack 6 / L4T R36.4.0 / Ubuntu 22.04 / glibc 2.35 / CUDA 12.6)
- `PROJECT_ROOT` = `/home/nvidia/vla/Thinker-IL-LAB/ubt_IL`
- Design principle: host scripts use `PROJECT_ROOT` + CLI overrides instead of the container paths hard-coded in the original scripts (`/ubt_IL/...`, `/lerobot/.venv/bin/...`). **No sudo, no symlinks, no modification of the original scripts.**

## 1. Runtime architecture (dual Python stacks)

The core idea is a **division of labor between two Python stacks**: `env_vla` (3.12) runs LeRobot inference/training, while system Python 3.10 runs hardware services that require cp310 (pyorbbecsdk camera, ROS 2 bridge). They are decoupled over ZMQ and do not pollute each other.

```
env_vla (conda, Python 3.12):  LeRobot + tienkung plugin + torch
   │  lerobot-rollout inference / lerobot-train training / image_client path check
   │
   ├── 5558 ◄── ImageServer (system py3.10 + pyorbbecsdk) ──► camera
   │
   └── 5559 ──► Bridge2 (system py3.10 + ROS 2) ──ROS 2 DDS──► TienKung hardware
       5560 ◄──┘
```

| ZMQ port | Direction | Purpose | Started by |
|----------|------|------|----------|
| 5558 | ImageServer -> env_vla | Camera JPEG frames | `image_server_host.sh` (manual, long-running) |
| 5559 | env_vla -> Bridge2 | Action commands | The plugin starts Bridge2 automatically during rollout |
| 5560 | Bridge2 -> env_vla | Robot status | Same as above |

**Stack ownership**:

- **env_vla (3.12)**: `rollout_host.sh`, `train_host.sh`, `image_client_host.sh`. Contains torch / LeRobot / the tienkung plugin; run `conda activate env_vla` first.
- **System python3 `/usr/bin/python3` (3.10.12) + `/opt/ros/humble`**: `image_server_host.sh`, `robot_ready.sh`, and Bridge2 (started by the plugin with `/usr/bin/python3`; **no rclpy needed in env_vla**).

**state/action are 26D**, ordered `[left arm 7 | right arm 7 | left hand 6 | right hand 6]` (defined by `TienKungRobotConfig.all_joints`).

## 2. Files

| File | Purpose | Stack |
|------|------|--------|
| `setup_env.sh` | One-shot build of the conda env `env_vla` (Python 3.12 + local wheels + LeRobot + tienkung plugin), with an import self-check at the end | Builder (conda) |
| `rollout_host.sh` | Policy inference deployment (`lerobot-rollout`) | env_vla (3.12) |
| `train_host.sh` | ACT training (`lerobot-train`, optional) | env_vla (3.12) |
| `image_server_host.sh` | Camera image service: camera -> ZMQ 5558 (JPEG) | System py3.10 + pyorbbecsdk |
| `image_client_host.sh` / `image_client.py` | Camera path validation client (receives frames on 5558, reports fps/latency and a verdict) | env_vla (3.12) |
| `robot_ready.sh` | Reset the robot to the preset pose (ready pose before inference) | System py3.10 + ROS 2 |
| `torch-*-cp312-cp312-linux_aarch64.whl` | Locally built PyTorch / torchvision wheels (Jetson-specific) | - |

## 3. Environment setup (first time)

Prerequisites:

- Project code copied to `/home/nvidia/vla/Thinker-IL-LAB/` (see step 0 of [Real-robot workflow · Option B](real-workflow.md#6b-jetson-agx-orin)).
- conda installed (miniconda / miniforge).
- ROS 2 Humble on the host (`/opt/ros/humble`, including rclpy, bodyctrl_msgs, cv_bridge).
- System Python 3.10 has `pyorbbecsdk2` / `cv2` / `zmq` / `numpy` (under `~/.local`). Install missing ones with:
  ```bash
  python3 -m pip install evdev pyorbbecsdk2
  ```
- **Real deployment**: change `192.168.41.99` in `ubt_IL/docker/fastdds_no_shm.xml` to **the IP of the NIC directly connected to the robot** (keep `127.0.0.1`), otherwise ROS 2 DDS cannot reach the robot.

Build:

```bash
cd /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64
bash setup_env.sh          # create conda env_vla (python=3.12), install wheels + LeRobot + plugin, self-check
```

| Variable | Default | Description |
|------|--------|------|
| `CONDA_BASE` | `conda info --base` or `~/miniconda3` | conda installation root |
| `ENV_NAME` | `env_vla` | conda environment name |
| `PIP_MIRROR` | Tsinghua TUNA | pip mirror |

After the build, `conda activate env_vla` and `which python` should point to env_vla (Python 3.12).

## 4. Deployment inference

Standard order (full steps in [Real-robot workflow · Option B](real-workflow.md#6b-jetson-agx-orin)): **start the camera service first, then inference**. The camera service runs on system Python 3.10 and must stay resident in a separate terminal or in the background.

```bash
bash image_server_host.sh    # ① camera service (system py3.10, separate terminal / background)
bash robot_ready.sh          # ② (optional) reset the robot to the preset pose (system py3.10 + ROS 2)
conda activate env_vla       # ③ policy inference (env_vla 3.12, new terminal)
bash rollout_host.sh
```

Common `rollout_host.sh` overrides:

```bash
POLICY_PATH=$PROJECT_ROOT/model/tienkung_pick_up_act/checkpoints/100000/pretrained_model \
DURATION=60 bash rollout_host.sh

# Deploy a 13-DOF model (right arm 7 + right hand 6; JOINT_CONFIG must match training)
JOINT_CONFIG=tienkung_13 \
POLICY_PATH=$PROJECT_ROOT/model/sim_pick_place_right13_act/checkpoints/last/pretrained_model \
DURATION=60 bash rollout_host.sh

DISPLAY_CAM=false bash rollout_host.sh        # disable camera display over SSH without X
```

| Variable | Default | Description |
|--------|--------|------|
| `POLICY_PATH` | `$PROJECT_ROOT/model/Pick_up_tiangong_all_act/checkpoints/last/pretrained_model` | ACT checkpoint |
| `JOINT_CONFIG` | `tienkung_26` | Joint DOF config (`tienkung_26` = full 26; `tienkung_13` = right arm 7 + right hand 6); must match the training DOF |
| `STRATEGY` | `base` | Inference strategy (`base` autonomous; `sentry`/`highlight`/`dagger` for recording/interaction) |
| `TASK` | `sim_pick_place` | Task description (injected as the policy condition) |
| `ZMQ_HOST` | `127.0.0.1` | image_server address (change to the robot IP when the camera runs there) |
| `DURATION` | `60` | Run time (seconds) |
| `FPS` | `30` | Control loop rate (aligned with training fps) |
| `DISPLAY_CAM` | `true` | Camera display (set `false` for SSH without X) |

### Verify the camera path

Before starting rollout, use `image_client` (env_vla side) to connect to 5558 and verify frames. No GUI required:

```bash
bash image_client_host.sh --count 60               # receive 60 frames, exit and report
bash image_client_host.sh --show                   # preview window (requires X: ssh -X/-Y or local DISPLAY=:0)
bash image_client_host.sh --address 192.168.41.2   # connect to image_server on the robot
```

`=> Camera path OK ✓` means the path is healthy.

## 5. On-board training (optional)

```bash
conda activate env_vla
bash train_host.sh
STEPS=100000 BATCH_SIZE=8 bash train_host.sh
```

| Variable | Default | Description |
|--------|--------|------|
| `CONFIG_PATH` | `$PROJECT_ROOT/scripts/deploy/train_config_tiangong_all.json` | Training config |
| `DATASET_ROOT` | `$PROJECT_ROOT/dataset` | LeRobotDataset root |
| `DATASET_REPO_ID` | `Pick_up_the_apple_all` | Must match the actual dataset directory name (overrides the config `repo_id`) |
| `OUTPUT_DIR` | `$PROJECT_ROOT/model/Pick_up_tiangong_all_act` | Output directory |
| `STEPS` | `500000` | Training steps |
| `BATCH_SIZE` | `8` | Batch size |
| `SEED` | `10000` | Random seed |
| `DEVICE` | `cuda` | Training device |
| `HF_HUB_OFFLINE` | `1` | Offline training (no HuggingFace Hub access) |

> **Note**: the config's `repo_id=Pick_up_tiangong_all` does not match the actual dataset directory `Pick_up_the_apple_all`; the script defaults to `DATASET_REPO_ID=Pick_up_the_apple_all` to match it. Keep them consistent if you change either.

## 6. Dataset replay (optional)

```bash
/usr/bin/python3 $PROJECT_ROOT/scripts/deploy/replay.py \
    --dataset $PROJECT_ROOT/dataset/Pick_up_the_apple_all --episode 0 --rate 30
```

## 7. Notes and troubleshooting

- **Python 3.12**: LeRobot 0.5.2 and the tienkung plugin run natively without source patches; do not downgrade to 3.10.
- **Local wheels**: torch/torchvision use the cp312 wheels in this directory (Jetson-specific, linked against glibc 2.35). Do not use PyPI aarch64 wheels (CPU-only, no CUDA) or prebuilt wheels requiring GLIBC_2.38 (they crash on 2.35).
- **numpy must be <2**: the local torch was built against numpy 1.x; 2.x triggers a "compiled using NumPy 1.x" crash. `setup_env.sh` pins `numpy==1.26.4`; do not upgrade it.
- **Do not mix stacks**: `image_server_host.sh` and `robot_ready.sh` use system Python 3.10 and **do not** need `conda activate env_vla`; `rollout_host.sh`, `train_host.sh`, and `image_client_host.sh` use env_vla (3.12) and require activation.
- **Camera display**: `DISPLAY_CAM=true` and `image_client --show` need X; set `false` or use `ssh -X/-Y` when `DISPLAY` is absent.
- **`conda activate` fails**: in non-interactive shells, run `source $CONDA_BASE/etc/profile.d/conda.sh` before `conda activate env_vla` (the scripts handle this).
- **Disk**: env_vla with torch is about 1.5 GB and eMMC is tight; pip always uses `--no-cache-dir`.
- **Jetson inference crashes (numpy)**: numpy was upgraded to 2.x; rerun `setup_env.sh` and keep `numpy==1.26.4`.
