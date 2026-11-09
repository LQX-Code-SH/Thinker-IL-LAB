# Walker TienKung Dex: On-Robot Deployment (Thor / Jazzy, No Container)

This page explains how to run the policy directly on the robot's ARM board (NVIDIA **Thor**, Ubuntu 24.04 + ROS 2 **Jazzy** + Python 3.12) without Docker. Data conversion and training are still recommended on a workstation/container; the robot only performs deployment.

Scripts: `ubt_IL/scripts/deploy/tienkung_dex/arm_64/`.

## 1. Runtime architecture (dual Python stacks, ZMQ decoupling)

```
Layer A  $VENV (3.12): lerobot-rollout / lerobot-train
        │ 5559 actions ▼        ▲ 5560 state       ▲ 5558 JPEG (640×360)
Layer B  system python3 (3.12) + Jazzy: ros2_deploy_bridge.py
        │ /arm/cmd, /right_hand/set_motor_multi          ▲ /robot_state (≈200Hz)
        └ /ob_camera_head/color/image_raw (1280×720 → resize 640×360 → JPEG)
Layer C  system python3 + Jazzy: robot_ready.py (ready pose, mutually exclusive with layer B)
```

| Layer | Interpreter | Runs | Ports / topics |
|---|---|---|---|
| A | `$PROJECT_ROOT/.venv-dex` (3.12) | `lerobot-rollout` / `lerobot-train` | 5559 actions / 5560 state |
| B | `/usr/bin/python3` + Jazzy | `ros2_deploy_bridge.py` | 5558 JPEG, `/arm/cmd`, hand commands |
| C | `/usr/bin/python3` + Jazzy | `robot_ready.py` (mutually exclusive with layer B) | `/arm/cmd` |

## 2. Why not Docker / conda

| Option | Verdict |
|---|---|
| docker | The repository's `Dockerfile.arm64` targets JetPack6 / 22.04 / Humble / Orin `sm_87`, which is incompatible with Thor (JetPack7 / 24.04 / Jazzy / `sm_110`); the current user is also not in the docker group and has no passwordless sudo |
| conda | The host torch comes from a deleted local wheel; PyPI aarch64 torch has no CUDA; the jetson-ai-lab `jp7/cu130` index has no cp312 wheel → no usable torch |
| **venv(`--system-site-packages`)** | Inherits the host `sm_110` torch directly; tested with numpy 2.2.6 and torch/torchvision/cv2 all work (the Orin `numpy<2` pitfall does not exist here) |

## 3. Files

| File | Purpose |
|---|---|
| `env.sh` | Shared constants (`PROJECT_ROOT`/`VENV`/`BRIDGE_SCRIPT`) + `dex_source_ros` (explicitly sources Jazzy) |
| `setup_env.sh` | Creates the venv, installs LeRobot/plugins (`--no-deps` to keep torch), self-checks the 13D contract |
| `robot_ready_host.sh` | Ready pose (system python + Jazzy) |
| `rollout_host.sh` | Policy inference (venv), with dimension precheck |
| `bridge_smoke.sh` | Read-only smoke test: verifies state/image/e-stop links without moving the robot |
| `camera_preview.sh` | Head camera preview: live window / `--snapshot` / `--size raw` for 720p |
| `hold_test.py` | Zero-displacement control-path acceptance (target = current measured angles, no motion) |
| `staged_test.sh` | **Staged micro-motion**: head / hand / arm ±0.05 rad (hand 80 codes) out and back, with measured verification |
| `robot_home_host.sh` | **Return to standing**: reverses the four ready-pose waypoints (hands stay above the table), target `HOME_POSE` + real-robot offsets |

## 4. Environment setup (first time)

The repository (including the `lerobot` submodule) must be synced to the robot — `PROJECT_ROOT` is derived from the script location as `ubt_IL/` inside the repository. LFS assets (`ubt_sim` simulation resources) are not needed.

**Prechecks**:

```bash
python3 -V                                   # 3.12
ls /opt/ros/jazzy/setup.bash
/usr/bin/python3 -c "import rclpy, ros2_bridge_msgs, bodyctrl_msgs, inspire_hand_msgs, brainco_hand_msgs"
/usr/bin/python3 -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_arch_list())"
```

Vendor message packages are installed system-wide by default; if they live in a custom workspace, set `TIENKUNG_DEX_MSGS_WS=<ws>` (default `/opt/tienkung_dex_msgs_ws`). `ROS_SETUP` can explicitly select the ROS distribution; by default it checks `/opt/ros/jazzy` then `/opt/ros/humble`.

**Create the environment**:

```bash
cd ubt_IL/scripts/deploy/tienkung_dex/arm_64
bash setup_env.sh
```

`setup_env.sh` is idempotent (reruns skip venv creation and dependency reinstallation). It performs:

| Step | Description |
|---|---|
| venv | `python3 -m venv --system-site-packages $PROJECT_ROOT/.venv-dex`; **does not install torch**, inheriting the host `sm_110` build |
| numpy | Installs `numpy==2.2.6` (LeRobot requires `>=2.0,<2.3`) |
| Runtime dependencies | draccus / gymnasium / einops / safetensors / huggingface-hub / datasets / av / pyzmq, etc.; **does not install opencv** (uses the host cv2 with GUI support) |
| numpy2-ABI packages | pandas / pyarrow / matplotlib / scikit-learn use `--ignore-installed`, otherwise numpy1-built packages in host `~/.local` raise `numpy.dtype size changed` |
| Editable install | `--no-deps -e lerobot` + `-e tienkung_dex` (`--no-deps` prevents pip from replacing torch with the PyPI CPU build) |
| Self-check | torch CUDA / `sm_110`, numpy version, the dimensions and hand contract of all three `JOINT_CONFIG` values, `lerobot-rollout --help` |

On success you should see `cuda=True`, `archs` containing `sm_110`, `numpy 2.2.6`, and:

```text
joint_config=tienkung_dex_brainco_13  dim=13 hand=brainco mode=rad13 ...
joint_config=tienkung_dex_20          dim=20 hand=inspire  mode=rad13 ...
joint_config=tienkung_dex_inspire_13  dim=13 hand=inspire  mode=pico6 ...
dex_camera registered | 13D contract: ...
[setup] lerobot-rollout OK
[setup] done.
```

Failures usually fall into two cases: `cuda=False` means the venv did not inherit the host torch; `numpy.dtype size changed` means the ABI packages were not installed into the venv (rerun `setup_env.sh`).

## 5. First integration (without the policy)

```bash
cd ubt_IL/scripts/deploy/tienkung_dex/arm_64

bash bridge_smoke.sh                     # read-only check: status ≥30 Hz, image ≥10 fps, estop=false
bash camera_preview.sh --snapshot        # capture the policy view (/tmp/dex_camera_preview.png)
source ./env.sh && dex_source_ros        # layer B uses system python + Jazzy
/usr/bin/python3 hold_test.py --groups arm   # zero-displacement path acceptance
bash staged_test.sh --step all           # head/hand/arm micro-motion and return, verify direction
bash robot_ready_host.sh --dry-run       # inspect the four targets first
bash robot_ready_host.sh --real          # send the ready pose (operator at the e-stop)
```

For the Inspire r13 path: `HAND_VARIANT=inspire HAND_ACTION_MODE=pico6 bash bridge_smoke.sh` (the `right_hand` in state should be 6D). `camera_preview.sh` starts its own subscribe-only Bridge2; `--size raw` shows native 1280×720, and `--port 5558` connects to an existing bridge.

## 6. Deployment inference

```bash
POLICY_PATH=/abs/path/pretrained_model \
JOINT_CONFIG=tienkung_dex_brainco_13 \
CAMERA_PORT=5558 IMAGE_ENABLED=1 FPS=30 \
DURATION=10 MAX_RELATIVE_TARGET=0.01 \
bash rollout_host.sh
```

| Variable | Default | Description |
|---|---|---|
| `POLICY_PATH` | None (required) | Checkpoint `pretrained_model` directory |
| `JOINT_CONFIG` | `tienkung_dex_brainco_13` | BrainCo 6 motors; use `tienkung_dex_20` for Inspire 20D; `tienkung_dex_inspire_13` for Inspire r13 datasets |
| `HAND_ACTION_MODE` | Empty (auto) | Empty derives from `JOINT_CONFIG`; `tienkung_dex_inspire_13` → `pico6` |
| `FPS` | `15` | Must equal the dataset frame rate |
| `CAMERA_PORT` / `IMAGE_ENABLED` | `5558` / `1` | Real robot uses Bridge2 transcoding |
| `CAMERA_WIDTH/HEIGHT` | `640×360` | Used for both Bridge2 resizing and the `dex_camera` crop window |
| `MAX_RELATIVE_TARGET` | Empty | `0.01` (rad/step) for the first run |
| `DISPLAY_CAM` | `true` | Set `false` when there is no X server (SSH) |

## 7. Comparison with the container version

| Container version | On-robot version | Notes |
|---|---|---|
| `robot_ready.sh --real` | `robot_ready_host.sh --real` | System python3 + Jazzy |
| `rollout.sh` | `rollout_host.sh` | Runs in the venv and starts Bridge2 automatically |
| `preview_camera.py` | `camera_preview.sh` | Bridge2 transcodes directly on the robot |
| `inference_server.sh` | Not available | The warm-started inference service still requires the container version |

## 8. Notes and troubleshooting

- **Camera size mismatch does not raise an error**: feeding 720p unchanged makes `dex_camera` slice columns by `offset_x+width` and take the "left half"; the policy still outputs, but the visual distribution is completely wrong. Bridge2's `image_width/image_height` now resize before encoding; always run `camera_preview.sh --snapshot` first to confirm the full view.
- **`bash -lc` does not source `.bashrc`**: Bridge2 inherits the ROS environment from its parent process; scripts explicitly source Jazzy via `dex_source_ros`. Source it manually when starting the bridge by hand.
- **`/arm/cmd` has no consumer**: the robot must be in topic-control mode (remote half-body policy toggled up), otherwise "the policy runs but the robot does not move".
- **`robot_ready` and Bridge2 are mutually exclusive**: both write `/arm/cmd`; the order is always ready first, then rollout.
- **Emergency stop**: Bridge2 monitors `is_estop` on `/power/board/key_status`; previously sent targets remain in the drives, so press the physical e-stop if necessary.
- **`wrist_roll_l` 40 Hz friction limit cycle**: the real-robot home/ready pose already adds +0.03 rad via `constants.DEPLOY_HOME_OFFSETS`; if the right wrist buzzes after swapping robots, add an entry to that table.
- **`cuda=False`**: the venv did not inherit the host torch; confirm `python3 -m venv --system-site-packages` created `$PROJECT_ROOT/.venv-dex`.
- **`numpy.dtype size changed`**: ABI packages were not installed into the venv; rerun `setup_env.sh`.

See `ubt_IL/scripts/deploy/tienkung_dex/arm_64/README.md` in the repository for the full file list and field measurements.
