# Container Environment (ubt_IL/docker)

`ubt_IL/docker/` provides a **LeRobot + ROS 2 Humble** containerized environment for training and deployment across the supported robots. Two architectures are supported: x86_64 workstations and Jetson/aarch64 robots. `env.sh` selects the Dockerfile and default parameters from the host architecture.

## Files

| File | Purpose |
| --- | --- |
| `Dockerfile` | x86_64 image build (base `huggingface/lerobot-gpu:latest`, ships `/lerobot/.venv`) |
| `Dockerfile.arm64` | Jetson aarch64 image build (base `dustynv/l4t-pytorch:r36.4.0-linuxarm64`, builds a Python 3.12 venv) |
| `run.sh` | CLI orchestrator: `build / start / stop / restart / bash / rm / check` |
| `entrypoint.sh` | Idempotent startup: editable installs lerobot and plugins, builds Walker ROS 2 messages, fixes opencv/numpy, validates sm_87 torch |
| `env.sh` | Selects default image / Dockerfile / GPU arguments from `uname -m` |
| `fastdds_no_shm.xml` | Disables FastDDS shared-memory transport to fix ROS 2 subscription failures inside Docker |
| `ros2_msgs/` | Prebuilt `ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb` (x86) + `bodyctrl_msgs_src/` source (built on arm64) |
| `torch-*-aarch64.whl` etc. | torch/torchvision wheels built from source for Jetson sm_87, shipped with the image (~261 MB) |

Project code is bind-mounted at `/ubt_IL`; lerobot and each plugin are installed in editable mode at every startup, so sources do not need to be copied into the image.

## Automatic architecture selection

| Host architecture | Dockerfile | Default image | Default GPU arguments |
| --- | --- | --- | --- |
| `x86_64` / `amd64` | `Dockerfile` | `lerobot-tienkung:humble` | `--gpus all` |
| `aarch64` / `arm64` | `Dockerfile.arm64` | `lerobot-tienkung:humble-arm64` | `--runtime nvidia` |

## Build and run commands

```bash
cd ubt_IL/docker

bash run.sh build     # build the image (first build takes several minutes)
bash run.sh start     # idempotent start: warn if running, start if stopped, run if absent
bash run.sh check     # health check: mounts / lerobot / plugins / ROS 2 msgs / GPU / network
bash run.sh bash      # enter the container shell (ROS 2 and walker workspace sourced)
bash run.sh stop      # stop the container
bash run.sh restart   # restart the container (stop + start)
bash run.sh rm        # stop and remove the container
```

`start` follows the container log in the background and waits until the entrypoint has installed/compiled lerobot, the robot plugins, and the Walker ROS 2 messages and they import successfully. If imports still fail after installation, it exits with an error. The container runs with `--network=host --shm-size=16g` and forwards X11 for GUI tools.

## Environment overrides

```bash
# Use another base image
BASE_IMAGE=nvcr.io/nvidia/l4t-pytorch:<tag> bash run.sh build

# Use another Dockerfile or output image name
DOCKERFILE=./Dockerfile.arm64 IMAGE=my-lerobot:arm64 bash run.sh build

# Newer NVIDIA Container Toolkit on Jetson can use --gpus all
DOCKER_GPU_ARGS="--gpus all" bash run.sh start

# Disable explicit GPU arguments for debugging
DOCKER_GPU_ARGS="" bash run.sh start

# Change the ROS 2 domain ID (default 0 for hardware)
DOMAIN_ID=1 bash run.sh start

# Change the container name
CONTAINER_NAME=my-container bash run.sh start
```

The ARM64 base image can also be pulled and tagged manually to avoid re-pulling on every build:

```bash
sudo docker pull swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64
sudo docker tag \
  swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/dustynv/l4t-pytorch:r36.4.0-linuxarm64 \
  docker.io/dustynv/l4t-pytorch:r36.4.0
BASE_IMAGE=docker.io/dustynv/l4t-pytorch:r36.4.0 bash run.sh build
```

## Notes

1. **Architecture-bound; both cannot coexist on one machine.** The arm64 image is not generic arm64 but Jetson-specific (JetPack 6 base, torch wheels built for Orin sm_87, GPU arguments requiring Jetson's `--runtime nvidia`). An x86 host can only build and run x86 containers; arm64 containers must be built and run on a Jetson.
2. **bodyctrl_msgs is available on both architectures but installed differently.** x86 uses a prebuilt deb (`dpkg` into `/opt/ros/humble`); arm64 has no matching deb and builds from source with colcon into `/opt/bodyctrl_msgs_ws` during `Dockerfile.arm64`. Walker S2 EDU does not depend on bodyctrl_msgs (its messages are built by `entrypoint.sh` from `/ubt_IL/walker/walker_sdk_ros2`, 8 packages).
3. **FastDDS shared memory must be disabled.** Even with `--network=host`, shared-memory transport inside the container makes `ros2 topic list` work while `echo`/`subscribe` fail. The `fastdds_no_shm.xml` whitelist includes `127.0.0.1` plus the robot's direct-link subnet by default. ⚠️ **This file changes with the robot subnet and must be edited before building** (it is copied into the image at build time; changes afterwards have no effect). See each robot's real-robot workflow.
4. **TORCH_HOME is redirected** to the bind mount `/ubt_IL/.cache/torch`, so torchvision pretrained weights such as ResNet can be downloaded and persisted.
5. **The entrypoint installs runtime dependencies on every start.** lerobot, `lerobot_robot_tienkung`, and `lerobot_robot_walker` are editable installs, so source changes take effect after a container restart. It also replaces `opencv-python-headless` with the GUI build and pins `numpy<2`; on Jetson it defensively checks that `torch.cuda.get_arch_list()` contains `8.7`.
6. **Jetson torch wheels ship with the image.** Generic cu128 wheels from `download.pytorch.org` are not built for SM_87 and fail with `no kernel image is available`; the wheels here are built locally (CUDA 12.6, cuDNN 9.4, cp312) and copied to `/opt/jetson-wheels` to avoid rebuilding on every Jetson.
7. **LeRobot's `[dataset]` extra is preinstalled** (`datasets pandas pyarrow av`, installed at build time), satisfying the hard dependency of the `rollout` module.
