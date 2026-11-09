# Imitation Learning Platform: Container Setup

`ubt_IL` runs in a **LeRobot + ROS 2 Humble** container. The build process is identical for all robots and maintained on this page; robot-specific steps start from the corresponding robot page.

The host project is mounted at `/ubt_IL`; all conversion/training/evaluation commands run inside the container.

## Build and enter the container (host)

```bash
cd ubt_IL/docker
bash run.sh build      # build the image (first time; selects the Dockerfile automatically)
bash run.sh start      # start the container
bash run.sh bash       # enter the container shell
```

## Automatic architecture selection

The build script selects a Dockerfile based on the host CPU architecture:

| Host architecture | Dockerfile | Notes |
|------------|-----------|------|
| `x86_64` / `amd64` | `Dockerfile` | x86 workstation training / simulation deployment |
| `aarch64` / `arm64` (Orin) | `Dockerfile.arm64` | Jetson Vision board deployment (arm image must be built on the Jetson) |
| `aarch64` / `arm64` (Thor) | `Dockerfile.thor` | Walker C1 EDU onboard compute (L4T r38 / sm_110); selected automatically by `run.sh` |

## Common run.sh commands

```text
build    Build the image
start    Idempotent start: warn if running, start if stopped, run if absent
check    Health check: mounts / lerobot / plugins / ROS 2 msgs / GPU / network
bash     Enter the container shell (ROS 2 and walker workspace sourced automatically)
stop     Stop the container
restart  Restart the container (stop + start)
rm       Stop and remove the container
```

## Python environments in the container

- Two Python environments: use `/usr/bin/python3` for ROS-related code and `/lerobot/.venv/bin/python` (default) for LeRobot scripts.
- The x86 image is based on `huggingface/lerobot-gpu:latest` and ships with `/lerobot/.venv`.
- The container runs with `--network=host --shm-size=16g` and forwards X11 for GUI tools.
- Communication: LeRobot (Python 3.12) -ZMQ-> Bridge2 (system Python 3.10) -ROS 2 DDS-> robot/simulator. Each robot uses its own ports: Walker TienKung Pro / Walker TienKung Dex `5559`/`5560` + camera `5558`; Walker S2 EDU `5561`/`5562`/`5563`; Walker C1 EDU `5661`/`5662`/`5663`.
- For real-robot deployment, the container is built on the robot **Vision board (Jetson / Thor)**; the build script detects the board type (Thor selects `Dockerfile.thor`).

> For full details on the image and `entrypoint.sh` (Dockerfile internals, environment overrides, FastDDS shared memory, Jetson torch wheels), see [Container Environment](../common/docker.md).

## Next steps

- [Platform overview](index.md) - back to the platform overview
- [Walker TienKung Pro](../tienkung/convert-train.md) - Walker TienKung Pro training workflow
- [Walker TienKung Dex](../tienkung-dex/convert-train.md) - single-arm dexterous manipulation training
- [Walker S2 EDU](../walker-s2/convert-train.md) - Walker S2 EDU training workflow
- [Walker C1 EDU](../walker-c1/convert-train.md) - Walker C1 EDU training workflow
