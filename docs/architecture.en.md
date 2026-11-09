# Architecture Overview

## Two sub-projects

| Sub-project | Responsibility | Stack | Runtime |
|--------|------|--------|----------|
| `ubt_sim` | Robotics simulation: validation, teleoperation, data collection | Isaac Sim 5.0 + Isaac Lab 2.2 + ROS 2 | Docker container (Isaac Sim Python 3.11 + system Python 3.10) |
| `ubt_IL` | Imitation learning: conversion, training, evaluation, deployment | LeRobot (vendored submodule) + ROS 2 | Docker container (Python 3.12 venv + system Python 3.10) |

## Data flow

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          ubt_IL imitation learning                       │
│              conversion ──► training ──► offline eval ──► deployment     │
└───────────▲───────────────┬──────────────────────▲──────────────┬────────┘
            │               │④ sim deployment       │              │⑤ real deployment
            │① data          ▼ (ROS bridge)          │② real data    │ (ROS bridge)
   ┌────────┴────────────────┐        ┌─────────────┴──────────────▼──────┐
   │      ubt_sim            │        │ Walker TienKung Pro /             │
   │   (Isaac Sim container) │        │ Walker TienKung Dex /             │
   │                         │        │ Walker S2 EDU / Walker C1 EDU     │
   │                         │        │ ROS_DOMAIN_ID=0 direct network    │
   └─────────────────────────┘        └───────────────────────────────────┘
```

- **① Data collection (simulation)**: built-in scripted tasks (pick / sort) collect 15 Hz HDF5 data; an episode is saved only when the task succeeds.
- **② Data collection (real robot)**: use the [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) teleoperation platform and export HDF5 or LeRobot datasets.
- **③ Conversion / training / evaluation**: all run inside the `ubt_IL` container. HDF5 is converted to LeRobot v3, ACT / Pi0.5 policies are trained, then evaluated offline with MSE.
- **④ Sim deployment**: the policy is fed back into Isaac Sim through the ROS2-ZMQ bridge using the same topics as the real robot, validating the policy before real deployment.
- **⑤ Real deployment**: `rollout.sh`, or a warm-started inference server for long-running deployment.

## ubt_sim architecture

Three cooperating processes (**Python versions must not be mixed**):

```
Isaac Sim (Py3.11)  ──ZMQ──►  ROS2-ZMQ Bridge (Py3.10)  ──ROS2──►  teleop / collection scripts
```

- **Simulation** `/isaac-sim/python.sh`: loads the Gym task (Walker TienKung Pro parlor / Walker TienKung Dex apple picking / Walker S2 EDU warehouse / Walker C1 EDU parlor pick-and-place) and sends images directly over ZMQ.
- **Bridge** `/usr/bin/python3`: bidirectional ZMQ↔ROS 2 forwarding, started automatically by `start_sim.sh`.
- **Collection** `/usr/bin/python3`: publishes commands and records 15 Hz HDF5.

The ROS2-ZMQ bridge exposes state and images on **the same ROS 2 topics as the real robot**, enabling sim-to-real consistent teleoperation and data collection.

## ubt_IL architecture

Communication path (Walker TienKung Pro shown; each robot has its own ports):

```
LeRobot (Python 3.12 venv)
   │  ZMQ 5559/5560 (Walker S2 EDU: 5561/5562; Walker C1 EDU: 5661/5662)
   ▼
Bridge2 (system Python 3.10 + ROS 2)
   │  ROS 2 DDS
   ▼
Robot / simulator
   ▲
   └── camera stream over ZMQ 5558 (ImageServer) / 5563 (Walker S2 EDU relay) / 5663 (Walker C1 EDU relay)
```

The container provides two Python environments: use `/usr/bin/python3` for ROS-related code and `/lerobot/.venv/bin/python` for LeRobot code.

## ZMQ ports

| Port | Module | Purpose |
|------|--------|---------|
| 5555 | Walker TienKung Pro / Walker TienKung Dex sim bridge | Control commands (bridge PUB) |
| 5556 | Walker TienKung Pro / Walker TienKung Dex sim bridge | Robot state (bridge SUB) |
| 5557 | Walker TienKung Pro sim bridge | Raw camera images (C++ bridge) |
| 5558 | Simulation / ImageServer | JPEG camera stream (published by simulation; on Walker TienKung Dex real robots, encoded by Bridge2 `--enable-image`) |
| 5559 / 5560 | ubt_IL ↔ Bridge2 (Walker TienKung Pro / Walker TienKung Dex) | Action commands / robot state |
| 5655 | Walker S2 EDU / Walker C1 EDU sim bridge | Control commands (bridge PUB) |
| 5656 | Walker S2 EDU / Walker C1 EDU sim bridge | Robot state (bridge SUB) |
| 5657 | Walker S2 EDU / Walker C1 EDU simulation | Camera RGB (direct collection) |
| 5658 | Walker S2 EDU / Walker C1 EDU simulation | JPEG images |
| 5561 / 5562 | Walker S2 EDU real-robot Bridge2 | Action commands / robot state |
| 5563 | Walker S2 EDU real-robot CameraRelay (separate process) | Image channel (ROS 2 DDS `shm_msgs` camera topics -> ZMQ) |
| 5570 / 5571 | Walker S2 EDU / Walker TienKung Dex inference server | ZMQ REP commands / PUB state |
| 5661 / 5662 | Walker C1 EDU real-robot Bridge2 | Action commands / robot state |
| 5663 | Walker C1 EDU CameraRelay (separate process) | Image channel (ROS 2 DDS camera topics -> ZMQ JPEG) |
| 5690 / 5691 | Walker C1 EDU inference server | ZMQ REP commands / PUB state |
| 5755 / 5756 | Walker TienKung Dex sim bridge | Control commands / robot state |
| 5757 / 5758 | Walker TienKung Dex simulation | Raw camera images (RGB + depth) / JPEG stream |

> Walker S2 EDU and Walker C1 EDU share simulation ports 5655-5658, so **only one of them can run at a time**. Their `ubt_IL` plugin ports differ (Walker S2 EDU: 5561-5563, Walker C1 EDU: 5661-5663), so each plugin can run alongside its own simulation bridge.

## Repository layout

```
Thinker-IL-LAB/
├── ubt_sim/                    # simulation platform
│   ├── assets/                 # 3D assets (USD/URDF/textures, Git LFS)
│   ├── config/                 # YAML task/scene configuration
│   ├── docker/                 # Docker configuration
│   ├── scripts/                # simulation launch scripts
│   ├── source/ubt_sim/         # Python package (Isaac Sim side, Py3.11)
│   └── teleoperation/          # robot control scripts (ROS 2 side)
│       ├── bridges/            # ROS2-ZMQ bridges
│       ├── control/            # robot control + data collection
│       ├── image/              # image transport
│       ├── msgs/               # custom ROS 2 messages
│       └── tools/              # diagnostics / playback tools
└── ubt_IL/                     # imitation learning platform
    ├── dataset/                # datasets and conversion output
    ├── docker/                 # Docker configuration
    ├── lerobot/                # vendored lerobot source (submodule)
    ├── model/                  # training output and pretrained weights
    ├── scripts/
    │   ├── convert/            # HDF5 -> LeRobot conversion
    │   ├── train/              # training launchers
    │   ├── deploy/             # deployment (reset / rollout / replay / inference server)
    │   └── eval/               # offline evaluation
    ├── tienkung/               # Walker TienKung Pro LeRobot plugin
    ├── tienkung_dex/           # Walker TienKung Dex plugin + ROS 2 messages + Bridge2
    ├── walker_c1/              # Walker C1 EDU plugin + Bridge2
    └── walker/                 # Walker S2 EDU plugin + ROS 2 SDK/messages + Bridge2
```
