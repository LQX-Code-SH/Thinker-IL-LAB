# License scope

Copyright © 2026 UBTECH ROBOTICS CORP LTD. All rights reserved.

This repository contains materials under different license terms.

## Apache License 2.0

Except for the proprietary assets listed below, all files in this repository —
including source code, scripts, build and container files, package metadata,
configuration files, documentation, images, and videos — are licensed under the
Apache License 2.0. The complete license text is available at the repository
root (`LICENSE`) and in `Apache-2.0.txt` in this directory.

## UBTECH proprietary assets — limited use only

The following assets are governed by `UBTECH-PROPRIETARY.txt`. They are NOT
licensed under the Apache License 2.0:

### 1. Documentation assets and SDK archives under `ubt_IL`

- Every file under `ubt_IL/docs/walker_s2/assets/`, including:

  - `ubt_api_tiny.20260203.zip` and
    `ubt_api_tiny_python_20260407.tar.xz`, and every file contained in,
    extracted from, or installed from these archives (for example
    `ubt_robot` wheels, `libcc_api_client`, and `libtbox_*` libraries),
    except for embedded third-party components governed by their respective
    license terms;

  - all screenshots and images in that directory (`image*`, `img_v3_*`,
    `*.jpeg`, `*.jpg`, `*.png`);

  **Exception:** `data_config.py` and `gr00t_finetune.py` in that directory
  are NVIDIA Corporation code (Isaac GR00T) distributed under the Apache
  License 2.0 with the NVIDIA copyright notice preserved, and are NOT
  UBTECH proprietary assets.

- Every file under `ubt_IL/docs/assets/`, including all demonstration videos
  (`*.mp4`) and images (`*.png`).

### 2. Simulation documentation assets under `ubt_sim`

- Every file under `ubt_sim/docs/assets/`, including the simulation interface
  preview images (`*.png`) and demonstration videos (`walker.mp4`,
  `tienkung.mp4`).

### 3. Precompiled binaries

- `ubt_sim/teleoperation/bridges/zmq_image_bridge`
- `ubt_sim/teleoperation/msgs/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb`
- `ubt_IL/docker/ros2_msgs/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb`

### 4. Robot description and model assets

- Every file under `ubt_sim/assets/robots/`, including robot URDF
  descriptions (`walker_s2/s2.urdf`, `tienkung_pro/tienkung_pro_with_hands.urdf`),
  robot USD models (`walker_s2/s2_v1.usd`, `walker_s2/s2_v1back.usd`,
  `tienkung_pro/tienkung_pro_v2.usd`), and all associated textures and SubUSD
  files;
- `ubt_sim/teleoperation/control/tienkung_pro/right_arm.urdf`;
- Any `.urdf`, `.stl`, or `.STL` robot description or mesh file added to this
  repository at a later date.

These assets may be used only under the limited permissions in
`UBTECH-PROPRIETARY.txt`. Modification is prohibited except for the expressly
authorized installation-time RUNPATH adjustment. Redistribution outside the
GitHub platform is prohibited.

Third-party components contained in or required by the SDK remain governed by
their respective terms. See `SDK-THIRD-PARTY-NOTICES.md` and the corresponding
license texts in this directory.

## Third-party code and binaries included in this repository

The following third-party materials are distributed under their own licenses
(they are neither Apache-2.0 repository code nor UBTECH proprietary assets):

- Isaac Lab project code (parts of `ubt_sim/source/ubt_sim/`), Copyright
  (c) 2022-2025 The Isaac Lab Project Developers — BSD-3-Clause
  (`BSD-3-Clause.txt`); original copyright headers are retained in the files.
- NVIDIA Isaac GR00T code (`ubt_IL/docs/walker_s2/assets/data_config.py` and
  `gr00t_finetune.py`), Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES —
  Apache-2.0; NVIDIA copyright headers are retained in the files.
- PyTorch and torchvision binaries (`ubt_IL/docker/*.whl` and
  `ubt_IL/scripts/deploy/tienkung_pro/arm_64/*.whl`) redistributed under the
  PyTorch project's BSD-3-Clause license (`BSD-3-Clause.txt`).
- HuggingFace LeRobot, included as the git submodule `ubt_IL/lerobot` under
  the Apache License 2.0 (see its own LICENSE file).
- Third-party components embedded in the UBTECH SDK — see
  `SDK-THIRD-PARTY-NOTICES.md`.
