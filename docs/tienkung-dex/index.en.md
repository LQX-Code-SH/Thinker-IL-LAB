# Walker TienKung Dex

Walker TienKung Dex is the single-arm dexterous manipulation variant of the TienKung series: a 7-DOF right arm plus a dexterous right hand, available with either the **Inspire RH5DG2** (67 revolute joints) or **BrainCo Revo2** (53 revolute joints) hand. The policy controls only the right arm and right hand; the left arm, waist, head, and legs are moved to the ready pose by `robot_ready.sh` and then held still.

![Walker TienKung Dex simulation preview](../assets/tienkung-dex仿真界面预览.png)

## Specifications

| Item | Specification |
|------|------|
| Configuration | 7-DOF right arm + dexterous right hand (Inspire 13 channels / BrainCo 6 motors) |
| Dexterous hand | Inspire RH5DG2 (67 revolute) / BrainCo Revo2 (53 revolute) |
| Common training dimensions | 20D (right arm 7 + right hand 13, Inspire) / 13D (right arm 7 + right hand 6, BrainCo or Inspire r13) |
| Joints not controlled by the policy | Left arm / waist / head / legs (constant in the data; held at the ready pose) |
| LeRobot plugin | `ubt_IL/tienkung_dex/lerobot_robot_tienkung_dex` (robot type `tienkung_dex`, camera type `dex_camera`) |
| USD model | `ubt_sim/assets/robots/tienkung_dex/usd/tienkung_dex_inspire.usd` / `tienkung_dex_brainco.usd` |
| Camera | Head-mounted monocular RGB: `/ob_camera_head/color/image_raw` |
| Training resolution | Simulation / BrainCo real: 640×360 RGB; Inspire r13 real config: 1280×720 |
| ROS domain | `ROS_DOMAIN_ID=0` by default (same domain as the real robot; set `146` on both sides to isolate) |

> ✅ Verified on hardware (Nov 2026): the full real-robot training and deployment loop has been validated with both Inspire and BrainCo hands. Verified dataset: `tienkung_dex_real_pick_apple_brainco_13d` (32 episodes / 16,956 frames / 30 fps, 13D = right arm 7 + BrainCo hand 6).

## Action space (three `JOINT_CONFIG` variants)

| `JOINT_CONFIG` | Dimension | Hand | Policy-side hand representation |
|---|---|---|---|
| `tienkung_dex_20` | 20D = right arm 7 + right hand 13 | Inspire | 13 joint angles in rad |
| `tienkung_dex_brainco_13` | 13D = right arm 7 + right hand 6 | BrainCo | 6 closure values in 0~1 |
| `tienkung_dex_inspire_13` | 13D = right arm 7 + right hand 6 | Inspire | 6 normalized values in 0~1, expanded by Bridge2 into 13 angle codes |

## Simulation tasks

| Task ID | Hand | Joints | Config |
|---------|------|--------|--------|
| `UBTSim-TienkungDex-Parlor-v0` | Inspire RH5DG2 | 67 | `config/tienkung_dex/parlor.yaml` |
| `UBTSim-TienkungDex-Brainco-Parlor-v0` | BrainCo Revo2 | 53 | `config/tienkung_dex/brainco_parlor.yaml` |

## Documentation

Platform entry points (shared architecture and commands are on the platform overview pages):

- [Simulation Platform · Walker TienKung Dex](sim-setup.md) - Container build, launching simulation, batch collection, camera test, playback
- [Imitation Learning Platform · Walker TienKung Dex](convert-train.md) - HDF5 -> LeRobot conversion, ACT training, offline evaluation, deployment parameters

In order of use:

1. [Simulation workflow](sim-workflow.md) - End to end: simulation collection -> conversion -> training -> evaluation -> sim deployment
2. [Real-robot workflow](real-workflow.md) - End to end: real collection -> conversion -> training -> evaluation -> real deployment
3. [On-robot deployment](arm-deploy.md) - Docker-free deployment on the Thor board (venv + system Python dual stack)

## Deployment results

**Simulation**

<video src="../assets/tienkung-dex仿真部署效果.mp4" controls muted width="100%"></video>

**Real robot**

<video src="../assets/tienkung-dex真机部署效果.mp4" controls muted width="100%"></video>
