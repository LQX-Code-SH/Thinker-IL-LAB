# Walker C1 EDU

Walker C1 EDU is a humanoid robot with 53 DOF and 41 controllable upper-body joints. Each arm has 7 DOF; each hand is an 11-joint dexterous hand (exposed as 6 SDK channels); the head carries a monocular RGB camera.

![Walker C1 EDU simulation preview](../assets/walker-c1仿真界面预览.png)

## Specifications

| Item | Specification |
|------|------|
| DOF | 53 (arms 14 + hands 11×2 + head 2 + waist 3 + legs 12) |
| End effectors | 2 × dexterous hands (11 joints each; 6 SDK channels: `thumb_swing` / `thumb_mcp` / `index_mcp` / `middle_mcp` / `ring_mcp` / `little_mcp`) |
| Common training dimensions | 26D (left arm 7 + right arm 7 + left hand 6 + right hand 6) / 13D (right arm 7 + right hand 6, single-arm tasks) |
| Joints not controlled by the policy | Head 2 / waist 3 / legs 12 (hold their current position; simulation anchors to HOME_POSE) |
| LeRobot plugin | `ubt_IL/walker_c1/lerobot_robot_walker_c1` (robot type `walker_c1`) |
| USD model | `ubt_sim/assets/robots/walker_c1/usd/walker_c1_with_hands.usd` |
| Cameras | Head monocular RGB: simulation `/sensor/camera/head/color/raw`, real robot `/sensor/camera/head_stereo_left/color/raw` |
| Training resolution | Simulation 320×240; real robot 640×512 (5:4) |
| ROS domain | `ROS_DOMAIN_ID=0` by default (simulation and real robot share domain 0; isolate by domain ID) |

> Verified on hardware (Nov 2026): 13D (right arm 7 + right hand 6), 30 fps, head camera 640×512. Dataset `walker_c1_real_13d_1RGB` (34 episodes / 11,406 frames) -> model `walker_c1_real_13d_act_1RGB`; the real-robot deployment loop is working.

## Simulation task

| Task ID | Scene | Description |
|---------|------|------|
| `UBTSim-WalkerC1-Parlor-v0` | Parlor pick-and-place | Apple pick-and-place with online IK collection (default) |

Walker C1 EDU and Walker S2 EDU use the same simulation ZMQ ports (5655-5658); **only one robot can run at a time**.

## Documentation

Platform entry points (shared architecture and commands are on the platform overview pages):

- [Simulation Platform · Walker C1 EDU](sim-setup.md) - Container build, launching simulation, batch collection, reset and playback
- [Imitation Learning Platform · Walker C1 EDU](convert-train.md) - HDF5 -> LeRobot conversion, ACT training, offline evaluation, deployment parameters

In order of use:

1. [Simulation workflow](sim-workflow.md) - End to end: simulation collection -> conversion -> training -> evaluation -> sim deployment
2. [Real-robot workflow](real-workflow.md) - End to end: half_sdk prerequisite -> real collection -> conversion -> training -> real deployment
3. [Real-robot deployment](deploy.md) - half_sdk guard, ready/home poses, ROBOT_MODELS registry, safety prechecks
4. [Inference server](inference-server.md) - Warm start, ZMQ commands, remote launch, model hot-swap
5. [Architecture](architecture.md) - 26D/13D contract, ZMQ ports, ROS 2 topics, hand mapping

## Deployment results

**Simulation**

<video src="../assets/walker-c1仿真部署效果.mp4" controls muted width="100%"></video>

**Real robot**

<video src="../assets/walker-c1真机部署效果.mp4" controls muted width="100%"></video>
