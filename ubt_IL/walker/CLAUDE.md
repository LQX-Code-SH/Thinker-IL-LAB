# Walker Plugin — Technical Architecture

## Architecture Overview

Dual-process bridge design (same pattern as TienKung): LeRobot (Python 3.12) communicates with `walker/ros2_walker_bridge.py` (Bridge2, Python 3.10) via ZMQ for actions/status. Bridge2 interfaces with Walker S2 body/hand/gripper hardware via ROS2 DDS. Camera images flow through a **separate** `walker_camera_relay.py` process (Python 3.10), which subscribes to ROS2 `shm_msgs/Image*` topics and publishes JPEG over ZMQ to LeRobot's `WalkerCamera`.

```
LeRobot Inference (Python 3.12)       Bridge2 (Python 3.10)
  ZMQ PUB → 5561 (actions)       →    ZMQ SUB ← 5561
  ZMQ SUB ← 5562 (status)        ←    ZMQ PUB → 5562
                                         │
                                    ROS2 DDS
                                         │
                                  ┌──────┴──────┐
                                  │             │
                            Walker S2 HW    Body / Hand / Gripper

LeRobot Inference (Python 3.12)       CameraRelay (Python 3.10, 独立进程)
  ZMQ SUB ← 5563 (images)        ←    ZMQ PUB → 5563
                                         │
                                    ROS2 DDS
                                         │
                                  shm_msgs Camera Topics
```

## Deployment Configs

Walker rollout is configured by JSON files under:

`/ubt_IL/scripts/deploy/walker_s2/configs/`

Initial models:

| Model | Dim | End effector | File |
|-------|-----|--------------|------|
| `walker_s2_v4_hand_31d` | 31 | left/right 7D V4 hands | `walker_s2_v4_hand_31d.json` |
| `walker_s2_gripper_19d` | 19 | left/right 1D PGC grippers | `walker_s2_gripper_19d.json` |

Walker sim/real deployment uses **per-camera `shm_msgs/Image*` types** — each camera topic may use a different buffer size. The authoritative mapping is defined in `constants.py` `RobotConfig.camera_msg_types`. Camera configs should set `msg_type` per camera:

| Camera | Type |
|--------|------|
| `camera_head` (stereo/color/raw) | `Image2m` |
| `camera_head_left` (stereo_left) | `Image6m` |
| `camera_head_right` (stereo_right) | `Image6m` |
| `camera_wrist_left` | `Image1m` |
| `camera_wrist_right` | `Image1m` |

Do not publish `sensor_msgs/Image` on the same camera topic name; use a separate debug topic if a standard ROS image stream is needed.

Run examples:

```bash
# 31D V4 hand model
ROBOT_MODEL=walker_s2_v4_hand_31d \
POLICY_PATH=/ubt_IL/model/<walker_31d_policy>/checkpoints/last/pretrained_model \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh

# 19D 1-DOF gripper model
ROBOT_MODEL=walker_s2_gripper_19d \
POLICY_PATH=/ubt_IL/model/<walker_19d_policy>/checkpoints/last/pretrained_model \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

Use `ROBOT_CONFIG=/path/to/config.json` to bypass `ROBOT_MODEL` preset lookup.

### Config naming rule

Config files use **real joint/actuator names** without `.pos`:

- Body: `L_elbow_roll_joint`, `R_shoulder_pitch_joint`, `head_pitch_joint`, `waist_yaw_joint`
- V4 hand: `left_thumb_swing`, `right_index_mcp`
- PGC gripper: `left_grip`, `right_grip`

The plugin derives LeRobot features by appending `.pos`, e.g. `L_elbow_roll_joint` → `L_elbow_roll_joint.pos`. Do not use placeholder names like `left_arm_j1` in new configs.

## ZMQ Port Reference

| Port | Direction | Purpose |
|------|-----------|---------|
| 5561 | LeRobot PUB → Bridge2 SUB | action commands |
| 5562 | Bridge2 PUB → LeRobot SUB | joint/gripper state |
| 5563 | CameraRelay PUB → LeRobot SUB | Camera images (JPEG over JSON, via `walker_camera_relay.py`) |

## State/Action Layouts

### 31D V4 hand layout

```
[0-6]   left arm real joints
[7-13]  right arm real joints
[14-15] head_pitch_joint, head_yaw_joint
[16]    waist_yaw_joint
[17-23] left V4 hand joints
[24-30] right V4 hand joints
```

### 19D PGC gripper layout

```
[0-6]   left arm real joints
[7-13]  right arm real joints
[14-15] head_pitch_joint, head_yaw_joint
[16]    waist_yaw_joint
[17]    left_grip
[18]    right_grip
```

Default locked joints are still included in the 19D action/state tensor but are not sent in `RobotCommand`:

- `head_pitch_joint`
- `head_yaw_joint`
- `waist_yaw_joint`

ZMQ internal message format remains stable across variants:

```json
{
  "left_arm": [7 floats],
  "right_arm": [7 floats],
  "head": [2 floats],
  "waist": [1 float],
  "left_hand": [7 floats for V4, 1 float for gripper],
  "right_hand": [7 floats for V4, 1 float for gripper],
  "ts": "timestamp"
}
```

## ROS2 Topics & Messages (Bridge2)

### Body

| Topic | Message Type | Purpose | QoS |
|-------|-------------|---------|-----|
| `/mc/sdk/robot_state` | `mc_state_msgs/RobotState` | body state | BEST_EFFORT + VOLATILE |
| `/mc/sdk/robot_command` | `mc_task_msgs/RobotCommand` (`JointCmd[]`) | body control | RELIABLE + VOLATILE |

Body command uses `JointCmd.MODE_POSITION = 2`. Locked joints are excluded from `JointCmd[]`.

### V4 hand

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/mc/left_hand/joint_states` | `sensor_msgs/JointState` | 7D left V4 hand state |
| `/mc/right_hand/joint_states` | `sensor_msgs/JointState` | 7D right V4 hand state |
| `/mc/left_hand/command` | `mc_task_msgs/JointCommand` | left V4 hand command |
| `/mc/right_hand/command` | `mc_task_msgs/JointCommand` | right V4 hand command |

V4 hand command uses `mode = [5, ...]` and clamps values by V4 joint limits.

### Camera

Walker sim and real camera topics use different `shm_msgs/Image*` types per camera (Image1m/Image2m/Image6m). The standalone `walker_camera_relay.py` process subscribes to the configured `msg_type`, resizes to the target resolution, and republishes JPEG frames over ZMQ 5563 for LeRobot.

| Topic | Message Type | Purpose | QoS |
|-------|-------------|---------|-----|
| `/sensor/camera/stereo/color/raw` | `shm_msgs/Image2m` | head RGB camera | BEST_EFFORT + VOLATILE |
| `/sensor/camera/stereo_{left,right}/image/raw` | `shm_msgs/Image6m` | stereo camera pair | BEST_EFFORT + VOLATILE |
| `/sensor/camera/wrist_{left,right}/color/raw` | `shm_msgs/Image1m` | wrist camera pair | BEST_EFFORT + VOLATILE |

Avoid using the same topic name for both `sensor_msgs/Image` and `shm_msgs/Image*`.
### PGC 1-DOF gripper

| Topic | Message Type | Purpose | QoS |
| `/ecat/left_grip/state` | `ecat_task_msgs/GripStatus` | left gripper state | BEST_EFFORT + VOLATILE |
| `/ecat/right_grip/state` | `ecat_task_msgs/GripStatus` | right gripper state | BEST_EFFORT + VOLATILE |
| `/ecat/left_grip/cmd` | `ecat_task_msgs/GripCmd` | left gripper command | RELIABLE + VOLATILE |
| `/ecat/right_grip/cmd` | `ecat_task_msgs/GripCmd` | right gripper command | RELIABLE + VOLATILE |

Default gripper limits:

- position: `[0.0, 0.05]` m
- force: `[41.0, 100.0]` N
- velocity: `[0.0, 0.01]` m/s
- acceleration: `[0.0, 3.0]` m/s² (`GripCmd.cur`)

## ⚠️ Critical: JointCmd vs JointCommand vs GripCmd

- **JointCmd**: nested inside `RobotCommand`; body control; `MODE_POSITION = 2`.
- **JointCommand**: standalone V4 hand control; V4 uses `mode = [5]`.
- **GripCmd**: standalone PGC gripper control; fields include `pos`, `vel`, `force`, `cur`, `mode`, `init`, `stop`, `reset`, `homing`.

Confusing these will cause silent failures or commands going to the wrong controller.

## ⚠️ Critical: QoS

Walker sensor topics require BEST_EFFORT + VOLATILE QoS. If the bridge uses the wrong QoS, it will silently receive no messages:

```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
```

## Prerequisites (before running)

1. Start motion container and switch to SDK controller:
   ```bash
   rosa run t800_mc_server start_mc_client
   rosa run rosa_controllers switch_controller config_mc_walker_s2_v1_sps
   ```
2. Use the project Docker default FastDDS no-SHM profile unless the site explicitly requires CycloneDDS. The default whitelist includes `127.0.0.1`, `192.168.41.99`, and `192.168.11.99`; adjust `/ubt_IL/docker/fastdds_no_shm.xml` to the host NIC IP before connecting hardware.
3. Ensure robot is in a safe position before enabling SDK control.
4. For gripper deployment, verify `/ecat/{left,right}_grip/state` first with `joint_test.py --grip-print`.

## Container Environment

The entrypoint auto-installs on container start:

| Component | Method | Location |
|-----------|--------|----------|
| lerobot | `uv pip install -e /ubt_IL/lerobot` | venv (editable) |
| walker plugin | `uv pip install -e /ubt_IL/walker/lerobot_robot_walker` | venv (editable) |
| ROS2 Humble | apt (pre-installed in image) | `/opt/ros/humble/` |
| mc_task_msgs | colcon build or .deb | `/opt/ros/humble/` |
| mc_state_msgs | colcon build or .deb | `/opt/ros/humble/` |
| ecat_task_msgs | colcon build or .deb | `/opt/ros/humble/` |
| shm_msgs | colcon build or .deb | `/opt/ros/humble/` |

## DOF Architecture

Walker S2 supports variable-DOF deployment via an `IntEnum` registry pattern, identical to the TienKung architecture.

### Key Files

| File | Purpose |
|------|---------|
| `lerobot_robot_walker/constants.py` | DOF enums, `JOINT_INDEX_ENUMS` registry, `inactive_fill_for()`, `joint_names_with_pos()` |
| `lerobot_robot_walker/config_walker.py` | `joint_config` field, `_rebuild_groups_from_enum()`, `__post_init__` derives `all_joints` + `_inactive_fill` |
| `lerobot_robot_walker/walker.py` | `send_action` uses `get_val()` + `_inactive_fill`; `get_observation` filters to `_all_joints` |

### Registered DOF Configs

| Name | Dim | Joints |
|------|-----|--------|
| `walker_s2_31d` | 31 | Body(17) + L V4 hand(7) + R V4 hand(7) |
| `walker_s2_19d` | 19 | Body(17) + L grip(1) + R grip(1) |
| `walker_s2_10d` | 10 | R arm(7) + head(2) + R grip(1) |

### How inactive_fill works

1. 6 hardware groups (`left_arm`, `right_arm`, `head`, `waist`, `left_hand`, `right_hand`) are fixed — bridge expects all 6 in ZMQ messages
2. Policy outputs only joints in the selected DOF enum
3. `send_action` looks up each hardware joint by name in the action dict
4. Missing joints get their value from `DEFAULT_INACTIVE_FILL`: body joints → `READY_POSE`, V4 hands → `0.0` (extended), grippers → `0.0` (safe closed)
5. A complete 6-group ZMQ message is assembled and sent to the bridge

### Adding a custom DOF

1. Define an `IntEnum` class in `constants.py` — member order = dataset order
2. Register in `JOINT_INDEX_ENUMS` dict
3. Deploy with `JOINT_CONFIG=my_dof_name`
4. Create matching convert config + train config with `shape: [N]`

### Deployment

```bash
# Deploy subset policy (10D: right arm + head + right grip)
JOINT_CONFIG=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/walker_s2_10d_policy/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

## Plugin Registration

Package name `lerobot_robot_walker` follows the `lerobot_robot_` prefix convention, auto-discovered by `register_third_party_plugins()`. No modifications to upstream lerobot source required.
