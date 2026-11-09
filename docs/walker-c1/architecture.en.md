# Walker C1 EDU: Architecture

> Code: `ubt_IL/walker_c1/` (LeRobot plugin `lerobot_robot_walker_c1`) and `ubt_IL/scripts/deploy/walker_c1/`

## Architecture

Dual-process bridge (same pattern as walker_s2 / tienkung). C1 is **fully decoupled** from Walker S2 EDU (separate package, registration name, and ports):

```
LeRobot inference (Python 3.12)                Bridge2 (Python 3.10)
  ZMQ PUB → 5661 (actions)        ──►         ZMQ SUB ← 5661
  ZMQ SUB ← 5662 (status)         ◄──         ZMQ PUB → 5662
  ZMQ SUB ← 5663 (JPEG images)    ◄──         camera relay ← ROS 2 /sensor/camera/head/color/raw
                                                  │
                                              ROS 2 DDS
                                                  │
                                        mc_state_msgs / mc_task_msgs
                                                  │
                                          Walker C1 EDU hardware / ubt_sim
```

## ZMQ ports

| Port | Purpose | Owner |
|---|---|---|
| 5661 | ZMQ actions (PUB→SUB) | plugin / bridge |
| 5662 | ZMQ state (PUB→SUB) | bridge / plugin |
| 5663 | ZMQ images (JPEG base64) | camera relay / plugin |
| 5690 / 5691 | Inference server REQ / PUB | inference_server |

Do **not** reuse walker_s2's 5561-5563 or the ubt_sim C1 bridge ports 5655-5658.

## 26D vector

```
[0-6]   Left arm 7    L_shoulder_pitch/roll/yaw, L_elbow_pitch/yaw, L_wrist_pitch/roll
[7-13]  Right arm 7   R_shoulder_pitch/roll/yaw, R_elbow_pitch/yaw, R_wrist_pitch/roll
[14-19] Left hand 6   left_thumb_swing/thumb_mcp/index_mcp/middle_mcp/ring_mcp/little_mcp
[20-25] Right hand 6  right_thumb_swing/thumb_mcp/index_mcp/middle_mcp/ring_mcp/little_mcp
```

- Single source of truth: `lerobot_robot_walker_c1/constants.py::ACTION_ORDER`
- Dataset names, deployment `action_order`, and bridge name-based scattering all follow this order
- **Head 2 / waist 3 / legs 12 are not part of the policy**: the bridge does not command them and the robot holds its current position. In simulation, `to_controller_data` uses `_hold_joint_targets` (anchored to HOME_POSE) to fill all 53 joints, so uncommanded joints do not fall.

## 13D variant (single arm)

```
[0-6]  Right arm 7   R_shoulder_pitch/roll/yaw, R_elbow_pitch/yaw, R_wrist_pitch/roll
[7-12] Right hand 6  right_thumb_swing/thumb_mcp/index_mcp/middle_mcp/ring_mcp/little_mcp
```

- Registry key `walker_c1_13d` (`constants.ROBOT_MODELS`); deployment config `configs/walker_c1_13d.json`
- `body.groups` still contains `left_arm` / `left_hand`; `inactive_fill` (13 keys) fills the left arm 7 + left hand 6 with home values and sends them — this keeps the left arm from going slack and isolates it from the policy
- Dataset `walker_c1_sim_13d_1RGB` (converted with `DIM=13`)

## 31-joint state order (state parsing only)

`BODY_JOINT_NAMES` (`constants.py`) = L arm 0-6 / R arm 7-13 / head_yaw 14 / head_pitch 15 / waist_yaw 16 / waist_pitch 17 / waist_roll 18 / left leg 19-24 / right leg 25-30. The bridge selects only the 14 arm joints by name (`body_groups` has empty head/waist lists).

## ZMQ messages

Action (single step):

```json
{"left_arm": [7], "right_arm": [7], "head": [], "waist": [],
 "left_hand": [6], "right_hand": [6], "ts": 1700000000.0}
```

Chunk (act_async): adds `n_points / fps / inference_time_sec / obs_time_sec / chunk_id`, with each group's value shaped `[C, n]`.

`{"hold": 1}` clears the trajectory immediately and **stops sending** RobotCommand (in half_sdk, C1 motion control holds the joint positions itself; the bridge no longer repeats the last frame at 300 Hz, avoiding servo jitter).

State (bridge → plugin): `{"left_arm": [7], "right_arm": [7], "head": [], "waist": [], "left_hand": [6], "right_hand": [6], "ts": ...}`.

## ROS 2 interfaces

| Direction | Topic | Type |
|---|---|---|
| sub | `/mc/sdk/robot_command` | `mc_task_msgs/RobotCommand` (`JointCmd.control_mode=MODE_POSITION=2`) |
| sub | `/mc/left_hand/command`, `/mc/right_hand/command` | `mc_task_msgs/JointCommand` (`mode=[5]`, `names` + `position`) |
| pub | `/mc/sdk/robot_state` | `mc_state_msgs/RobotState` (`joint_states`, 31 joints) |
| pub | `/mc/{left,right}_hand/joint_states` | `sensor_msgs/JointState` (SDK 6-joint order) |
| pub | `/sim/cmd_reset` | `std_msgs/Bool` (simulation only) |
| Camera (sim) | `/sensor/camera/head/color/raw` | `sensor_msgs/Image` (rgb8, BEST_EFFORT) |
| Camera (real) | `/sensor/camera/head_stereo_left/color/raw` | `shm_msgs/Image6m` (1920×1536 yuv422/UYVY, BEST_EFFORT) |
| Camera override | `WALKER_C1_CAMERA_TOPIC` / `WALKER_C1_CAMERA_MSG_TYPE` | Override topic and type; `/compressed` is inferred as a compressed image; `shm_msgs/Image6m` must be specified explicitly |

The `mc_task_msgs` / `mc_state_msgs` packages match the fields in walker_s2's `walker_sdk_ros2` workspace and reuse `/ubt_IL/walker_s2/walker_sdk_ros2/install` by default. Set `WALKER_C1_ROS_WS` to use a separate workspace.

## File responsibilities

| File | Responsibility |
|---|---|
| `constants.py` | Joint names/limits/`READY_POSE`/`ACTION_ORDER`/`ROBOT_MODELS`/`inactive_fill_for` |
| `config_walker_c1.py` | `WalkerC1RobotConfig` (registration name `walker_c1`), JSON loading and validation, `to_bridge_config` |
| `walker_c1.py` | `WalkerC1Robot`: connect / state thread / get_observation / send_action / chunk / home / disconnect |
| `hand_utils.py` | Hand 6-channel limit clipping (limits passed to the bridge once via `to_bridge_config`) |
| `camera/*` | ZMQ image subscription (base64 JPEG → ndarray, resized by W/H) |
| `ros2_walker_c1_bridge.py` | ZMQ↔ROS 2, name-based scattering, chunk blending/upsampling/300 Hz execution, stop publishing, singleton guard |
| `walker_c1_camera_relay.py` | `sensor_msgs/Image` → resize → JPEG → ZMQ 5663 |
| `chunk_processor.py` | Pure-numpy blending/interpolation/filtering (identical to and reused from walker_s2) |

## Key constraints

1. `lock_joints` means **skip, do not command**. It must be an empty list on C1; `_validate()` drops entries outside the body groups.
2. `CANONICAL_JOINT_NAMES` = 26 (excluding head/waist/legs). If you include non-group joints as walker_s2 does, `_inactive_fill` generates keys that cannot be assigned to the empty groups and `send_action` grouping fails.
3. `_inactive_fill` is generated only when `joint_config` hits `ROBOT_MODELS`; **the deployment JSON must also pass** `--robot.joint_config=walker_c1_26d`.
4. Hand limits are defined once: plugin `hand_joint_limits` → `to_bridge_config()` → bridge clipping; the bridge no longer hard-codes the V4 limit table.
5. Arm joint 4 is called `elbow_roll` in the SDK docs and `elbow_pitch` in the URDF; the ubt_sim bridge tolerates aliases, but verify the sign on hardware before deployment.
6. The camera is head monocular only (no wrist, no depth); relay QoS must be BEST_EFFORT. Simulation publishes uncompressed `sensor_msgs/Image`, hardware publishes `shm_msgs/Image6m` (1920×1536 yuv422/UYVY); the relay accepts both (including CompressedImage), decodes, resizes, and JPEG-encodes into ZMQ.
7. The plugin runs in the same container as walker_s2 but with a different registration name (`walker_c1` vs `walker`); do not import one from the other.

## Quick self-check

```bash
# 1. Package registration
python /ubt_IL/walker_c1/init_walker_c1.py

# 2. Config derivation (26D + 1 camera)
PYTHONPATH=/ubt_IL/lerobot/src:/ubt_IL/walker_c1/lerobot_robot_walker_c1 python - <<'PY'
from lerobot_robot_walker_c1 import WalkerC1RobotConfig
cfg = WalkerC1RobotConfig()
print(len(cfg.all_joints), list(cfg.cameras), cfg.zmq_cmd_port, cfg.lock_joints)
PY

# 3. Bridge defaults (no ROS required)
/usr/bin/python3 -c "
import sys; sys.path.insert(0,'/ubt_IL/walker_c1')
import ros2_walker_c1_bridge as b
print(b._DEFAULT_CFG['robot_model'], len(b._DEFAULT_CFG['body_joint_names']))
"
```
