# Walker C1 插件技术架构

> 本文与 `walker_c1/CLAUDE.md` 内容同步（仓库忽略 `CLAUDE.md`，故以本文为入库版本）。

## 架构

双进程桥接（与 walker_s2 / tienkung 同模式），C1 与 walker_s2 **完全解耦**（独立包、独立注册名、独立端口）：

```
LeRobot 推理 (Python 3.12)                    Bridge2 (Python 3.10)
  ZMQ PUB → 5661 (actions)        ──►         ZMQ SUB ← 5661
  ZMQ SUB ← 5662 (status)         ◄──         ZMQ PUB → 5662
  ZMQ SUB ← 5663 (JPEG images)    ◄──         camera relay ← ROS2 /sensor/camera/head/color/raw
                                                  │
                                              ROS2 DDS
                                                  │
                                        mc_state_msgs / mc_task_msgs
                                                  │
                                          Walker C1 真机 / ubt_sim
```

| 端口 | 用途 | 占用方 |
|---|---|---|
| 5661 | ZMQ 动作（PUB→SUB） | 本插件 / bridge |
| 5662 | ZMQ 状态（PUB→SUB） | bridge / 本插件 |
| 5663 | ZMQ 图像（JPEG base64） | camera relay / 本插件 |
| 5690 / 5691 | 常驻推理服务 REQ / PUB | inference_server |

**不要**复用 walker_s2 的 5561-5563 或 ubt_sim C1 桥接的 5655-5658。

## 26 维向量

```
[0-6]   左臂 7     L_shoulder_pitch/roll/yaw, L_elbow_pitch/yaw, L_wrist_pitch/roll
[7-13]  右臂 7     R_shoulder_pitch/roll/yaw, R_elbow_pitch/yaw, R_wrist_pitch/roll
[14-19] 左手 6     left_thumb_swing/thumb_mcp/index_mcp/middle_mcp/ring_mcp/little_mcp
[20-25] 右手 6     right_thumb_swing/thumb_mcp/index_mcp/middle_mcp/ring_mcp/little_mcp
```

- 顺序唯一来源：`lerobot_robot_walker_c1/constants.py::ACTION_ORDER`
- 训练数据集 names、部署 `action_order`、bridge 按名散射全部以此为准
- **头 2 / 腰 3 / 腿 12 不在策略中**：bridge 不下发，机器人保持当前位；仿真侧 `to_controller_data`
  用 `_hold_joint_targets`（锚定 HOME_POSE）填充全部 53 关节，未下发关节不会掉落

## 13 维变体（单臂）

```
[0-6]  右臂 7   R_shoulder_pitch/roll/yaw, R_elbow_pitch/yaw, R_wrist_pitch/roll
[7-12] 右手 6   right_thumb_swing/thumb_mcp/index_mcp/middle_mcp/ring_mcp/little_mcp
```

- 注册表键 `walker_c1_13d`（`constants.ROBOT_MODELS`），部署配置 `configs/walker_c1_13d.json`
- `body.groups` 仍保留 `left_arm`/`left_hand`，`inactive_fill`（13 个键）把左臂 7 + 左手 6
  填成 home 并下发——这样左臂不会被放松，也不会被策略干扰
- 数据集 `walker_c1_sim_13d_1RGB`（`DIM=13` 转换）

## 31 关节状态顺序（仅用于状态解析）

`BODY_JOINT_NAMES`（`constants.py`）= L 臂 0-6 / R 臂 7-13 / head_yaw 14 / head_pitch 15 /
waist_yaw 16 / waist_pitch 17 / waist_roll 18 / 左腿 19-24 / 右腿 25-30。
bridge 只按名取 14 个臂关节（`body_groups` 的 head/waist 为空列表）。

## ZMQ 消息

动作（单步）：

```json
{"left_arm": [7], "right_arm": [7], "head": [], "waist": [],
 "left_hand": [6], "right_hand": [6], "ts": 1700000000.0}
```

chunk（act_async）：追加 `n_points / fps / inference_time_sec / obs_time_sec / chunk_id`，
每组的值为 `[C, n]`。
`{"hold": 1}` 立即清空轨迹并**停止下发** RobotCommand（C1 在 half_sdk 下由运控自行保持
关节位置，桥接不再以 300 Hz 重复末帧，避免伺服抖动）。

状态（bridge → 插件）：`{"left_arm": [7], "right_arm": [7], "head": [], "waist": [],
"left_hand": [6], "right_hand": [6], "ts": ...}`。

## ROS2 接口

| 方向 | 话题 | 类型 |
|---|---|---|
| sub | `/mc/sdk/robot_command` | `mc_task_msgs/RobotCommand`（`JointCmd.control_mode=MODE_POSITION=2`） |
| sub | `/mc/left_hand/command`、`/mc/right_hand/command` | `mc_task_msgs/JointCommand`（`mode=[5]`，`names`+`position`） |
| pub | `/mc/sdk/robot_state` | `mc_state_msgs/RobotState`（`joint_states`，31 关节） |
| pub | `/mc/{left,right}_hand/joint_states` | `sensor_msgs/JointState`（SDK 6 关节序） |
| pub | `/sim/cmd_reset` | `std_msgs/Bool`（仅仿真） |
| 相机（仿真） | `/sensor/camera/head/color/raw` | `sensor_msgs/Image`（rgb8，BEST_EFFORT） |
| 相机（真机） | `/sensor/camera/head_stereo_left/color/raw` | `shm_msgs/Image6m`（1920×1536 yuv422/UYVY，BEST_EFFORT） |
| 相机覆盖 | `WALKER_C1_CAMERA_TOPIC` / `WALKER_C1_CAMERA_MSG_TYPE` | 覆盖话题与类型；`/compressed` 自动推断为压缩图，`shm_msgs/Image6m` 必须显式指定 |

消息包 `mc_task_msgs`/`mc_state_msgs` 与 walker_s2 的 `walker_sdk_ros2` 工作空间字段一致，
默认复用 `/ubt_IL/walker_s2/walker_sdk_ros2/install`；可用 `WALKER_C1_ROS_WS` 指向独立工作空间。

## 文件职责

| 文件 | 职责 |
|---|---|
| `constants.py` | 关节名/限位/READY_POSE/`ACTION_ORDER`/`ROBOT_MODELS`/`inactive_fill_for` |
| `config_walker_c1.py` | `WalkerC1RobotConfig`（注册名 `walker_c1`）、JSON 加载与校验、`to_bridge_config` |
| `walker_c1.py` | `WalkerC1Robot`：connect / 状态线程 / get_observation / send_action / chunk / home / disconnect |
| `hand_utils.py` | 手部 6 通道限位裁剪（限位经 `to_bridge_config` 单点传给 bridge） |
| `camera/*` | ZMQ 图像订阅（base64 JPEG → ndarray，按 W/H resize） |
| `ros2_walker_c1_bridge.py` | ZMQ↔ROS2、按名散射、chunk 融合/加密/执行期 300 Hz、stop 停发、单例守卫 |
| `walker_c1_camera_relay.py` | `sensor_msgs/Image` → resize → JPEG → ZMQ 5663 |
| `chunk_processor.py` | 纯 numpy 融合/插值/滤波（与 walker_s2 相同，原样复用） |

## 关键约束

1. `lock_joints` 语义 = **跳过不下发**。C1 必须为空列表；`_validate()` 会丢弃不在 body 组内的条目。
2. `CANONICAL_JOINT_NAMES` = 26（不含头/腰/腿）。若改成像 walker_s2 那样包含非 group 关节，
   `_inactive_fill` 生成的键无法归属到空组，`send_action` 分组会出错。
3. `_inactive_fill` 仅在 `joint_config` 命中 `ROBOT_MODELS` 时生成；**部署 JSON 也要传**
   `--robot.joint_config=walker_c1_26d`。
4. 手部限位单点定义：插件 `hand_joint_limits` → `to_bridge_config()` → bridge 裁剪；
   bridge 内不再硬编码 V4 限位表。
5. 第 4 个臂关节 SDK 文档叫 `elbow_roll`、URDF 叫 `elbow_pitch`；ubt_sim bridge 做别名容忍，
   真机部署前需现场核对符号。
6. 相机只有头部单目（无腕部、无深度）；relay QoS 必须 BEST_EFFORT。仿真发未压缩
   `sensor_msgs/Image`，真机发 `shm_msgs/Image6m`（1920×1536 yuv422/UYVY）；relay 两者都能吃
   （含 CompressedImage 兼容），解码后统一 resize 再 JPEG 编码进 ZMQ。
7. 与 walker_s2 的插件运行在同一容器内，注册名不同（`walker_c1` vs `walker`），
   不要互相 import。

## 快速自检

```bash
# 1. 包注册
python /ubt_IL/walker_c1/init_walker_c1.py

# 2. 配置派生（26 维 + 1 相机）
PYTHONPATH=/ubt_IL/lerobot/src:/ubt_IL/walker_c1/lerobot_robot_walker_c1 python - <<'PY'
from lerobot_robot_walker_c1 import WalkerC1RobotConfig
cfg = WalkerC1RobotConfig()
print(len(cfg.all_joints), list(cfg.cameras), cfg.zmq_cmd_port, cfg.lock_joints)
PY

# 3. bridge 默认配置（无需 ROS）
/usr/bin/python3 -c "
import sys; sys.path.insert(0,'/ubt_IL/walker_c1')
import ros2_walker_c1_bridge as b
print(b._DEFAULT_CFG['robot_model'], len(b._DEFAULT_CFG['body_joint_names']))
"
```
