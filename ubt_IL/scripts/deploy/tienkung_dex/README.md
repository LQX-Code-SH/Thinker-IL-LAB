# TienKung Dex（天工行者 Dex V3）部署

容器内执行。动作空间 **20D = 右臂 7 + 右手 13（因时 Inspire）**；强脑 BrainCo 用
13D = 右臂 7 + 右手 6。左臂 / 腰 / 头 / 腿**不进模型**——数据里它们全程恒定，由
`robot_ready.sh` 摆到预备位后保持不动。

| 脚本 | 用途 |
|------|------|
| [robot_ready.sh](robot_ready.sh) | 预备位（四段到位 + 手型）。**跑策略前必须先执行** |
| [rollout.sh](rollout.sh) | 单次 rollout（DURATION 秒） |
| [inference_server.sh](inference_server.sh) / [inference_client.sh](inference_client.sh) | 常驻推理服务 + 客户端 |
| [reset.py](reset.py) | 复位**仿真**场景（`/sim/cmd_reset`） |

## 正确顺序

```bash
# 1. 机器人到预备位（仿真 ROS_DOMAIN_ID=146 / 真机 0）
bash robot_ready.sh            # 仿真
bash robot_ready.sh --real     # 真机

# 2. 再跑策略（rollout 或常驻服务）
POLICY_PATH=... bash rollout.sh
```

`robot_ready.py` 与 Bridge2 都写 `/arm/cmd`。Bridge2 在跑时 `robot_ready.py` 会拒绝执行
（`--force` 可跳过）——**永远不要把两者同时跑**。

## robot_ready.sh

```bash
bash robot_ready.sh                       # 仿真（最终目标 READY_POSE ∪ READY_POSE_TARGET）
bash robot_ready.sh --real                # 真机（最终目标 READY_POSE）
bash robot_ready.sh --dry-run             # 只打印四段目标，不发指令
bash robot_ready.sh --stage-durations 4,4,4,4 --rate 30
bash robot_ready.sh --hands right         # 只摆右手（both/left/right/none）
```

| 变量 / 参数 | 默认值 | 说明 |
|------|--------|------|
| `ROS_DOMAIN_ID` | `0` | 真机 0；仿真 146 |
| `TIENKUNG_DEX_MSGS_WS` | `/opt/tienkung_dex_msgs_ws` | dex 消息包工作空间 |
| `--real` | 关 | 真机目标位（不含仿真重力前馈补偿） |
| `--rate` | `50` | 指令下发频率 Hz |
| `--stage-durations` | `5,5,5,5` | 四段时长（秒） |
| `--settle` | `3` | 最后一段结束后等稳定的秒数 |
| `--tolerance` | `0.25` | 到位报告的超差阈值 rad |

**为什么要分四段**：站立位手臂自然下垂时手在世界 0.80 m，比桌面（0.987 m）还低。
从站立位直达预备位，手会贴着桌面从体侧扫过去被顶住、肘收不到位。四段是
`肩外展抬臂过桌面 → 高位折前臂 → 大臂转回近垂直 → 前臂落到预备位`，每段都验证过
手部离桌面 ≥ 0.23 m。

**发哪些组**：

| 组 | 话题 | 发不发 | 原因 |
|---|---|---|---|
| head | `/head/cmd`（`HeadCtrl`） | 发 | `head_pitch = 0.785398`（低头 45°）**是训练视角的前提**，不发就是另一个观测分布 |
| waist | `/waist/cmd`（`ArmCtrl`） | 发 | 预备位腰部有姿态 |
| arm | `/arm/cmd`（`ArmCtrl`） | 发 | **左臂也要发**：预备位左肩外展 ±0.60，数据集中左臂恒定在该姿态 |
| leg | `/leg/cmd` | **永远不发** | 真机腿由行走控制器独占；仿真里腿保持初始 standing 目标，与数据集一致 |

**仿真 vs 真机目标位不同，别弄反**：

- 仿真 `READY_POSE ∪ READY_POSE_TARGET`。`READY_POSE_TARGET` 是"让指令 = 实际"的重力
  前馈补偿（只覆盖双臂 14 + 腰 3，**不含 head**，head 从 `READY_POSE` 取）。
- 真机 `READY_POSE`。真机驱动器闭环，本来就没有仿真那段下垂；用 TARGET 会站成旧版
  仿真姿态（**低 13 cm**）。

预备位是**绝对**目标，不是"相对当帧读数"，更不是回 0（站立位膝盖本就微屈 0.368/0.341）。

## rollout.sh

```bash
POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_20 FPS=15 DURATION=60 \
bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
```

真机：

```bash
ROS_DOMAIN_ID=0 POLICY_PATH=... CAMERA_PORT=5558 IMAGE_ENABLED=1 \
DURATION=10 MAX_RELATIVE_TARGET=0.01 \
bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
```

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填** | checkpoint 目录（含 `config.json`） |
| `JOINT_CONFIG` | `tienkung_dex_20` | `tienkung_dex_20`（因时 13 通道）/ `tienkung_dex_brainco_13`（强脑 6 电机） |
| `HAND_VARIANT` | 空 | 留空跟随 `JOINT_CONFIG`；显式填写时两者必须一致 |
| `STRATEGY` | `base` | rollout 策略类型 |
| `FPS` | `15` | 控制环频率（= 数据集帧率） |
| `DURATION` | `60` | 运行时长（秒） |
| `TASK` | `pick apple` | 任务描述 |
| `ZMQ_HOST` | `127.0.0.1` | 桥接地址（跨机部署时填对端 IP） |
| `CAMERA_PORT` | `5758` | **仿真 5758**（ubt_sim 直发 JPEG）/ **真机 5558**（Bridge2 转码） |
| `IMAGE_ENABLED` | `0` | 真机设 `1`，Bridge2 才把头相机编成 JPEG |
| `MAX_RELATIVE_TARGET` | 空（不限） | 单步相对目标限幅 rad；真机首轮建议 `0.01` |
| `INFERENCE_TYPE` | `sync` | **只支持 sync**（Dex 桥接没有 chunk 消费者） |

### 相机

`dex_camera`（不是 `image_server`）——三个机型的相机各自注册独立类型名，两个包同时被
`register_third_party_plugins()` 自动 import 时不会撞注册表。

| 场景 | 链路 |
|------|------|
| 仿真 | ubt_sim 的 `AsyncJpegCameraSender` 直接在 **5758** 发 JPEG（12 字节 `dI` 头），`dex_camera` 原生会剥 |
| 真机 | 没有这条链路：Bridge2（`--enable-image`）把 `/ob_camera_head/color/image_raw` 编成 JPEG 发 **5558** |

### 安全预检

与 `walker_s2/rollout.sh` 同一套：比 `policy.output_features.action.shape[0]` 与
`len(joint_names_with_pos(JOINT_INDEX_ENUMS[JOINT_CONFIG]))`，不匹配直接退出。

对 **ACT 名字比对不会触发**：ACT 的 `pretrained_model/config.json` 里
`output_features.action` 只有 `{"type":"ACTION","shape":[N]}`，没有 names
（`PolicyFeature` 本身就没有 names 字段）。所以 `ALLOW_DIM_ONLY_POLICY` 默认 `1`。
顺序的真正保险是：`TienKungDex20JointIndex` 的成员顺序 == 转换配置里的 `names` ==
数据集顺序，Bridge2 全程**按关节名**寻址、不按位。

## inference_server.sh（常驻预热）

`rollout.sh` 每次都冷启动（policy 加载到 CUDA + 桥接拉起）。常驻服务一次性预热，
之后 `start/stop/home/status/shutdown` 秒级响应。

```bash
# 1. 拉起（容器内，后台预热；引擎 paused，不动机器人）
POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_20 FPS=15 \
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_server.sh

# 2. 轮询到 READY
bash inference_client.sh status

# 3. 开始 / 停止（动！）
bash inference_client.sh start
bash inference_client.sh stop

# 4. 回原点 / 关停
bash inference_client.sh home
bash inference_client.sh shutdown
```

`home` 是**右臂**从当前读数插值 2 s 到 `home_position`（站立位手臂），**手指保持不动**。
它走逐帧单步下发而不是 walker 的整块 chunk——Dex 桥接只认单步消息，整块会被当成维度
不符丢掉。回**预备位**（含腰/头/手型）仍要用 `robot_ready.sh`，`home` 只动右臂。

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填** | pretrained_model 目录 |
| `JOINT_CONFIG` | `tienkung_dex_20` | 同上 |
| `FPS` | `15` | 控制环 FPS |
| `CAMERA_PORT` | `5758` | 相机端口（真机 5558） |
| `IMAGE_ENABLED` | `0` | 真机 1 |
| `MAX_RELATIVE_TARGET` | 空（不限） | 单步相对目标限幅 rad；真机常驻服务同样建议 `0.01` |
| `SERVER_CMD_PORT` / `SERVER_STATUS_PORT` | `5570` / `5571` | ZMQ REP / PUB 端口 |
| `PID_FILE` / `LOG_FILE` | `/tmp/tienkung_dex_inference_server.pid` / `.log` | 单例与日志 |

> **没有 `load`（热切换）**：服务端重建机器人的代码路径写死了 `--robot.type=walker`
> 且不带 `--robot.cameras`，对 Dex 会重建出错误的机器人（无相机）。换模型请
> `shutdown` + 重新 `launch`（客户端会直接拒绝 `load` 并提示）。

## reset.py（仅仿真）

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=146 /usr/bin/python3 reset.py           # 默认重复发 3 次
```

真机没有这个复位话题，真机的"复位"就是 `robot_ready.sh --real`。

## 20D 向量

```
[0-6]   右臂 7 : shoulder_pitch_r, shoulder_roll_r, shoulder_yaw_r,
                 elbow_pitch_r, elbow_yaw_r, wrist_pitch_r, wrist_roll_r
[7-19]  右手13 : thumb_yaw, thumb_mcp, thumb_pip, index_yaw, index_mcp, index_pip,
                 middle_yaw, middle_mcp, middle_pip, ring_mcp, ring_pip,
                 pinky_mcp, pinky_pip
```

⚠️ **数据集手序 ≠ 因时通道序**。数据集按 URDF（thumb→pinky），`INSPIRE_CHANNELS`
按 pinky→thumb，落地映射是 `[10,11,12,5,4,9,3,2,8,1,7,0,6]`。Bridge2 按关节名
散列，不按位；启动时逐字比对关节名，不一致直接拒启。

## 已知待确认

- **仿真实际控制频率**：仿真主循环约 11 Hz（RTF≈0.11）而数据是 15 Hz，同一串动作在
  仿真时间轴上会被拉长。上线第一步应打印实际达到的 action 下发频率与 `/robot_state`
  回传频率，再决定 `FPS` 取值（数据集保持 15 Hz 不动）。
- **真机 `READY_POSE` 验证**：该值来自仿真的重力前馈标定，上真机第一步就是只跑
  `robot_ready.sh --real`，比对姿态与仿真画面是否一致，再上策略。
