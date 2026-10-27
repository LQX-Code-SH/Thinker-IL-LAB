# TienKung Dex 控制与数据采集

本目录脚本跑在 **ROS2 侧**（系统 Python 3.10），不依赖 Isaac Sim；
仿真进程由 `scripts/start_sim.sh` 启动，两者通过 ZMQ 5755-5758 端口通信。

## 一套逻辑，靠 ROS_DOMAIN_ID 分流

话题、消息类型、关节编址方式与具身天工 3.0 真机**完全一致**，没有"仿真档/真机档"
之分。选择控制哪台机器人只改一个环境变量：

| 通道 | `ROS_DOMAIN_ID` | 说明 |
|------|-----------------|------|
| 仿真 | `146`（默认） | `start_sim.sh` 起的桥接发布/订阅同一套话题 |
| 真机 | `0` | 厂商驱动发布的原话题 |

```bash
export ROS_DOMAIN_ID=146    # 控仿真
export ROS_DOMAIN_ID=0      # 控真机
```

于是同一份脚本、同一套 API，改环境变量即可从仿真换到真机，不用改代码。

## 启动顺序

```bash
# 0) 容器内
cd /ubt_sim
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash   # run.sh init 编译过才有

# 1) 启动仿真 + ROS2-ZMQ 桥接（domain 默认 146）
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh

# 2) 控制（另开终端）
export ROS_DOMAIN_ID=146
/usr/bin/python3 teleoperation/control/tienkung_dex/tienkung_dex_controller.py --print-state
```

上真机时第 1 步换成厂商驱动，只把 `ROS_DOMAIN_ID` 设为 0。

## 统一 CLI

`tienkung_dex_controller.py` 是唯一入口，子命令转发到对应实现：

| 子命令 | 作用 | 典型用法 |
|--------|------|----------|
| `state` | 状态、单关节移动、预备位、回站立位 | `state --print-state` / `state --init` / `state --home` |
| `joint` | 关节调试、手部开合、连续监视 | `joint --move elbow_pitch_r_joint=-0.5` |
| `endpoint` | 末端位姿 / 笛卡尔 IK / EE 相对位移 | `endpoint --side right --delta-xyz 0.05 0 0` |
| `hand` | 灵巧手开合 / 指定角度 / 闭合度 / 单指正弦 | `hand --side right --close` |
| `home` | 分段回站立位 + 张开双手 | `home --duration 10` |
| `play` | 播放预备位 / 回站立位，定时抓拍全程 | `play ready` / `play ready --from-home` / `play home --rounds 2` |
| `analyze` | 关节阶跃/正弦响应分析 + CSV | `analyze --joint elbow_pitch_r_joint --step -0.6 --duration 5` |
| `camera` | 相机 RGB/深度 信息、预览、存盘 | `camera --preview` / `camera --save --count 5` |
| `teleop` | 键盘末端遥操作（需 X11） | `teleop --side right` |

无子命令时默认走 `state`，兼容旧式 `--print-state` 调用；`--help` 看子命令清单，
`<子命令> --help` 看该子命令的参数。

目录里只有 `dex_controller.py` 一个"控制器"实现（`DexController` 类 + 各子命令入口），
其余是入口与工具：

| 文件 | 角色 |
|------|------|
| `dex_controller.py` | `DexController` 类 + state/joint/endpoint/hand/home 的 argparse 实现 |
| `tienkung_dex_controller.py` | 统一 CLI 分发（只做子命令转发，不含控制逻辑） |
| `keyboard_joint_control.py` | 关节空间键盘遥操作（分组 1~6，w/s 增量） |
| `keyboard_ee_control.py` | 末端位姿键盘遥操作（WASDQE + 数字键旋转） |
| `utils/{ik,camera,recorder,joint_analyzer,pose_play}.py` | IK / 相机 / 采集 / 响应分析 / 预备位·站立位播放 |

安全默认值：**腿部关节默认锁定**（真机站立时腿由行走控制器持有），
要动腿加 `--include-legs`；轨迹下发前做限位裁剪与最大速度检查。

## 末端遥操作的三个变量

`keyboard_ee_control.py` 把"意图"和"读数"彻底分开，三者互不回写：

| 变量 | 谁写 | 谁读 | 说明 |
|------|------|------|------|
| `_target_pose` | 键盘 | 发送器 | 目标末端位姿，**唯一事实源**（实测永不回写） |
| `_sent_pose` | 发送器（仅下发成功时前推） | 发送器 | 已下发到的位姿；被拒就停住，下一拍重试 |
| `_actual_pose` | 定期刷新（`--actual-hz`） | 状态行显示 | 实测末端，**绝不回写目标** |

**没有跟随误差门限**。防"目标跑飞"靠输入节流，不靠比较实测：

| 参数 | 默认 | 说明 |
|------|------|------|
| `--input-hz` | 1 Hz | 按键采样频率：一个采样窗口最多消化一步 |
| `--translation-step` | 12 mm | 每次按键的平移步长（= 单次下发上限，一次按键 = 一次下发） |
| `--rotation-step` | 0.05 rad | 每次按键的键盘旋转步长 |
| `--seed` | 预备位期望 | 首次目标初值（6 个数，基座系 m/rad）；重启遥操作时传上次的目标位姿即可接着走 |

脚本只认"按下"边沿：**按一次 = 走一步**；1 s 采样窗口内的多次按键合并成一步
（连点三下也只走一步），按住不放时 X 自动重复会补边沿，等效每秒一步——不像
旧版 `--poll-hz 200 × 3 mm/tick` 那样按住一秒就累出 600 mm 的目标。目标速度
因此被夹在 `translation_step × input_hz` = `12 mm/s`，而发送器 0.2 s 就能走完
12 mm —— 手臂始终跟得上，误差攒不起来，也就不需要门限。每走一步打印一行目标
位姿与随动误差供观察。鼠标通道已移除，全部输入都来自键盘。

长按的累加次数是 `⌊按住时长 / 采样周期⌋ + 1`，**不是** `按住时长 / 采样周期`：
按下瞬间所在的那个窗口会立刻兑现一步，之后靠自动重复每窗口再走一步。实测
1.5 s → 2 步、2.0 s → 3 步、3.0 s → 4 步。注意这一步依赖 X 自动重复是开着的
（`xset r on`，默认开）；关掉的话长按只会走 1 步。速度上限 `12 mm/s` 不受影响。

### 键位

| 键 | 作用 |
|----|------|
| `W`/`S` | 前后移动（**基座系** ±x） |
| `A`/`D` | 左右移动（基座系 ±y） |
| `Q`/`E` | 上下移动（基座系 ±z） |
| `1`/`2` | 绕**掌根 z** 轴旋转（roll，沿手指轴拧转）−/+ |
| `4`/`5` | 绕**掌根 x** 轴旋转（yaw，手掌左右摆）−/+ |
| `7`/`8` | 绕**掌根 y** 轴旋转（pitch，手掌上下摆）−/+ |
| `3` | 切换左右臂（切换后目标重置为**该臂预备位的期望末端**） |
| 空格 | 当前手臂灵巧手 预备位张开 / 握拳 切换 |
| `T` | 启停切换（**臂+手一起停**，暂停时空格也被忽略），**不重置目标变量**，恢复后接着原目标走 |
| `ESC` | 退出 |

平移走基座系是为了让方向不随手掌姿态漂移（预备位手掌前倾时按 `A` 仍是水平
左移）；旋转留在掌根帧是因为左右手掌根坐标系不同（roll 轴差约 10°、yaw/pitch
轴差约 103°），"相对手转"才符合操作者直觉。数字键奇数 = 负方向、偶数 = 正方向
（右手定则）。

目标变量没有任何"对齐实测"的快捷键：初值和切臂时的重置都用 `READY_POSE` 的
正运动学（`fk_ee_pose`），实测只进 `_actual_pose` 显示随动误差。

发送器与输入解耦：主循环以 `--poll-hz`（默认 200 Hz）跑发送器，发送器按
`--send-hz`（默认 30 Hz）读目标、按 `--max-delta-xyz`（默认 12 mm）分步下发。
`--duration` 与 `--max-delta-xyz` 必须和安全阈值配套：DexController 的限速检查
是 `max(Δs)·Δq/0.02 ≤ 1.5 rad/s`，0.2 s 轨迹（10 点）的 Δq 上限是 0.145 rad，
12 mm 步长约需 0.09 rad。旧的 0.12 s / 20 mm 组合需要 0.153 rad（2.8 rad/s），
会被稳定拒绝。

## DexController（基础控制器）

上层脚本（采集、抓取、遥操作）只依赖 `dex_controller.DexController`，不直接碰话题：

```python
import sys
sys.path.insert(0, "/ubt_sim/teleoperation/control/tienkung_dex")
from dex_controller import DexController

ctrl = DexController(variant="inspire")        # 同一套 API，仿真/真机都一样
ctrl.wait_for_state()

ctrl.move_to_ready_pose(duration_sec=10.0)     # READY_POSE（四段：外展抬臂→折前臂→转垂直→落到预备位）
ctrl.move_joint("head_yaw_joint", 0.3)         # 单关节
ctrl.open_hand("right", wait=True)             # 灵巧手
ctrl.move_arm_ee_delta("right", delta_xyz=(0.05, 0, 0))          # 末端相对位移
ctrl.move_arm_ee_delta_local("right", delta_xyz=(0, 0, 0.03))    # 末端局部系
ctrl.move_arm_ik("right", target_xyzrpy=[0.35, -0.25, -0.10, 3.0, 0.0, 1.6])
ctrl.home()                                    # 分段回站立位
```

要点：

- **预备位要避开正前方桌面**：parlor 场景原始桌面 0.897 m，本任务把 `table` 沿 z
  缩放到 1.1，桌面抬到 0.987 m（`config/tienkung_dex/parlor.yaml` 的
  `scene.table_z_scale`，苹果和盘子同步上移）；站立位手臂下垂时手在 0.80 m
  （比桌面还低）。`move_to_ready_pose` 照 tiangong2dex `reset` 的顺序分四段：
  肩外展抬臂（`READY_CLEARANCE_POSE`，手 1.41 m）→ 高位折前臂（1.35 m）→ 大臂转回
  近垂直（1.43 m）→ 前臂落到预备位（`READY_POSE`，肩外展 ±0.6 rad，手 1.29 m，
  桌面之上 0.30 m，左右手分开到 y=6.51 / 5.64）。预备位同时**低头 45°**
  （`head_pitch_joint`，正值为低头），让平装的头部相机对准桌面工作区。
  段间插值 FK 复核（含腰下垂、只算桌面投影范围内的点）最低 +0.30 m；
  `--no-staged` 一步到位已被新桌面挡住，只适合真机无桌面或手持物已抬高的场景。
  `home` 是这四段的**严格倒放**（前臂收回高位 → 大臂外展 → 伸直侧向 → 归位），
  每一段都复用正向已走过的路径。**不要**简化成"直接外展再落下"：那样肘还在
  -1.10，指尖会从桌面正上方 0.98 m（x≈0.50）扫过去，仿真 PD 下垂 0.1-0.3 rad
  就会碰桌面；倒放路径该段指尖最低 1.26 m。外展段容差 0.55 是刻意放宽的：
  仿真隐式 PD 扛不住水平伸直的手臂，肩 roll 稳态差 0.34-0.48 rad，此时手仍有
  1.17 m，属正常下垂而非被挡住。
- **位姿都在 URDF 基座系（pelvis 原点）**，`[x, y, z, roll, pitch, yaw]`，单位 m/rad；
  世界系换算用 `world_xyzrpy_to_base` / `base_xyzrpy_to_world`
  （机器人世界位姿见 `constants.ROBOT_WORLD_POS`，与 `config/tienkung_dex/parlor.yaml` 一致）。
- 末端 frame 按手型自动选：因时 `*_hand_base`，强脑 `*_base_link`，兜底 `wrist_roll_*_link`。
- IK 是 pinocchio DLS + 热启动 → 中立位 → 随机种子回退；未严格收敛但残差
  < 2 cm 时仍执行（`allow_approximate`），`require_success=True` 时可调小。
- 关节名可省略 `_joint` 后缀（`constants.normalize_joint_name`）。
- **预备位手型**：`move_to_ready_pose` 会顺带把双手摆成 `constants.ready_hand_pose()`
  ——因时拇指侧摆（通道 10）内收到码 850 = 90°，其余手指伸直；要给
  `hands=False` 才跳过。**全伸直**（`open_hand`，拇指侧摆回 1750）只在
  `home` / `state --home` 里出现；`pick_apple` 的 `_open()` 用的也是预备位手型，
  否则它会把刚摆好的拇指推回伸直。遥操作（`teleop` 按空格）两个状态都保持
  这个手型：张开走 `apply_ready_hand_pose(side=...)`，握拳走
  `close_hand(..., base_pose=ready_hand_pose())`——偏航/外展轴不随 `grip`
  缩放，拇指不会在合手瞬间甩回全外展。

## 话题（与真机一致）

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 指令 | `/arm/cmd` | `ros2_bridge_msgs/ArmCtrl` | `ctrl[].name` = 关节 ID 11-17/21-27，`pos` 绝对角度 |
| 指令 | `/head/cmd` | `ros2_bridge_msgs/HeadCtrl` | ID 1/2 |
| 指令 | `/waist/cmd`、`/leg/cmd` | `ros2_bridge_msgs/ArmCtrl` | ID 31-33 / 51-56,61-66 |
| 指令 | `/left_hand/angle_set`（因时）/ `/left_hand/set_motor_multi`（强脑） | 厂商消息 | 因时 13 通道、单位 0.1° 绝对角码（1200=120°）；强脑 1-1000 6 电机 |
| 状态 | `/robot_state` | `ros2_bridge_msgs/RobotState` | `head/waist/arm/leg` 分组 `MotorStatus` |
| 状态 | `/left_hand/angle_actual` / `/left_hand/motor_status` | 厂商消息 | |
| 图像 | `/ob_camera_head/color/image_raw` / `depth/image_raw` | `sensor_msgs/Image` | rgb8 + 16UC1（mm） |
| 复位 | `/sim/cmd_reset` | `std_msgs/Bool` | 仅仿真有效；真机没人订阅，发它无副作用 |

关节 ID ↔ 仿真关节名映射见 `constants.JOINT_ID_TO_SIM`
（注意真机 15/25 是 `wrist_yaw`，对应仿真 `elbow_yaw_{l,r}_joint`）。
话题字符串的唯一定义处是 `dex_controller.TOPICS` 与 `bridges/tienkung_dex/bridge_config.yaml`。

## 上真机前

1. **消息包**：`ros2_bridge_msgs` / `inspire_hand_msgs` / `brainco_hand_msgs` 在仿真机上
   是 `teleoperation/msgs/` 下按 SDK 用法重建的，字段布局未与厂商包核对——真机上
   要 source 厂商自带的消息包，重建版只用于仿真联调（桥接启动时会打 WARN）。
2. **驱动参数**：`constants.GROUP_CMD_LIMITS` 里的 `spd/cur`（臂 0.5/40、头腰 0.3/10）
   取自 SDK 示例；`CONTROL_MODE_IMPEDANCE` 只在真机有意义，仿真忽略 `mode`。
3. **腿**：默认不下发腿部指令，避免与行走控制器打架。
4. **急停**：`Ctrl+C` 会停止下发，但已发出的目标由真机驱动器继续保持；真机务必留硬件急停。
5. **domain**：真机 `ROS_DOMAIN_ID=0`。仿真默认 146，两者同时在线也不会串。

## 数据采集

```bash
bash teleoperation/control/tienkung_dex/save_data.sh                      # Enter 开始/停止，s 保存，q 退出
bash teleoperation/control/tienkung_dex/save_data.sh --variant brainco    # BrainCo 双手
```

`utils/recorder.py` 订阅的就是上面这套统一话题（`/robot_state` + 厂商手部状态 +
`/ob_camera_head/*`），action 取 `/arm|head|waist|leg/cmd` 与灵巧手指令，
所以**仿真和真机用同一个采集脚本**。

产物：

```
dataset/tienkung_dex/episode_0001.hdf5
├── joint_observations/{joint_position, joint_velocity, joint_effort}    # 31
├── hand_observations/{hand_left_*, hand_right_*}                        # Inspire 18+18 / BrainCo 11+11
├── action_observations/{joint_position, hand_left_position, hand_right_position}
├── camera_observations/images/head_rgb                                  # PNG 字节流
├── camera_observations/depth_images/head_depth                          # uint16 mm
└── timestamp
```

`attrs` 内含 `robot / hand_variant / body_joint_names / left_hand_joint_names / right_hand_joint_names / capture_hz / task_name`。

回放用 `teleoperation/tools/playback/`，适配层同样发这套话题。

## 已知限制

- 手部 `mimic` 关节（每手 5 个：`thumb_pip`、各指 `*_dip`）由 `action_process` 按 URDF
  倍率展开，ROS 侧无需下发；真机话题下这些关节没有独立读数，录制里取驱动展开值。
- 厂商灵巧手状态只有位置，没有速度/力矩，因此 `hand_*_velocity` / `hand_*_effort`
  在真机数据里恒为 0（仿真侧同样填 0，保证两边 schema 一致）。
- 手部下发按厂商接口换算，两个变体的语义不同：
  - **因时**：13 通道、单位 0.1° 的绝对角码，**每通道一个关节**（通道顺序见
    `constants.INSPIRE_CHANNELS`）。屈曲通道**码值越大越伸直**，`弯曲角 =
    (码上限 − code) × 0.1°`；侧摆通道 `摆角 = code × 0.1°`。量化步长 0.1°，
    通道 12（拇指二关节）在 URDF 里是 `thumb_pip` 的 mimic，只上报不生效。
  - **强脑**：6 电机 1-1000 闭合度，步长约 1/1000 量程。
  - 换算入口统一是 `constants.hand_code_to_rad` / `hand_rad_to_code`；真机若改
    协议只需改这两个函数与 `INSPIRE_CHANNELS`。
- 仿真的手臂执行器是隐式 PD（`stiffness=80`，无重力补偿），重力载荷下肩/肘稳态
  误差可达 0.02–0.06 rad；真机是驱动器闭环，不会有这个偏差。`move_to_pose` 的
  `settle_tolerance` 默认因此放宽到 0.08 rad。
- 相机外参取 `camera_head_link` 原点，`rot` 是单位四元数：URDF 的
  `camera_head_joint`（rpy −90/0/−90）已经把该 link 摆成 ROS 光学系，相机与
  `head_pitch_link` 同向。**视角由 `head_pitch_joint` 决定**（正值为低头，
  预备位低头 45°），不要再往 `rot` 里加安装俯角。
