# Walker C1 PICO 仿真遥操作

这套程序不使用 GMR，也不需要腰绑带或腿部 Motion Tracker。当前配置只遥操作机器人右臂和
右手；头部、左臂和左手保持 task reset 姿态。

## 每次使用：按顺序执行

### 1. 宿主机启动 PC Service

```bash
sudo /usr/bin/systemctl start thinker-studio.service
curl -fsS -X POST http://127.0.0.1:9999/api/control/robotics_service_start
curl -fsS http://127.0.0.1:9999/api/control/services_status
```

状态中应看到 `robotics_service.running=true`。

### 2. PICO 端连接

```text
PC Service:     192.168.1.4
Head:           On
Controller:     On
Hand:           Optional
Motion Tracker: None
Data & Control: Send
```

PICO 必须与电脑处于同一局域网。不要填写 PICO 自己的 IP、`127.0.0.1` 或 Docker 的
`172.x.x.x` 地址。

### 3. 启动容器与仿真（终端 1）

```bash
sudo docker start walker-c1-ubt-sim
xhost +SI:localuser:root
sudo docker exec -it \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e ROS_DOMAIN_ID=146 \
  walker-c1-ubt-sim bash
```

进入容器后：

```bash
cd /ubt_sim
bash scripts/start_c1_pick_place_sim.sh
```

等待终端出现 `RateLimiter sleep_duration`，并保持该终端运行。

### 4. Reset（终端 2）

```bash
sudo docker exec -it -e ROS_DOMAIN_ID=146 walker-c1-ubt-sim bash
```

两个容器 shell 保持打开即可；后续命令都在容器内执行，不要为每一步重新运行 `docker exec`。

从这里开始，所有 `/opt/ros/...`、`/opt/ubt_sim/...`、`ros2`、`reset.py` 和 teleop 命令都必须
在容器 shell 内运行。在宿主机执行会出现 `setup.bash: 没有那个文件` 和消息类型
`mc_state_msgs/msg/RobotState is invalid`；这不代表仿真损坏。

进入容器后：

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
cd /ubt_sim/teleoperation/control/walker_c1
ROS_DOMAIN_ID=146 /usr/bin/python3 reset.py --mode task
```

必须使用 `/usr/bin/python3`，不要使用 Isaac 的 Python 3.11。

### 5. 启动遥操作

在终端 2 的同一容器 shell 中运行。

只遥操作，不保存数据：

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command
```

遥操作并保存多条数据：

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command --record
```

把仿真画面投到 PICO 内，并用手柄完成采集（新方式）：

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command --headset-mode
```

`--headset-mode` 会自动启用录制；默认旧命令不启动视频、不读取 Grip，也不新增 FFmpeg 依赖。
新模式启动后，在 PICO 的 `Remote Vision` 中选择 `ZEDMINI`，点 `Listen`，输入与 PC Service
相同的电脑 IP（当前为 `192.168.1.4`），再点 `Confirm`。必须先启动上面的命令，再点
`Listen`。

在 PICO 的 `Tracking -> Data & Control` 中关闭 `Switch w/ A Button`，再手动点一次 `Send`。
否则 A 会被 XRoboToolkit 当作全局 Send 开关，造成数据流反复启停。

看到 `PICO teleop mode=sim, COMMAND` 后即可操作：

- 所有模式均按住右手柄 A 控制，松开 A 暂停。
- B 不参与机器人控制；头显模式下 B 只留给 XRoboToolkit 切换画面。
- 右扳机：控制右手开合。
- 启用录制时：松开 A 和 Grip，再单独长按右 Grip 1 秒，保存本条并 reset；电脑空格也可以。
- 左摇杆按下锁定急停；右摇杆按下解除急停。
- `Ctrl+C`：关闭遥操作。

XRoboToolkit 的 Remote Vision 界面自身也使用 B 在平面/双目显示间切换，因此视频窗口打开时，
所有模式统一改用 A 控制机器人，避免 B 同时改变画面并污染同一条采集数据。

## PICO 裸手 tracking MVP（不控制机器人）

裸手 receiver 与 controller teleoperation 相互独立。它只读取和显示 OpenXR 26-joint skeleton，
不会发布 `/mc/sdk/robot_command` 或左右手命令。PICO 系统设置中先启用 Hand Tracking，然后在
XRoboToolkit 中设置：

```text
Hand:           On
Data & Control: Send
Controller:     On 或 Off 均可
```

在已有 PICO SDK runtime 的容器 shell 中运行紧凑调试输出（默认 8 Hz）：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
./run_pico_hand_tracking.sh
```

程序为左右手打印 wrist pose、五个 fingertip、食指 flexion 和 thumb-index 距离。查看全部 26 个
joint 的 name、position 和 quaternion：

```bash
./run_pico_hand_tracking.sh --all-joints --print-hz 5
```

约定固定为：

- PICO/OpenXR 输入：右手系 `+X 右、+Y 上、-Z 前`；
- 输出：保留 OpenXR tracking origin，换成机器人轴 `+X 前、+Y 左、+Z 上`；
- position 单位：meter；
- pose 数组/日志 quaternion：`xyzw`，即 `qx,qy,qz,qw`；
- joint 0 是 `palm`，joint 1 才是 `wrist`。

可选发布 wrist 和五个 fingertip 的 ROS `visualization_msgs/MarkerArray`：

```bash
./run_pico_hand_tracking.sh --ros-markers
ROS_DOMAIN_ID=146 ros2 topic echo /pico/hand_tracking/markers --once
```

marker frame 默认为 `pico_tracking_robot_axes`；它只说明轴方向，origin 仍是 PICO tracking origin，
尚未声明为机器人 `base_link`。可在 RViz 中把 fixed frame 设为同名以查看 spheres。当前没有为了
debug marker 修改 Isaac 控制器；在没有明确外参前，不应把 tracking origin 冒充机器人 base。

现场检查顺序：

1. 两手伸入头显视野，确认 `is_tracked=True` 且左右各 26 点；
2. 手向前/右/上移动，确认输出分别沿机器人前/对应右/上方向；
3. 弯食指，确认 `index_tip` 和 `index flexion` 明显变化；
4. thumb-index pinch，确认两指距离明显减小；
5. 手移出视野，确认 `is_tracked=False`，marker 被删除；
6. 退出 debug 后按原命令启动 controller teleoperation，行为应与之前一致。

当前 `xrobottoolkit_sdk 1.0.2` 只有整手 `isActive`，没有把 PICO 已发送的逐关节
`locationStatus/radius` 暴露给 Python。MVP 不伪造逐关节 confidence；详情和最小 binding 升级方案
见根目录 `PICO_HANDOFF.md` 第 19 节。

## 无 PICO 的 26-joint → C1 11DoF 离线验证

这部分读取和控制的主资产是：

```text
assets/robots/walker_c1/Collected_walker_c1_v1_sensorKpkd/
  Collected_walker_astron_v1_sensorKpkd/walker_astron_v1_sensorKpkd_hands.usd
```

不是把 URDF 当作运行时真值。`inspect_c1_hand_usd.py` 用 `pxr.Usd` 直接检查主 USD，再用 AST
读取 `devices/walker_c1/config.py` 中 `ActionsCfg` 使用的 hand joint list。当前核对结果为每侧 11 个
独立、启用、带 angular drive 的 `PhysicsRevoluteJoint`：

| order | semantic | left joint / range(rad) | right joint / range(rad) | USD kinematic role |
|---:|---|---|---|---|
| 0 | thumb_cmp | `L_thumb_cmp_joint` `[-0.96,0]` | `R_thumb_cmp_joint` `[0,0.96]` | palm→thumb base 镜像摆动/对掌 |
| 1 | thumb_mpp | `L_thumb_mpp_joint` `[0,1.04]` | `R_thumb_mpp_joint` `[0,1.04]` | thumb 近端屈曲 |
| 2 | thumb_ip | `L_thumb_ip_joint` `[0,1.05]` | `R_thumb_ip_joint` `[0,1.05]` | thumb 远端屈曲 |
| 3–4 | index mpp/ip | `L_index_*` `[0,1.46]/[0,1.62]` | `R_index_*` 同范围 | 食指近端 + 压缩后的远端屈曲 |
| 5–6 | middle mpp/ip | `L_middle_*` `[0,1.46]/[0,1.62]` | `R_middle_*` 同范围 | 中指近端 + 压缩后的远端屈曲 |
| 7–8 | ring mpp/ip | `L_ring_*` `[0,1.46]/[0,1.62]` | `R_ring_*` 同范围 | 无名指近端 + 压缩后的远端屈曲 |
| 9–10 | little mpp/ip | `L_little_*` `[0,1.46]/[0,1.62]` | `R_little_*` 同范围 | 小指近端 + 压缩后的远端屈曲 |

USD 的 `physxJoint:maxJointVelocity` 为 `119.748 deg/s = 2.09 rad/s`；映射采用这个主资产上限，
即使 Isaac actuator config 的 `velocity_limit_sim=3` 更宽松也不放大命令速度。可在容器内复核：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
/isaac-sim/python.sh -B inspect_c1_hand_usd.py
```

所有可调参数都在 `pico_hand_retarget_config.json`：joint sign、scale、offset、range、q limits、
velocity limit、smoothing、flexion sensitivity、丢失跟踪策略和 thumb opposition 阈值。几何映射使用
MCP/PIP/DIP 方向夹角总和、metacarpal→tip reach ratio、palm width 和 thumb-index tip 距离；先归一化，
再把人的多个关节压缩到机器人的 mpp/ip，不逐角度硬复制。

不连接 PICO，先打印六个 mock 姿态的完整 11D 目标：

```bash
./run_pico_hand_offline_demo.sh --dry-run --side both --cycles 1
```

在正常 C1 仿真已经按前文启动、ROS bridge 正常运行后，另一个容器 shell 中执行：

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
cd /ubt_sim/teleoperation/control/walker_c1
ROS_DOMAIN_ID=146 ./run_pico_hand_offline_demo.sh --side both
```

默认无限循环：

```text
open → fist → open → thumb-index pinch → tripod
→ open → ring flex → open → little flex → open → cylindrical grasp
```

其中 `ring_flex` 只驱动无名指的 `mpp/ip`，`little_flex` 只驱动小指的 `mpp/ip`，用于明确验证
主 USD 中这两根手指及其控制通道彼此独立；动作之间插入 open，方便观察平滑过渡后的单指结果。

默认 transport 是 ROS，发布完整 11-name `JointCommand`，由现有 bridge 转成 full 11DoF dict；不要在
controller teleoperation 或其他 hand command publisher 运行时同时启动。只有使用
`UBT_SIM_NO_BRIDGE=1` 启动仿真时才可以显式用 `--transport zmq` 直连 5655。`Ctrl+C` 后仿真保持最后
一个已限速目标；再次运行 open 或执行 task reset 可张开。

正常容器已有依赖，不需要 PICO SDK，也不新增神经网络依赖。只有在容器外单独运行 USD audit 时
才需要可选的 `usd-core`；实际 retarget、mock 和 dry-run 只依赖 NumPy，仿真播放复用现有 ROS 消息。

## 真实 PICO 裸手 → C1 仿真 teleop

先停止 `pico_hand_debug.py`、offline demo、controller teleoperation 和其他 hand/body command publisher，
避免多个进程同时读取 SDK 或向同一关节发布命令。默认仍是只控制灵巧手手指的低风险模式。

### 只控制手指（默认）

默认 preview 直接读取真实 PICO 双手并实时计算 11D，但不导入 ROS、不发布命令：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
./run_pico_hand_live.sh --mode preview --side both
```

要直接控制 Isaac Sim，先确认 C1 仿真和现有 ROS bridge 正常运行。建议第一次只试右手：

```bash
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --mode sim --side right --enable-command
```

程序启动为 `DISARMED`。保持真实手在视野中并先张开，然后使用终端按键：

- `a`：请求解锁；选定手连续有效 0.5 秒后才开始发布；
- `d`：立即撤销解锁并停止发布；
- `q` 或 `Ctrl+C`：退出。

追踪短暂无效时立即停止发布；持续 0.30 秒后自动退回 `DISARMED`，恢复 tracking 也不会自动重新
解锁。第一次发布从 open command 按主 USD 的 2.09 rad/s 限速进入当前姿态。右手确认后测试双手：

```bash
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --mode sim --side both --enable-command
```

`sim` 同时要求 `ROS_DOMAIN_ID=146`、`--enable-command` 和交互式终端；代码没有 `real` 模式，不能
通过这个入口向真机发布。

### 右手腕＋右臂＋右手 11DoF（可选）

联合模式只读取右手 skeleton 和头显，不读取或要求 controller。它只发布右臂 7DoF 与右手 11DoF；
不会发布左手、左臂、头、腰或腿命令。第一版故意拒绝 `--side left/both`，这样左手可继续操作键盘。
以下命令必须在刚才能正常运行手指 teleop 的同一个 C1 容器终端内执行。先把机器人移动到弯肘的
task pose：

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
cd /ubt_sim/teleoperation/control/walker_c1
ROS_DOMAIN_ID=146 /usr/bin/python3 reset.py --mode task
```

reset 完成并停止后，再启动唯一一个 teleop 进程：

```bash
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --control hand-arm --mode sim --side right --enable-command
```

程序必须看到持续的 `/mc/sdk/robot_state`、有效的头显/右手腕 tracking、右肘不接近伸直奇异位形，
并检测到右手基本张开，才接受键盘 `a`。按 `a` 后再连续有效 0.5 秒，捕获两组相对锚点：当前真实
右手腕 pose 和当前机器人右掌 FK pose。因此启动瞬间不会把 PICO tracking origin 当作机器人绝对目标，
也不会因手腕放置位置不同而直接跳到某个绝对位置。按 `d` 立即停发，空格复位场景和机器人，
`q`/`Ctrl+C` 退出。

联合模式下按一次空格会按固定顺序执行：立即 `DISARMED` → 向 `/sim/cmd_reset` 发送一次场景复位
（苹果等场景状态回到初始状态）→ 复用 `reset.py --mode task` 的 arm-clear、elbow-clear、task-reset
三阶段运动 → 清除旧 wrist anchor 和旧 RobotState。复位完成后不会自动恢复控制；等日志显示
`RESET COMPLETE`，张开右手并重新按 `a`。

启动时建议面向机器人前方，右肘自然弯曲，右手腕放在舒适活动范围的中间并保持静止，手指张开。
手腕位置/旋转都以这一瞬间为零点；姿态放歪不会在启动瞬间造成绝对跳变，但之后的相对旋转范围会
以这个歪姿态为基准，容易更早触发工作空间或 IK 拒绝。头显朝向决定操作者的“前/右”水平坐标，
所以按键时不要低头或转身。闭拳会被 open-hand gate 拒绝，右肘过直也会被拒绝。

首次只做小幅逐轴验证：真实手向前、向右、向上时，机器人右掌应分别向机器人前、右、上移动；再
小幅验证 wrist roll/pitch/yaw，最后才弯手指。任一 tracking、RobotState 或 IK 帧无效时，臂和手在
该帧同时停止发布；连续失效 0.30 秒后自动退回 `DISARMED`，恢复后必须再次按 `a`。

当前启动方式是键盘：`a` 启动、`d` 停发、空格复位场景＋机器人、`q` 退出。因为它与右手动作
解耦且最容易明确急停。若之后不想碰键盘，优先增加一个
USB 脚踏/HID deadman；也可做左手特定手势或语音，但左手在键盘附近时手势容易误触，语音也不适合
承担唯一的安全 deadman。不要用右手 pinch 作为启动手势，它会与灵巧手控制目标冲突。

#### 苹果抓取现场经验

真实 PICO 已完成右手腕＋右臂＋11DoF 联合控制和苹果抬起验证，操作者确认手臂平移/旋转整体跟随
正常；当前抓取仍然比较难，属于后续 mapping 调参项。不要用完全闭拳抓取：现场日志显示完全闭拳时
middle/ring 等关节会达到主 USD 的 `1.46/1.62 rad` 上限，而已验证的自动苹果抓取闭合量约为
`0.70–0.85 rad`。过度闭合既可能挤出苹果，也会让 PICO 裸手因手指自遮挡而把整手 `isActive` 置为
invalid。

建议使用半闭合 C 形手势，让 normalized flexion 大致保持 `0.4–0.7`、机器人主要手指关节约
`0.7–0.9 rad`，不要长时间把五指 flexion 压到 `1.0` 或把真实 thumb-index 距离缩到几毫米。抓取
调试时可把短暂 tracking-loss 容忍从默认 0.30 秒提高到 2.5 秒：

```bash
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --control hand-arm --mode sim --side right --enable-command \
  --tracking-timeout-s 2.5
```

invalid 期间不会生成新 wrist/hand 目标，仿真保持最后一次安全命令；2.5 秒内 tracking 恢复可继续
当前锚点，超时仍会安全退回 `DISARMED`，且不会自动重新解锁。这个参数只缓冲短暂自遮挡，不能让
无效 skeleton 继续控制机械臂，也不应无限增大。

## 原 controller 遥操作的安全与模式边界

- 当前任务只控制右臂和右手；头部、左臂和左手始终使用 task reset 标准姿态。
- 默认 `--mode preview` 不发送机器人动作；仿真命令必须使用 `--mode sim --enable-command` 和
  `ROS_DOMAIN_ID=146`。
- 所有模式的右 A 都是 deadman；左摇杆按下锁定急停，松开所有按钮后按右摇杆清除，
  但不会自动恢复运动。
- PICO 位姿非法、机器人状态不完整、IK 残差/跳变超限或右肘接近伸直奇异位形时，拒绝危险帧
  并保持上一目标。
- `--source mock` 禁止用于真机。真机使用 `ROS_DOMAIN_ID=0`，还必须显式增加
  `--confirm-real-robot` 并先完成仓库根目录 `C1_REAL_ROBOT_READONLY_CHECK.md`；当前流程只验收
  仿真，不要直接切到真机。

## 采集多条数据

使用带 `--record` 的命令启动后，每条任务按以下循环操作：

1. 按住右 A 完成任务，过程中松开 A 即可暂停。
2. 完成后松开 A 和 Grip。可以在 Isaac Sim 主窗口按一次空格，也可以长按右 Grip 1 秒。
3. 程序保存 HDF5，并自动执行场景 reset 和 task reset。
4. 等待日志出现 `ready for next episode`，松开 Grip，再按住 A 开始下一条。

空格只按一次；保存和 reset 期间重复按键会被忽略。`R` 只重置仿真场景，不保存数据；按
`R` 后还应重新运行 `reset.py --mode task`。

数据默认保存到：

```text
/ubt_sim/dataset/walker_c1_pico/<时间戳>/trajectory.hdf5
```

查看已经保存的条数：

```bash
find /ubt_sim/dataset/walker_c1_pico -name trajectory.hdf5 -type f | sort
```

HDF5 中 `record_hz` 是相机时间戳测得的实际采样率，`requested_record_hz` 是采样上限。CPU
仿真的实际相机速率可能明显低于 30 Hz，转换训练数据时应使用实际速率。

## 常见问题

### PICO 显示 Connection Error

先停止遥操作，再重启 PC Service：

```bash
curl -fsS -X POST http://127.0.0.1:9999/api/control/robotics_service_stop
curl -fsS -X POST http://127.0.0.1:9999/api/control/robotics_service_start
curl -fsS http://127.0.0.1:9999/api/control/services_status
```

然后在 PICO 端依次点 `Reconnect` 和 `Send`。

### 按 B 没反应

在容器中检查：

```bash
pgrep -af pico_teleop.py
ROS_DOMAIN_ID=146 ros2 topic hz /mc/sdk/robot_state
```

只应有一个 `pico_teleop.py`。机器人状态应持续输出；日志若显示 `PICO data stale`，需要在
PICO 端重新 `Send`。

### 仿真窗口开了但机器人不动

确认启动命令包含 `--mode sim --enable-command`，并且已经执行 task reset。若
`/mc/sdk/robot_state` 没有数据，应彻底停止旧仿真和 bridge 后只重启一套。

### PICO Remote Vision 没画面

确认新模式日志出现 `PICO Remote Vision ready on 0.0.0.0:13579`，PICO 与电脑同一局域网，
并且 `Remote Vision` 填的是电脑 IP。容器内检查：

```bash
command -v ffmpeg
ffmpeg -hide_banner -encoders 2>/dev/null | grep libx264
ROS_DOMAIN_ID=146 ros2 topic hz /sensor/camera/head/color/raw
```

启动失败会直接说明是 13579 端口占用、FFmpeg/libx264 缺失，还是相机 topic 无数据；不会静默
退回旧模式。

## 首次部署才需要

### 头显画面模式：安装 FFmpeg

只有 `--headset-mode` 需要 FFmpeg。若启动时报：

```text
RuntimeError: --headset-mode requires ffmpeg in PATH
```

请在容器内以 `root` 执行；不要在宿主机安装，也不需要 `sudo`：

```bash
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y ffmpeg
ffmpeg -hide_banner -encoders 2>/dev/null | grep ' libx264 '
```

看到 `libx264` 后重新启动：

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command --headset-mode
```

安装保存在当前容器中，只需执行一次；默认旧遥操作模式不需要 FFmpeg。

### PICO SDK runtime

XRoboToolkit 必须使用 CPython 3.10。若容器中还没有 `/opt/pico-runtime`，先把厂家 wheel 和
`libPXREARobotSDK.so` 放入 `/opt/pico-sdk`，然后运行：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
./prepare_pico_runtime.sh /opt/pico-sdk /opt/pico-runtime
```

日常启动不需要重复执行。Python 模块名是 `xrobottoolkit_sdk`，`robot` 后只有一个 `t`。

## 架构

```text
PICO Head + Controller
  -> PC 192.168.1.4:4000
  -> Thinker Studio RoboticsServiceProcess
  -> 127.0.0.1:60061 gRPC
  -> xrobottoolkit_sdk
  -> pico_teleop.py（坐标映射、右臂 IK、A 键保护）
  -> ROS_DOMAIN_ID=146
  -> Walker C1 ROS2-ZMQ bridge
  -> Isaac Sim

Isaac 头部 RGB
  -> pico_headset_view.py
  -> FFmpeg H.264
  -> PC:13579 控制连接 + PICO 回连视频端口
  -> PICO Remote Vision（与 PC Service 独立）

Isaac 头部 RGB + 机器人状态 + 遥操作命令
  -> pico_episode_recorder.py
  -> trajectory.hdf5

Isaac Space / 右 Grip 长按
  -> 保存当前 episode
  -> 场景 reset
  -> task reset
  -> 等待下一次右 A
```

历史排查、实现细节和跨会话继续点见仓库根目录的
[PICO_HANDOFF.md](../../../../PICO_HANDOFF.md)。
