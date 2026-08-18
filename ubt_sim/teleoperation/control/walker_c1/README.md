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

## 安全与模式边界

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
