# Walker C1 PICO 遥操作与数据采集

本文只保留日常使用步骤。默认方式使用 PICO 头显和右手柄控制 Walker C1 仿真的右臂与右手；
左臂、左手、头部和下半身保持 task reset 姿态。

当前流程只用于仿真：`ROS_DOMAIN_ID=146`。不要把本文命令改成 Domain 0 后直接连接真机。

## 一、启动 PICO 与仿真

### 1. 启动电脑端 PICO 服务

在宿主机执行：

```bash
sudo /usr/bin/systemctl start thinker-studio.service
curl -fsS -X POST http://127.0.0.1:9999/api/control/robotics_service_start
```

### 2. 设置 PICO

PICO 与电脑必须在同一局域网。XRoboToolkit 使用以下设置：

```text
PC Service:     电脑的局域网 IP（当前为 192.168.1.4）
Head:           On
Controller:     On
Hand:           Off（使用可选裸手模式时再打开）
Motion Tracker: None
Switch w/ A Button: Off
Data & Control: Send
```

`PC Service` 填电脑 IP，不要填 PICO 自己的 IP、`127.0.0.1` 或 Docker 的 `172.x.x.x` 地址。

### 3. 启动 C1 仿真（终端 1）

在宿主机执行：

```bash
sudo docker start walker-c1-ubt-sim
xhost +SI:localuser:root
sudo docker exec -it \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e ROS_DOMAIN_ID=146 \
  walker-c1-ubt-sim bash
```

进入容器后执行，并保持该终端运行：

```bash
cd /ubt_sim
bash scripts/start_c1_pick_place_sim.sh
```

### 4. Reset（终端 2）

在宿主机另开一个终端：

```bash
sudo docker exec -it -e ROS_DOMAIN_ID=146 walker-c1-ubt-sim bash
```

后续 reset 和遥操作命令都在这个容器 shell 中运行：

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
cd /ubt_sim/teleoperation/control/walker_c1
ROS_DOMAIN_ID=146 /usr/bin/python3 reset.py --mode task
```

必须使用容器内的 `/usr/bin/python3`，不要使用 Isaac Python 3.11。

## 二、选择一种手柄遥操作方式

以下两种方式控制逻辑相同，只选择一种运行。启动前应停止其他 PICO teleop 进程。

### 方式 A：不投影到头显

只遥操作，不采集数据：

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command
```

遥操作并以 30 Hz 采集数据：

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command --record
```

操作者直接查看电脑上的 Isaac Sim 画面。

### 方式 B：把仿真画面投影到头显

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command --headset-mode
```

`--headset-mode` 会自动启用数据采集。命令启动后，在 PICO 的 `Remote Vision` 中执行：

1. 选择 `ZEDMINI`；
2. 点击 `Listen`；
3. 输入与 `PC Service` 相同的电脑 IP；
4. 点击 `Confirm`。

必须先启动遥操作命令，再在 PICO 中点击 `Listen`。右手柄 B 仅用于 Remote Vision 的画面切换，
不控制机器人。

如果首次启动提示缺少 FFmpeg，在容器内安装一次即可：

```bash
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y ffmpeg
```

## 三、手柄操作与采集

- 按住右手柄 A：控制机器人；松开 A：暂停。
- 右扳机：控制右手开合。
- 左摇杆按下：锁定急停；松开所有按键后按右摇杆：解除急停。
- `Ctrl+C`：退出遥操作。

启用录制后，每条数据按以下顺序操作：

1. 按住右 A 完成任务，松开 A 可以暂停。
2. 完成后松开 A 和 Grip。
3. 单独长按右 Grip 1 秒；也可以在电脑端按一次空格。
4. 等待程序保存 HDF5，并自动完成场景 reset 和 task reset。
5. 日志显示可以开始下一条后，再按住右 A。

数据默认保存到：

```text
/ubt_sim/dataset/walker_c1_pico/<时间戳>/trajectory.hdf5
```

保存/reset 期间不要重复按空格或 Grip。两种手柄遥操作方式都以 30 Hz 生成相同的
26D 数据格式。PICO 的读取、控制与数据采集频率均为 30 Hz。

## 四、可选：PICO 裸手控制

裸手模式不使用手柄 pose。使用前停止 `pico_teleop.py` 和其他 PICO/hand command 进程，然后在
PICO 系统中启用 Hand Tracking，并把 XRoboToolkit 设置为：

```text
Head:           On
Hand:           On
Controller:     On 或 Off
Data & Control: Send
```

### 1. 只查看裸手 tracking，不控制机器人

```bash
cd /ubt_sim/teleoperation/control/walker_c1
./run_pico_hand_tracking.sh
```

该命令只显示左右手 tracking 状态、wrist、指尖和手指弯曲信息，不发布机器人命令。

### 2. 裸手控制右臂和 11DoF 右手

先按第一部分完成仿真启动和 task reset，然后运行：

```bash
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --control hand-arm --mode sim --side right --enable-command
```

需要采集裸手示教时增加 `--record`：

```bash
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --control hand-arm --mode sim --side right --enable-command --record
```

裸手模式使用终端键盘：

- `A`：请求启动；右手张开并连续 tracking 有效 0.5 秒后开始控制和采集。
- `D`：暂停控制，但不结束当前 episode。
- 空格：保存当前 episode，并自动执行场景 reset 和 task reset。
- `Q` 或 `Ctrl+C`：退出。

裸手数据保存到：

```text
/ubt_sim/dataset/walker_c1_pico_hand/<时间戳>/trajectory.hdf5
```

裸手数据是双臂 14D 加双手完整 22D，共 36D。转换 LeRobot 时使用：

```text
ubt_IL/scripts/convert/configs/Walker_C1_36_1RGB.json
```

不要把裸手 36D 数据和手柄 26D 数据放入同一个数据目录。

## 五、常见问题

### PICO 显示 Connection Error

停止遥操作，在宿主机重启 PICO 服务，然后在 PICO 中重新点击 `Reconnect` 和 `Send`：

```bash
curl -fsS -X POST http://127.0.0.1:9999/api/control/robotics_service_stop
curl -fsS -X POST http://127.0.0.1:9999/api/control/robotics_service_start
```

### 仿真窗口存在，但机器人不动

确认已经执行 task reset、PICO 显示 `Send`，并且只运行了一个遥操作进程。重新启动遥操作不会自动
代替 task reset。

### 头显没有画面

确认遥操作命令使用了 `--headset-mode`，并且先启动命令、后在 Remote Vision 中点击 `Listen`。
Remote Vision 和 `PC Service` 必须填写同一个电脑 IP。

## 六、首次部署 PICO SDK（已有环境无需执行）

如果容器中还没有 `/opt/pico-runtime`，将厂家 wheel 和 `libPXREARobotSDK.so` 放入
`/opt/pico-sdk`，然后在容器内执行：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
./prepare_pico_runtime.sh /opt/pico-sdk /opt/pico-runtime
```
