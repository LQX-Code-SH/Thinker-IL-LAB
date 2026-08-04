# Walker C1 真机 ROS 只读检查

本文档用于在另一台 Ubuntu 24.04 笔记本上，通过 ROS 2 查看 Walker C1
真机的 topic、消息类型、QoS、关节名和关节反馈。

整个流程只进行查询和订阅，不发布控制消息，不会主动控制机器人运动。

## 0. 安全边界

本轮检查可以执行：

- `ping`
- `ssh`
- `docker ps`
- `ros2 node list`
- `ros2 topic list`
- `ros2 topic type`
- `ros2 topic info`
- `ros2 topic echo`
- `ros2 topic hz`
- `ros2 interface show`

本轮检查不要执行：

- `ros2 topic pub`
- 任何 `/mc/sdk/robot_command` 发布命令
- 任何 `/mc/{left,right}_hand/command` 发布命令
- `ros2 service call /sys/task/developer_mode ...`
- `rosa run ... switch_controller ...`
- `reset.py`
- `joint_test.py --move`、`--shift`、`--hand-move` 等运动参数
- `rollout_walker_c1.sh`

检查期间仍应保证机器人周围无人、遥控器和急停可用。

## 1. 连接关系

项目代码和 ROS 2 Humble Docker 运行在笔记本上，不需要把整个仓库复制到真机：

```text
Ubuntu 24.04 笔记本
  └─ ROS 2 Humble Docker + 本项目消息包
                    │
                    │ 有线网 / ROS 2 DDS
                    ▼
Walker C1 真机
  └─ 厂家已有的运控、ROS 和 SDK
```

SSH 只用于检查真机内部服务。若真机 ROS 服务已经正常运行，可以不 SSH，直接从笔记本订阅。

## 2. 笔记本检查有线网络

当前仓库的 Fast DDS 配置使用笔记本地址 `192.168.11.3`，真机地址暂按
`192.168.11.2` 检查。如果现场 C1 使用其他网段，以现场实际地址为准，并同步修改
`ubt_IL/docker/fastdds_no_shm.xml`。

在笔记本宿主机执行：

```bash
ip -br link
ip -br addr
```

确认有线网卡具有下面的地址：

```text
192.168.11.3/24
```

测试真机网络：

```bash
ping -c 3 192.168.11.2
```

预期结果：收到 3 个回复，并且没有 `Destination Host Unreachable`。

如果 ping 不通，先不要继续 ROS 检查。检查网线、网卡 IPv4、真机地址和是否存在 IP 冲突。

## 3. 可选：SSH 进入真机做内部只读检查

如果已经知道真机 SSH 用户和地址：

```bash
ssh walker@192.168.11.2
```

进入后先查看容器，不启动、不停止、不切换任何控制器：

```bash
hostname
ip -br addr
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
```

如果已确认哪个容器是厂家运控/ROS 容器，可以进入它：

```bash
docker exec -it <真机运控容器名> bash
```

在真机容器内执行：

```bash
source /opt/walker/setup.bash
printenv ROS_DOMAIN_ID
ros2 node list
ros2 topic list | sort
ros2 topic info /mc/sdk/robot_state -v
```

如需查看一帧真机内部状态：

```bash
timeout 10 ros2 topic echo --once --no-daemon \
  /mc/sdk/robot_state mc_state_msgs/msg/RobotState \
  --qos-reliability best_effort \
  --qos-durability volatile
```

退出真机容器和 SSH：

```bash
exit
exit
```

如果真机内部能收到 `/mc/sdk/robot_state`，而笔记本收不到，问题通常在笔记本
网卡、DDS、Domain ID、Fast DDS 网卡白名单或防火墙。

## 4. 启动笔记本 ROS 2 Humble 容器

以下命令在笔记本仓库中执行。假设仓库目录名为 `walker_c1`：

```bash
cd ~/walker_c1/ubt_IL/docker
```

如果容器尚未创建：

```bash
DOCKER_GPU_ARGS="" DOMAIN_ID=0 bash run.sh start
```

检查容器环境：

```bash
bash run.sh check
```

进入容器：

```bash
DOMAIN_ID=0 bash run.sh bash
```

后续第 5～10 节命令均在这个笔记本容器内执行。

## 5. 检查 ROS 和 DDS 环境

```bash
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
```

预期至少包含：

```text
ROS_DISTRO=humble
ROS_DOMAIN_ID=0
FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_no_shm.xml
```

确认自定义消息可用：

```bash
ros2 interface show mc_state_msgs/msg/RobotState
ros2 interface show mc_task_msgs/msg/RobotCommand
ros2 interface show mc_task_msgs/msg/JointCommand
```

`RobotState` 应包含：

```text
sensor_msgs/JointState joint_states
```

清除的只是笔记本本地 ROS CLI daemon 缓存，不影响真机：

```bash
ros2 daemon stop
```

## 6. 查看真机 ROS 图

```bash
ros2 node list
ros2 topic list | sort
```

只筛选本次关心的 topic：

```bash
ros2 topic list | grep -E '^/mc/|^/sensor/|^/sys/'
```

至少重点寻找：

```text
/mc/sdk/robot_state
/mc/sdk/robot_command
/mc/left_hand/joint_states
/mc/right_hand/joint_states
/mc/left_hand/command
/mc/right_hand/command
```

注意：看到 command topic 不等于发送了命令；`topic list` 只是查看 DDS 图。

## 7. 检查身体状态 topic

查看类型、发布者和 QoS：

```bash
ros2 topic type /mc/sdk/robot_state
ros2 topic info /mc/sdk/robot_state -v
```

预期消息类型：

```text
mc_state_msgs/msg/RobotState
```

订阅一帧完整状态：

```bash
timeout 10 ros2 topic echo --once --no-daemon \
  /mc/sdk/robot_state mc_state_msgs/msg/RobotState \
  --qos-reliability best_effort \
  --qos-durability volatile
```

连续检查发布频率，看到稳定输出后按 `Ctrl+C`：

```bash
ros2 topic hz /mc/sdk/robot_state
```

重点记录完整输出中的：

```yaml
joint_states:
  name: [...]
  position: [...]
  velocity: [...]
  effort: [...]
```

## 8. 单独导出真机关节名

先尝试直接输出嵌套字段：

```bash
timeout 10 ros2 topic echo --once --no-daemon \
  --field joint_states.name \
  /mc/sdk/robot_state mc_state_msgs/msg/RobotState \
  --qos-reliability best_effort \
  --qos-durability volatile
```

如果当前 ROS CLI 不支持嵌套 `--field`，使用第 7 节的完整 echo 输出，复制
`joint_states.name` 数组即可。

必须重点核对左右臂第 4 个关节：

```text
L_elbow_pitch_joint  还是  L_elbow_roll_joint
R_elbow_pitch_joint  还是  R_elbow_roll_joint
```

还要逐项核对：

- 左臂 7 个关节名
- 右臂 7 个关节名
- 头部关节名
- 腰部关节名
- 数组 `name` 和 `position` 的长度是否一致

当前项目的 C1 期望关节名记录在仓库根目录 `C1_joint_map.md`，真机反馈是最终依据。

## 9. 检查左右手状态

检查左手：

```bash
ros2 topic type /mc/left_hand/joint_states
ros2 topic info /mc/left_hand/joint_states -v

timeout 10 ros2 topic echo --once --no-daemon \
  /mc/left_hand/joint_states sensor_msgs/msg/JointState \
  --qos-reliability best_effort \
  --qos-durability volatile
```

检查右手：

```bash
ros2 topic type /mc/right_hand/joint_states
ros2 topic info /mc/right_hand/joint_states -v

timeout 10 ros2 topic echo --once --no-daemon \
  /mc/right_hand/joint_states sensor_msgs/msg/JointState \
  --qos-reliability best_effort \
  --qos-durability volatile
```

重点记录：

- 左右手实际消息类型
- 左右手 `name` 数组
- 左右手 `position` 数组维度
- 真机使用 6D 逻辑手命名还是其他命名
- 发布者 QoS 是否为 `BEST_EFFORT`、`VOLATILE`

## 10. 只读检查命令通路是否存在

下面只查看 command topic 的订阅者，不发布消息：

```bash
ros2 topic type /mc/sdk/robot_command
ros2 topic info /mc/sdk/robot_command -v

ros2 topic type /mc/left_hand/command
ros2 topic info /mc/left_hand/command -v

ros2 topic type /mc/right_hand/command
ros2 topic info /mc/right_hand/command -v
```

记录每个 command topic：

- 消息类型是否与项目一致
- 是否存在真机侧 Subscription
- Subscription 的节点名
- QoS 是否与项目发布端兼容

不要为了测试 command topic 而运行 `ros2 topic pub`。

## 11. 可选：只读查看开发者模式状态

先确认状态 topic 是否存在及其类型：

```bash
ros2 topic list | grep walker_mode
ros2 topic type /sys/state/walker_mode
ros2 topic info /sys/state/walker_mode -v
```

若存在，读取一次：

```bash
timeout 10 ros2 topic echo --once /sys/state/walker_mode
```

本轮不要调用 `/sys/task/developer_mode` 服务，也不要切换控制器。

## 12. 可选：只读检查相机 topic

先从真机实际 ROS 图中寻找相机：

```bash
ros2 topic list | grep -E 'camera|image|raw'
```

对找到的每个候选 topic 查询实际类型：

```bash
ros2 topic type <相机topic>
ros2 topic info <相机topic> -v
```

不要先假设相机一定是 `sensor_msgs/msg/Image`。真机也可能使用
`shm_msgs/msg/Image1m`、`shm_msgs/msg/Image2m` 等固定缓冲区消息。

如果只想确认是否持续发布，可以运行：

```bash
ros2 topic hz <相机topic>
```

看到稳定频率后按 `Ctrl+C`。

## 13. 结果判断

### 情况 A：`ping` 不通

优先检查：

- 网线和网口
- 笔记本是否为 `192.168.11.3/24`
- 真机实际 IP
- IP 冲突

### 情况 B：`ping` 通，但 `ros2 topic list` 看不到真机 topic

优先检查：

- 笔记本和真机 `ROS_DOMAIN_ID` 是否一致
- `/opt/fastdds_no_shm.xml` 是否包含笔记本实际网卡 IP
- 容器是否使用 host network
- 防火墙是否阻止 DDS UDP/组播
- 真机 ROS/运控容器是否运行

### 情况 C：能看到 topic，但 `topic echo` 超时

优先检查：

- 是否显式使用了 `--qos-reliability best_effort`
- 消息类型是否一致
- 自定义消息包是否成功编译和 source
- `ros2 topic info <topic> -v` 中发布端 QoS

### 情况 D：身体有状态，手没有状态

优先检查：

- 真机手部控制器/状态节点是否运行
- 手部 topic 名是否与项目假设一致
- 手部实际消息类型
- 手部状态 QoS

### 情况 E：状态正常，但关节名与配置不一致

先保存真机完整 `joint_states.name`，不要发送运动命令。应先修改 C1
真机配置和名字映射，再编写单关节、小幅、可立即停止的运动验收流程。

## 14. 建议保存的检查结果

完成后至少保存以下输出：

```text
笔记本 ip -br addr
ping 结果
ROS_DOMAIN_ID
ros2 node list
ros2 topic list
ros2 topic info /mc/sdk/robot_state -v
/mc/sdk/robot_state 的一帧完整消息
左右手 joint_states 的一帧完整消息
身体和双手 command topic 的 topic info -v
相机 topic 名、消息类型和 topic info -v
```

拿到这些结果后，才能可靠判断当前代码中的 topic、消息类型、QoS、关节名和维度是否适配 C1 真机。
