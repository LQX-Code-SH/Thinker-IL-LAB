# robot_control — Walker S2 EDU 探索者 真机 Python 控制包

从 [executor_node_sdk.py](../rosa_vla_additional/vla-motionx86/src/vla_executor/vla_executor/executor_node_sdk.py) 提取的纯机器人控制版本，**去除 VLA 推理依赖**，提供 Python API 直接控制真机。

控制方法：方法 2（SDK 控制器，`RobotCommand`/`JointCmd`，`MODE_POSITION=2`）。详见 [../../CLAUDE.md](../../CLAUDE.md) 中"两层控制"章节。

## 安装

本包作为 `robot_control` ament_python 包构建，需先编译消息包：

```bash
# 阶段 1：消息包
colcon build --packages-select \
  shm_msgs mc_state_msgs mc_task_msgs emb_task_msgs sys_task_msgs rosa_msgs
source install/setup.bash

# 阶段 2：robot_control 包
colcon build --packages-select robot_control
source install/setup.bash

# 验证
python3 -c "from robot_control import RobotController; print('OK')"
```

也可通过 `ros2 run` 调用：
```bash
ros2 run robot_control robot_control --print-state
ros2 run robot_control camera --print
ros2 run robot_control joint_test --print --joints R_elbow_yaw_joint
```

---

## 与 executor_node_sdk.py 的差异

| 特性 | executor_node_sdk.py | robot_control.py |
|---|---|---|
| 输入源 | 订阅 `/vla_inference_result`（VLA 推理） | Python API 调用 |
| 维度映射 | 20→17（丢弃 lifter） | 直接用配置维度 |
| 过渡段 | 自动 30 点过渡 | 由 API 决定（`move_to_position` 自动生成） |
| 安全检查 | 内置 | 保留（可关闭） |
| 关节锁定 | 硬编码 | 可配置（构造参数 + 运行时修改） |
| 关节范围裁剪 | 无 | 新增（默认开启） |
| 控制频率 | 200 Hz | 200 Hz（可配置） |
| RobotCommand 构造 | 跳过锁定关节 | 同 |
| 手指关节状态 | 无 | 新增（订阅 `/mc/{left,right}_hand/joint_states`） |
| 手指关节控制 | 仅周期运动 | 新增 `move_hand` / `shift_hand` / `send_hand_position` 等 |

---

## 前置条件

### 1. 启动运控 + 切换 SDK 控制器（Motion 板）

```bash
ssh walker@192.168.11.2
docker exec -it walker-motion.manipulation_robot_app-1 bash
source /opt/walker/setup.bash

rosa run t800_mc_server start_mc_client
rosa run rosa_controllers switch_controller config_mc_walker_s2_v1_sps
```

> ⚠️ 切换控制器前必须用遥控器把机器人移到安全位置。
>
> **手指关节不需要切换控制器**——手部控制器始终监听 `/mc/{left,right}_hand/command`。

### 2. 在控制容器中 source 环境

```bash
docker start vla_control_node_sdk
docker exec -it vla_control_node_sdk bash
source /home/ubt/additional/scripts/setup.sh
```

---

## 两层控制架构

本包同时支持两层控制通路，它们完全独立、可同时运行：

| | 身体关节 | 手指关节 |
|---|---|---|
| **关节** | 17 自由度（7 左臂 + 7 右臂 + 头 2 + 腰 1） | 14 自由度（7 左手 + 7 右手，V4 版本） |
| **消息** | `RobotCommand`（内含 `JointCmd[]`） | `JointCommand`（并行数组） |
| **命令话题** | `/mc/sdk/robot_command` | `/mc/{left,right}_hand/command` |
| **状态话题** | `/mc/sdk/robot_state` | `/mc/{left,right}_hand/joint_states` |
| **mode 值** | `MODE_POSITION = 2` | `5`（手部控制器自定义） |
| **需要 switch_controller** | ✅ 是 | ❌ 否（始终监听） |
| **关节锁定** | 受 `lock_joints` 控制 | 不受影响 |
| **安全速度检查** | 6.28 rad/s | 无 |
| **限位裁剪** | `BODY_JOINT_LIMITS` | `V4_HAND_JOINT_LIMITS` |

⚠️ **mode 值不可互混**：`JointCmd` 中 `MODE_POSITION=2`，`JointCommand` 中 position mode 对应 `5`。互相复制会静默出错。

---

## 命令行用法

### robot_control.py — 基础控制脚本

```bash
# 仅打印当前关节状态（不发送任何指令）
python3 robot_control.py --print-state

# 运行内置安全演示：右臂 elbow_yaw ±0.05 rad
python3 robot_control.py --demo

# 移动到 VLA 抓取任务预备位姿
python3 robot_control.py --vla-ready

# 头部周期 sin 运动测试
python3 robot_control.py --head-test                          # 默认振幅 0.5 rad, 周期 6.28s, 2 个循环
python3 robot_control.py --head-test --head-amplitude 0.3     # 自定义振幅
python3 robot_control.py --head-test --yaw-only               # 仅 head_yaw
python3 robot_control.py --head-test --pitch-only             # 仅 head_pitch
python3 robot_control.py --head-test --head-cycles 5          # 5 个循环

# V4 手部周期 sin 运动测试
python3 robot_control.py --hand-test                          # 默认振幅 0.6 rad, 双手
python3 robot_control.py --hand-test --hand-amplitude 0.4     # 自定义振幅
python3 robot_control.py --hand-test --left-only              # 仅左手
python3 robot_control.py --hand-test --right-only             # 仅右手
python3 robot_control.py --hand-test --hand-phase-diff 0      # 关闭波浪效果

# 保持节点运行，供外部 Python 调用
python3 robot_control.py --interactive

# 不锁定头/腰关节
python3 robot_control.py --print-state --no-lock

# 禁用安全速度检查（不推荐）
python3 robot_control.py --demo --no-safety
```

### joint_test.py — 交互式关节测试脚本

`joint_test.py` 继承 `RobotController`，提供**身体关节 + 手指关节**的统一命令行接口，适用于单关节调试、限位验证、零点标定等场景。

#### 身体关节

```bash
# 查询指定关节状态
python3 joint_test.py --joints R_elbow_yaw_joint L_shoulder_pitch_joint

# 移动关节到目标角度（rad）
python3 joint_test.py --move R_elbow_yaw_joint=0.5 --duration 2.0

# 多关节同时移动
python3 joint_test.py --move R_elbow_yaw_joint=0.5 L_shoulder_pitch_joint=-0.3

# 相对当前位置偏移
python3 joint_test.py --shift R_elbow_yaw_joint=+0.1

# 持续监控（10Hz 刷新）
python3 joint_test.py --monitor --joints head_pitch_joint head_yaw_joint

# 交互模式
python3 joint_test.py --interactive --joints R_elbow_yaw_joint
```

#### 手指关节

```bash
# 查询左手手指状态（用 --hand + --print）
python3 joint_test.py --hand left --print
python3 joint_test.py --hand both --print

# 也可以用手指关节全名直接传 --joints + --print
python3 joint_test.py --print --joints left_thumb_swing left_index_mcp

# 身体关节和手指关节混用
python3 joint_test.py --print --joints R_elbow_yaw_joint left_thumb_swing

# 移动单个手指关节（短名或全名，需配合 --hand 指定手别）
python3 joint_test.py --hand left --hand-move thumb_swing=0.5
python3 joint_test.py --hand right --hand-move right_index_mcp=0.8

# 手指关节相对偏移
python3 joint_test.py --hand right --hand-shift index_mcp=+0.2

# 设置整手姿态（7 个角度值，按 V4_HAND_*_JOINTS 顺序）
python3 joint_test.py --hand left --hand-pose 0.5 0.3 0.1 0.8 0.8 0.8 0.8

# 预设姿态：张开 / 握拳
python3 joint_test.py --hand both --hand-open
python3 joint_test.py --hand left --hand-close

# 双手周期波形运动
python3 joint_test.py --hand both --hand-wave

# 监控手指关节
python3 joint_test.py --monitor --joints left_thumb_swing left_index_mcp
```

#### 可用关节名

**身体关节（17 个）：**

```
L_elbow_roll_joint   L_elbow_yaw_joint   L_shoulder_pitch_joint
L_shoulder_roll_joint L_shoulder_yaw_joint L_wrist_pitch_joint
L_wrist_roll_joint   R_elbow_roll_joint   R_elbow_yaw_joint
R_shoulder_pitch_joint R_shoulder_roll_joint R_shoulder_yaw_joint
R_wrist_pitch_joint  R_wrist_roll_joint   head_pitch_joint
head_yaw_joint       waist_yaw_joint
```

**手指关节（V4，每手 7 个）：**

```
# 左手
left_thumb_swing  left_thumb_mcp  left_thumb_pip    # V4 独有
left_index_mcp    left_middle_mcp  left_ring_mcp  left_little_mcp

# 右手
right_thumb_swing  right_thumb_mcp  right_thumb_pip  # V4 独有
right_index_mcp    right_middle_mcp  right_ring_mcp  right_little_mcp
```

> 手指关节短名（去掉 `left_`/`right_` 前缀）可在 `--hand-move` / `--hand-shift` 中使用；`--joints` / `--move` / `--shift` 须用全名。

---

## Python API 用法

### 基本初始化

```python
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_control import RobotController

rclpy.init()
controller = RobotController(
    lock_joints=['head_pitch_joint', 'head_yaw_joint', 'waist_yaw_joint'],
    enable_safety_check=True,
    enable_limit_check=True,
)

executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(controller)
threading.Thread(target=executor.spin, daemon=True).start()

# 等待身体状态
controller.wait_for_state(timeout=5.0)
```

### 身体关节 — 平滑移动

```python
import numpy as np

# 获取当前位置（17 维 numpy 数组）
pos = controller.get_current_position()

# 让右臂的 elbow_yaw 增加 0.1 rad
target = pos.copy()
target[controller.joint_index('R_elbow_yaw_joint')] += 0.1

# 2 秒平滑过渡，阻塞等待完成
controller.move_to_position(target, duration_sec=2.0, wait=True)
```

### 身体关节 — 字典式移动

```python
# 只关心你要改的关节，其余保持当前位置
controller.move_to_pose(
    {
        'head_yaw_joint':         -0.65,
        'L_shoulder_pitch_joint': -0.6,
        'R_shoulder_pitch_joint': -0.6,
    },
    duration_sec=2.0,
    unlock_required_joints=True,  # 自动解锁头/腰等锁定关节
)
```

### 身体关节 — 执行预定义轨迹

```python
# N 个时间步 × 17 维关节位置
# 点间距 = 1/200Hz = 5ms，400 点 = 2 秒
trajectory = np.zeros((400, controller.n_joints))
for t in range(400):
    trajectory[t] = pos.copy()
    trajectory[t][controller.joint_index('R_elbow_yaw_joint')] += 0.1 * np.sin(t * 0.01)

controller.execute_trajectory(trajectory, wait=True)
```

### 身体关节 — 中断执行

```python
controller.move_to_position(target, duration_sec=10.0, wait=False)
time.sleep(2.0)
controller.stop()    # 立即停止，保持在最后一个发送的位置
```

### VLA 预备位姿

```python
# 双臂、头、腰同时移动到 VLA 预备姿态
controller.move_to_vla_ready_pose(duration_sec=1.5, wait=True)

# 完成后 head/waist 关节保持解锁，如需重新锁定：
from robot_control import DEFAULT_LOCK_JOINTS
controller.set_lock_joints(DEFAULT_LOCK_JOINTS)
```

⚠️ `move_to_vla_ready_pose` 仅复现 XML 阶段1（4 个 MetaMove 并行复位），不包含阶段2 的 `clamp_s2_joints_trajectory` 命名轨迹。

### 头部周期 sin 运动测试

```python
# 默认：振幅 0.5 rad, 周期 2π s, 2 个循环
controller.head_periodic_motion()

# 自定义
controller.head_periodic_motion(
    amplitude=0.3,
    period_sec=4.0,
    cycles=5,
    move_yaw=True,
    move_pitch=False,
    return_to_zero=True,
)
```

### 手指关节 — 状态读取

```python
# 等待手指状态
controller.wait_for_hand_state(timeout=5.0)

# 获取整手位置（7 维 numpy 数组）
left_pos = controller.get_hand_position('left')
right_pos = controller.get_hand_position('right')

# 获取单关节位置
thumb = controller.get_hand_joint_position('left', 'thumb_swing')  # 短名
thumb = controller.get_hand_joint_position('left', 'left_thumb_swing')  # 全名

# 查看关节名列表
controller.hand_joint_names('left')   # ['left_thumb_swing', ..., 'left_little_mcp']
```

### 手指关节 — 位置控制

```python
# 字典式移动（短名或全名均可）
controller.move_hand('left', {'thumb_swing': 0.5, 'index_mcp': 0.8}, duration_sec=2.0)

# 单关节移动
controller.move_hand_joint('right', 'index_mcp', 1.0, duration_sec=2.0)

# 相对偏移
controller.shift_hand('left', 'thumb_swing', +0.3, duration_sec=2.0)

# 直接发送 7 维位置（无插值，单次发送）
controller.send_hand_position('left', [0.5, 0.3, 0.1, 0.8, 0.8, 0.8, 0.8])
```

### 手指关节 — 预设姿态

```python
from robot_control import V4_HAND_OPEN_POSE, V4_HAND_CLOSE_POSE

# 张开（所有关节归零）
controller.move_hand('left', V4_HAND_OPEN_POSE, duration_sec=2.0)

# 握拳（所有关节到限位上限）
controller.move_hand('right', V4_HAND_CLOSE_POSE, duration_sec=2.0)
```

### 手指关节 — 周期波形运动

```python
# 默认：振幅 0.6 rad, 双手, 相邻手指相位差 0.2 rad
controller.hand_periodic_motion()

# 自定义
controller.hand_periodic_motion(
    amplitude=0.4,
    period_sec=4.0,
    cycles=3,
    phase_diff=0.0,       # 关闭波浪效果
    left_hand=True,
    right_hand=False,
    return_to_zero=True,
)
```

> ⚠️ V4 手 = 7 关节（含 `thumb_pip`），V3 手 = 6 关节。本包仅支持 V4 手。

### 清理

```python
controller.destroy_node()
rclpy.shutdown()
```

---

## API 参考

### `RobotController(...)`

| 参数 | 默认 | 含义 |
|---|---|---|
| `node_name` | `'robot_control_node'` | ROS2 节点名 |
| `command_topic` | `/mc/sdk/robot_command` | 身体控制指令发布话题 |
| `state_topic` | `/mc/sdk/robot_state` | 身体状态订阅话题 |
| `control_hz` | `200` | 控制频率（Hz） |
| `lock_joints` | `None` | 锁定关节名列表（构造参数为 None 时不锁定，命令行默认锁 head/waist） |
| `max_joint_speed` | `6.28` rad/s | 安全速度上限（仅身体关节） |
| `enable_safety_check` | `True` | 是否启用速度安全检查 |
| `enable_limit_check` | `True` | 是否裁剪到关节限位 |

### 身体关节方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `wait_for_state(timeout=5.0)` | `bool` | 阻塞等待第一个 RobotState 消息 |
| `get_current_position()` | `np.ndarray \| None` | 最新关节位置（17 维） |
| `joint_index(joint_name)` | `int` | 关节名→索引 |
| `joint_names` | `list[str]` | 所有身体关节名（只读属性） |
| `move_to_position(target, duration_sec, wait)` | `bool` | 线性插值到目标（17 维向量） |
| `move_to_pose(pose_dict, duration_sec, wait, unlock_required_joints)` | `bool` | 按字典移动（未指定关节保持当前位置） |
| `move_to_vla_ready_pose(duration_sec, wait)` | `bool` | 移动到 VLA 任务预备位姿 |
| `head_periodic_motion(amplitude, period_sec, cycles, ...)` | `bool` | 头部周期 sin 运动测试 |
| `execute_trajectory(trajectory, wait)` | `bool` | 执行预定义轨迹（N×17） |
| `stop()` | `None` | 立即停止发布 |
| `set_lock_joints(joint_names)` | `None` | 动态修改锁定关节 |
| `is_busy` | `bool` | 是否正在执行轨迹（只读属性） |

### 手指关节方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `wait_for_hand_state(side, timeout=5.0)` | `bool` | 阻塞等待手部状态（side=`None` 等待双手） |
| `get_hand_position(side)` | `np.ndarray \| None` | 整手位置（7 维），side="left"/"right" |
| `get_hand_joint_position(side, joint_name)` | `float \| None` | 单关节位置（支持短名和全名） |
| `hand_joint_names(side)` | `list[str]` | 指定手的关节名列表 |
| `hand_joint_index(side, joint_name)` | `int` | 手指关节名→索引（支持短名和全名） |
| `move_hand(side, pose_dict, duration_sec, wait)` | `bool` | 按字典移动手指（线性插值 + 200Hz） |
| `move_hand_joint(side, joint_name, target_rad, duration_sec, wait)` | `bool` | 单手指关节移动 |
| `shift_hand(side, joint_name, delta_rad, duration_sec, wait)` | `bool` | 手指关节相对偏移 |
| `send_hand_position(side, positions)` | `None` | 单次发送 7 维位置（无插值） |
| `hand_periodic_motion(amplitude, period_sec, cycles, ...)` | `bool` | V4 手部周期 sin 运动测试 |

---

## 关节限位

### 身体关节限位（rad）

| 关节 | 下限 | 上限 |
|---|---|---|
| `L/R_elbow_roll_joint` | -2.6180 | 0.0 |
| `L/R_elbow_yaw_joint` | -2.9147 | 2.9147 |
| `L/R_shoulder_pitch_joint` | -2.8274 | 2.8274 |
| `L/R_shoulder_roll_joint` | -1.85 | 0.0873 |
| `L/R_shoulder_yaw_joint` | -2.8972 | 2.8972 |
| `L/R_wrist_pitch_joint` | -1.5882 | 1.5882 |
| `L/R_wrist_roll_joint` | -1.9897 | 1.9897 |
| `head_pitch_joint` | -0.6807 | 0.5061 |
| `head_yaw_joint` | -1.6406 | 1.6406 |
| `waist_yaw_joint` | -2.7925 | 2.7925 |

### V4 手指关节限位（rad，左右手相同）

| 关节（短名） | 下限 | 上限 |
|---|---|---|
| `thumb_swing` | 0.0 | 2.11 |
| `thumb_mcp` | 0.0 | 1.85 |
| `thumb_pip` | 0.0 | 1.09 |
| `index_mcp` | 0.0 | 1.71 |
| `middle_mcp` | 0.0 | 1.71 |
| `ring_mcp` | 0.0 | 1.71 |
| `little_mcp` | 0.0 | 1.71 |

> 手指关节下限全部为 0.0（完全张开），负值会被裁剪。硬件零位偏移可能导致实际读数略低于 0（如 `thumb_mcp` 读到 -0.002），限位裁剪会在发送命令时自动修正。

---

## 安全机制

1. **速度限制**：身体关节最大角速度 6.28 rad/s（约 360°/s），超出即拒绝整段轨迹
2. **关节范围裁剪**：自动 clip 到 `BODY_JOINT_LIMITS` / `V4_HAND_JOINT_LIMITS`
3. **关节锁定**：锁定关节即使在 target 中有值也不会发布
4. **状态超时检查**：`wait_for_state` / `wait_for_hand_state` 超时即报错
5. **维度校验**：target/trajectory 维度不匹配立即拒绝

---

## 常见问题

**Q: 报错 "Timeout waiting for robot state"**
A: 检查 `rosa run t800_mc_server start_mc_client` 是否启动；检查 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 是否设置（默认 FastDDS 会静默匹配失败）。

**Q: 身体指令发布了但机器人不动**
A: 没切 SDK 控制器，必须先 `rosa run rosa_controllers switch_controller config_mc_walker_s2_v1_sps`。

**Q: 手指指令发了但手指不动**
A: 手部控制器始终监听，不需要 switch_controller。检查话题名是否正确（`/mc/left_hand/command`）；检查是否用了错误的 mode 值（应为 5，不是 2）。

**Q: `--print` 显示手指关节 `⚠️BELOW`**
A: 硬件零位偏移导致实际位置略低于 0.0 限位，这是正常的。发送命令时会自动裁剪到合法范围。

**Q: 安全检查失败**
A: 减小 `duration_sec`（运动太快），或检查 target 与当前位置差距过大。如确认安全可临时 `enable_safety_check=False`，但不推荐。

**Q: 想让头/腰也运动**
A: 构造时传 `lock_joints=[]` 或运行时调用 `controller.set_lock_joints([])`。

**Q: 如何切回 VLA 控制模式**
A: 停止本脚本，运行 `python3 executor_node_sdk.py` 即可，无需切换控制器。
