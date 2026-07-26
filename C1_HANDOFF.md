# Walker C1 / Astron Handoff Notes

本文档给后续接手的 AI/工程师使用。当前有效工作目录已经切换到新 clone 的 `walker_c1` 分支仓库。

## 2026-07-26 LeRobot 常驻多轮评估尝试、根因分析与回滚

### 当前结论

Diffusion Policy 的单次训练和单次推理链路可以继续使用，但不要再使用后来试验的
`walker_c1_eval` 常驻多 episode 策略。用户明确更看重少改 Mentor 原代码，可以接受每个
episode 重新加载模型。当前已经回到与 Walker S2/Tienkung 一致的方式：
LeRobot 原生 `base` strategy 一次只执行一个固定时长的 rollout，结束后停止推理引擎、
断开相机和 Walker Bridge2；下一次运行重新创建并加载。

当前 C1 单次入口：

```bash
docker exec -it lerobot-walker-c1 bash -lc '
ROS_DOMAIN_ID=146 DURATION=30 \
POLICY_PATH=/ubt_IL/model/Walker_C1_26_1RGB_diffusion/checkpoints/050000/pretrained_model \
bash /ubt_IL/scripts/deploy/rollout_walker_c1.sh'
```

第一次推理前仍应由用户在仿真容器里手动运行 `reset.py`，让策略从与训练数据一致的 ready
姿势开始。不传参数时 `rollout_walker_c1.sh` 运行1次；它后来重新提供了 `--episodes N`，
但实现已经改为脚本外层进程循环：每轮重新加载 checkpoint、Bridge2 和相机，再调用一次
原生 `base` rollout。它不是已经删除的常驻 `walker_c1_eval`。

例如运行10次：

```bash
docker exec -it lerobot-walker-c1 bash -lc '
ROS_DOMAIN_ID=146 \
POLICY_PATH=/ubt_IL/model/Walker_C1_26_1RGB_diffusion/checkpoints/050000/pretrained_model \
bash /ubt_IL/scripts/deploy/rollout_walker_c1.sh \
  --episodes 10 --duration 30 --seed 1000'
```

外层脚本在每轮启动模型前通过 ROS 放置苹果，正常 rollout 结束后读取苹果最终位置并按固定
盘心统计成功率。因为上一轮的 Bridge2 已经完全退出，下一轮放置苹果时不存在常驻进程中的
500 Hz body hold 竞争。盘子仍没有独立归位接口；盘子被碰走或 rollout 异常退出时应停止
批量运行并人工恢复场景/机器人。

2026-07-26 用户根据实际推理现象选择取消 C1 机械臂相对目标限幅：
`walker_c1_26d.json` 的 `safety.max_relative_target` 当前为 `null`。这只关闭
LeRobot `send_action()` 中相对实测关节位置的单周期裁剪；机械臂硬关节限位、手部范围裁剪和
Bridge2 的 500 Hz 五次曲线插值仍保留。不限幅时曾观察到策略有成功案例，但手臂会来回修正；
若后续需要恢复折中限幅，优先重新测试 `0.10 rad/周期`，不要直接照搬 S2 的 `0.02`。

### 做过的常驻方案

曾在 LeRobot rollout 核心中新增 `walker_c1_eval` strategy，目标是在同一个进程内只加载
一次 checkpoint、相机和 ROS/ZMQ bridge，然后连续运行 N 个 episode。该策略还负责：

- 按种子在采集方形区域内随机放置苹果；
- 每轮重置 policy/interpolator 状态；
- 失败后把右臂和手恢复 ready；
- 根据苹果到固定盘心的距离判断成功，并输出累计成功率；
- 成功后提前结束本轮，避免始终等待完整 duration。

为了支持它，试验性修改过 LeRobot 的 rollout config、strategy factory、
`ThreadSafeRobot` 辅助接口和测试，并新增
`src/lerobot/rollout/strategies/walker_c1_eval.py`。这些核心修改现已全部定点回滚，
新增 strategy 文件已经删除，`ubt_IL/lerobot` 子仓库恢复为无本地修改状态。

### 苹果从第二轮开始放置超时的确定根因

典型错误：

```text
Timed out waiting for the simulator to place the apple
(target=[8.171047420351096, 5.856062510050574, 0.9421],
 last=[8.464376449584961, 6.263848781585693, 0.9419999718666077])
```

这里的 target 按 `seed=1000` 和当前随机算法反算，精确对应第 2 个 episode。last 与 target
水平相差约 0.50 m，是上一轮动作后遗留的苹果位置，不是苹果从目标点自然滚动造成的厘米级
偏差。因此问题不是 `DURATION` 太短、苹果 z 高度不对或等待落稳不足，而是第 2 轮的放置
命令没有在仿真端执行。

完整链路中的冲突为：

1. `ros2 topic pub --once /sim/cmd_set_object_pose` 只能确认 ROS CLI 发布进程正常结束，
   不能确认仿真最终执行了 teleport。
2. ROS2-ZMQ bridge 将苹果放置消息和机器人 body/hand 指令发到同一个 ZMQ command socket。
3. 第 1 轮策略动作开始后，Walker Bridge2 的 body hold 线程会持续以 500 Hz 发布关节目标，
   即使该轮策略已经结束，常驻进程内仍会继续 hold。
4. 仿真 `WalkerC1Controller` 的同一 command SUB socket 使用 `RCVHWM=1`。低频、单次的
   `set_object_pose` 与 500 Hz body 消息竞争时会被队列丢弃。
5. 第 1 轮放置通常正常，是因为策略还没发过动作、500 Hz hold 尚未启动；从第 2 轮开始
   稳定复现，重启仿真或重新创建 Bridge2 后也只会暂时恢复第一轮。

因此，单纯把等待从 10 秒增到 30 秒、降低苹果落下高度、重发若干次或重启后再试，都没有
解决传输层根因。消息一旦在共享队列中丢失，继续等待不会让它重新出现。可靠的长期方案应是
给仿真场景控制使用独立的可靠通道及执行 ACK，或明确停止高频 hold 后再发送场景命令；但这
会改动 Mentor 的桥接/仿真控制结构，当前按用户要求不继续实施。

### 盘子归位是另一个独立问题

当前 `/sim/cmd_set_object_pose` 的实现只控制 `env.scene["object"]`，即任务苹果，没有盘子
参数，也没有独立的盘子 pose topic。粉色可见盘是场景 USD 内部节点，并不是该接口管理的
任务刚体，所以现有 C1 推理代码不能保证盘子被碰走后自动归位。

期间尝试过两条路线，均已放弃：

- 在 episode 间执行整个 `/sim/cmd_reset`：常驻 rollout 中会扰动相机/场景生命周期，曾出现
  后续取不到相机帧，且不能把它当成可靠的盘子专用 ACK；
- 在 `scene_v2_c1.usda` 中移除粉色盘内部网格的 RigidBody/Collision API，使其不可移动：
  用户明确不希望为了评估修改 USD，因此该试验性修改已经用 Git 恢复。

若以后仍要自动评估多轮且允许盘子移动，必须在“每次重启完整仿真恢复场景”和“增加盘子
专用 reset 接口”之间明确选择；不要声称现有苹果 topic 可以恢复盘子。

### 本次回滚范围与保留内容

已经回滚：

- LeRobot 中所有 `walker_c1_eval` config/factory/export/wrapper/test 修改；
- 新增的 `walker_c1_eval.py`；
- `rollout_walker.sh` 中对常驻 C1 strategy 的特殊分支；
- `scene_v2_c1.usda` 中固定粉色盘的试验性覆盖。

继续保留：

- `walker_c1_26d.json` 的26维动作映射；
- Walker C1 相机、手部和 Bridge2 的基础适配；
- HDF5 → LeRobot 数据转换配置；
- Diffusion Policy 训练配置和 checkpoint；
- `rollout_walker_c1.sh` 单次推理入口；
- 原 `rollout_walker.sh` 自带的动作维度/配置预检。

后续跑 100 次使用 `rollout_walker_c1.sh --episodes 100`；该参数现在只控制外层进程循环，
不要重新修改 LeRobot strategy 核心。脚本已负责随机放苹果和按最终位置统计成功率，但盘子
恢复仍未自动化，因此批量运行期间需要观察盘子是否被碰走。

---

## 2026-07-21 mentor 反馈：固定抓取倾角并恢复 8 cm 苹果

抓取掌心姿态不再复制历史成功轨迹中带侧歪的 3×3 朝向矩阵。现在完整姿态固定为 base frame
下的水平掌心向下矩阵 `diag(1, -1, -1)`；抬升和释放沿用同一倾角，搬运只允许绕竖直轴增加
5° yaw，不改变倾角。标称 approach 的自然 IK 种子腕 roll 从 `-1.376 rad` 降为
`-0.144 rad`，肘 pitch 为 `-1.671 rad`，即用收肘来替代扭腕；实际闭合前腕 roll 实测
`-0.432 rad`，旧方案约 `-1.35 rad`。

任务苹果恢复为与 Tiankung 原场景相近的 8 cm：可见包围盒约
`77.9 x 80.0 x 78.9 mm`，不可见球碰撞半径 `0.040 m`，质量和摩擦仍为
`0.10 kg / 1.20`。固定位置完整物理回归首轮通过：苹果落稳中心 z=`0.942 m`，抬升
`9.2 cm`，静置 1.4 仿真秒后 `STABLE`，最终距盘心 `3.2 cm`，结果 `1/1 SUCCESS`。

根据后续目视反馈，盘上松手净空又从 `0.060 m` 提高 `0.050 m` 至 `0.110 m`；只抬高
松手点，盘上 xy、搬运高度和撤离路径均不变。完整回归中掌心释放目标 base-z 从约
`0.161 m` 提高到 `0.211 m`，实测到达 `0.195 m`；任务 `1/1 SUCCESS`，但苹果最终距盘心
`0.107 m`，已经接近 `0.120 m` 成功边界；盘子随后移到正前方后该落点问题已消失。

这里的“抓取标定”只指苹果中心相对掌心、手口和指笼中心的任务几何偏移以及手指闭合量，
不是相机内外参或机器人关节零位标定。固定水平掌心朝向是明确的姿态约束，不再称作经验标定。

后续盘子从右前方 `(8.190, 5.710)` 移到机器人身体中线正前方 `(8.190, 6.083)`。粉色可见
盘模型、不可见圆柱碰撞体和在线 IK/成功判定目标三者同步移动，盘面高度保持不变。抓稳优先：
通过 1.4 秒 `STABLE` 检查后才开始搬运；取消旧的右侧固定路点和后续尝试的几何中点，直接
用 3 秒平滑轨迹移动到正前方盘子上方，同时渐变到 `+35° yaw` 以避开腕 pitch 限位。
掌心倾角和闭手目标保持不变，盘上搬运点始终比松手点高 `0.040 m`。
正前方盘位首轮物理回归中苹果—手口距离全程保持约 `0.014-0.015 m`；释放前实测腕
pitch/roll 为 `-0.035/-0.068 rad`、肘 pitch 为 `-1.488 rad`。苹果最终落在
`(8.156, 6.081)`，距新盘心 `0.034 m`，结果 `1/1 SUCCESS`。

C1 手动启动入口的默认 `step_hz` 从 30 提高到 100，使 100 Hz 物理仿真不再以约 0.3 倍速
慢放；动作轨迹、关节目标、闭手时间和 1.4 仿真秒防滑检查均未缩短。若机器带五相机渲染时
负载较高，可用 `UBT_SIM_C1_STEP_HZ=60` 启动；默认/最快使用 `100`。

在线控制器另有 `--motion-speed`，默认 `5.0`，只压缩机械臂插值时间；闭手轨迹和
`HELD/STABLE` 安全检查保留。已把重复 ready 手部命令及苹果落稳、抓后静置、松手落稳等固定
等待压缩到必要时长，静态防滑检查由 1.4 缩短为 0.8 仿真秒。保守速度可用
`--motion-speed 1.0`、`1.5` 或 `2.0`。`5.0x` 连续两轮完整回归均耗时约 25 秒，苹果都通过
`HELD/STABLE`，最终距盘心分别为 `0.024/0.021 m`，两轮均为 `1/1 SUCCESS`。记录的 HDF5
会在根属性保存本局 `motion_speed`。

---

## 2026-07-21 最终状态总览（后续接手先读本节）

### 1. 当前完成度与版本

Walker C1 已完成一条可实际运行的 Isaac Sim + ROS 2 闭环任务链：机器人进入安全准备姿势，
在线读取苹果位置，使用多种子 6D IK 完成抓取、抬升、静置防滑检查、经身体前方路点搬运、
放入粉色盘子，最后双臂安全归位。ROS 运行时可同步记录左右臂、左右手、动作目标、时间戳和
头部 RGB，成功后保存 HDF5，因此目前具备自动执行和自动采集固定位置示范数据的能力。

本节对应的实现基线：

```text
branch: walker_c1
commit: d8e9320 feat(walker-c1): merge mentor sensors and restore task visuals
remote: origin/walker_c1（已 push）
```

commit `d8e9320` 已包含 mentor 原始 USD、合并后的双手 USD、苹果复合 USD、构建脚本、控制
代码和本文档。两个主 USD 分别约 51.7/54.8 MB，GitHub 已接受，但因为本机没有 `git-lfs`，
它们是普通 Git blob，push 时只有超过推荐 50 MB 的警告，没有失败。无关 Astron SDK `.docx`
始终未提交。

### 2. 推荐启动方式（两个终端进入同一个容器）

不要同时启动两套 Isaac Sim。终端 A 只负责仿真，终端 B 只运行 ROS pick-and-place；这里的
“两个终端”是两次 `docker exec` 进入同一个 `walker-c1-ubt-sim` 容器，不是再创建一个容器。

终端 A，启动 mentor 机身 + 当前灵巧手 + ROS 控制 + 五相机视窗：

```bash
docker exec -it walker-c1-ubt-sim \
  bash /ubt_sim/scripts/start_c1_mentor_sensor_sim.sh --control
```

等仿真完成场景加载并出现 ZMQ controller 日志后，终端 B 执行一次任务并保存轨迹：

```bash
docker exec -it walker-c1-ubt-sim \
  bash /ubt_sim/scripts/run_c1_pick_place_once.sh
```

仅验动作、不保存数据：

```bash
docker exec -it walker-c1-ubt-sim \
  bash /ubt_sim/scripts/run_c1_pick_place_once.sh --no-record
```

修改采集帧率（默认 30 Hz，例如 20 Hz）：

```bash
docker exec -it walker-c1-ubt-sim \
  bash /ubt_sim/scripts/run_c1_pick_place_once.sh --record-hz 20
```

成功轨迹保存到 `/ubt_sim/dataset/walker_c1_ros/<毫秒时间戳>/trajectory.hdf5`。失败默认丢弃，
调试时可加 `--save-on-failure`。ROS 记录器优先使用 `/sim/object_state` 的 100 Hz `sim_step`
调度 3/3/4 步采一帧以得到平均 30 Hz；没有仿真步（真机）时回退 ROS 图像时间戳节流。
连续 `--episodes N` 运行时，上一轮成功结束已经回到 ready，下一轮会跳过重复的开场归位；
归位以实测关节最大误差不超过 `0.05 rad` 为完成条件，而不是只看指令是否发完。如果上一轮
失败或关节未收敛，下一轮仍重新执行归位以保证安全。
`--randomize` 以 `APPLE_SPAWN_W` 为中心，在 X/Y 各 `±0.025 m` 的 `5 cm × 5 cm` 正方形
区域内均匀采样，Z 高度保持不变。区域中心为 `(8.170, 5.860)`，相对最初中心沿机器人右侧
（世界 Y 负方向）共移动 `0.040 m`，进一步远离盘子。可用 `--apple-xy X Y` 指定世界坐标，
用于确定性验证中心和四个角点；不能与 `--randomize` 或 `--use-existing-apple` 同时使用。
曾测试扩大至 `10 cm × 10 cm`：中心及前三个角点可完成抓放，但靠盘子的角点
`(8.220, 5.910)` 抓取失败，且用户观察到左上角路径会碰盘，因此该范围判定为不安全并撤回。
最终 `5 cm × 5 cm` 区域最靠盘子的两个角点 `(8.145, 5.885)`、`(8.195, 5.885)` 均完整通过
`HELD/STABLE/SUCCESS`；落盘距盘心分别为 `0.065/0.050 m`。

`run_c1_online_ik_batch.sh` 会自行管理 Isaac 进程，不应在 GUI 仿真已经运行时再启动；此前
“一运行就 Killed”主要来自同时运行两套 Isaac 的内存/GPU 资源竞争。全局默认已切换为 mentor
机身/传感器与当前灵巧手的合并 USD，因此普通 GUI、batch 和 ROS 抓放均使用同一机器人资产。

### 3. 机器人 USD 的实际组成

mentor 资产目录：

```text
ubt_sim/assets/robots/walker_c1/Collected_walker_c1_v1_sensorKpkd/
  Collected_walker_astron_v1_sensorKpkd/
    walker_astron_v1_sensorKpkd.usd          # mentor 原始身体/传感器，无灵巧手
    walker_astron_v1_sensorKpkd_hands.usd    # 当前推荐的合并结果
    SubUSDs/                                  # 双目、鱼眼、下巴相机子层
```

mentor 原始资产有 31 个身体可动关节、40 个身体刚体、5 个相机和 2 个 IMU，但缺少双灵巧手。
`merge_walker_c1_mentor_usd.py` 从已经通过抓放验证的 `walker_c1_force_drive_grip.usd` 拼入：

```text
30 个手部刚体
22 个手指 revolute joints
8 个手部 fixed joints
56 个实际被手部使用的独立 prototype
4 个手部视觉材料
1 个手部高摩擦物理材料
```

最终组合为 53 revolute + 17 fixed + 70 rigid body。mentor 身体原有 80 条视觉/碰撞引用逐条
保持不变。绝不能把原 C1 的全部 `Flattened_Prototype_N` 原路径复制进 mentor USD：两边同名
编号对应不同网格，会覆盖身体 prototype，表现就是机器人四分五裂。当前脚本把手部模板放在
独立 `/C1HandPrototypes` 并重写引用，已经解决该问题。

手并不是选择一种统一颜色。原手模型的 4 种视觉材料同时按子网格出现：白色、银灰和淡蓝灰
分别用于掌壳、手指等零件；左右手的具体分配沿用原 USD。物理抓取材料独立位于 default prim
内部 `/walker_astron_v1/PhysicsMaterials/HandGripMaterial`，绑定 30 个手部 collision instance，
不会因 Robot reference scope 丢失。

mentor 左鱼眼主层曾多出 0.646 m 局部平移，合并脚本会清除该错误 opinion，使左右鱼眼恢复
约 `+/-71.4 mm` 的对称安装位置。五个 GUI viewport 分别显示 Stereo Left/Right、Fisheye
Left/Right 和 Jaw Camera。

默认 `start_sim.sh` 和 `start_c1_pick_place_sim.sh` 均使用 mentor 合并版本。
`start_c1_mentor_sensor_sim.sh` 仍显式选择同一 USD，并额外打开五个传感器 viewport；不带
`--control` 时该脚本只是 load-only 传感器预览，不启动 ROS bridge，做抓放必须加 `--control`。
如需临时回退旧资产，可通过 `UBT_SIM_WALKER_C1_USD_PATH` 指向
`walker_c1_force_drive_grip.usd`，无需修改代码。

### 4. 苹果、桌子和盘子的最终几何

苹果不再显示为纯红球，但物理抓取条件没有改变。`c1_task_apple.usda` 的结构是：

```text
/C1Apple                         RigidBody，mass=0.10 kg
  /Collision                     不可见 Sphere，radius=0.027 m
  /PhysicsMaterials/...          static/dynamic friction=1.20
  /Visual                        引用 Tiankung scene_v2.usd 的原苹果网格/材质
```

Visual 使用原 `Yellow_Red_Nectarine` 材质，Albedo、Normal、Roughness 纹理均可解析；缩放并
居中后外观包围盒约 `52.6 x 54.0 x 53.3 mm`。原网格上的 RigidBody/Mass/Collision API 已全部
删除，所以物理引擎只看到原来成功版本的 27 mm 球碰撞体，不会因复杂三角网格改变抓取。
`build_c1_apple_usd.py` 可重建该小型组合层，但成品已经随 commit `d8e9320` 提交。

最终场景参数：

```text
robot base world xy: 约 (7.803, 6.083)
apple fixed world:    (8.170, 5.860)，落稳中心 z=0.929
plate center world:   (8.190, 6.083)，机器人身体中线正前方，粉色原盘模型保留
table top world z:    0.902
table x shift:        -0.160 m（只把桌子向机器人靠近，苹果/盘子 xy 不跟随）
apple collision:      sphere r=0.040 m, mass=0.10 kg, friction=1.20
hand collision:       friction=1.50，手指 effort 仍为 2 N.m
```

白色厚圆柱不是显示盘子，而是 `visible=False` 的盘子碰撞体；画面中应看到原粉色薄盘。场景
装饰苹果在 `scene_v2_c1.usda` 中保持 deactivate，避免与任务刚体出现两个苹果。

### 5. 在线 IK、抓取和最终姿势参数

控制入口是 `teleoperation/control/walker_c1/pick_place_controller.py`。任务不是回放固定关节
轨迹：每局从 `/sim/object_state` 读取实时苹果位置，在线生成掌心路点并用多种子 6D IK 求解。
完整掌心朝向约束仍保留，抓取阶段固定为水平掌心向下；抓稳后直接用 3 秒平滑轨迹移动到
正前方盘子上方，并在水平面内渐变到 `+35 deg` place yaw。

关键阶段：安全 staged reset -> approach -> hover -> descend/闭环掌口对准 -> close -> lift ->
1.4 仿真秒静置防滑 -> direct carry over plate -> lower/release -> retreat -> staged
reset。`HELD` 只说明刚抬起，必须随后看到 `static hold ... STABLE` 才算抓持可靠。

最终盘上释放净空为 110 mm，目标掌心 base-z 约 `0.211 m`，避免张开手指碰粉色盘沿；盘上
搬运点再高 40 mm，保证动作仍然是先高位搬运、再下降松手。

最终 ready pose 的左臂是右臂严格镜像：肩/肘/腕 pitch 同号，roll/yaw 反号；左右手均发送
相同的 6 个主动关节全开命令，其余 5 个/手从动关节按镜像机构联动。归位仍采用安全三阶段：
双臂侧向抬起 0.6 仿真秒 -> 收肘 0.6 仿真秒 -> ready 1.0 仿真秒，避免向前扫桌和盘。

### 6. 最终验证结果

mentor 合并 USD + 原苹果外观 + 球碰撞版本完成完整 ROS 回归：

```text
apple settled:       world z=0.929 m（与旧球版本一致）
grasp tracking:      典型 7-8 mm
lift:                8.3-8.4 cm
apple-mouth:         2.4 cm
static hold:         STABLE after 1.4 simulated seconds
release target:      base z=0.161 m
final plate distance:2.2 cm（success limit 12 cm）
result:              1/1 SUCCESS，安全归位
```

苹果视觉刚替换时、释放净空还是 40 mm 的回归落点为 1.9 cm；净空提高到 60 mm 且左臂镜像后
再次完整回归，落点为 2.2 cm。两次均 `HELD`、`STABLE`、`SUCCESS`，证明视觉替换、释放抬高
和左臂镜像没有破坏抓放。

### 7. 已知限制与排查顺序

1. 当前 CPU 仿真约为实时的 0.25 倍，日志中的 0.6/1.0/1.4 秒都是仿真时间，墙钟约放大
   4 倍；reset 和抓放中的短暂停顿是按物理步等待，不是程序卡死。
2. 启动时 `Clear_Glass.mdl` 缺失警告来自客厅玻璃材质，与机器人、手和苹果无关。
3. mentor USD 内含 drive/Kp/Kd，但当前 Isaac Lab `ImplicitActuatorCfg` 会覆盖 USD drive 参数；
   当前成功只证明几何、惯量、传感器和现有控制增益兼容，尚未直接使用 mentor 的原 Kp/Kd。
4. 运行很久或经历剧烈失败后的 PhysX 进程可能出现接触状态退化；若连续抓空，不要立刻继续
   改摩擦/IK，先完整重启仿真做一次冷启动判决。
5. 回真机前必须核对 SDK joint order、限位和安全控制；当前成果是 Isaac Sim + ROS 控制链，
   不能把仿真成功直接等同于真机已验证。
6. 固定位置高成功率是当前验收目标；随机苹果位置仍是后续增强项。批量采集前应先做多次
   冷启动固定点回归，再逐步扩大随机范围。

---

## 2026-07-20 Online IK Pick-and-Place 已完成

非 replay 路线已经连续两次独立冷启动完整成功：`reset.py` 准备姿势 → 在线读取
`/sim/object_state` → 6D IK 接近/闭环对准 → 物理抓取 → 放入盘子 → 回准备姿势。
两次抬升分别为 8.3 cm、8.4 cm，最终落点距盘心 1.4 cm、1.5 cm。

端到端确定性抓空的关键修复不是继续增加夹持力，而是消除盘子对张开手指的碰撞干扰：苹果固定在
`(8.17, 5.90)`，盘子最终放在 `(8.19, 5.71)`。此前盘子中心在 `(8.20, 5.76)`
时，盘沿虽不碰苹果，却进入无名指/小指 6-8 cm 的张开范围，导致确定性抓空。

连续成功后又按用户目视反馈增加了延长静置测试，发现标准摩擦下存在另一个独立问题：
苹果刚抬起时 `dz=7.7-8.1 cm`、距掌口 `3.2-3.4 cm`，但静置约 1.4 个仿真秒后会
慢慢滑回桌面。继续卷曲四指、单独增加拇指角度都不能解决，前者还会把球向下挤出。
因此最终启用 `walker_c1_force_drive_grip.usd`（手部碰撞摩擦 1.5），但手指力矩仍保持
2 N.m、初始闭合姿势仍保持 `[0.7,0.85,0.8,0.8,0.8,0.8]`；用户 GUI 反馈保持效果
可接受。控制器保留 `static hold check`，只有延长静置后仍满足高度和掌口距离阈值才搬运。

场景最终状态：物理桌面和视觉桌一起向机器人方向移动 16 cm，但苹果、盘子和机器人
水平坐标不变；桌面及其上的苹果、粉色盘子和隐藏碰撞体整体抬高 5 mm。白色厚圆柱仅作为
不可见盘子碰撞体，原粉色盘子模型移动到目标位置显示。
归位路径恢复 `reset.py` 的安全顺序：双臂先侧向外展，再收肘，最后进入准备姿势，
避免手向前扫到桌子或盘子。

### 2026-07-20 桌面位置微调

按用户目视反馈，C1 覆盖层中的桌子先由相对 Tiankung 原场景向机器人移动 4 cm 改为 6 cm，
随后又在当前基础上靠近 10 cm，最终累计移动 16 cm。桌子近侧边缘约为 `x=7.984`，距机器人
基座中心约 18.4 cm；苹果和盘子的 x/y 始终保持成功版本不变。
桌面高度由 `z=0.897` 改为 `z=0.902`。为避免只移动视觉模型造成物理错位，视觉桌、隐藏
桌面碰撞体、粉色盘子、盘子碰撞体、苹果初始高度和 ROS 放置目标全部同步抬高 5 mm。
桌子与 Tiankung 使用同一个 `scene_v2` 网格和材质，只改组合场景中的 transform。

没有把 C1 的 5.4 cm 物理球换成 Tiankung 原场景约 8.5 cm 的苹果网格：C1 当前成功抓取的
五指笼只有约 5-6 cm，直接换大苹果既放不进指笼，复杂网格接触也会增加卡碰和挤出风险。
如后续实验尺寸，只建议先把球半径从 27 mm 增至 28 mm，并重新标定掌口高度。

改动后完整冷启动回归首次成功：苹果落稳中心 `z=0.929`，抬升 8.0 cm，静置 1.4 仿真秒后
苹果到掌口 2.7 cm、判定 `STABLE`，最终距盘心 1.3 cm，成功放盘并归位。验证日志位于容器
`/tmp/c1_online_ik_batch_1784604440`，同步轨迹为
`/ubt_sim/dataset/walker_c1_ros/1784604575962/trajectory.hdf5`（820 帧）。
桌子继续靠近 10 cm 后再次冷启动首轮成功，抓持和静置指标不变，最终距盘心 1.2 cm；验证
日志为 `/tmp/c1_online_ik_batch_1784607925`，轨迹为
`/ubt_sim/dataset/walker_c1_ros/1784608057157/trajectory.hdf5`（821 帧）。

### 2026-07-21 IK 自然姿态优化（腰部仍锁定）

用户目视认为抓取姿势略显扭转。根因不是 PD 或右臂自由度不足，而是完整掌心姿态约束、固定
成功种子和 ikpy 局部收敛共同选择了偏直肘、靠近肩/腕限位的解。腰部三关节继续固定为零，
没有引入相机视角、真机重心和躯干碰桌的新变量。

`robot_controller.py` 新增多种子全姿态 IK：从成功种子、当前实测/命令姿势和 reset ready
姿势分别求解，硬性保留掌心位置与完整朝向约束，再按关节限位余量、肘部弯曲、ready 距离和
轨迹连续性选解；没有合格候选时回退首个已验证种子。approach 之后的 hover、grasp 和掌口
闭环不再每段重置为同一个固定种子，而是以上一命令解连续求解。

仅对已验证掌心姿态增加绕 base-Y `+5 deg` 的小幅放松，固定指笼 yaw 仍保留。抓取前实际
右臂由旧的 `[-0.322,+0.114,+0.728,-0.337,+0.219,-0.101,-1.466]` 改为
`[-0.230,+0.043,+0.706,-0.624,+0.227,+0.112,-1.349]`：肘部弯曲增加，肩 roll 和腕 roll
均远离原极限方向；掌口、抓取中心和各指尖相对苹果仅变化约 1-4 mm。

三次独立冷启动全部首次成功。每次苹果均提升 8.3 cm，静置后苹果到掌口 2.4 cm 且判定
`STABLE`，最终距盘心 2.3-2.4 cm，完整放盘并安全归位。验证日志：
`/tmp/c1_online_ik_batch_1784616029`（1/1）和
`/tmp/c1_online_ik_batch_1784616259`（2/2）；后两条同步轨迹为
`/ubt_sim/dataset/walker_c1_ros/1784616403263/trajectory.hdf5`（820 帧）与
`/ubt_sim/dataset/walker_c1_ros/1784616538009/trajectory.hdf5`（821 帧）。

### 2026-07-21 自然抬升与中间搬运路点

用户认为抓起后的上升和横向运输仍显生硬。苹果和盘子位置明确保持不变：苹果仍为
`(8.17,5.90)`，盘心仍为 `(8.19,5.71)`；当前苹果中心相对基座水平约 41 cm，盘心因偏右
约 54 cm。盘子不能沿 y 靠近苹果，历史上 `(8.20,5.76)` 会进入张开无名指/小指的扫掠区。

抓取和最初抬升继续严格保持抓取朝向，但抬升关节插值改用 smoothstep。静置验证通过后，
新增 base frame 身体前方中间掌心点 `(0.290,-0.285,0.220)`，经过该点时在已持稳苹果的前提下
增加 `+5 deg` carry yaw，随后平滑移动到原盘子上方；下降进盘时撤销这 5 deg。仅抬升、
中间搬运、盘上方移动、下降和空手撤离使用 smoothstep，抓取下降/对准/闭手参数没有改动。

中间点实际右臂稳定为约 `[+0.236,-0.034,-0.030,-1.622,+0.783,+0.181,-0.490]`：相比抓取时
腕 roll `-1.349`，运输时降到 `-0.490`，同时保持约 93 deg 弯肘；多种子 IK 最小限位余量
8.8-8.9%。中间点苹果到掌口 1.8-1.9 cm，释放前约 1.6 cm，没有渐进滑落。盘上方误差由旧
18 mm 降到 15 mm，空手撤离误差由 24 mm 降到 17 mm。因盘子位置保持较远且偏右，下降释放
时肘角仍会伸到约 `-0.29 rad`，这是当前固定场景几何下保留的取舍。

三次独立冷启动均首次成功，最终距盘心均约 2.2 cm。验证日志：
`/tmp/c1_online_ik_batch_1784617945`（1/1）和
`/tmp/c1_online_ik_batch_1784618360`（2/2）；轨迹：
`/ubt_sim/dataset/walker_c1_ros/1784618092800/trajectory.hdf5`（856 帧）、
`/ubt_sim/dataset/walker_c1_ros/1784618507830/trajectory.hdf5`（856 帧）和
`/ubt_sim/dataset/walker_c1_ros/1784618646525/trajectory.hdf5`（855 帧）。

### 2026-07-21 盘上方释放净空

用户目视发现下降释放时手指可能碰到粉色盘沿。盘子、苹果和桌子的坐标均未改动，仅把
`place_and_return()` 中掌心释放点相对盘面锚点的净空由 25 mm 提高到 40 mm。掌心目标
`z` 因此由约 `0.126 m` 提高到 `0.141 m`；受跟踪误差影响，实测到达由此前约 `0.119 m`
提高到 `0.128 m`，实际增加约 9 mm。释放姿态、盘上方水平位置和撤离路径保持不变。

三次独立冷启动均首次成功，苹果最终距盘心 19-20 mm，没有因提高释放点而弹出盘面。
验证日志：`/tmp/c1_online_ik_batch_1784620056`（1/1）和
`/tmp/c1_online_ik_batch_1784620336`（2/2）；新增同步轨迹：
`/ubt_sim/dataset/walker_c1_ros/1784620203958/trajectory.hdf5`、
`/ubt_sim/dataset/walker_c1_ros/1784620483823/trajectory.hdf5` 和
`/ubt_sim/dataset/walker_c1_ros/1784620622695/trajectory.hdf5`（均为 856 帧）。

### 2026-07-21 Mentor C1 身体/传感器与当前灵巧手合并预览

Mentor 提供的 `Collected_walker_c1_v1_sensorKpkd` 可正常用 pxr 打开，包含 31 个身体可动
关节、40 个身体刚体、左右双目、左右鱼眼、下巴相机和两个 IMU，但完全没有当前抓放依赖的
22 个手指关节与 30 个手部刚体。它和当前 USD 的 31 个共同关节在轴、限位、local pose、
质量和惯量上逐项一致，因此可以安全挂接现有灵巧手，但不能直接替换机器人 USD。

新增 `merge_walker_c1_mentor_usd.py`：复制当前 `walker_c1_force_drive_grip.usd` 中缺失的
30 个手部刚体、22 个手指转动关节和 8 个手部固定关节，输出独立的
`walker_astron_v1_sensorKpkd_hands.usd`。原成功 USD 保持默认，不被覆盖。Mentor 主层错误地给
左鱼眼 Camera 额外写入 0.646 m 局部位移，组合后离头部约 0.718 m；合并脚本将该 Camera
局部位移恢复为子 USD 中的零值，使左右安装点回到对称的约 +/-71.4 mm。

重要合并坑：Mentor 和当前 USD 分别有 177/128 个隐藏 `Flattened_Prototype_N`，相同自动
编号代表不同视觉/碰撞网格。第一版错误地原位复制当前 128 个模板，覆盖 Mentor 同名模板，
导致身体 link 引用错误网格，GUI 中机器人四分五裂。最终实现只读取 30 个手部刚体实际引用
的 56 个模板，将它们复制到独立 `/C1HandPrototypes` 并重写手部 instance reference；Mentor
身体的 80 条 visuals/collisions 引用在合并前后逐项一致（0 diff），根级 177 个模板不改动。
抓取材质也不能放在 default prim 外的 `/PhysicsMaterials`（作为 Robot reference 时目标会被
USD 丢弃），现复制到 `/walker_astron_v1/PhysicsMaterials/HandGripMaterial`，并绑定到 30 个
实际手部 collision instance，Isaac Lab spawn 时不再有越过 reference scope 的警告。

`UBT_SIM_WALKER_C1_USD_PATH` 可临时选择机器人 USD；`start_c1_mentor_sensor_sim.sh` 会选择
合并资产、关闭 ROS bridge、启用 load-only，并打开 Stereo Left/Right、Fisheye Left/Right、
Jaw Camera 五个 GUI viewport。命令：
`docker exec -it walker-c1-ubt-sim bash /ubt_sim/scripts/start_c1_mentor_sensor_sim.sh`。
加 `--control` 则启用物理和 ROS bridge，保留五相机窗口，可在另一终端运行原 pick-place：
`docker exec -it walker-c1-ubt-sim bash /ubt_sim/scripts/start_c1_mentor_sensor_sim.sh --control`。

合并后 pxr 校验为 53 个转动关节、17 个固定关节、70 个刚体、5 个相机、2 个 IMU，左右
掌心的 instance 均可解析视觉 Mesh 和 CollisionAPI；Isaac Lab spawn smoke test 通过，53 个
home pose 关节无缺失/无多余，Action Manager 总维度 53（双手各 11）。五个相机 prim 均在
GUI 启动时找到并成功创建 viewport。当前 Isaac Lab `ImplicitActuatorCfg` 仍会覆盖 USD 中的
Kp/Kd，因此合并预览没有启用 Mentor 的高跨度 PD 数值；后续若测试该增益应另做可切换配置。

修复 prototype 冲突和材质 reference scope 后，合并 USD 完成一次独立冷启动在线 IK 抓放并
首轮成功：抬升 8.4 cm、苹果距掌口 2.4 cm，1.4 仿真秒静置后判定 `STABLE`，最终距盘心
1.9 cm，正常归位并保存 856 帧同步轨迹。日志：`/tmp/c1_online_ik_batch_1784623309`；轨迹：
`/ubt_sim/dataset/walker_c1_ros/1784623456143/trajectory.hdf5`。

### 2026-07-20 动作节奏优化与实测

用户反馈 reset 第一段突跳、第二段慢且阶段间像卡住，同时整套 pick-and-place 墙钟耗时过长。
根因有两个：旧 `go_ready()` 每段直接发送固定目标并重复保持，没有关节轨迹插值；CPU 仿真
实时倍率约 0.25，所以 1 个仿真秒约等于 4 个墙钟秒，代码中的稳定等待会被视觉放大。

当前 `go_ready()` 和独立 `reset.py` 都改为基于实时关节反馈的 smoothstep 插值，并严格只按
“肩部侧抬 0.6 仿真秒 → 肘部折叠 0.6 仿真秒 → 全身 ready 1.0 仿真秒”依次动作。
在线归位由 6.0 缩短到 2.2 个仿真秒；在线任务从开始归位到设置苹果实测约 13 秒墙钟，
原版本约 30 秒。运行日志出现这三个新时长即可确认控制器已加载新代码。

Pick-and-place 同时缩短了空中接近、重复对齐、搬运、下降和撤离，但没有改抓取手型、摩擦、
苹果/盘子坐标，也保留闭手稳定和 1.4 仿真秒慢滑检查。优化后完整回归一次成功：苹果提升
8.3 cm，静置判定 `STABLE`，最终距盘心 1.8 cm，完整任务约 128 秒墙钟完成。日志现在会
明确写出 reset 阶段以及苹果落稳、抓取位姿稳定、静置防滑和释放落稳等待，不应将这些日志
明确标记的等待误判为控制器卡死。

后续曾尝试把下降、对齐和提起进一步压缩，结果一次直接抓空、一次抬起后静置滑落，因此已
回退所有抓取临界段到上述完整成功版本，只保留与物体接触无关的 reset 进一步提速。连续失败
后 ROS scene reset 未恢复稳定接触，符合此前观察到的 PhysX 长进程接触状态问题；下次端到端
验收应完整重开仿真，而不是继续在当前长时间运行的进程里判断抓取参数。

### ROS 在线 IK 同步数据采集

`run_c1_pick_place_once.sh` 和 `run_c1_online_ik_batch.sh` 现在默认在执行最新 ROS 在线 IK 的
同时记录训练数据，不再依赖另一套 Isaac 进程内采集轨迹。记录器由头部 RGB topic
`/sensor/camera/head/color/raw` 触发，默认以 30 Hz 同步保存：左右臂和左右手实测位置、
实际下发的位置目标、ROS 图像时间戳和 JPEG RGB。仿真 100 Hz 物理步下采用 3/3/4 步
交替调度以保持平均 30 Hz；真机无仿真步计数时按 ROS 图像时间戳节流。可通过
`--record-hz <Hz>` 修改。HDF5 仍使用现有 26 维
observation/action + `camera_head` schema，可继续使用 `Walker_C1_26_1RGB.json` 转换。
仿真 HDF5 的 `observations/timestamp` 使用物理仿真时间，避免 CPU 实时倍率较低时把 30 Hz
误写成约 8 Hz；真机没有 `sim_step` 时自动使用 ROS 图像时间戳。根属性
`timestamp_clock` 会明确记录实际时钟来源。

成功轨迹保存到 `/ubt_sim/dataset/walker_c1_ros/<毫秒时间戳>/trajectory.hdf5`，失败默认只
丢弃缓存而不污染训练集。单次脚本可追加 `--no-record` 关闭记录，或追加
`--save-on-failure` 保留失败轨迹用于调试。HDF5 根属性包含 `recording_source=ros2`、任务名、
成功标记、相机 topic 和记录频率。

30 Hz 默认值与 Tienkung ROS 采集器、Walker S2 仿真采集器以及
`Walker_C1_26_1RGB.json` 的视频配置一致；S2 真机部分数据配置使用 12.5 Hz，因此换真机或
做对照实验时直接在脚本后追加 `--record-hz 12.5`，无需改代码。2026-07-20 最终干净回归
首轮成功，生成 `/ubt_sim/dataset/walker_c1_ros/1784601728511/trajectory.hdf5`：820 帧，
状态/动作均为 26 维，所有字段长度一致，时间戳严格递增且分析器实测为 30.00 Hz，全部 JPEG
均可解码为 480x640x3，文件约 26.96 MB。无仿真状态的预期失败测试确认输出 0 个文件，
失败过滤有效。

入口：

```bash
docker exec walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_online_ik_batch.sh 1 3
```

需要看 GUI、并将仿真和任务分成两个终端时，分别运行：

```bash
docker exec -it walker-c1-ubt-sim bash /ubt_sim/scripts/start_c1_pick_place_sim.sh
docker exec -it walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_pick_place_once.sh
```

第二个脚本只连接现有仿真执行一次任务，不会启动、关闭或重启仿真。

验证日志：`/tmp/c1_online_ik_batch_1784544284`、
`/tmp/c1_online_ik_batch_1784544582`（容器内）。批处理每次失败都会重启完整仿真栈，
退出/中断也会清理控制器和仿真子进程，避免多个控制器同时发命令。

### 多日调试经验总结

1. **先分清资产、控制、规划、接触四层问题。** C1 最早的手臂/手指下垂不是 IK 或
   增益问题，而是 USD drive 使用 acceleration 语义，微小惯量使等效力矩接近零；转换为
   force-drive USD 后才真正解决。后来接触弹射则相反，是 force 语义下沿用过大增益造成，
   回退到 stiffness 10 / damping 1 / effort 2 才正常。
2. **仿真时间必须按物理步推进。** CPU 仿真明显慢于墙钟；用 `time.sleep()` 控动作会把
   物理动作加速数倍，导致接触冲击和苹果弹飞。所有动作、闭合和稳定等待统一使用
   `wait_sim_steps()`，真机无步计数时再退化为墙钟等待。
3. **已知成功轨迹适合做判决实验，不适合作为最终任务。** replay 证明 ROS bridge、手指
   指令和物理抓取链路可行，也提供成功姿态/手部几何标定；但用户要求在线感知和 IK，最终
   控制器每局都读取 `/sim/object_state` 并重新规划，没有逐帧回放。
4. **仅做位置 IK 不够。** 自由 yaw 会让同一个掌心位置对应不同指笼朝向，表现为时好时坏。
   最终使用完整掌心姿态的 6D IK、成功构型种子和实时罩口中心闭环，抓取前实际误差稳定到
   约 5-7 mm。
5. **球形物体的抓取要看整个指笼几何。** 五指 `[0.2]*6` 预张开可避免下探时小指扫球；
   苹果半径 0.027 m 能建立对向接触且仍放得进 5-6 cm 指笼；继续深卷曲并不等于更牢，
   可能把球从下方挤出。
6. **视觉不碰撞，碰撞不一定可见。** 原场景桌子/盘子主要是视觉资产，任务必须补齐准确的
   桌面和盘子碰撞体；碰撞圆柱不应直接显示成白色厚盘，应隐藏并让粉色模型与其共位。
7. **检查物体间距时必须包含机器人扫掠体积。** 盘子和苹果不重叠仍不够：盘沿进入张开的
   无名指/小指 6-8 cm 范围，同样会让抓取确定性失败。盘心移到 `(8.19,5.71)` 后，首轮
   立即恢复实际抓起、放盘和归位。
8. **“抬起瞬间成功”不是稳定抓住。** 原先 60 步检查只能发现抓空，发现不了慢滑；延长
   静置后才暴露刚性指尖摩擦不足。应同时检查抬升高度、苹果到掌口距离和延长静置结果。
9. **安全归位需要明确的避障顺序。** `reset.py` 是先肩部侧向外展、再收肘、最后进准备姿势；
   在线控制器曾把前两步写反，手会向前扫桌。阶段值相同但顺序不同，安全性完全不同。
10. **仿真进程生命周期会影响复现。** PhysX 接触状态在失败后可能稳定进入重复失败分支；
    批量验收采用“每次失败重启完整仿真栈”，而 GUI 调试采用独立的仿真终端和一次性控制器。
    中断外层 `docker exec` 不一定会结束容器内子进程，因此批处理必须注册清理 trap，启动前也
    要检查是否有多个控制器同时发布命令。
11. **日志判定必须覆盖完整任务。** 最终成功同时要求：真实位置输入、抓后高度提升、苹果与
    掌口距离合理、静置不滑、落点在盘内、机械臂回准备姿势。只看到手闭合或物体瞬时上升
    不能报告完成。
12. **动作时长和目标保持必须分开设计。** 直接重复发布最终目标会造成开头突跳、到位后假停顿；
    归位应从实时反馈连续插值，并让肩侧抬、肘折叠只控制各自关节。提速优先缩短空中运动和
    已经收敛的重复校正，不能删除闭手接触建立、慢滑验证和释放落稳等与物理正确性相关的等待。

---

## ⭐ 新会话从这里开始读（2026-07-20 更新，最新最权威）

下面这一节是当前真实状态的完整摘要。本文档很长（2000+ 行），下面这节之后的内容是按时间顺序的历史记录，**仅在需要查具体某天的调试细节时再翻**；正常接手只看这一节 + 代码本身就够了。

### 现在要做的任务（用户目标，务必先读）

用户的最终目标是**做一套 ROS 工具链，让同一份控制代码既能操作仿真、也能操作真机**（切 `ROS_DOMAIN_ID` 146↔0）。当前具体任务：Walker C1 在仿真里完成"从 reset.py 准备姿势出发 → 识别桌上苹果的真实位置 → 用 IK 现场规划抓取路径 → 抓起 → 放进盘子 → 归位"。

**验收标准（用户明确）**：只需要保证**苹果位置固定**时成功率比较高；随机位置是锦上添花，不是硬指标。

**安全红线（用户明确，真机安全相关）**：手/指尖任何时候不许碰到桌面。

**✅ 2026-07-20 用户关于 replay 的反馈已按在线 IK 路线解决（以下保留背景）：**

> "虽然成功了，但是和初衷背离了……我需要之前的那种先跑 reset.py 然后在这基础上去做抓取的，我觉得你不能这么 replay，很作弊。"

现状是：`pick_place_replay.py` 用的是**逐帧回放一条录制好的固定轨迹**（示教再现/teach-and-repeat），机器人不看苹果实际在哪，只是机械地重放当年成功那次的关节角度序列。这在仿真里能稳定复现"成功"，**但这是作弊，不是真抓取**——苹果位置只要有一点点偏差这套就完全失效，也完全违背了"先 reset 到位、再现场感知抓取"的初衷。**这个方案已经被用户否决，不要在这个基础上继续优化，要重做。**

### 正确方向：参照 walker_s2 的在线 IK 规划模式

同仓库里 `teleoperation/control/walker_s2/pick_part.py`（+ `pick_part_save_data.py`）是成熟的、已验证的同类任务参照，**新会话开工前必须先读这两个文件**。核心做法：

1. **每一局都实时读取物体当前真实位置**（S2 读 `/sim/part_states`；C1 对应 `/sim/object_state`，已经有了），不是读录像；
2. **每一步用 IK 现场解算**该走到哪个关节角度去够物体，不是回放固定轨迹；
3. **抓取重试在同一进程内做**：抬起来后检查物体有没有真的跟着手一起动（`min_lift_delta` 高度阈值 + `max_part_to_ee_dist` 距离阈值双重判定），没抓住就**重新读物体最新位置**、原地重新规划、重新抓，不需要重启任何东西；
4. S2 的抓取姿态用**球面采样**：绕物体撒一圈候选抓取方向，挑一个 IK 有解、姿态朝下合理的。

C1 侧最终复用并完成的零件：
- `teleoperation/control/walker_c1/robot_controller.py`：ikpy 位置+姿态 IK、掌心 z 安全下限（不碰桌）、`go_ready()` 分阶段归位、`wait_sim_steps()` 仿真步同步（见下方"极重要的技术教训"）；
- `teleoperation/control/walker_c1/pick_place_controller.py`：当前在线 IK 主实现，包含实时物体位置、
  6D IK、罩口闭环、静置保持验证、搬运/释放和安全归位；
- bridge 已经支持 `/sim/cmd_set_object_pose`（仿真专用，命令苹果瞬移到指定世界坐标）和 `/sim/object_state`（回读苹果位置、机器人根位姿、右手各指链节世界坐标）。

**上述方向已完成**：当前实现为 go_ready → 读取 `/sim/object_state` → world/base 变换 →
6D IK 路点 → 罩口闭环 → 五指合围 → 抬升与延长静置双重校验 → 盘子路点 → 释放 →
侧向避障归位。固定位置已经完成端到端验证。

### 极重要的技术教训（不遵守会重新踩坑，务必先读）

1. **仿真时间 ≠ 墙钟时间，一切动作必须按仿真物理步计时，不能按秒睡眠**。这台机器 CPU 物理只能跑 ~28 步/秒（仿真时间流速只有真实时间的 0.28 倍）。如果代码里写 `time.sleep(2.0)` 想让机械臂"用 2 秒缓降"，物理引擎实际只推进了 0.56 秒，等于把所有动作都在物理世界里加速了 3.6 倍执行，抓取瞬间的接触会因此变得粗暴、直接把物体弹飞。**`robot_controller.py` 里的 `wait_sim_steps(k)` 就是为了解决这个问题**：它不数秒钟，而是数仿真物理引擎真正推进了多少步，物理引擎快就少等、慢就多等，永远和物理时间对齐。真机上没有这个"仿真慢速"问题（真机的仿真时间=墙钟时间），`wait_sim_steps` 检测不到步数计数器时会自动退化成普通睡眠，两边代码不用分叉。**新写的抓取逻辑里，所有"等待到位/等待稳定"的地方都必须用 `wait_sim_steps`，绝对不能用 `time.sleep` 或墙钟版的 `spin_for`。**

2. **仿真进程本身会"变质"，只有整个重启才能治好**：反复验证过很多轮，规律很稳定——全新重启的 Isaac 仿真进程,第一次抓取尝试成功率约 65%（不是 100%，这点纠正过一次误判）；但只要某一次尝试失败了，同一个仿真进程后面无论怎么重试都会用几乎一模一样的方式重复失败（失败后物体最终停的坐标精确到小数点后几位都重复）。已确认排除的原因：不是场景复位（`/sim/cmd_reset`）导致的，不是腰部/头部关节漂移，也不是我们自己 ROS 控制脚本进程里的残留状态（换全新进程测试过，一样会退化）。真正原因大概率是 PhysX 物理引擎内部的接触/摩擦缓存状态，具体机制没有继续深挖（性价比已经不高）。**唯一验证有效的应对方案：失败就把整个仿真栈（`sim_runner.py` + bridge + 镜像子进程）杀掉重启，不要在同一个仿真进程里指望"再试一次"能恢复。** 批处理脚本 `ubt_sim/scripts/run_c1_teach_and_repeat_batch.sh` 已经实现了"失败自动重启整个仿真栈再试，最多 N 次"的逻辑，可以直接复用这个重试外壳，把里面调用的 Python 脚本换成新写的在线 IK 版即可。

3. **`ros2` 命令行工具自带的后台发现进程（daemon）会在反复快速重启 ROS 节点后"卡死"，报出 `RuntimeError: !rclpy.ok()` 这种和仿真无关的假故障**，会被误判成"仿真启动失败"。修法：`ros2 daemon stop && ros2 daemon start`，或者干脆不依赖 `ros2` 命令行工具做健康检查，改用一小段原生 `rclpy` 脚本直接订阅话题确认（`run_c1_teach_and_repeat_batch.sh` 里的 `restart_stack()` 函数已经改成这种写法，可以直接抄）。

4. **命令行粘贴多行命令容易被终端断行，断在关键位置会导致命令"看起来在跑但其实跑错了"**（比如 `python3` 后面缺了脚本路径，直接进了 Python 交互界面）。粘贴长命令前确认是完整一行，或者用一个短的 wrapper 脚本文件代替长命令。

5. **容器操作提醒**：`walker-c1-ubt-sim` 是本项目专属容器（不要碰旧的 `ubt-sim` 容器）；仿真 GUI 启动命令去掉 `--headless` 即可看到界面；常见的"端口被占用"报错（`Address already in use`）几乎总是上一次没清理干净的残留进程，用 `pgrep -af 'sim_runner|walker_c1_ros2|zmq_image' | grep -v pgrep | awk '{print $1}' | xargs -r kill -9` 清理后再重启即可。

### 已验证可用、可以直接复用的基础设施（不用重新做）

- **ROS2-ZMQ bridge**（`teleoperation/bridges/walker_c1/`）：SDK 话题面完整可用，身体/手/相机三路回路实测通过，仿真专用扩展话题（挪物体、读物体+机器人+手指链节状态）都在。
- **`robot_controller.py`**：ikpy IK/FK、安全 z 下限、`go_ready()`、`wait_sim_steps()` 全部可用。
- **场景**：桌子/盘子使用独立碰撞体；视觉桌向机器人移动 4 cm 并与桌面碰撞体对齐；
  苹果和盘子任务坐标不变；粉色盘子模型与不可见盘子碰撞体共位；刚体苹果 r=0.027。
- **物理配置**：手部增益保持 10/1/2（力矩 2 N.m 上限）；当前默认使用 mentor 机身/传感器
  与 `walker_c1_force_drive_grip.usd` 灵巧手合并得到的 `walker_astron_v1_sensorKpkd_hands.usd`。
  合并版保留 force-drive 和高摩擦手部材料，解决关节重力下垂与苹果慢滑；不要用加大力矩或
  继续卷曲四指替代。
- **相机**：`parlor.yaml` 里的头部相机朝向已修正（之前有个存在很久的 bug 是朝天花板拍），第一视角俯视桌面正确。
- **in-process 采集器**（`ubt_sim/scripts/collect_walker_c1_pick_place.py`）：这是另一条独立的、纯仿真内部的抓放实现（不走 ROS），保留作为参照和快速验证物理可行性的工具，**不是这次任务要交付的东西**，但它的抓取配方（转腕掌心朝下、闭环罩口对准、五指微张再合围）逻辑是这次 IK 重写的重要参考来源。

### 本节之后的内容

再往下是从项目最早期开始的完整历史记录（URDF 导入、USD 导出、站姿调试、手部重力下垂根因、ROS bridge 搭建过程、示教再现调试的完整账本等），按时间顺序排列，仅供需要时查阅细节，不是接手必读。

---


## Current Workspace

```text
/home/changzhang/VLA/walker_c1
```

Git 分支：

```text
walker_c1
```

当前只改 `ubt_sim` 相关内容。`ubt_IL` 暂时不动。

旧探索目录仍在：

```text
/home/changzhang/VLA/C1-IL-LAB
```

旧目录只作为参考，不再作为主工作区。

## Naming

- `C1 = Astron`。
- 新仓库里统一使用 `walker_c1` 作为目录/代码前缀，和现有 `walker_s2` 对齐。
- 原始 URDF 文件名暂时保留 `walker_astron_*`，因为这是来源资产名。

## User Preference

- 用户希望一步一步慢慢做。
- 每一步执行前需要说明要做什么、为什么做。
- 不要直接大范围改主链路。
- 当前优先级：先跑通 Walker C1/Astron 在 `ubt_sim` 仿真中的基础加载链路，再接 task/controller/ROS。
- 当前只做仿真，不做真机。

## Current Git Status Summary

截至本接力文件更新时，新仓库新增未跟踪内容主要是：

```text
ubt_sim/assets/robots/walker_c1/
ubt_sim/scripts/export_walker_c1_usd.py
ubt_sim/scripts/import_walker_c1_urdf_test.py
ubt_sim/scripts/test_walker_c1_cfg_spawn.py
ubt_sim/source/ubt_sim/devices/walker_c1/
ubt_sim/source/ubt_sim/task/walker_c1_parlor/
ubt_sim/config/walker_c1/
```

注意：运行测试后可能生成：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/__pycache__/
ubt_sim/source/ubt_sim/task/walker_c1_parlor/__pycache__/
```

这是 Python 运行缓存，不是源码改动。

## Commit / Push Status

本地分支：

```text
walker_c1
```

当前本地已完成 commit：

```text
41e2603 Add Walker C1 parlor load-only task
b6e8a57 Add Walker C1 assets and Isaac spawn config
```

当前工作区在 commit 后是 clean 的；如果后续只看到 `C1_HANDOFF.md` 变更，是因为本段接力信息是在 commit 后补写的。

用户尝试首次 push：

```bash
git push -u origin walker_c1
```

失败原因：

```text
remote: Invalid username or token. Password authentication is not supported for Git operations.
fatal: Authentication failed for 'https://github.com/UBTECH-Robot/TienKung-IL-LAB.git/'
```

说明：这不是代码或 commit 问题，是 GitHub HTTPS 不再支持账号密码 push。后续需要任选一种认证方式：

```text
1. 使用 GitHub Personal Access Token 作为 HTTPS password。
2. 改用 SSH remote，例如 git@github.com:UBTECH-Robot/TienKung-IL-LAB.git。
3. 使用 gh auth login 完成 GitHub CLI 认证。
```

认证完成后再执行：

```bash
git push -u origin walker_c1
```

## Docker / Isaac Container State

为了不影响旧的 `ubt-sim` 容器，已新建专用容器：

```text
walker-c1-ubt-sim
```

镜像：

```text
ubt-sim-isaac:latest
```

挂载：

```text
/home/changzhang/VLA/walker_c1/ubt_sim -> /ubt_sim
```

旧容器 `ubt-sim` 仍然挂载旧项目：

```text
/home/changzhang/VLA/TienKung-IL-LAB/ubt_sim -> /ubt_sim
```

所以后续 Walker C1 测试应使用：

```bash
docker exec walker-c1-ubt-sim ...
```

不要误用旧的 `ubt-sim` 容器。

## Assets Done

Walker C1/Astron 资产已经放到：

```text
ubt_sim/assets/robots/walker_c1/
```

关键文件：

```text
ubt_sim/assets/robots/walker_c1/walker_astron_v2.urdf
ubt_sim/assets/robots/walker_c1/walker_astron_v2_fixed.urdf
ubt_sim/assets/robots/walker_c1/walker_astron_v2_hand_v3.urdf
ubt_sim/assets/robots/walker_c1/walker_astron_v2_hand_v3_no_sixforce_mesh.urdf
ubt_sim/assets/robots/walker_c1/walker_c1.usd
```

手部 mesh/URDF：

```text
ubt_sim/assets/robots/walker_c1/meshes/hand_v3/left/
ubt_sim/assets/robots/walker_c1/meshes/hand_v3/right/
ubt_sim/assets/robots/walker_c1/urdf/hand_v3/left/
ubt_sim/assets/robots/walker_c1/urdf/hand_v3/right/
```

## URDF Notes

原版：

```text
walker_astron_v2_hand_v3.urdf
```

当前不能直接可靠导入 Isaac，因为缺两个 mesh：

```text
L_sixforce_link.STL
R_sixforce_link.STL
```

推荐当前使用：

```text
walker_astron_v2_hand_v3_no_sixforce_mesh.urdf
```

这个版本只移除了 `L_sixforce_link` / `R_sixforce_link` 的 visual/collision mesh 引用，保留 link、inertial 和 fixed joint 结构。

## Completed Tests

### 1. URDF Import Smoke Test

新增脚本：

```text
ubt_sim/scripts/import_walker_c1_urdf_test.py
```

运行命令：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/import_walker_c1_urdf_test.py --headless
```

结果：

```text
[OK] Articulation is valid: /walker_astron_v1/root_joint
[INFO] dof_count=53
```

### 2. URDF -> USD Export

新增脚本：

```text
ubt_sim/scripts/export_walker_c1_usd.py
```

运行命令：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/export_walker_c1_usd.py --headless
```

输出：

```text
ubt_sim/assets/robots/walker_c1/walker_c1.usd
```

结果：

```text
[OK] Articulation is valid: /walker_astron_v1/root_joint
[INFO] dof_count=53
[OK] Exported USD: /ubt_sim/assets/robots/walker_c1/walker_c1.usd
```

`walker_c1.usd` 是 USD crate 文件，约 16 MB。

### 3. devices/walker_c1/config.py

新增：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
ubt_sim/source/ubt_sim/devices/walker_c1/__init__.py
```

已定义：

```text
WALKER_C1_USD_PATH
WALKER_C1_URDF_PATH
WALKER_C1_LEFT_ARM_JOINTS
WALKER_C1_RIGHT_ARM_JOINTS
WALKER_C1_LEFT_HAND_JOINTS
WALKER_C1_RIGHT_HAND_JOINTS
WALKER_C1_HEAD_JOINTS
WALKER_C1_WAIST_JOINTS
WALKER_C1_LEFT_LEG_JOINTS
WALKER_C1_RIGHT_LEG_JOINTS
WALKER_C1_UPPER_BODY_JOINTS
WALKER_C1_HOME_POSE
WALKER_C1_CFG
```

关节校验结果：

```text
movable_joint_count=53
home_pose_count=53
home_pose_extra=[]
home_pose_missing_movable=[]
```

说明：`WALKER_C1_HOME_POSE` 正好覆盖 URDF 里的 53 个可动关节。

很多 home pose 值为 `0.0` 是第一版中立姿态，不代表关节不能动。膝盖因为 URDF 下限是 `0.08`，所以设置为：

```text
L_knee_pitch_joint = 0.08
R_knee_pitch_joint = 0.08
```

### 4. WALKER_C1_CFG Spawn Test

新增脚本：

```text
ubt_sim/scripts/test_walker_c1_cfg_spawn.py
```

运行命令：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/test_walker_c1_cfg_spawn.py --headless --device cpu
```

结果：

```text
[OK] robot_spawned=True
[INFO] joint_count=53
[INFO] home_pose_missing_from_spawn=[]
[INFO] spawn_extra_joints=[]
[OK] spawn_smoke_test_passed
```

注意：这个 smoke test 脚本在成功后使用 `os._exit(0)` 直接退出，原因是 Isaac/AppLauncher 在这个最小脚本里关闭阶段会卡住。这个处理只影响测试脚本，不影响正式仿真入口。

### 5. Walker C1 Parlor Load-Only Task

新增：

```text
ubt_sim/source/ubt_sim/task/walker_c1_parlor/__init__.py
ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py
ubt_sim/config/walker_c1/parlor.yaml
```

已注册 task：

```text
UBTSim-WalkerC1-Parlor-v0
```

同时小范围更新：

```text
ubt_sim/source/ubt_sim/__init__.py
ubt_sim/scripts/start_sim.sh
ubt_sim/scripts/sim_runner.py
```

更新目的：

```text
注册 Walker C1 task
start_sim.sh 识别 WalkerC1，不再误走 tienkung_pro
Walker C1 暂时跳过 ROS2-ZMQ bridge
start_sim.sh 自动加入 /ubt_sim/source 到 PYTHONPATH
start_sim.sh 使用 /isaac-sim/python.sh -u 输出实时日志
sim_runner.py 对 Walker C1 非 load-only 明确报错，避免误接旧 controller
```

验证命令：

```bash
docker start walker-c1-ubt-sim
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_LOAD_ONLY=1 timeout 120s bash scripts/start_sim.sh --headless"
```

结果：

```text
[INFO]: Parsing configuration from: ubt_sim.task.walker_c1_parlor.walker_c1_parlor_env_cfg:WalkerC1ParlorEnvCfg
[INFO] Action Manager: <ActionManager> contains 8 active terms.
Active Action Terms (shape: 53)
left_arm_action=7
right_arm_action=7
left_hand_action=11
right_hand_action=11
head_action=2
waist_action=3
left_leg_action=6
right_leg_action=6
[INFO] Observation Manager: policy joint_pos shape=(53,)
[INFO]: Completed setting up the environment...
[INFO] Walker C1 load-only mode: ROS control and action preprocessing are disabled.
[INFO] Load-only app update enabled: physics/action/observation stepping is disabled.
```

说明：

```text
Walker C1 USD 已能在 parlor 场景中通过 Isaac Lab task 正常加载
相机配置未报错
action manager 已识别全部 53 个可动关节
```

### 6. Tiangong Parlor Local Scene Assets

用户打开 GUI 后只看到机器人和空白背景。原因是新 clone 里的：

```text
ubt_sim/assets/scenes/parlor/scene_v2.usd
```

以及 parlor 下很多 USD/PNG 都是 Git LFS pointer 文件，当前机器没有 `git lfs`，所以实际客厅/桌子/水果资产没有拉下来。

为了不把 tracked LFS pointer 文件直接替换成大二进制，已从旧探索目录复制 Tienkung 可见的 parlor 真实资产到本地忽略目录：

```text
/home/changzhang/VLA/C1-IL-LAB/ubt_sim/assets/scenes/parlor/
  -> ubt_sim/assets/local_scenes/tiangong_parlor/
```

并在 `.gitignore` 增加：

```text
ubt_sim/assets/local_scenes/
```

`ubt_sim/config/walker_c1/parlor.yaml` 当前场景路径已改为：

```text
scene:
  usd_path: "local_scenes/tiangong_parlor/scene_v2.usd"
```

重新验证：

```bash
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_LOAD_ONLY=1 timeout 120s bash scripts/start_sim.sh --headless"
```

结果：

```text
环境 setup 完成
Action Manager shape=53
进入 Walker C1 load-only mode
```

注意：日志里仍有少量 MDL/默认贴图 warning，例如 `Clear_Glass.mdl` 和 fruit 默认贴图路径；这不阻止 scene setup。如果 GUI 中水果材质显示异常，后续再单独整理这些材质路径。

## ROS / SDK Notes

SDK 文档显示 Astron/C1 低层 ROS2 接口和当前 `walker_s2` 使用的 SDK message 基本一致：

身体状态：

```text
/mc/sdk/robot_state
mc_state_msgs/RobotState
```

身体控制：

```text
/mc/sdk/robot_command
mc_task_msgs/RobotCommand
JointCmd.MODE_POSITION = 2
```

灵巧手状态：

```text
/mc/left_hand/joint_states
/mc/right_hand/joint_states
sensor_msgs/JointState
```

灵巧手控制：

```text
/mc/left_hand/command
/mc/right_hand/command
mc_task_msgs/JointCommand
```

新仓库已有 vendored ROS message 源码：

```text
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/mc_task_msgs
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/mc_state_msgs
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/shm_msgs
ubt_sim/teleoperation/msgs/walker_sdk_ros2_msgs_src/src/ecat_task_msgs
```

后续重点不是重造 message，而是确认 C1/Astron 的真实 joint order、手部 JointCommand 顺序、相机 topic/type。

## Current Architecture Decision

现在已经从 URDF 路线推进到 USD/task 路线：

```text
URDF import OK -> USD export OK -> WALKER_C1_CFG spawn OK -> Walker C1 parlor load-only task OK
```

后续应按 `walker_s2` 结构继续新增同级目录，而不是覆盖已有文件：

```text
ubt_sim/source/ubt_sim/task/walker_c1_parlor/
ubt_sim/config/walker_c1/parlor.yaml
ubt_sim/teleoperation/bridges/walker_c1/
ubt_sim/teleoperation/control/walker_c1/
```

## Recommended Next Step

下一步建议继续保持小步：

```text
1. 确认 C1/Astron 真实 SDK joint order，尤其身体 RobotCommand 顺序和左右手 JointCommand 顺序。
2. 再按 walker_s2 结构新增 walker_c1 controller/action_process 的最小版本。
3. controller 通过后再新增 teleoperation/bridges/walker_c1/，不要先写 ROS bridge。
```

## Important Do Not Do Yet

暂时不要：

```text
不要直接改旧 ubt-sim 容器
不要覆盖 walker_s2 文件
不要先写 ROS bridge
不要先写 controller
不要动 ubt_IL
不要把原版 walker_astron_v2_hand_v3.urdf 当作默认加载文件
```

当前最稳顺序：

```text
asset -> URDF import -> USD export -> devices config -> CFG spawn test -> task -> controller -> ROS bridge
```

目前已经完成到：

```text
Walker C1 parlor load-only task
```

## 2026-07-10 Update: C1 Controller / ZMQ / Hand Debug

本节是最新状态。前面的部分记录了早期 load-only 阶段；当前已经继续推进到 C1 controller、ZMQ 仿真控制和手部调试。

### Files Changed In This Round

新增：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/action_process.py
ubt_sim/source/ubt_sim/devices/walker_c1/controller.py
```

修改：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/__init__.py
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
ubt_sim/scripts/sim_runner.py
ubt_sim/docker/env.sh
```

文档/参考文件：

```text
C1_joint_map.md
C1_ubt_sim改造顺序清单.md
【CC-API】Astron优必选SDK二次开发文档【对内】.docx
```

### Docker State

`ubt_sim/docker/env.sh` 已把容器名改为：

```text
CONTAINER_NAME="walker-c1-ubt-sim"
```

原因：旧的 `ubt-sim` 容器挂的是旧项目路径，C1 应该使用新容器 `walker-c1-ubt-sim`，避免 `bash run.sh init` / `check` 误操作旧容器。

用户已在新容器里跑过：

```bash
bash run.sh check
```

关键结果：

```text
[OK] Project mounted: /ubt_sim
[OK] Assets directory: /ubt_sim/assets
[OK] Isaac Sim Python: Python 3.11.13
[OK] ubt_sim package: installed
[OK] pyzmq: installed
[OK] numpy: 1.26.0 (< 2)
[OK] ROS2 Humble: installed
[OK] bodyctrl_msgs: installed
[FAIL] Walker SDK ROS2 messages: NOT built (run: bash run.sh init)
```

说明：当前 C1 仿真使用 `UBT_SIM_NO_BRIDGE=1`，暂时不走 ROS bridge，所以 `Walker SDK ROS2 messages` 未 build 不是当前 blocker。后续接真机/ROS bridge 时再处理。

### Controller / ZMQ Status

当前 C1 非 load-only 仿真已能启动 `WalkerC1Controller`：

```text
Command Sub: tcp://127.0.0.1:5655
Status Pub:  tcp://*:5656
Image Pub:   tcp://*:5657
JPEG Pub:    tcp://*:5658
```

推荐调试命令：

```bash
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_NO_BRIDGE=1 timeout 360s bash scripts/start_sim.sh --headless --device cpu --step_hz 30"
```

注意：GPU PhysX 曾出现 CUDA/PhysX allocator 相关问题；当前 debug 推荐用 `--device cpu`。

### Action Mapping Status

C1 当前外部第一版控制输入定义为 26 维：

```text
left_arm[7] + left_hand_sdk[6] + right_arm[7] + right_hand_sdk[6]
```

手部 SDK 6 维顺序来自 Astron 内部 SDK 文档：

```text
left_thumb_swing
left_thumb_mcp
left_index_mcp
left_middle_mcp
left_ring_mcp
left_little_mcp

right_thumb_swing
right_thumb_mcp
right_index_mcp
right_middle_mcp
right_ring_mcp
right_little_mcp
```

仿真内部手是每侧 11 个 revolute joints，所以 `action_process.py` 做了 6D -> 11D 展开：

```text
thumb_swing -> thumb_cmp
thumb_mcp   -> thumb_mpp + thumb_ip
index_mcp   -> index_mpp + index_ip
middle_mcp  -> middle_mpp + middle_ip
ring_mcp    -> ring_mpp + ring_ip
little_mcp  -> little_mpp + little_ip
```

控制器支持：

```text
26-list/tuple/tensor
dict: left_arm/right_arm/left_hand/right_hand/head/waist/left_leg/right_leg
{"walker_c1": ...}
body 或直接 joint-name dict
```

### `to_ros_data()` Note

C1 的 `to_ros_data()` 当前和 S2 一样，暂时不读 `robot.data.joint_vel`，而是发布 zero velocity：

```text
joint_vel = [0.0] * len(joint_names)
```

原因：Isaac Sim / Isaac Lab 在 CUDA 场景下读 velocity tensor 时可能触发 PhysX device/readback 问题，导致 status 发布失败。当前控制闭环主要需要 position feedback，所以先保证 status 稳定。

### Hand Debug Result

已验证：

```text
ZMQ command -> WalkerC1Controller -> action_process -> Action Manager target
```

链路是通的。发送：

```python
{"right_hand": [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]}
```

status 中右手 11 个 sim joints 的 target 都能正确变成 `0.8`。

关键发现：target 正确，但手部实际关节位置跟随很弱，不是命令链路问题。

在把 hand actuator 临时调强到：

```text
stiffness=200
damping=20
effort_limit_sim=50
velocity_limit_sim=10
```

之后，持续发送闭合命令 20 秒，结果仍大致停在：

```text
R_index_mpp_joint  target=0.8 pos=0.336
R_index_ip_joint   target=0.8 pos=0.035
R_middle_mpp_joint target=0.8 pos=0.327
R_middle_ip_joint  target=0.8 pos=0.034
R_ring_mpp_joint   target=0.8 pos=0.336
R_ring_ip_joint    target=0.8 pos=0.047
R_little_mpp_joint target=0.8 pos=0.353
R_little_ip_joint  target=0.8 pos=0.061
```

结论：

```text
1. C1 手部 command mapping 正确。
2. ZMQ/status/controller 链路正确。
3. 问题集中在仿真手模型/actuator/USD 物理驱动上。
4. 只继续改 ZMQ 或 6D->11D 映射没有意义。
```

下一步应重点查：

```text
walker_c1.usd 中右手 joint 的 physics drive / limit 是否正确
URDF -> USD 导出时是否没有正确设置手指关节 drive
手部 collision/约束是否导致关节卡在某个物理平衡点
是否需要重新导出 USD 或单独简化/禁用手部 collision
```

### Current Hand Actuator Params

当前 `config.py` 中 C1 hand actuator 是调试后的强参数：

```text
WALKER_C1_HAND_STIFFNESS = 200
WALKER_C1_HAND_DAMPING = 20
hand effort_limit_sim = 50
hand velocity_limit_sim = 10
```

这比原始值强很多。它让 MPP 关节从约 `0.22` 改善到约 `0.33~0.36`，但 IP 关节仍几乎不动。后续如果发现 USD/drive/collision 根因，可以再把这组参数收敛到更合理值。

### Important Runtime Notes

`robot.data.joint_names` 的顺序和 Action Manager action order 不一样。调试时必须按 joint name 建 map，不能按 index 对齐。

当前 Action Manager order 中右手段是：

```text
R_index_mpp_joint
R_little_mpp_joint
R_middle_mpp_joint
R_ring_mpp_joint
R_thumb_cmp_joint
R_index_ip_joint
R_little_ip_joint
R_middle_ip_joint
R_ring_ip_joint
R_thumb_mpp_joint
R_thumb_ip_joint
```

而 `robot.data.joint_names` 是 Isaac 内部顺序。status debug 必须使用：

```python
pos = dict(zip(status["joint_names"], status["joint_pos"]))
target = dict(zip(status["target_joint_names"], status["target_joint_pos"]))
```

### Validation Done

只读语法检查已通过：

```text
OK ubt_sim/source/ubt_sim/devices/walker_c1/config.py
OK ubt_sim/source/ubt_sim/devices/walker_c1/action_process.py
OK ubt_sim/source/ubt_sim/devices/walker_c1/controller.py
```

普通 `python3 -m py_compile` 在宿主侧会因为 `__pycache__` 权限报错：

```text
Permission denied: ubt_sim/source/ubt_sim/devices/walker_c1/__pycache__/...
```

这是容器/root 生成缓存导致的文件权限问题，不代表源码语法错误。

### Current Process State

手部 debug 用的 headless CPU sim 已停止。当前没有残留：

```text
/ubt_sim/scripts/sim_runner.py
```

如果后续 `timeout` 后残留 Isaac Python 进程，先查：

```bash
docker exec walker-c1-ubt-sim pgrep -af /ubt_sim/scripts/sim_runner.py
```

只杀精确 PID，例如：

```bash
docker exec walker-c1-ubt-sim kill -9 <pid>
```

不要对整个容器做宽泛 `pkill -f`。

## Morning Standup Summary For 2026-07-11

可以这样汇报：

```text
昨天我把 C1/Astron 从 load-only 推到了可通过 ZMQ 控制的仿真链路。

具体完成了三件事：
1. 新增了 C1 的 action_process 和 controller，sim_runner 已能启动 WalkerC1Controller。
2. 对齐了 Astron SDK 文档里的手部 6 维命令顺序，并实现了 6D SDK hand command 到仿真 11D hand joints 的映射。
3. 做了手部闭环 debug，确认 ZMQ 命令、Action Manager target 和 status 发布链路都是通的。

当前发现的问题是：手部 target 已经正确写入，但实际仿真关节跟随很弱。即使用更强 actuator 参数持续发闭合命令，MCP/MPP 只能动到约 0.33，IP 关节几乎不动。因此问题不在 ZMQ 或映射，而更可能在 C1 USD 资产的 joint drive、URDF->USD 导出参数或手部碰撞/物理约束上。

今天下一步我会集中查 walker_c1.usd 里手部关节的 drive/limit/collision 配置，必要时重新导出或修正 C1 USD。修完后再继续推进 ROS bridge/真机接口，不会先碰真机。
```

如果要更短，可以说：

```text
C1 仿真控制链路昨天已经跑通到 ZMQ controller，手部 6D 到 11D 映射也验证正确。当前 blocker 是手部物理模型：target 到了，但实际关节卡住或跟随很弱。下一步查 USD 里的手部 drive/collision，必要时重新导出 USD，然后再推进 ROS bridge。
```

## Immediate Next Step

下一步建议只做一件事：

```text
用 Isaac/pxr 工具读取 walker_c1.usd 中右手 11 个 hand joints 的 physics drive、limit、joint type、collision API 信息。
```

目标是回答：

```text
为什么 target=0.8 后 IP joints 基本不动？
是 USD drive 没设置/被覆盖？
是关节 limit 或单位转换问题？
是 collision/约束把手指卡住？
```

确认根因后，再决定：

```text
1. 只改 ArticulationCfg actuator 参数；
2. 重新导出 walker_c1.usd；
3. 修改导出脚本参数；
4. 单独处理手部 collision/drive。
```

## 2026-07-10 Update: Hand Gravity Root Cause / Temporary Fix

已完成上一节的 immediate next step，并继续做了运行时隔离测试。

### Added Debug Scripts

新增两个只读/探针脚本：

```text
ubt_sim/scripts/inspect_walker_c1_usd.py
ubt_sim/scripts/probe_walker_c1_hand_runtime.py
```

用途：

```text
inspect_walker_c1_usd.py
  读取 walker_c1.usd 中手部 joint 的 drive、limit、body mass/inertia、collision API。

probe_walker_c1_hand_runtime.py
  直接 spawn WALKER_C1_CFG，绕过 ZMQ/action manager，测试手部 actuator 运行时参数和关节响应。
```

### USD Inspection Result

右手 11 个 joints 在 USD 中都有：

```text
type = PhysicsRevoluteJoint
axis = Z
PhysicsDriveAPI:angular
PhysxJointAPI
IsaacJointAPI
```

limit 从 URDF 正确转换成 USD 角度值，例如：

```text
R_index_mpp_joint lower=0 upper=83.6518 deg  # 约 1.46 rad
R_index_ip_joint  lower=0 upper=92.8192 deg  # 约 1.62 rad
```

USD 里 hand drive 仍保留 URDF effort：

```text
stiffness=625
damping=0
max_force=1.35
type=acceleration
```

但 Isaac Lab 运行时 `config.py` 的 hand actuator 覆盖是生效的：

```text
stiffness=200
damping=20
effort_limit_sim=50
velocity_limit_sim=10
```

hand link mass/inertia 也正常，没有明显异常大质量：

```text
R_index_mpp_link mass ~= 0.0147 kg
R_index_ip_link  mass ~= 0.0133 kg
R_palm_link      mass ~= 0.3822 kg
```

runtime 还确认：

```text
num_fixed_tendons = 0
joint_armature = 0
joint_friction_coeff = 0
joint_dynamic_friction_coeff = 0
joint_viscous_friction_coeff = 0
```

所以当前已排除：

```text
1. ZMQ/action mapping 问题
2. Action Manager target 顺序问题
3. USD joint limit 单位问题
4. hand actuator config 没写入的问题
5. fixed tendon / friction / armature hidden constraint 问题
6. hand link mass/inertia 明显异常问题
```

### Root Cause Found

关键对照测试：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8
```

在 gravity 开启时，即使直接对右手 11 个 joints 设置 target=0.8，非拇指关节仍明显跟随失败，effort 会打满：

```text
R_index_mpp_joint target=0.8 pos ~= 0.116
R_index_ip_joint  target=0.8 pos ~= 0.000
applied_effort    ~= 50 saturated
```

把 effort/stiffness 临时加到更高只能部分改善，但仍无法稳定到 target：

```text
--hand-effort 500 --hand-stiffness 1000 --hand-damping 40
R_index_mpp_joint pos ~= 0.447
R_index_ip_joint  pos ~= 0.125
```

禁用 robot gravity 后，同一个 target=0.8 能稳定跟随：

```text
R_thumb_cmp_joint  pos=0.806
R_thumb_mpp_joint  pos=0.801
R_thumb_ip_joint   pos=0.799
R_index_mpp_joint  pos=0.799
R_index_ip_joint   pos=0.797
R_middle_mpp_joint pos=0.799
R_middle_ip_joint  pos=0.798
R_ring_mpp_joint   pos=0.799
R_ring_ip_joint    pos=0.799
R_little_mpp_joint pos=0.800
R_little_ip_joint  pos=0.799
```

结论：

```text
当前 C1/Astron hand 在 Isaac 中的主要问题是 gravity 下的手指驱动不足/下垂，不是控制链路或 USD joint metadata 错误。
```

### Applied Temporary Fix

已修改：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
```

C1 `ArticulationCfg.spawn.rigid_props` 现在为：

```python
sim_utils.RigidBodyPropertiesCfg(disable_gravity=True)
```

理由：

```text
C1 当前阶段是 fixed-root upper-body simulation，目标是先跑通 ZMQ/controller/task 链路。
开启 gravity 时手部姿态无法跟随，且会误导后续任务调试。
禁用 C1 robot gravity 后，手部 target 跟随稳定，物体/场景 gravity 不受影响。
```

保留当前 hand actuator 调试参数：

```text
WALKER_C1_HAND_STIFFNESS = 200
WALKER_C1_HAND_DAMPING = 20
hand effort_limit_sim = 50
hand velocity_limit_sim = 10
```

不要立刻降回原始 `10/2/2`；禁用 gravity 后低参数仍不能稳定驱动所有 IP joints。

### Validation After Fix

已运行：

```bash
PYTHONPYCACHEPREFIX=/tmp/c1_pycache python3 -m py_compile \
  ubt_sim/source/ubt_sim/devices/walker_c1/config.py \
  ubt_sim/scripts/inspect_walker_c1_usd.py \
  ubt_sim/scripts/probe_walker_c1_hand_runtime.py
```

结果：通过。

已运行默认配置探针：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8
```

结果：右手 11 个关节全部稳定到约 `0.797~0.806`，证明 `config.py` 里的 `disable_gravity=True` 已生效。

### Recommended Next Step

下一步可以回到 C1 ZMQ 仿真链路，重新跑：

```bash
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_NO_BRIDGE=1 timeout 360s bash scripts/start_sim.sh --headless --device cpu --step_hz 30"
```

然后发送：

```python
{"right_hand": [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]}
```

预期：

```text
status target_joint_pos 和实际 joint_pos 都应接近 0.8。
```

如果这个验证通过，再继续推进：

```text
1. 清理/保留 debug scripts 的取舍；
2. C1 task/controller 的更完整 smoke test；
3. 后续 ROS bridge 或数据采集链路。
```

## 2026-07-10 Update: Mimic Ratio Trial With Gravity Enabled

用户指出 Tiankung/S2 都没有禁用 gravity。已确认：

```text
walker_s2/config.py      disable_gravity=False
tienkung_pro/config.py   disable_gravity=False
```

因此 C1 也已改回：

```text
ubt_sim/source/ubt_sim/devices/walker_c1/config.py
disable_gravity=False
```

尝试过“主关节 + mimic ratio”方向，但只在 probe 脚本里做测试，没有写入正式 ZMQ 映射。

新增/扩展 probe 参数：

```text
--ip-ratio
--thumb-ip-ratio
```

测试 1：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8 --ip-ratio 0.5 --thumb-ip-ratio 0.5
```

结果：非拇指 MPP 仍然只有约 `0.11~0.15`，IP 仍接近 `0`：

```text
R_index_mpp_joint  target=0.8 pos=0.1167
R_index_ip_joint   target=0.4 pos≈0.0
R_middle_mpp_joint target=0.8 pos=0.1076
R_middle_ip_joint  target=0.4 pos≈0.0
R_ring_mpp_joint   target=0.8 pos=0.1224
R_little_mpp_joint target=0.8 pos=0.1475
```

测试 2：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/probe_walker_c1_hand_runtime.py --headless --device cpu --steps 600 --target 0.8 --ip-ratio 0.0 --thumb-ip-ratio 0.0
```

结果：只驱动主关节也没有明显改善：

```text
R_index_mpp_joint  target=0.8 pos=0.1168
R_middle_mpp_joint target=0.8 pos=0.1077
R_ring_mpp_joint   target=0.8 pos=0.1225
R_little_mpp_joint target=0.8 pos=0.1473
```

结论：

```text
mimic ratio 是以后让手指动作更自然的合理映射方式，但它不能解决当前 gravity 开启时非拇指 MPP 主关节本身就抬不起来的问题。
所以暂时不要把 mimic ratio 写进 C1 正式 action_process.py。
```

当前更可信的后续方向：

```text
1. 保持 C1 disable_gravity=False，和 Tiankung/S2 一致；
2. 不靠 mimic ratio 修 gravity 下垂；
3. 下一步查手指关节 frame/axis、手部姿态相对 gravity 的方向、是否需要只对 hand links 做重力补偿或修 USD/URDF 物理参数。
```

## 2026-07-10 Update: C1 Reset Semantics

这里明确区分两种 reset：

```text
1. 按 R / ZMQ {"reset": true}
   这是 Isaac 场景 reset：sim_runner.py 执行 env.sim.reset(); env.reset()。
   用途是把仿真状态回到 config.py 里的 WALKER_C1_HOME_POSE / 0 位。

2. ubt_sim/teleoperation/control/walker_c1/reset.py
   这是抓取任务开始前的 task reset pose。
   用途是把 C1/Astron 摆到当前 parlor/tabletop 抓取任务的初始姿态。
```

新增文件：

```text
ubt_sim/teleoperation/control/walker_c1/__init__.py
ubt_sim/teleoperation/control/walker_c1/constants.py
ubt_sim/teleoperation/control/walker_c1/reset.py
```

`constants.py` 里的 `TASK_RESET_BODY_POSE` 参考 Tiankung `ARM_HOME_PICK_PLACE` 的思路：

```text
头部先给任务视角；
手部张开；
双臂先经过 clear pose 避碰；
最后发布抓取任务初始姿态。
```

默认真机/ROS 用法：

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/reset.py
```

仿真 ZMQ 任务姿态用法：

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/reset.py --mode sim-task
```

如果只想做“按 R 那种场景 reset”，不要用默认 `reset.py`，用键盘 R；脚本里仅保留显式 fallback：

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/reset.py --mode sim-scene
```

注意：当前 `TASK_RESET_BODY_POSE` 只是按 Tiankung 抓放姿态语义映射到 C1 关节名的第一版，需要后续根据 C1 在 parlor 场景里的实际抓取高度、桌面位置、相机视角继续调。

## 2026-07-10 Update: C1 Action Offset Trial Reverted

用户反馈 C1 启动仿真后脚/初始姿态不对。排查结论：

```text
WALKER_C1_HOME_POSE 最近提交没有改腿部姿态；
腿部自然站姿仍是 hip/ankle=0，knee=0.08；
parlor.yaml root 高度仍是 z=0.90，和 MJCF base_link pos=0.91 基本一致。
```

试过一个假设：C1 controller/action_process 输出的是绝对 joint target，而 `walker_c1_parlor_env_cfg.py` 里除了手部以外，arm/head/waist/leg 没有显式 `use_default_offset=False`，可能导致 default offset 与绝对 target 叠加。

但用户反馈重启后腿部仍不对/疑似更差，所以该试验已撤回，当前不要保留这个改动。

```text
ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py
```

当前状态：

```text
只有 hand action 保持 use_default_offset=False，和 Walker S2 写法一致；
arm/head/waist/leg action 未显式设置 use_default_offset。
```

下一步不要继续猜 offset；应优先确认：

```text
1. 当前启动是否仍带 UBT_SIM_LOAD_ONLY=1；
2. 是否有多个 Isaac/sim_runner 残留进程；
3. load-only 下脚是否正常；
4. 非 load-only controller 启动后脚是否变化；
5. 如果 load-only 也飘，重点查 parlor.yaml 的 robot init_state.z / 场景地面高度 / USD root 定义。
```

## 2026-07-10 Update: 41e2603 Confirmed As Good Standing Baseline

用户临时切到：

```text
41e260371fe96206903a9805fe2bd5538c26b7a3
```

并用昨天的 load-only 命令观察，确认该状态下 C1/Astron 初始站姿自然：

```text
腿部正常站着；
脚尖朝前；
身体挺直。
```

因此用户之前关于“昨天看到的姿态是正常的”的记忆是有效 baseline，不应再假设用户看错。

随后用户已执行：

```bash
git switch walker_c1
git stash pop
```

当前回到：

```text
walker_c1
```

`stash pop` 成功，恢复了之前未提交改动，并删除了 `refs/stash@{0}`。

当前工作区仍有未提交内容：

```text
M  C1_HANDOFF.md
?? ubt_sim/teleoperation/control/walker_c1/
?? 【CC-API】Astron优必选SDK二次开发文档【对内】.docx
```

注意：`.docx` 是本地 SDK 文档参考文件，不要默认加入 git。

### Verified Diff From Good Baseline To walker_c1

已对比 `41e2603` 和当前 `walker_c1` 分支：

```bash
git diff --name-status 41e260371fe96206903a9805fe2bd5538c26b7a3..walker_c1 -- \
  ubt_sim/assets/robots/walker_c1 \
  ubt_sim/source/ubt_sim/devices/walker_c1 \
  ubt_sim/source/ubt_sim/task/walker_c1_parlor \
  ubt_sim/config/walker_c1 \
  ubt_sim/scripts
```

结果关键点：

```text
C1 robot assets 没变：
  ubt_sim/assets/robots/walker_c1/*.usd / *.urdf / *.xml 没有 diff

parlor task 配置没变：
  ubt_sim/config/walker_c1/parlor.yaml 没有 diff
  ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py 没有 diff

C1 HOME_POSE 腿部没变：
  hip/ankle 仍是 0
  knee 仍是 0.08

gravity/root 没变：
  disable_gravity=False
  fix_root_link=True
```

相对 good baseline，`walker_c1` 分支相关变化主要是：

```text
1. sim_runner.py 允许 Walker C1 非 load-only 启动 WalkerC1Controller；
2. 新增 ubt_sim/source/ubt_sim/devices/walker_c1/action_process.py；
3. 新增 ubt_sim/source/ubt_sim/devices/walker_c1/controller.py；
4. 新增 hand debug/inspect 脚本；
5. config.py 只改了 hand actuator 参数。
```

`config.py` 中唯一物理配置差异是手部 actuator 从原始弱参数改成调试强参数：

```text
41e2603:
  WALKER_C1_HAND_STIFFNESS = 10
  WALKER_C1_HAND_DAMPING = 2
  hand effort_limit_sim = 2
  hand velocity_limit_sim = 3

walker_c1:
  WALKER_C1_HAND_STIFFNESS = 200
  WALKER_C1_HAND_DAMPING = 20
  hand effort_limit_sim = 50
  hand velocity_limit_sim = 10
```

理论上手部 actuator 不应影响腿部站姿，但从 git diff 看，这是当前分支和 good baseline 之间唯一会影响 C1 articulation 物理配置的变更。事实优先，后续应做最小 A/B 验证。

### Load-only Command Status

用户昨天命令：

```bash
docker exec -it walker-c1-ubt-sim bash -lc \
  "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_LOAD_ONLY=1 bash scripts/start_sim.sh"
```

已确认当前脚本仍会正确展开成：

```text
/isaac-sim/python.sh -u /ubt_sim/scripts/sim_runner.py \
  --task UBTSim-WalkerC1-Parlor-v0 \
  --enable_cameras \
  --num_envs 1 \
  --load_only \
  --device cpu
```

短启动验证日志中确实出现：

```text
[INFO] Walker C1 load-only mode: ROS control and action preprocessing are disabled.
[INFO] Load-only app update enabled: physics/action/observation stepping is disabled.
```

所以 `UBT_SIM_LOAD_ONLY=1` 本身没有失效。

判断标准：

```text
看到 "Walker C1 load-only mode" => load-only 生效；
看到 "Walker C1 Controller: simulation ZMQ interface enabled" => 当前不是 load-only，或者另有非 load-only 进程。
```

load-only 下 `reset.py --mode sim-task` 不会生效，因为 load-only 不启动 `WalkerC1Controller`，也不会监听 ZMQ 命令。

### Current Best Next Step For Standing Pose

当前不要再大范围猜 gravity、root height、parlor init pose 或 action offset；这些和 good baseline 的 diff 不匹配。

建议下一步只做一个最小验证：

```text
在 walker_c1 分支，把 hand actuator 参数临时恢复成 41e2603 的值；
重新跑 load-only；
看腿部/身体初始姿态是否恢复自然。
```

如果恢复自然：

```text
说明强 hand actuator 对整机 articulation 初始稳定性产生了副作用；
应拆分手部跟随修复，不要直接保留强参数进主配置。
```

如果仍不自然：

```text
说明差异可能不在 git-tracked 源码，需检查：
1. 容器内是否真挂载当前工作区；
2. 是否有残留 Isaac 进程；
3. 是否有未跟踪/生成的 USD 或缓存状态；
4. 视觉观察是否在同一视角/同一启动模式下比较。
```

### Branch Safety

`41e2603` 是 detached HEAD baseline，只用于对照观察，不要在 detached HEAD 上继续开发。

继续开发应回到：

```bash
git switch walker_c1
```

如果需要永久保留 good baseline，可新建分支：

```bash
git switch -c walker_c1_good_baseline 41e260371fe96206903a9805fe2bd5538c26b7a3
```

## 2026-07-14 Update: 站姿根因修复 + 左腿碰撞（含一条死胡同）

本节是最新、最重要的状态。上面很多"站姿不对/内八"的猜测在这一轮被**数据推翻并定案**了。

### 一句话总览

```
站姿核心问题已修好并 push（commit f4a4c2e）：
  1. C1 actuator 刚度过弱 -> 对齐 S2；
  2. controller 把 reset 乱帧当保持目标 -> 改为锚定 HOME_POSE。
剩一个纯视觉小尾巴：左腿在 parlor 里撞家具被拧 ~35°（腿是固定基座、不参与任务）。
关腿碰撞的"运行时"做法被证明是死胡同（见下），已回退，sim 恢复正常。
```

### 关键结论 1：站姿"不对"不是代码回归，"脚尖朝前的好版本"= 关了重力

用 `dump_walker_c1_joint_state.py` 在 `41e2603`（用户记忆里"好"的 baseline）和 `dc2fc83`（当时 HEAD）各跑一次 load-only：

```
两次 env.reset() 后的 53 个关节值逐字节完全相同。
```

含义：

```
- load-only 下这两个 commit 在所有影响姿态的输入上等价，必然渲染成同一姿态。
- 用户记忆里"41e2603 脚尖朝前"其实是 41e2603 + 未提交的 disable_gravity=True
  （那次实验后来 revert 了）。commit 哈希是障眼法，真正区别是"重力关 vs 开"。
```

### 关键结论 2：load-only 不能用来判断站姿（physics 没跑）

```
sim_runner.py 的 load-only 分支只调 simulation_app.update()（纯渲染），
从不调 env.step()，所以 physics 不推进，机器人冻结在 reset 后那一帧。
改 actuator 刚度对 load-only 姿态零影响（实测 0 步 dump 改刚度前后逐字节相同）。
=> 判断站姿必须用 controller 模式（env.step 在跑），或用 dump 脚本 --steps 让物理 settle。
```

### 关键结论 3：站姿根因 = C1 刚度是没调过的占位弱值

对照 `walker_s2/config.py`（重力下能站住的成熟参考）：

```
部位   S2 stiffness        C1 修复前   C1 修复后(=对齐S2)
腿 hip_roll/yaw   1100     200        1100
腿 hip_pitch/knee 1500     200        1500
腿 ankle          1600     200        1600
头 head            600      80         600
腰 waist           600     120         600
臂 arm         500~600      80      500~600
手 hand      (夹爪1200)    200       200(未动，另一条线)
```

修复（`config.py`，已提交 f4a4c2e）：把臂/腰/腿/头刚度阻尼对齐 S2 量级，阻尼腿 55/65/70、头腰 60、臂 40。**手部刚度未动。**

### 关键结论 4：controller "拿 reset 乱帧当目标"是第二个 bug（已修）

现象：刚度提上去后，controller 模式启动时**左腿平飞到侧面、右腿甩到头顶**。

根因（`action_process.py` 的 `to_controller_data`）：

```
原逻辑：_hold_joint_targets 首次从"当前关节位置"捕获（好意：启动先 hold 当前位，别猛跳）。
但 env.reset() 后"当前位置"正是那一帧乱姿态（L_hip_roll≈2.94、R_hip_pitch≈2.5）。
弱刚度时电机够不到乱目标，只是软塌（看不出）；强刚度时电机有劲，
就把腿死命甩到乱目标：L_hip_roll 2.94->左腿平展、R_hip_pitch 2.5->右腿到头。
```

修复（`action_process.py`，已提交 f4a4c2e）：

```python
# 启动保持目标锚定 HOME_POSE，而不是读乱掉的当前位置
_hold_joint_targets = {name: WALKER_C1_HOME_POSE.get(name, 0.0) for name in action_joint_names}
```

验证（走**真实 controller 动作路径** `to_controller_data + env.step` 300 步）：

```
头 head_pitch 0.065、腰≈0、双臂<0.05、右腿全部≈0。
max|diff|=0.81 落在 L_thumb_mpp（手，未动）。
=> 头/躯干/臂/右腿全部 hold 住 HOME_POSE。
```

### 关键结论 5：reset 会甩腿的机制（为什么左右腿路径不同）

```
机器人固定在桌边(root z=0.9, fix_root_link=True)，腿垂下来时和 parlor 家具几何重叠；
PhysX 初始化时猛推消穿模 -> reset 出不对称乱帧（左腿甩得比右腿远）；
电机往 HOME 拉时，两腿从不同乱起点绕回：左腿"往前绕"路径扫过桌子、被别住；
右腿"往后绕"路径是空的、干净归位。
```

### 关键结论 6：左腿残留 = parlor 场景碰撞（已坐实，非机器人问题）

剩余现象：修好后左腿仍拧 `L_hip_yaw≈0.6`（~35°），右腿完美(<0.13)。诊断：

```
- 隔离测试 probe_walker_c1_leg_isolation.py（机器人单独 spawn、无场景）：
    两腿完美对称，max|L|-|R|=0.0000  => 排除机器人/刚度/控制，锁定"场景碰撞"。
- 定位 probe_walker_c1_leg_collision.py（parlor 内，dump 左右腿连杆世界坐标）：
    两脚同高 Z≈0.05（左脚没踩在桌面上），但左脚在 +Y 被撇出去(左+0.16 vs 右-0.10)
    => 左脚/左小腿在脚踝高度被侧向顶（大概率桌腿），右腿路径干净。
- 脚是悬空的（fix_root_link=True，上半身操作标准做法），所以不是穿地。
- URDF 里左右 hip_yaw 限位是正常镜像对，0 在合法范围内 => 不是关节限位。
```

### 死胡同（重要，别再走）：运行时关腿碰撞

目标：腿是固定不用的，关掉腿↔场景碰撞即可根治左腿被别。做了开关设计（默认关）：

```
config.py:   WALKER_C1_DISABLE_LEG_COLLISION = True
env 覆盖:     UBT_SIM_C1_DISABLE_LEG_COLLISION=0  (打开腿碰撞)
helper:      scene_setup.py  apply_leg_collision_setting(env) / disable_leg_collision(env)
```

尝试与失败链：

```
1. env.sim.stage 遍历 -> 找到 0 个碰撞 prim（错的 stage 句柄）。
   正解：omni.usd.get_context().get_stage()。碰撞 prim 在
   /World/envs/env_0/Robot/<link>/collisions/<link>/mesh（12 个腿部）。
2. 直接改碰撞属性 -> 报错 "authoring to an instance proxy is not allowed"
   （腿连杆子树是 USD 实例代理，不能直接改）-> 崩、机器人卡死。
3. 先 SetInstanceable(False) 去实例化再改 -> 能关掉 12 个、不报那个错，
   但 (a) 仿真变得极慢（120 步 9 分钟都跑不完，正常 300 步 5 分钟），
   (b) env.reset() 崩："Simulation view object is invalidated ...
       Failed to set DOF actuation forces"。
   => 结论：sim play 之后再改 USD（去实例化/碰撞）会让 PhysX 视图失效。
      运行时关碰撞在原理上走不通，不是调参能救的。
```

处置：

```
- 已从 sim_runner.py 撤回 apply_leg_collision_setting 调用，sim 恢复正常（能启动、不崩）。
- scene_setup.py / config.py 的 WALKER_C1_DISABLE_LEG_COLLISION flag / inspect 脚本
  仍留在工作区但【未提交】，标记为死胡同实验，勿再用运行时路径。
```

### 左腿的正确修法（尚未做）

```
在 sim 加载之前把腿碰撞关掉，而不是运行时改。方案：离线烤一个
walker_c1_no_leg_collision.usd（打开 walker_c1.usd，把 12 个腿部
collision mesh 的 collisionEnabled 设 False，另存），config 按
WALKER_C1_DISABLE_LEG_COLLISION 选择加载哪个 USD。
优点：解析时碰撞就是关的，PhysX 从头认，不崩、不慢，仍可切回原 USD 打开腿碰撞。
（编辑源 USD 时 prim 不是实例代理，可直接改，无 instance proxy 问题。）
```

或者：左腿纯视觉、腿不干活，**先搁置**当已知小尾巴也完全可以。

### 本轮 commit / 文件状态

已提交并 push（remote 已是 SSH `git@github.com:UBTECH-Robot/TienKung-IL-LAB.git`，push 通）：

```
f4a4c2e Fix Walker C1 standing pose: S2-aligned gains + HOME_POSE hold targets
  - config.py（刚度对齐 S2）
  - action_process.py（锚定 HOME_POSE）
  - scripts/dump_walker_c1_joint_state.py（新，只读诊断）
  - scripts/probe_walker_c1_leg_isolation.py（新）
  - scripts/probe_walker_c1_leg_collision.py（新）
1db36ba Add Walker C1 teleoperation reset scaffolding and handoff notes
  - teleoperation/control/walker_c1/{__init__,constants,reset}.py
  - C1_HANDOFF.md
```

工作区【未提交】（多为死胡同实验，谨慎处理）：

```
M dump_walker_c1_joint_state.py         (加了 --use-controller / --disable-leg-collision 开关)
M config.py                             (加了 WALKER_C1_DISABLE_LEG_COLLISION flag)
?? scene_setup.py                       (运行时关碰撞 helper —— 死胡同)
?? scripts/inspect_c1_collision_prims.py(找碰撞 prim 的探针)
?? 【CC-API】...SDK...【对内】.docx      (对内文档，勿入 git)
```

### 诊断脚本速查（都是只读/探针）

```
dump_walker_c1_joint_state.py
  建 parlor env、reset 后 dump 53 关节 vs HOME_POSE。
  --steps N            reset 后驱动 HOME_POSE 目标 step N 步再 dump（=物理 settle）
  --use-controller     走真实 controller 动作路径(to_controller_data+env.step) 而非手喂
  --disable-leg-collision  调用 scene_setup 关腿碰撞（死胡同，会崩，勿用于正式）
probe_walker_c1_leg_isolation.py  机器人单独 spawn（无场景），验左右腿对称
probe_walker_c1_leg_collision.py  parlor 内 dump 左右腿连杆世界坐标，定位撞点
inspect_c1_collision_prims.py     打印 body 名 + 碰撞 prim 路径（用 omni.usd stage + 实例代理遍历）
```

运行例（headless，注意这台机器 Isaac 启动+300步约 5 分钟）：

```bash
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u \
  /ubt_sim/scripts/dump_walker_c1_joint_state.py \
  --headless --device cpu --enable_cameras --steps 300 --use-controller
```

GUI 看站姿（controller 模式，物理在跑）：

```bash
docker exec -it walker-c1-ubt-sim bash -lc \
  "cd /ubt_sim && UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 UBT_SIM_NO_BRIDGE=1 \
   bash scripts/start_sim.sh --device cpu --step_hz 30"
```

### 运行时踩坑备忘

```
- 判断站姿别用 load-only（physics 不跑，冻在 reset 乱帧）；用 controller 模式。
- env.sim.stage 看不到 spawn 出来的碰撞 prim；要用 omni.usd.get_context().get_stage()。
- 机器人腿连杆是 USD 实例代理，不能直接改属性。
- sim play 之后改 USD（碰撞/实例化）会让 PhysX tensor view 失效 -> env.reset() 崩。
- 这台机器 Isaac 单次 headless（启动+300 步+相机）约 5 分钟，多实例会更慢/抢显存；
  timeout/残留进程用 pgrep -af 精确杀 PID，别宽泛 pkill。
```

### 下一步建议

```
1.（可选）左腿：走"烤 USD"正解，或先搁置（纯视觉、腿不用）。
2. 把未提交的死胡同实验清理或明确标注后再决定去留。
3. 回主线：确认 C1/Astron 真实 SDK joint order（身体 RobotCommand 顺序、
   左右手 JointCommand 顺序），这才是影响真机的关键；仿真物理调参不影响真机。
4. 手部 droop 仍是独立未结项（拇指等在重力下 hold 不住，disable_gravity 时正常）。
```

## 2026-07-14 Update（下午）：目标转仿真数据采集 + 上肢重力 droop 根因定案

本节是最新、最重要的状态。**注意：上面几节把"手部 droop / 手臂 hold 不住"当独立未结项，这一轮把它们
一锅端定案了——根因是 USD 关节 drive 是 acceleration 型，不是 force 型。**

### 一句话总览

```
1. 目标变了：用户明确"暂不做真机，先在仿真里跑通规划数据采集，只需要数据生成 HDF5"。
   （LeRobot 转换 out of scope；ROS bridge / 真机搁置。）
2. M1 已完成：脚本化动作 -> 录制 -> HDF5（26 维 obs/action + camera_head），已验证。
3. M2（真实抓取）挖出并定案了 droop 根因：
   C1 所有关节 drive 是 type=acceleration -> 等效力矩刚度 = stiffness×inertia ≈ 0
   -> 上肢重力下没劲、垂下去。改成 force 驱动后彻底 hold 住（已验证）。
4. 正式修法（烤 force-drive USD）脚本已写好，尚未执行（等用户过一遍再动）。
```

### 目标转向（重要）

用户新方向：**在仿真里跑通"脚本化运动规划 + 数据采集"，产物就是 HDF5**。
- 采集链路对标 `ubt_IL/dataset/sim_pick_place`（天工采的：26 维 obs/action + `camera_head`，fps15）。
- **只要 HDF5**：LeRobot 转换（`convert_to_lerobot.py` + 配置 `Walker_C1_26_1RGB.json`）备着但不跑。
- 驱动方式用户选定：**脚本化运动规划**（自动抓放、批量录，无人 teleop），后来进一步要求**真实物理抓取**（不接受吸附/关重力）。

### M1 已完成：HDF5 采集链路（in-process，不走 ZMQ/ROS）

新脚本 `ubt_sim/scripts/collect_walker_c1_pick_place.py`：
- 直接建 `UBTSim-WalkerC1-Parlor-v0` env（非 load-only，物理在跑），脚本化右臂波点 + 逐帧录制。
- 产出 `ubt_sim/dataset/walker_c1/<ts>/trajectory.hdf5`，schema 与天工一致
  （`puppet/*` + `action/*` + `camera_observations/color_images/camera_head` JPEG）。
- 已验证：`observation.state`=26、`action`=26、相机解码 (480,640,3)。复用 `action_process.py`
  的 `to_controller_data`/`to_ros_data`。
- 运行：`docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/collect_walker_c1_pick_place.py --headless --device cpu --enable_cameras --episodes 1`

### M2（真实抓取）路上发现的场景问题

- **parlor 场景自带的桌子没有碰撞**（只有外观）——物体会穿过去掉地上。
- 场景里的 **fruit USD 不是刚体**（没有 RigidBodyAPI），不能直接当 RigidObject spawn。
- 处理：在 `walker_c1_parlor_env_cfg.py` 里**自己加了一张静态碰撞桌 `GraspTable`（Cuboid）+ 一个球
  RigidObject**（先用 primitive sphere，保证是正经刚体），放在右手够得到的桌面。
- 抓取测试 `probe_walker_c1_grasp.py`：inconclusive——**卡在"手臂够不到球"**（手停在桌子近边后
  15cm，抬手也几乎抬不动），暴露出真正的上游问题是**手臂在重力下 hold 不住**。

### 关键结论：上肢 droop 根因 = 关节 drive 是 acceleration 型（已定案 + 已验证修复）

用隔离探针 `ubt_sim/scripts/probe_walker_c1_arm_tracking.py`（机器人单独 spawn、量 命令vs实际 关节角）
系统排查，**逐项确认正常并排除**：关节限位、config 刚度/阻尼/effort（runtime 确认生效 500/40/80）、
连杆质量（臂+手 4.85kg、全身 57kg）、惯量（~0.0015）、质心偏移（几 cm）、USD 单位（1.0/1.0）、
碰撞/tendon/friction/armature。**effort 从 25 提到 300 都扛不住，且停位与 effort 无关；重力关掉则一切正常。**

根因（读 USD drive type 定案）：

```
[USD DRIVE] R_elbow_pitch_joint: TYPE=acceleration  maxForce=25  stiffness=625  damping=0
```

C1 所有关节 drive 是 **`type=acceleration`（加速度驱动）**，不是 `force`。加速度驱动下 PhysX 把驱动按
关节等效惯量缩放：等效力矩刚度 ≈ stiffness×inertia ≈ 500×0.0015 ≈ 近 0 → 扛不住重力；提 effort 也
被惯量缩放故无效。之前"重力像放大 40 倍"是假象——是驱动力太弱。S2/天工用 force 驱动所以正常。
来源：URDF→USD 导入时 drive 默认成 acceleration。

**验证**（`probe_walker_c1_arm_tracking.py --force_drive`，spawn 前把 drive 改 force，重力开 + effort 80）：

```
             was(accel)   now(force)
ready  elbow  err 0.68  -> err 0.028   holding torque ~10 N·m
reach_fwd     err 0.49  -> err 0.026
lift_up       err 1.02  -> err 0.027
arm_down      err 0.35  -> err 0.021
```

**根因确认、修法有效。手指 droop 大概率同源（同一 acceleration 问题），改 force 后需一并复测。**

### 正式修法：烤 force-drive USD（脚本已写，尚未执行）

`ubt_sim/scripts/bake_walker_c1_force_drive_usd.py`：打开 `walker_c1.usd`，把每个关节 DriveAPI 的 type
从 acceleration 改 force，**另存 `walker_c1_force_drive.usd`（原文件不动）**；只改 type，不碰
stiffness/maxForce。之后 config 的 `WALKER_C1_USD_PATH` 指向新 USD 即可（回退就指回原文件）。

```bash
# 先 dry-run 看扫到多少关节（应 ~53 个可动关节）
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/bake_walker_c1_force_drive_usd.py --dry_run
# 真烤
docker exec walker-c1-ubt-sim /isaac-sim/python.sh -u /ubt_sim/scripts/bake_walker_c1_force_drive_usd.py
```

⚠️ 改 force 后**腿/腰/头刚度会真正生效**（之前 acceleration 下它们几乎没力、全靠 fix_root_link 挂着）。
烤完必须**重跑完整任务确认整机站姿/稳定**，再复测手指 droop（可能好了，也可能手部 stiffness 要重调）。

### 本轮提交内容（checkpoint push，force-drive 修复前）

以下改动在本节写入后作为一个 checkpoint 版本提交/推送（**force-drive 修复本身尚未落地**）：

```
M  config.py                       arm effort 60/25 -> 100/80（S2 对齐；对 acceleration 无用，等 force 后重估）
M  walker_c1_parlor_env_cfg.py     加 GraspTable 碰撞桌 + 球 RigidObject（M2 用）
M  C1_HANDOFF.md                   本文档
+  scripts/collect_walker_c1_pick_place.py     M1 采集主脚本
+  scripts/probe_walker_c1_workspace.py        右手工作空间探针
+  scripts/probe_walker_c1_grasp.py            抓取测试
+  scripts/probe_walker_c1_arm_tracking.py     ★手臂跟踪诊断（--no_gravity/--force_drive/读 drive type等）
+  scripts/bake_walker_c1_force_drive_usd.py   force-drive 烤 USD（脚本，未执行）
+  ubt_IL/scripts/convert/configs/Walker_C1_26_1RGB.json   LeRobot 转换配置（暂不用）
```

不入 git：`【CC-API】...SDK...docx`（对内文档）。更早的死胡同文件（scene_setup.py / inspect_c1_collision_prims.py）
已在早前清理时删除，不在工作区。`C1_joint_map.md` 已在 commit 3c90d98 提交（含 SDK cross-check）。

诊断脚本 `probe_walker_c1_arm_tracking.py` 是本轮最有用的资产：支持 `--no_gravity`、`--force_drive`，
并打印 sim 生效的 刚度/阻尼/effort、关节限位、连杆质量/惯量/质心、USD 单位、USD drive type。

### 下一步顺序

```
1. （等用户点头）跑 bake_walker_c1_force_drive_usd.py 烤 force-drive USD；config 指向它。
2. 重跑完整任务确认整机站姿/稳定（force 后腿/腰/头刚度真生效）。
3. 复测手指 droop（同源，大概率好转）。
4. 回到 M2：手臂能 hold 了，重做抓取（IK 或波点 -> 抓球 -> 移到目标），成功才存 HDF5。
5. 最后统一整理工作区未提交改动 + 提交。
```

## 2026-07-14 Update（傍晚）：force-drive 修复已落地 + 验证 + 准备姿势已调

### force-drive 修复已烤 USD 并落地（步骤 1、2 完成）

- dry-run 确认：`walker_c1.usd` 里 **全部 53 个可动关节** drive 都是 `acceleration`。
- 已烤 **`ubt_sim/assets/robots/walker_c1/walker_c1_force_drive.usd`**（15.8MB，53/53 acceleration→force，
  只改 type，不动 stiffness/damping/maxForce；**原 walker_c1.usd 不动**，回退就把 config 指回去）。
- `config.py` 的 `WALKER_C1_USD_PATH` 已指向 `walker_c1_force_drive.usd`。
- **完整 parlor 任务复测通过**（`probe_walker_c1_workspace.py`，force USD + 重力开）：
  root 稳在 [7.80, 6.08, 0.90]、不炸；R_palm 抬到 z=0.905（修复前垂在 ~0.80）；腿/腰因刚度真生效没抖没崩。
  **手臂在重力下能 hold 住准备姿势了。** 用户 GUI 目视确认满意。

### 准备姿势已"略微张开"（用户要求，已满意）

`constants.py::TASK_RESET_BODY_POSE` 的肩外展从近 0 调到 ±0.30 rad（约 17°）让双臂外张：
`L_shoulder_roll 0.068→0.30`、`R_shoulder_roll -0.003→-0.30`（方向：L 正/R 负 = 外展；range L[-0.139,1.884]/R[-1.884,0.139]）。
`collect_walker_c1_pick_place.py` 的 READY_*ARM 已同步同值，两边一致。

### 手部 droop：根因同源，已被同一修复覆盖，但【尚未复测】——下次做

- **手指 droop 和手臂 droop 是同一个 bug**：手指关节 drive 也是 acceleration。手指链惯量极小（~0.00002，
  手指 13g），acceleration 缩放后等效刚度 ≈ 200×0.00002 ≈ 0.004 N·m/rad ≈ 0 → 比手臂更彻底没劲。
  这解释了 handoff 早前"target 对但手指不跟、提 stiffness/effort 没用、关重力就正常"的全部现象。
- force-drive bake 已把 22 个手指关节一并改成 force，机制上应一并解决。
- **但两点未结（下次会话优先做）**：
  1. **手指闭合还没专门复测**（force USD 下发 `right_hand=[0.8]×6` 看手指跟不跟、稳不稳）。
  2. **手部 stiffness=200 / effort=50 是 acceleration 坏掉时代为硬凑调的强参数**，force 语义下对 13g 手指
     严重过强（200 N·m/rad），很可能手指会猛闭合/抖，**需要往回调到 force 下合理的一组**，再复测。

### 本轮新增/改动（本次提交）

```
+  ubt_sim/assets/robots/walker_c1/walker_c1_force_drive.usd   force-drive USD（15.8MB，修复核心）
M  ubt_sim/source/ubt_sim/devices/walker_c1/config.py          USD_PATH 指向 force USD
M  ubt_sim/teleoperation/control/walker_c1/constants.py        准备姿势肩外展 ±0.30
M  ubt_sim/scripts/collect_walker_c1_pick_place.py             READY_*ARM 同步
M  C1_HANDOFF.md                                               本节
```

### 更新后的下一步顺序

```
1.【下次先做】复测手指闭合（force USD）；若过猛/抖，把手部 stiffness/effort 从 200/50 回调到 force 合理值。
2. 回 M2：手臂能 hold 了，做"够到球"的 IK/波点，抓球→移到目标→成功才存 HDF5。
   （注意：目视 R_palm 离球还有 ~14cm，主要 Y 向 10cm，需要 reach 逻辑把手对准球。）
3. M3：批量随机化刷数据。
4. 收尾整理工作区。
```

## 2026-07-15 Update：M2 完成——真实物理抓放全流程 SUCCESS（正统抓法，非运气）

本节是最新状态。当天经历了 ~17 轮抓取调试（中途曾误报"完成"两次——deterministic
的运气 SUCCESS 在随机位置下 0/3），最终用**可解释的正统抓法**跑通：

```
下探: 苹果全程零扰动；提起: HELD (z 0.924->1.027)；放置: 离盘心 5.2cm SUCCESS。
```

### 制胜配方（collect_walker_c1_pick_place.py 当前实现）

```
1. 软手部增益 stiffness 10 / damping 1 / effort 2（近 URDF 原厂 force 语义值）。
   调试期 200/20/50 是接触弹射的元凶：空载跟踪正常 ≠ 增益正确。
2. 转腕 90°（wrist_roll = ready-1.57）掌心朝下，之后腕/肘yaw 冻结
   （IK joint_subset=(0,1,2,3) 只用肩3+肘pitch 做位置）。
3. 五指微张 [0.2]*6 当笼栅（掌下抓时收指反而把指节卷到罩腔正中央挡路——
   小指是全天惯犯，每轮首触都是 R_little_ip）。
4. 闭环罩口对准：servo_mouth_xy —— xy 误差用"罩口实时中心"（四指尖均值与
   拇指尖的中点）到球心度量，z 走 grasp center；消灭一切手系偏置盲猜。
5. 两段进场：高空(+0.22)对位 → 垂直降(+0.12) → 慢下探(ball+0.02, max_dq 0.006，
   靠接触自然停住即可，不必到底)。
6. 合围 [0.7,0.85,0.8,0.8,0.8,0.8] 一步完成 + 40 步稳定夹持再动臂。
7. 提升/搬运/下放全程 joint_subset 冻腕 + 慢速；放盘前先降到贴盘 1cm 再张手。
8. 球 r=0.027 m=0.1kg：实测这只手口袋有效孔径 5-6cm，7cm 球几何上装不下
   （满力闭合手图显示无任何指节能触到 7cm 球面）。
```

### IK/采集框架关键实现点

```
- 逐步 DLS（lambda=0.1, dq<=0.01 rad/步），palm jacobian；固定基座 jacobian 行号
  = body_idx-1。
- ★IK 增量必须累积在【命令】上（cmd_state），不能是 实测+dq：位置控制器有重力
  滞后，后者永远差 4-5cm 收敛不到。防饱和：命令超前实测 <=0.2 rad + 软限位内。
- 误差 <1.2cm 提前退出阶段；HOME->READY 用 100 步 ramp（一步跳变会甩臂扫到桌上物体）。
- --debug_watch：每 5 步打印 苹果位置+速度+最近机器人连杆+距离——本轮定案全靠它；
  _print_hand_map 打印 11 个手链节相对 grasp center 的坐标（口袋几何实测）。
- 成功判定：苹果落点距盘心 <=0.085m 且高度在盘面区间；成功才存 HDF5；
  --save_on_failure 调试用；--randomize 苹果初始位置随机（M3 主开关）。
```

### 抓取调试账本（17 轮的教训，按发现顺序）

```
1. 相机 rot=(1,0,0,0)+ros = 朝天花板拍（M1 就存在，只验过尺寸没验内容）。
   修正 rot=(0.40558,-0.57923,0.57923,-0.40558)=朝前+下俯20°，ready 低头 0.50。
2. 角阻尼 2.0 防滚 → 球不能滚就变"捏西瓜子"挤压弹射，更糟。回退。真正防滚 = 贴盘轻放。
3. 手部增益 200/20/50：接触时上百牛，球被弹飞 2-5 米。回调 10/1/2。
4. 张开指尖在斜线进场路径上扫飞球（位置 IK 不控姿态，腕自然低头）→ 两段进场。
5. 侧向口袋（掌心朝+y）：装 7cm 球孔径不够；口袋结构=指尖帘+高位拇指，
   拇指闭合弧根本够不到桌上球的高度 → 死路，转腕掌下才是正解。
6. 每轮首触都是小指(R_little_ip) → 掌下抓时五指微张，不收指。
7. deterministic 同种子成功 ≠ 可靠：必须 --randomize 多局验证。
```

### 本轮文件改动（待提交清单见 git）

```
M  ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py
   隐形桌面碰撞板/盘碰撞盘/刚体苹果(r0.027,100g)；两张桌子问题修复
M  ubt_sim/config/walker_c1/parlor.yaml   场景指向 scene_v2_c1.usda + 相机 rot 修正
M  ubt_sim/scripts/collect_walker_c1_pick_place.py   M2 完整抓放（上面的配方）
M  ubt_sim/scripts/probe_walker_c1_workspace.py      ready 同步
M  ubt_sim/teleoperation/control/walker_c1/constants.py  head_pitch 0.50
M  ubt_sim/source/ubt_sim/devices/walker_c1/config.py    手部增益 10/1/2
+  ubt_sim/assets/local_scenes/tiangong_parlor/scene_v2_c1.usda（gitignore 内：
   6 行 usda，subLayers 引 scene_v2.usd + over "apple" active=false，机器丢失按此重建）
```

### 随机化收尾（31e6a5d push 之后的迭代）

固定位置 SUCCESS 后，--randomize 连续 0/3 了四批，逐层修掉：

```
1. 冻腕后 IK 只剩肩3+肘pitch 4 关节，随机点（尤其 +x 远侧）出了可达域，
   下探停在差 5cm 处 -> 解冻 elbow_yaw（subset=(0,1,2,3,4)）。
2. 解冻后 yaw 自由漂移 -> 抓取时刻手朝向和成功局不一致，闭手落空 ->
   ★零空间偏置：DLS 加 null-space 项把 elbow_yaw 持续拉回转腕后的参考值
   （位置任务不受影响）。_ik_arm_step(null_ref={4: roll_arm[4]})。
3. 下探高度敏感 ±1cm：目标=球+0.04（加早退容差 ~1cm 恰落在成功高度 球+0.05）。
   低了指尖插桌被压住卷不动，高了笼子合在球顶上方。
4. 随机范围对齐可达域：x∈[-0.03,+0.01]、y∈[-0.05,+0.01]（+y 侧仍吃紧故收窄）。
```

结果：随机位置首次 SUCCESS（1/3，第 3 局 HELD+落点 7cm）。

### 随机化成功率现状（调参已到收益上限，2026-07-15 晚）

后续又加了三层（本 commit）：

```
1. 抓取校验+重试：提起后查苹果 z；没抓住 -> 张手回 [0.2] 预合拢，对苹果当前位置
   重来（最多 3 次）；苹果掉桌则提前中止。重试有实测兑现（一局第2次抓住并成功）。
   注意：失败大多是"位置确定性"的（同一点重试同样失败），重试主要救随机扰动型失败。
2. 苹果 r=0.027 -> 0.022（4.4cm 李子级）：罩笼孔径 5-6cm，5.4cm 球只剩毫米级余量，
   出生点差 6mm 结果就翻转；4.4cm 给 ±0.8cm 余量，和伺服精度匹配。成功率 1/5 -> 2/5。
3. 下探收敛容差 1.2cm -> 0.8cm；合围加深 [0.7,0.9,0.95x4]（副作用：抓空时更容易
   把苹果打飞出桌，中止该局——可接受，反正失败局不存）。
```

**当前测得成功率 ~30%（3/10，批间波动 20-40% 属噪声）。盲罩式抓取（位置 IK 无
姿态控制）已到上限，不要再花 50 分钟/轮磨 cm 级参数。**

### 下一步（按性价比）

```
1. 直接用：30-40% 成功率对刷数据可用（失败局不存，只费仿真时间）。
   一次启动多刷：--episodes 25 约 4 小时 -> ~8-10 条轨迹。
2. 提吞吐：精简阶段步数（很多是调试余量，可砍 30-40%）；num_envs>1 并行采集
   （标准做法，采集脚本要改批量版，半天活）。
3. 提成功率的正道：6D IK（位置+姿态同控，palm 姿态显式指定）代替
   "位置IK+转腕+零空间"的拼积木——这是把成功率推到 80%+ 的正确工程路线。
4. 抽查已存 HDF5（dataset/walker_c1/ 下已有多条成功轨迹）。
5. 归位→抓取语义已内嵌（每局 ramp 到 TASK_RESET_BODY_POSE 开始、结束回同一姿势），
   用户已 GUI 确认；姿势权威出处如需统一可抽 JSON。
6. 左腿碰撞小尾巴、真机 joint order 校对：老未结项不变。
```

## 2026-07-17 Update：优先级转向 ROS 工具链 + C1 bridge 完成并验证

**用户明确新目标：不是刷数据，是工具链——同一套控制代码通过 ROS SDK 话题，仿真和
真机行为一致（切 ROS_DOMAIN_ID 146↔0）。** in-process 采集器保留作调试用。

### 已完成（commit 820faee）

```
+ teleoperation/bridges/walker_c1/walker_c1_ros2_zmq_bridge.py + yaml
  SDK 话题面：/mc/sdk/robot_command|robot_state、/mc/{left,right}_hand/command|
  joint_states、/sensor/camera/head/color|depth/raw、/sim/cmd_reset（仿真 only）。
  身体命令按名映射（含 elbow_roll/elbow_pitch 双别名兼容——SDK 文档 vs URDF 命名
  歧义两头都认）；手部 6D SDK 名映射到 11 关节仿真手（复用 action_process 逻辑）。
M scripts/start_sim.sh  C1 分支启动 bridge；RMW 默认 rmw_fastrtps_cpp。
```

### 端到端验证（全部实测通过）

```
ros2 pub RobotCommand head_pitch 0.3      -> 仿真 head_pitch 0.301 ✓
ros2 pub RobotCommand R_elbow_roll -0.8   -> 仿真 R_elbow_pitch -0.788 ✓（别名兼容）
ros2 pub JointCommand right_hand 0.5x6    -> 仿真 SDK 手关节 0.495 ✓（与身体并发）
/mc/sdk/robot_state / hand joint_states   -> 数据流 ✓
/sensor/camera/head/color/raw             -> 25.4 Hz ✓（C++ image bridge）
```

### 踩坑记录（重要）

```
1. 容器没装 colcon —— run.sh init 的"Build Walker SDK ROS2 messages"一直静默失败。
   已 pip 装 colcon-common-extensions 并编译到 /opt/ubt_sim/walker_sdk_ros2_msgs。
   （注意 colcon 是装在容器里的，容器重建后要重装。）
2. S2 分支的 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 在本容器会让 rclpy 启动即退
   （只装了 fastrtps）。C1 分支默认 rmw_fastrtps_cpp。
3. bridge cmd socket 不能抄 S2 的 SNDHWM=1：身体+手是两条背靠背消息，1 深队列
   会系统性丢掉先发的一条（手部命令 100% 被丢）。C1 控制器 advance() 是
   "排干全部+合并"语义，SNDHWM=16 正确。
4. 杀 start_sim.sh 时 kit python 进程可能残留占住 5655/5656 端口
   （Address already in use）——pgrep -af sim_runner 精确清理。
```

### 启动方式

```bash
# 全栈（仿真+bridge），仿真域 146：
docker exec walker-c1-ubt-sim bash -lc "cd /ubt_sim && \
  UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 ROS_DOMAIN_ID=146 \
  bash scripts/start_sim.sh --headless --device cpu --step_hz 30"
# ROS 侧环境：
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
export ROS_DOMAIN_ID=146   # 真机=0
```

### 下一步

```
1. 抓放控制移植成 rclpy 脚本（teleoperation/control/walker_c1/，Py3.10，只用 SDK 话题）：
   - IK 不能再用仿真雅可比 -> URDF + ikpy（参照天工 robot_controller.py）；
   - 苹果位置作参数/配置（仿真=已知出生点；真机=将来接感知）；
   - reset.py 已有 ROS 骨架，可从"归位"开始验证。
2. 上真机第一件事不变：dump /mc/sdk/robot_state 的 joint_states.name，
   核对肘部命名（bridge 已双兼容，但控制脚本侧要以真机名为准）。
3. 左腿碰撞小尾巴等老未结项不变。
```

## 2026-07-19 Update：★ROS 抓放总根因定案——仿真时间 vs 墙钟不同步（已修复+判决实验验证）

本节是全项目最重要的一条结论。ROS 版抓放此前连败十几轮（增益/摩擦/姿态/偏移怎么调
都在"打飞球↔握不住"间震荡），最终用"回放冠军轨迹"判决实验定案：

### 根因

```
本机 CPU 物理仅 ~28 步/秒（实测），每步 0.01s 仿真时间
=> 仿真时间流速 = 0.28x 真实时间（慢 3.6 倍）
=> ROS 侧按墙钟计时的一切动作，在仿真世界里被加速 3.6 倍执行
=> "2.5 秒缓降"在物理引擎里是 0.7 秒的快落，接触必炸
in-process 采集器从不受此影响：命令与 env.step() 逐步同锁。
```

### 判决实验（replay_trajectory.py，commit f5887e7）

```
同一条 in-process 冠军轨迹（1784105817）通过 ROS SDK 话题重放：
  按墙钟 45ms/帧发   -> 球被打飞下桌（复现 ROS 版全部症状）
  按物理步 3步/帧发  -> 抓→提→运→放全成功，落点距盘心 4.8cm ✓✓
```

### 修复设施（已提交 f5887e7）

```
- controller.py: ZMQ 状态带 sim_step 物理步计数器
- bridge: /sim/object_state JSON 透传 sim_step
- robot_controller.py: wait_sim_steps(k)——按物理步等待；无计数器时自动退化
  为墙钟等待（真机语义：仿真时间==真实时间，同代码两端通用）
- replay_trajectory.py: 数据集 HDF5 轨迹回放器（sim/真机通用，IL 演示重放工具）
- move_right_arm_joints: ramp 改为每 3 物理步发一条命令（=冠军轨迹录制节奏），
  duration 语义变为"仿真秒"
- 物理配置已回退 in-process 已证值（手 10/1/2 + force_drive.usd）；
  调试期的摩擦烘焙（bake_walker_c1_hand_friction_usd.py + grip.usd）保留备用未启用
```

### 教训（重要，写给后来者）

```
1. 慢于实时的仿真里，跨进程控制必须按仿真步对齐，不能按墙钟。这是所有
   "仿真里 ROS 控制"项目的通用陷阱。
2. 调试期间的增益/摩擦/姿态/偏移十几轮调参全是给这个根因打的错误补丁——
   当"多方向调参都在两种失败模式间震荡"时，应怀疑存在未建模的系统级因素。
3. 判决实验（回放已知成功的轨迹）比继续调参高效得多，应更早使用。
4. 感谢用户两次关键质疑："如果是摩擦为什么 in-process 能成功"（排除摩擦）、
   "频率可能有关系"（直指根因）。
```

### 当前状态与下一步

```
- ROS 工具链完整：bridge + ikpy 控制 + 闭环对准 + 安全下限 + 回放器 + 步同步。
- pick_place_controller 正在切换到全步同步节奏（每局墙钟时间变长 3.6 倍属预期，
  是正确性的代价；物理跑得快时自动缩短）。
- 待办：步同步版抓放成功率验证 -> --randomize 验证 -> step_hz 100 吞吐测试
  （CPU 已近饱和，预期增益有限）-> 更新采集链路。
- 真机 joint order 核对、左腿小尾巴等老未结项不变。
```

## 2026-07-19 Update（晚）：验收标准确定 + 主路线切换为示教再现

### 用户验收标准（重要）

```
"只需要保证苹果位置固定的时候能成功率比较高，随机的是锦上添花。"
=> 固定位置高成功率 = 硬指标；随机位置 = 加分项，不阻塞验收。
用户已授权完全自主工作（不删成果、保护仓库、勤记录为红线）。
```

### 主路线：示教再现（teach-and-repeat）

```
新增 teleoperation/control/walker_c1/pick_place_replay.py：
  go_ready -> 苹果摆到示教位置 (8.207,5.877) -> 按物理步重放冠军轨迹
  (dataset/walker_c1/1784105817) -> 成功判定 -> 多局统计。
判决实验已证此路端到端 100% 可行（落点距盘心 4.8cm）。
这就是工业标准的固定位置抓放做法，也直接对应真机示教流程。
```

### 示教再现调试进展（自主工作期间实时记录，含两次假设纠错——诚实过程）

```
- 判决性发现②：新启动仿真栈上回放稳定成功（裸回放+包装版 back-to-back 双成功,
  落点 1.0cm）,运行数小时的老栈上失败。
- 【已证伪①】最初怀疑"reset_sim（reset_scene_to_default）复位会污染物理状态"，
  加了显式复位调用。复位版 5 局=1 成 4 败（ep1 成功,ep2 起确定性同败）。
  关掉显式复位后重测：全程不调用 reset_sim，结果依然是 ep1-2 成功、ep3 起败光。
  => 与是否调用复位无关，证伪。
- 【已证伪②】"体内摇晃未散尽"（关节残余速度未衰减导致下一局失真）。加了速度
  探针诊断，实测速度读数在多个"1s/3s/5s 静置"检查点之间逐位数字冻结不变——
  说明探针本身有问题（未验证真实衰减），但顺带检查腰/头/腿角度也全程稳定在
  0 附近（<0.02 rad），排除了"腰部在抓取冲击下悄悄跑偏"这个更具体的猜测。
- 【重大修正,原结论过于乐观】原以为"全新重启的仿真栈上第 1 局必成功"（早期
  3-4 次巧合都成功导致误判为确定性规律）。扩大到 6 次独立全新栈试验后，
  真实成功率约 4/6 ≈ 65%（不是 100%！也测到过全新栈 ep1 直接失败的情况）。
  但"一旦某局在某进程上失败，同进程后续几乎必败"这条规律在所有多局测试里
  稳定重现，没被推翻。
  => 根本内部机制（大概率是 PhysX 接触缓存或求解器状态在某次失败性剧烈接触后
  进入了某种非典型/退化分支，且不会自愈）仍未查到底，留作后续可选深挖项。
- 【当前可靠方案，已按新数据修正】run_c1_teach_and_repeat_batch.sh：
  单次全新栈尝试成功率~65%，若失败则丢弃该进程、整套重启再试（进程内重试
  已证无效）。按此重试直到命中，理论 P(3 次内成功)≈96%。
  用法（N=目标成功局数，第二参数=每局最多重试次数,默认5）：
    docker exec walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_teach_and_repeat_batch.sh 3 5
  代价：~5 分钟启动/次尝试,平均约 1.5 次尝试换 1 个成功局。
```

### 支线（IK 编排版）状态：实验性，暂停打磨

```
步同步修好时序后，IK 版残余问题 = mode-Z 的 yaw 自由度让指笼落位旋转不定
（球有时压在无名指/小指下而非拇指对握区）。尝试"冠军关节构型作 IK 种子"
反而劣化（种子与路点目标体系不自洽，对准环报 447mm 级误差、手臂被拉飞）。
代码全部保留（pick_place_controller.py），标记实验性；随机位置需求提上来时
再继续，或改走"冠军轨迹+笛卡尔偏移扭曲"的路线。
```

### 场景改造（walker_c1_parlor_env_cfg.py + scene_v2_c1.usda + parlor.yaml）

```
- 场景装饰桌 /World/table：x[8.144,8.744] y[5.483,6.683] 桌面 z=0.897（视觉专用，无碰撞）。
- TableTopCollider：隐形薄板 (0.60,1.20,0.06)，顶面与视觉桌面齐平 z=0.897。
- 场景盘子 /World/plate：中心 (8.374,6.046)，半径~0.10，盘沿 z=0.931（视觉专用）。
- PlateCollider：隐形圆盘 r=0.085 h=0.05，顶面 z=0.925（放置面+成功判定区）。
- Object：红色刚体苹果 r=0.035、100g、摩擦 1.2，初始 (8.21,5.90,0.934)（桌面右手区）。
- 场景装饰苹果 /World/apple 用覆盖层 deactivate（避免画面里两个苹果干扰 IL）：
  ubt_sim/assets/local_scenes/tiangong_parlor/scene_v2_c1.usda
  （6 行 usda：subLayers 引 scene_v2.usd + over "apple" active=false；
   在 gitignore 的 local_scenes 里，机器丢了按此描述重建即可。）
- parlor.yaml scene.usd_path 已指向 scene_v2_c1.usda。
```

### 相机朝向 bug（重要教训）

```
parlor.yaml 头部相机 offset rot 原来是 (1,0,0,0) + convention=ros
= 沿 head_pitch_link +Z 看 = 朝天花板！（画面里的灰色大圆角块=顶灯）
M1 当时只验证了"能解码出 480x640"，没验证内容。
修正值 rot=(0.40558,-0.57923,0.57923,-0.40558)（朝前+额外下俯 20°），
加上 ready 低头 0.50 rad，合计约 49° 俯视桌面。
验证帧：桌面/粉盘/红苹果/双手都在画面内，carry 阶段能看到黄色右臂抓着苹果。
READY_HEAD pitch 0.35->0.50 已同步 collect 脚本 / workspace probe / constants.py。
```

### IK 抓放实现（collect_walker_c1_pick_place.py）

```
- 伺服点"抓取中心"= R_thumb_mpp/R_index_ip/R_middle_ip 三链节中点。
- 每步阻尼最小二乘（lambda=0.1，dq<=0.01 rad/步），用 R_palm 的 jacobian。
- 关键坑：IK 增量必须累积在【命令】上（cmd_state["right_arm"]），不能加在实测角上——
  位置控制器有重力滞后，命令=实测+dq 永远差 4~5cm 收敛不到，第一次试跑就因此
  把苹果拨飞到地上。加防饱和：命令超前实测<=0.20 rad + 关节软限位内。
- 误差<1.2cm 提前退出阶段；下探前手指预合拢 0.3 减少拨飞。
- 阶段：settle60 -> hover240(苹果上方12cm) -> 预合拢25 -> descend160 -> 闭手0.85x60
  -> lift100(+15cm) -> carry180(盘上方) -> 张手50 -> retreat60 -> 回READY120 -> settle40。
- 成功判定：苹果落点离盘心 <=0.085m 且高度在盘面区间 -> 才存 HDF5。
  --save_on_failure 失败也存（调试）；--randomize 苹果初始位置随机
  （x+[-0.02,0.04], y+[-0.06,0.06]），M3 直接用。
- 典型成功日志：hover err 0.010 / descend err 0.032（指笼罩住苹果）/
  lift 后苹果 z 0.932->1.063 HELD / 落点离盘心 0.073 SUCCESS / 280 帧。
```

### 本轮改动文件

```
M  ubt_sim/source/ubt_sim/task/walker_c1_parlor/walker_c1_parlor_env_cfg.py  隐形碰撞体+苹果
M  ubt_sim/config/walker_c1/parlor.yaml        场景指向 scene_v2_c1.usda + 相机 rot 修正
M  ubt_sim/scripts/collect_walker_c1_pick_place.py  M2 IK 抓放 + 成功判定 + randomize
M  ubt_sim/scripts/probe_walker_c1_workspace.py     ready 同步（肩外展/低头）
M  ubt_sim/teleoperation/control/walker_c1/constants.py  head_pitch 0.50
+  ubt_sim/assets/local_scenes/tiangong_parlor/scene_v2_c1.usda（gitignore 内，见上）
```

### 下一步

```
1. M3：批量 --randomize 刷数据（验证成功率后放大 episodes 数）。
2. 数据规模上来后抽查 HDF5（帧数/图像内容/obs-action 对齐）。
3. （可选）左腿碰撞小尾巴仍在（纯视觉）；真机 joint order 校对仍是回真机前的第一件事。
```

## 2026-07-21 Update：mentor 机身、灵巧手材质与苹果外观

### 当前机器人资产

```
- mentor 提供的 Collected_walker_c1_v1_sensorKpkd 用作 C1 机身和传感器主体。
- merge_walker_c1_mentor_usd.py 只拼入原来已验证的双灵巧手、手部关节、碰撞体和材料，
  输出 walker_astron_v1_sensorKpkd_hands.usd。
- 不再复制整套 prototype；只重映射 56 个手部 prototype，避免机器人链接四分五裂。
- 手部 4 种视觉材料按原 USD 的子网格分配同时生效，并非运行时四选一：白、银灰、
  淡蓝灰等分别用于手掌外壳和各手指零件；手部物理摩擦材料独立保留。
- mentor 原有 80 个机身视觉/碰撞引用保持不变；合并后 53 revolute + 17 fixed、
  70 rigid body。完整 pick-and-place 已验证成功。
```

启动带 ROS 控制和五个传感器视窗的版本：

```bash
docker exec -it walker-c1-ubt-sim bash /ubt_sim/scripts/start_c1_mentor_sensor_sim.sh --control
```

### 苹果外观恢复，碰撞不变

```
- build_c1_apple_usd.py 生成 assets/robots/walker_c1/c1_task_apple.usda。
- Visual 引用原 Tiankung 客厅 /World/apple 的网格和 Yellow_Red_Nectarine 材质，
  Albedo/Normal/Roughness 纹理均能解析；可见尺寸 52.6 x 54.0 x 53.3 mm。
- 原视觉网格上的 RigidBody/Mass/Collision API 全部删除，防止双重碰撞。
- Physics 仍只有一个不可见 Sphere：radius=27 mm、mass=100 g、friction=1.2，
  因此控制器的目标点、桌面接触高度和抓取参数均无需调整。
- walker_c1_parlor_env_cfg.py 的 Object 从 SphereCfg 改为该 UsdFileCfg。
```

2026-07-21 完整 ROS 验证（`--no-record`）：抓起 8.4 cm，静置检查 `STABLE`，
最终落点距盘心 1.9 cm，`1/1 SUCCESS`。这证明视觉替换未改变既有抓放行为。

### 2026-07-21 最终姿势微调

```
- 盘上释放净空 PLATE_RELEASE_CLEARANCE_B：4 cm -> 6 cm，手掌目标 z 实测由
  0.141 m -> 0.161 m，盘子和运输路点不变。
- TASK_RESET_BODY_POSE 左臂改为右臂的严格镜像：pitch 同号，roll/yaw 反号；
  左右手本来就使用相同的 6 个主动关节全开命令，从动关节由镜像机构联动。
- 完整 ROS 回归：HELD、STABLE，最终苹果距盘心 2.2 cm，1/1 SUCCESS。
- 两项都是 ROS 控制侧常量修改，已有仿真无需重启，重新运行抓放脚本即生效。
```
