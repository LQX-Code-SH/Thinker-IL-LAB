# TienKung Dex 仿真快速上手

天工行者 Dex（V3 模型 + Inspire RH5DG2 双手）在 Isaac Sim 中的仿真、遥操作与数据采集。

## 1. 准备容器

```bash
cd ubt_sim/docker
bash run.sh build          # 首次
bash run.sh start
bash run.sh init           # 安装 ubt_sim / pyzmq / bodyctrl_msgs / 图像桥
bash run.sh bash           # 进入容器，项目挂在 /ubt_sim
```

## 2. 生成 USD 资产

仓库只入库 URDF/STL（`assets/robots/tienkung_dex/`），USD 需在容器内生成：

```bash
cd /ubt_sim
# Inspire RH5DG2 双手（67 revolute）
/isaac-sim/python.sh scripts/import_tienkung_dex_usd.py --variant inspire --force
/isaac-sim/python.sh scripts/check_tienkung_dex_usd.py --variant inspire

# BrainCo Revo2 双手（53 revolute）
/isaac-sim/python.sh scripts/import_tienkung_dex_usd.py --variant brainco --force
/isaac-sim/python.sh scripts/check_tienkung_dex_usd.py --variant brainco
```

`check` 会校验关节集合、限位（与 URDF 对比）、articulation root 与刚体质量，
失败时退出码为 1，需先修正再启动仿真。两个变体互不依赖，可只转换要用的那套。

转换结束会自动套用外观配色（`scripts/recolor_tienkung_dex_usd.py`，默认
`--scheme pro`，直接复用 `tienkung_pro` 的黑色/灰色/深灰/金属/白色材质）：
黑色主体与胸甲、灰色大腿外壳与小腿、深灰肩肘腕关节、金属踝关节、白色手掌与指根。
只改 USD 材质，不影响几何与物理；不想配色时给转换脚本加 `--no-recolor`。
单独调色只需重跑配色脚本：

```bash
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/recolor_tienkung_dex_usd.py --variant inspire

# 早期按参考照片分的五色方案
... scripts/recolor_tienkung_dex_usd.py --variant inspire --scheme reference
```

要按零件细调（同一个 link 内多色），走宿主机 Blender 往返：`build_dex_blender_scene.py`
建场景（`--split-links` 顺带按 loose parts 拆零件）→ 人工改色 →
`read_dex_blender_colors.py` 导出 JSON → 容器内 `apply_dex_colors_json.py` 写回，
零件颜色以 GeomSubset 落在 base 层的 mesh 上。机身 logo 同理：`add_dex_blender_decal.py`
生成带贴图的小平面，在调色场景里 Append 后手动摆位，随同一份 JSON 写成
`/visuals/<link>/decals/` 下的贴花。详见
[`assets/robots/tienkung_dex/README.md`](../../assets/robots/tienkung_dex/README.md)
的「在 Blender 里人工调色」。

## 3. 启动仿真

```bash
# 只加载场景（不启动 ROS 控制，适合先确认位姿/相机）
UBT_SIM_LOAD_ONLY=1 \
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 \
bash scripts/start_sim.sh

# 完整模式：同时拉起 ROS2-ZMQ 桥接（5755-5758）
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh
```

两个场景（手部变体）分别对应独立任务：

| 任务 ID | 手部 | 关节数 | 配置 |
|---------|------|--------|------|
| `UBTSim-TienkungDex-Parlor-v0` | Inspire RH5DG2 | 67 | `config/tienkung_dex/parlor.yaml` |
| `UBTSim-TienkungDex-Brainco-Parlor-v0` | BrainCo Revo2 | 53 | `config/tienkung_dex/brainco_parlor.yaml` |

`start_sim.sh` 会从任务名推断变体（含 `Brainco` → brainco），并把 `--variant`
传给 ROS2-ZMQ 桥接，无需额外参数。

可用环境变量：

| 变量 | 默认 | 说明 |
|------|------|------|
| `UBT_SIM_TASK` | `UBTSim-TienkungPro-Parlor-v0` | 设为 `UBTSim-TienkungDex-Parlor-v0` 启用 DEX |
| `UBT_SIM_NUM_ENVS` | 1 | 并行环境数 |
| `UBT_SIM_LOAD_ONLY` | 未设置 | 仅渲染，不接 ROS |
| `UBT_SIM_TIENKUNG_DEX_CMD_PORT` / `_STATUS_PORT` / `_IMAGE_PORT` / `_JPEG_PORT` | 5755/5756/5757/5758 | ZMQ 端口 |
| `UBT_SIM_NO_BRIDGE` | 未设置 | 置 1 时不启动桥接 |
| `ROS_DOMAIN_ID` | 146 | 仿真 146 / 真机 0；话题完全相同，只靠 domain 分流 |

## 4. 遥操作

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash
export ROS_DOMAIN_ID=146          # 真机改成 0
# Inspire 双手
/usr/bin/python3 teleoperation/control/tienkung_dex/keyboard_joint_control.py --step 0.02
# BrainCo 双手
/usr/bin/python3 teleoperation/control/tienkung_dex/keyboard_joint_control.py --step 0.02 --variant brainco
```

键位：`1`~`6` 切分组（左臂/右臂/左手/右手/头腰/腿），`[`/`]` 选关节，
`w`/`s` 增减，`o`/`c` 手张开/闭合，`r` 同步回当前姿态，`q` 退出。

统一 CLI（状态/单关节/末端 IK/手部/回站位/分析/相机/EE 遥操作）见
[control/tienkung_dex/README.md](../../teleoperation/control/tienkung_dex/README.md)。

外部遥操作端直接向真机同款话题发指令：`/arm|head|waist|leg/cmd`
（`ros2_bridge_msgs`，`ctrl[].name` = 关节 ID）与 `/left_hand/angle_set`
（因时）/ `/left_hand/set_motor_multi`（强脑）。因时手是 13 通道、单位 0.1°
的绝对角码（码越大越伸直，标定与握拳示例见
`teleoperation/msgs/tienkung_dex/dex_hand_msgs_src/src/inspire_hand_msgs/msg/SetAngle.msg`）。

## 5. 采集数据

```bash
# 交互：Enter 开始/停止，s 保存并下一集，q 退出
bash teleoperation/control/tienkung_dex/save_data.sh

# 无人值守：每集 30s，共 5 集
bash teleoperation/control/tienkung_dex/save_data.sh --auto-start --duration 30 --episodes 5 --task-name "pour water"

# BrainCo 变体需显式指定（影响手部列数与 attrs）
bash teleoperation/control/tienkung_dex/save_data.sh --variant brainco

# 批量采集：每集自动「复位 → 预备位 → 抓 → 放到盘上」并落盘，失败集不保存
bash teleoperation/control/tienkung_dex/save_data.sh --batch 20
```

批量采集的成败按仿真真值判：`/sim/task_completed` 给出苹果中心到盘心的距离，
小于 `--task-dist-tol`（默认 0.15 m，同 `tienkung_pro` 的 `TASK_SUCCESS_DIST`）
才落盘并算成功，否则丢弃、计入失败。收不到这个话题（真机 / 旧桥接）时跳过判定。

苹果默认固定在场景里；加 `--randomize-apple` 后每集随机挪一次（世界系相对复位
位姿 `x=0~10 cm`、`y=0~-10 cm`），抓取航点跟着平移，用于采集有位置多样性的数据：

```bash
bash teleoperation/control/tienkung_dex/save_data.sh --batch 20 --randomize-apple
```

产物：一集一个时间戳目录 `dataset/tienkung_dex/<epoch秒>/trajectory.hdf5`
（与 `tienkung_pro` / `walker_s2` 同布局，目录 0777、文件 0666），字段见
[control/tienkung_dex/README.md](../../teleoperation/control/tienkung_dex/README.md)。

## 6. 回放

```bash
# 预览（Rerun）；不给 --episode 就取 dataset/tienkung_dex 下最新的时间戳目录
/usr/bin/python3 teleoperation/tools/playback_tienkung_dex.py --mode preview

# 控制（向桥接话题回放）
/usr/bin/python3 teleoperation/tools/playback_tienkung_dex.py \
    --episode dataset/tienkung_dex/<epoch秒>/trajectory.hdf5 --mode control
```

## 7. 关键参数

| 项 | 位置 |
|----|------|
| 关节分组 / HOME / 增益 | `source/ubt_sim/devices/tienkung_dex/config.py` |
| 外观配色（按 link） | `scripts/recolor_tienkung_dex_usd.py` 的 `PRO_PALETTE` / `pro_palette_for_link`（默认），`REFERENCE_PALETTE` 为旧五色方案 |
| 外观配色（按零件） | Blender 往返：`scripts/build_dex_blender_scene.py`（`--split-links`）→ `scripts/read_dex_blender_colors.py` → `scripts/apply_dex_colors_json.py` |
| 机身 logo（贴花） | `scripts/add_dex_blender_decal.py` 生成贴花 `.blend` → 调色场景 Append 摆位 → 同一份 JSON 写回；贴图在 `assets/robots/tienkung_dex/textures/` |
| 场景与相机 | `config/tienkung_dex/parlor.yaml` |
| 任务注册 | `source/ubt_sim/task/tienkung_dex_parlor/__init__.py`（`UBTSim-TienkungDex-Parlor-v0`） |
| 桥接话题与端口 | `teleoperation/bridges/tienkung_dex/bridge_config.yaml` |
| 资产说明与转换 | `assets/robots/tienkung_dex/README.md` |

## 8. 已知事项

- USD 转换未完成前，启动仿真会因 USD 缺失失败——先执行第 2 步；
- 初始站姿按实机 `home_standing` 标定：pelvis 离地 0.998 m，膝微屈（左 0.368 / 右 0.341 rad）；
  改站姿需同时更新 `devices/tienkung_dex/config.py` 的 HOME_POSE 与 yaml 的 `init_state.pos[2]`；
- 相机取 `camera_head_link` 原点、`rot` 为单位四元数（URDF 已把该 link 摆成 ROS 光学系）；
  画面朝向由 `head_pitch_joint` 决定（正值为低头，范围 -15°~+58°），不要往 `cameras.*.rot` 里加安装俯角；
- 仿真主循环约 11 Hz（`scripts/start_sim.sh --perf_stats` 实测每步 Phys ~30 ms + Render ~40 ms，
  实时率 ≈0.11）：墙钟时长的轨迹（如预备位每段 2.5 s）换算到仿真时间远超关节速度上限，
  头/臂会明显滞后，轨迹走完后还要几秒才追上。这是时间基准问题，不是执行器刚度的锅——
  头以 1.27 rad/s 走 45° 只要 0.62 s，真机上 2.5 s 轨迹完全跟得上；
- Inspire RH5DG2（67 revolute）与 BrainCo Revo2（53 revolute）两套场景均已接入，
  各自需要单独生成 USD 并使用对应任务 ID；录制数据通过 `hand_variant` attr 区分。
