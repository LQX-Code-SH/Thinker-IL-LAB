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

## 4. 遥操作

```bash
source /opt/ros/humble/setup.bash
# Inspire 双手
/usr/bin/python3 teleoperation/control/tienkung_dex/teleop_controller.py --step 0.02
# BrainCo 双手
/usr/bin/python3 teleoperation/control/tienkung_dex/teleop_controller.py --step 0.02 --variant brainco
```

键位：`1`~`6` 切分组（左臂/右臂/左手/右手/头腰/腿），`[`/`]` 选关节，
`w`/`s` 增减，`o`/`c` 手张开/闭合，`r` 归零，`q` 退出。

外部遥操作端也可直接向 `/tienkung_dex/{body,left_hand,right_hand}_command`
发布 `sensor_msgs/JointState`。

## 5. 采集数据

```bash
# 交互：Enter 开始/停止，s 保存并下一集，q 退出
bash teleoperation/control/tienkung_dex/save_data.sh

# 无人值守：每集 30s，共 5 集
bash teleoperation/control/tienkung_dex/save_data.sh --auto-start --duration 30 --episodes 5 --task-name "pour water"

# BrainCo 变体需显式指定（影响手部列数与 attrs）
bash teleoperation/control/tienkung_dex/save_data.sh --variant brainco
```

产物：`dataset/tienkung_dex/episode_000N.hdf5`，字段见
[control/tienkung_dex/README.md](../../teleoperation/control/tienkung_dex/README.md)。

## 6. 回放

```bash
# 预览（Rerun）
/usr/bin/python3 teleoperation/tools/playback_tienkung_dex.py \
    --episode dataset/tienkung_dex/episode_0001.hdf5 --mode preview

# 控制（向桥接话题回放）
/usr/bin/python3 teleoperation/tools/playback_tienkung_dex.py \
    --episode dataset/tienkung_dex/episode_0001.hdf5 --mode control
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
- 相机默认取 `camera_head_link` 原点，画面朝向需目视确认后调整 yaml；
- Inspire RH5DG2（67 revolute）与 BrainCo Revo2（53 revolute）两套场景均已接入，
  各自需要单独生成 USD 并使用对应任务 ID；录制数据通过 `hand_variant` attr 区分。
