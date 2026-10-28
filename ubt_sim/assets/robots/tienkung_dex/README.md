# tienkung_dex 机器人资产

天工行者 Dex（TienKung Dex，V3 模型）的仿真资产包，用于 Isaac Sim / Isaac Lab。

## 来源

| 内容 | 来源 |
|------|------|
| 机身 URDF / STL、关键参数表 | `Tienkung_Dex_Model_All-main`（V3 模型包，内部交付件） |
| 灵巧手（Inspire RH5DG2 / BrainCo Revo2） | 同上，`Inspire_RH5DG2/`、`BrainCo_revo2/` |

本目录为**入库副本**，只做两处改动：

1. 网格引用由 `package://` 改写为相对路径（见下表），避免依赖 ROS 包搜索路径；
2. URDF `<robot name>` 改为 `tienkung_dex_*`，与仓库机型命名一致。

原始几何、惯量、关节限位、`mimic` 关系均未修改。

## 目录结构

```
tienkung_dex/
├── urdf/
│   ├── tienkung_dex_base.urdf      # 无手基础模型：39 links / 38 joints
│   ├── tienkung_dex_inspire.urdf   # + Inspire RH5DG2 双手：101 links / 100 joints（67 revolute）
│   └── tienkung_dex_brainco.urdf   # + BrainCo Revo2 双手：83 links / 82 joints（53 revolute）
├── meshes/                         # 40 个机身 STL
├── hands/
│   ├── rh5dg2_l/{meshes,urdf,config}
│   ├── rh5dg2_r/{meshes,urdf,config}
│   └── revo2/{left,right}/meshes
├── usd/                            # URDF→USD 转换产物（inspire / brainco 已入库，改 URDF 后重新生成）
│   └── configuration/              # Isaac Lab 生成的 base/physics/robot/sensor 分层 USD
├── textures/                       # logo 贴图（从 tienkung_pro 克隆，避免跨机型依赖）
└── 关键参数/                        # URDF关键参数表（质量/质心/惯量/关节轴向/限位/速度/力矩）
```

## 网格路径改写对照

| 原始引用 | 本目录内路径 |
|----------|--------------|
| `package://tiangong3_urdf/meshes/` | `../meshes/` |
| `package://RH5DG2_L/meshes/` | `../hands/rh5dg2_l/meshes/` |
| `package://RH5DG2_R/meshes/` | `../hands/rh5dg2_r/meshes/` |
| `package://revo2_description/meshes/revo2_left_hand/` | `../hands/revo2/left/meshes/` |
| `package://revo2_description/meshes/revo2_right_hand/` | `../hands/revo2/right/meshes/` |

校验结果：三套 URDF 的 200 / 146 / 76 处网格引用全部可在本目录内解析，无缺失。

## USD 转换

`usd/` 下的 **Inspire / BrainCo 两套 USD 已随仓库入库**（Git LFS），正常仿真无需执行本节；
只有修改 URDF/网格、或需要换配色重新生成时才需要重跑。

转换需要 Isaac Sim 容器（`ubt_sim/docker`）与 GPU。容器内项目挂载在 `/ubt_sim`：

```bash
bash docker/run.sh start                       # 首次需先 build / init

# 全部变体（base / inspire / brainco）
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh scripts/import_tienkung_dex_usd.py --force

# 仅首期使用的双手 Inspire 模型
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/import_tienkung_dex_usd.py --variant inspire --force

# 转换后自检（关节名、顺序、限位、质量）
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/check_tienkung_dex_usd.py --variant inspire
```

转换参数约定：

- `fix_base=False`（机器人以基座位姿放置在场景中，与 `tienkung_pro` / `walker_s2` 一致）；
- `merge_fixed_joints=False`（保留 `*_tcp_link`、相机、IMU 等固定件，便于挂载传感器）；
- `mimic` 关节转普通关节（`--mimic-as-normal`，默认开启）：V3 每只手 5 个 `mimic`（`thumb_pip←thumb_mcp`、`index_dip←index_pip`、`middle_dip`、`ring_dip`、`pinky_dip`），转普通关节后由驱动层按同样倍率下发，避免依赖 PhysX mimic 支持；
- 关节驱动默认 `stiffness=0`、`damping=1.0`，实际控制增益在 `ubt_sim/source/ubt_sim/devices/tienkung_dex/config.py` 按关节分组覆盖。

### 外观配色

URDF 里机身所有 link 共用同一个 0.753 灰，转换出来是均匀灰模。转换脚本结束时会调用
`scripts/recolor_tienkung_dex_usd.py`，按 link 名把 `configuration/<variant>_base.usd`
里的 OmniPBR 材质改成 `tienkung_pro` 已调好的配色（`--scheme pro`，默认）：
黑色主体/胸甲/骨盆/脚掌、灰色大腿外壳与小腿、深灰肩肘腕关节、金属踝关节、
白色手掌与指根外壳。只改材质的颜色/金属度/粗糙度，不动几何、质量与关节。

pro 的机身本身也是纯色材质（`tienkung_pro_v2.usd` 的 `black`/`grey`/`dark_grey`/
`metal`/`white`/`head_black`），没有贴图（仅胸前 logo 用 PNG，dex 网格没有 UV，
改用下面的[贴花平面](#贴-logo贴花平面)实现）。
调色板除上述基础色外还有 `ProHeadBlack`（纯黑 + roughness 0，对应 pro 头部外壳与摄像头的
镜面高光黑，用在 `head_yaw_link` / `camera_head_link`），以及 dex 上没有对应零件的
备用色 `ProBlue` / `ProStopRed`（建 `.blend` 时用 fake user 保活，便于随手换色）。

```bash
# 单独调整配色（改完 PRO_PALETTE 或 pro_palette_for_link 后重跑）
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/recolor_tienkung_dex_usd.py --variant inspire

# 早期按参考照片分的五色方案（亮银/枪灰/石墨/蓝/青）
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/recolor_tienkung_dex_usd.py --variant inspire --scheme reference

# 只预览分组、不写 USD
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/recolor_tienkung_dex_usd.py --variant all --dry-run
```

转换时加 `--no-recolor` 可保留 URDF 原色；配色写在 base 层，重新转换会覆盖，
所以需要重新执行上面的命令（或直接重跑 `import_tienkung_dex_usd.py`，默认自动配色）。

#### 在 Blender 里人工调色

脚本里的 `pro_palette_for_link` 只能做到「一个 link 一种颜色」。要按零件细调
（比如 `waist_pitch_link` 里胸甲、腰环、内部结构分开上色），走 Blender 往返流程，
三个脚本都在宿主机用 Blender 4.0.2 跑（发行版不带 USD，所以直接读 URDF 的 STL，
几何与 USD 同源、面顺序一致）：

```bash
# 1) 建调色场景；--split-links 在同一次保存里把大 link 按 loose parts 拆成零件，
#    材质按区域拆分（Head_* / Torso_* / LeftArm_* / RightArm_* / LeftLeg_* /
#    RightLeg_* / LeftHand_* / RightHand_*），不同区域互不影响
blender -b --factory-startup --python scripts/build_dex_blender_scene.py -- \
    --variant inspire --split-links waist_pitch_link \
    --out ~/dex_recolor/tienkung_dex_inspire.blend

# 2) 在 Blender 里改颜色后保存，导出每个物体/零件的颜色
blender -b --factory-startup --python scripts/read_dex_blender_colors.py -- \
    --blend ~/dex_recolor/tienkung_dex_inspire.blend --out /tmp/dex_colors.json

# 3) 写回 USD（容器内）
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/apply_dex_colors_json.py --variant inspire --colors /tmp/dex_colors.json
```

- `split_blend_link_parts.py` 也可单独对已有 `.blend` 拆分（`--link` 可重复），
  零件命名为 `<link>_pNN`、各自独立材质副本，面号映射存进文本块 `part_faces`；
- 已拆分的 link 写回时在 `/visuals/<link>/<link>/mesh` 上按零件面号建 GeomSubset，
  每个 subset 单独绑 `Custom_<HEX>_m<金属度>_r<粗糙度>` 材质；原来的强绑定从视觉
  Xform 挪到 mesh 上，避免盖住 subset；
- 写回前会校验零件面号是否**不重不漏**覆盖 mesh 的全部面，不满足则跳过该 link；
- 只改材质与 subset，几何、质量、关节、碰撞体不动。重复执行会先清掉旧 subset，
  结果幂等。

##### 贴 logo（贴花平面）

dex 的 STL 没有 UV，logo 不能直接贴到机身上。做法是先在世界坐标系里建一块带贴图的
小平面（decal），在调色场景里手动摆位，再随配色一起写回 USD。贴图用本目录
`textures/` 下的副本（`logo_xingzhe.png` 胸口、`logo_zuhe.png` 腰部，均从
`tienkung_pro/textures/` 克隆），不再引用 pro 的目录。

```bash
# 1) 生成贴花平面（读调色场景取 link 位姿，只读不保存；输出独立的 .blend）
blender -b --factory-startup --python scripts/add_dex_blender_decal.py -- \
    --blend ~/dex_recolor/tienkung_dex_inspire.blend \
    --texture assets/robots/tienkung_dex/textures/logo_xingzhe.png \
    --name xingzhe --link waist_pitch_link \
    --out ~/dex_recolor/decal_xingzhe.blend
```

在调色场景里 `File -> Append` 选中该 `.blend` 的 Object `decal_<link>_<name>`，
用 `G`/`S`/`R` 手动对位（贴花是独立物体，不影响机器人网格），保存调色场景；
之后的导出/写回命令与上一节相同。

- 贴花物体带自定义属性 `decal` / `source_link` / `texture`，`read_dex_blender_colors.py`
  会把它单独识别出来，按**四个角点的 link 局部坐标**写进 JSON；
- 摆位后贴花往往已离开创建时的 link，导出时按到各 link 网格表面的最近距离重新归属
  （阈值 0.08 m），日志会打印改挂结果；
- 写回时建 `Decal_<name>`（OmniPBR + 漫反射贴图，透明度取同一张 PNG 的 alpha）与
  `/visuals/<link>/decals/decal_<name>` 四边形。贴花挂在视觉 Xform **外面**：
  继承 link 变换，但不继承普通 link 上的 `strongerThanDescendants` 强绑定，
  否则贴花材质会被机身材质盖住；
- 只新增 decals 子树与 `Looks/Decal_*`，几何、质量、关节、碰撞体不动，重复执行覆盖同名贴花。

> 两套手部变体各自对应独立任务：`UBTSim-TienkungDex-Parlor-v0`（Inspire）与
> `UBTSim-TienkungDex-Brainco-Parlor-v0`（BrainCo），USD 也分别生成。
>
> 注意：`USD候选资产包/`（x-humanoid 候选模型，81 links）与 V3 不是同一套几何，未采用，避免腕部/手部安装位姿错位。

## 关节结构（V3 + Inspire）

| 分组 | 数量 | 关节 |
|------|------|------|
| 左/右腿 | 6 + 6 | `hip_pitch/hip_roll/hip_yaw/knee_pitch/ankle_pitch/ankle_roll` |
| 腰 | 3 | `waist_yaw / waist_roll / waist_pitch` |
| 头 | 2 | `head_yaw / head_pitch` |
| 左/右臂 | 7 + 7 | `shoulder_pitch/shoulder_roll/shoulder_yaw/elbow_pitch/elbow_yaw/wrist_pitch/wrist_roll` |
| 单手（RH5DG2） | 18 | 拇指 `yaw/mcp/pip/dip`，食指 `yaw/mcp/pip/dip`，中指 `yaw/mcp/pip/dip`，无名指 `mcp/pip/dip`，小指 `mcp/pip/dip` |
| 合计 | 67 revolute + 33 fixed | 101 links |

与旧 EVT2 参考实现（右手 13 通道、含 `*_index_pitch_joint`）不同，V3 手部为 18 关节、含 `*_yaw_joint`，通道映射以本模型为准。

## 许可

UBTECH 专有资产，授权范围见 `ubt_IL/docs/LICENSES/`；仅限本项目内使用，不得单独分发。
