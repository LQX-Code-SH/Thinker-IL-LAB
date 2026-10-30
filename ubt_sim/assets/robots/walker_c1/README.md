# Walker C1（Astron）机器人资产

Walker C1 的整机模型包（URDF / meshes / USD / 关节参数），用于 Isaac Sim / Isaac Lab。

## 来源

| 内容 | 来源 |
|------|------|
| 整机模型包 | `Walker_C1_Model_All-main`（Apache-2.0，上游说明见 [README_upstream.md](README_upstream.md)） |
| 身体 | `walker_astron_description` |
| 左右手 | `ubt_left_hand_v3_lite_description` / `ubt_right_hand_v3_lite_description` |

上游原始包还带 `mujoco/`、`config/`、`launch/`、`docs/`、`script/urdf2usd.py`，
本仓库只入库仿真需要的部分。

## 入库后的两处改动

1. **URDF mesh 路径改写**：`package://walker_c1_description/meshes/...` → `../meshes/...`
   （避免依赖 ROS 包搜索路径；URDF 位于 `urdf/`，meshes 位于上一级 `meshes/`）；
2. **目录重排**：按本仓库其他机型（`tienkung_dex`）的结构组织为 `urdf/` + `meshes/` + `usd/`，
   旧资产（含来源不明的 `*_hands.usd`）移入 `legacy/` 仅用于本地回退，不入库。

上游 README 里的展示图（`docs/*.png`）未入库，`README_upstream.md` 中的图片链接会失效。

## 目录结构

```
walker_c1/
├── urdf/
│   ├── walker_c1_with_hands.urdf      # 带双手整机：65 joint（53 驱动）→ 仿真/IK 使用
│   ├── walker_c1.urdf                 # 无手整机：39 joint
│   ├── walker_c1_with_dummyhands.urdf # 带占位手
│   └── ubt_{left,right}_hand_v3_lite.urdf
├── meshes/{body,visual,left_hand,right_hand,dummy_hand}/   # STL
├── usd/
│   ├── walker_c1_merged.usd           # 仿真主资产：53 驱动关节 / 0 固定 / 5 相机 / 2 IMU + 配色
│   ├── configuration/                 # merged 的分层 USD（base/physics/robot/sensor）
│   ├── walker_c1_with_hands.usd       # 未合并固定关节的原始转换产物（对比/回溯用）
│   └── original/                      # 原始 sensorKpkd（无手，含 SubUSDs），仅作参考
├── colors.csv                         # 配色真源（83 条：link/visual → hex/roughness/metallic）
├── head_parts.csv                     # 头部 22 个 loose part 的面数与世界 bbox（用于匹配）
├── textures/logubt_warm_white.png     # 胸口 logo（暖白底不透明，贴合躯干色）
├── 关键参数/                           # 关节/连杆的质量、惯量、轴向、限位（CSV）
├── c1_task_apple.usda                 # 抓放任务苹果（本仓库追加，非上游）
└── legacy/                            # 旧资产回退（.gitignore，不入库）
```

## 三份 USD 的取舍

| 文件 | 关节 | 相机 | 用途 |
|------|------|------|------|
| `usd/walker_c1_merged.usd` | 53 驱动 + 0 固定 | 5（`head_stereo_left/right`、`head_fisheye_left/right`、`head_jaw`） | **仿真主用**，与 `devices/walker_c1/config.py` 的 53 关节分组一一对应 |
| `usd/walker_c1_with_hands.usd` | 53 驱动 + 12 固定辅助 | 同上 | 未合并版；12 个 `PhysicsFixedJoint` 在 Isaac 里不生效会导致上肢散架，仅作对比 |
| `usd/original/walker_astron_v1_sensorKpkd.usd` | 身体 31（无手） | 同上 5 路 | 仅回溯：与带手 URDF 的装配不是同一变体 |

两份 USD 的 articulation root 都在 `base_link`，默认 prim 为 `/walker_c1`；
单位米制、Z 轴向上，需 Isaac Sim 4.5+。

generated 版由 URDF 重建，**不自带 PD 增益/阻尼**，执行器参数完全由
`devices/walker_c1/config.py` 的 `ArticulationCfg` 提供。

## 与仿真配置的对应

| 配置项 | 值 | 可覆盖 |
|--------|-----|--------|
| 仿真 USD | `usd/walker_c1_merged.usd` | `UBT_SIM_WALKER_C1_USD_PATH` |
| IK/限位 URDF | `urdf/walker_c1_with_hands.urdf` | `UBT_SIM_ASSETS_ROOT`（改资产根） |

采集相机 `Camera_RGB` **不是**资产自带，而是 `task/walker_c1_parlor` 在
`head_pitch_link` 下按 YAML 的 offset 新建；USD 自带的 5 路相机保持原状。

## USD 生成与配色

流程与 `tienkung_dex` 一致，两个脚本都在 Isaac Sim 容器内运行：

```bash
# 1) URDF -> USD（merge_fixed_joints=True，转换结束自动配色）
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/import_walker_c1_usd.py --variant hands --force

# 2) 只重跑配色（改完 colors.csv 后用；幂等，可反复执行）
docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
    scripts/recolor_walker_c1_usd.py --force
```

### 为什么必须是 `merge_fixed_joints=True`

C1 的 URDF 有 12 个固定关节（肩基座、palm、hand_mount 等）夹在链条中间，
不合并时它们会变成独立刚体，仿真里直接“爆炸”。C1 没有 `mimic`、也没有
`*_tcp_link`，合并不会丢功能件。合并后：53 revolute / 0 fixed / 1 articulation root。

### 配色实现（复用 dex 方案）

- 材质：克隆 importer 生成的 **OmniPBR** 模板，只改
  `diffuse_color_constant` / `metallic_constant` / `reflection_roughness_constant`；
- 绑定：写在 `/visuals/<link>/<visual>` 的 Xform 上，`strongerThanDescendants`；
- **头部**：`head_pitch_link` 用 **22 个 `GeomSubset`**（面号由世界坐标 bbox 匹配
  `head_parts.csv` 得到）——有 subset 时必须先删掉该 link 的 Xform 强绑，否则会被盖住；
- **腕部**：`merge_fixed_joints` 会把 `hand_base` / `palm` 的 visual 挂到
  `wrist_roll_link` 下（**不是**融成一个 mesh），所以按 **visual Xform 名**取色、
  取不到再回退 link 名——这是白/黑/银/蓝灰能分层的关键；
- **胸口 logo**：`/visuals/waist_roll_link/decals/decal_logubt`，与视觉 Xform
  **平级**（避开 `strongerThanDescendants`），贴图用相对路径 `../../textures/...`。

### 配色表（`colors.csv`）

| 颜色 | Roughness | Metallic | 部位 |
|------|-----------|----------|------|
| `#FFF5E8` | 0.10 | 0.00 | 躯干、上臂、大腿、腕部法兰 |
| `#101010` | 1.00 | 0.00 | 肩 pitch、髋 pitch、踝 roll、腕 pitch、头部外壳 |
| `#3B3E4C` | 0.35 | 0.22 | 手指、手掌 |
| `#C0C0C0` | 0.35 | 0.80 | 手基座（银） |
| `#000000` | 0.00 | 0.00 | 头部前面罩 |
| `#010101` | 0.65 | 0.02 | 头部镜面黑件 |
| `#969696` | 0.50 | 0.00 | 内部件兜底 |

改配色只需改 `colors.csv` 再跑第 2 条命令；几何、质量、关节、碰撞体零改动。
