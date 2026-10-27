"""场景 USD 的任务级覆盖（只在加载后的 stage 生效，不改资产文件）。

``assets/scenes/parlor/scene_v2.usd`` 被四种机型共用（walker_s2、tienkung_pro、
tienkung_dex 的两个手型变体），直接改文件会连带影响其他任务。所以需要"这台机器
看到的场景高一点/低一点"时，在任务 YAML 的 ``scene`` 段落里描述覆盖，由这里在
场景加载完成后对 prim 打补丁。

目前支持一种覆盖：沿 z 轴拉伸 ``table`` 来抬高桌面，并把摆在桌面上的物体
（苹果、盘子）按桌面抬升量同步上移。抬升量由加载后的世界包围盒算出来，桌面资产
换了也不会对不上。
"""

from __future__ import annotations

from collections.abc import Sequence

import isaaclab.sim as sim_utils
import omni.usd
from pxr import Gf, Usd, UsdGeom


def _find_xform_op(xform: UsdGeom.Xformable, op_type) -> UsdGeom.XformOp | None:
    """按类型取 xformOp（``GetScaleOp`` 等即使在 op 不存在时也会返回句柄，不能直接用）。"""
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == op_type:
            return op
    return None


def lift_table(
    scene_path: str,
    z_scale: float,
    table_prim: str = "table",
    lifted_prims: Sequence[str] = (),
) -> float:
    """把 ``scene_path/<table_prim>`` 沿 z 轴拉伸到 ``z_scale`` 倍，并抬高台上物体。

    拉伸以 table 自身原点为基准（parlor 场景里原点在地面，z=0），所以桌面高度
    直接乘 ``z_scale``——桌腿和台面一起变高，桌子整体不会浮起来。

    Args:
        scene_path: 场景根 prim（如 ``/World/envs/env_0/Scene``）。
        z_scale: z 轴缩放倍数，1.1 表示桌面抬高 10%。
        table_prim: 桌面 prim 名（相对 ``scene_path``）。
        lifted_prims: 需要跟随桌面抬高的 prim 名。

    Returns:
        桌面抬升的高度（米），同时就是台上物体的上移量。
    """
    stage = omni.usd.get_context().get_stage()
    table = stage.GetPrimAtPath(f"{scene_path}/{table_prim}")
    if not table.IsValid():
        raise ValueError(f"场景中没有 {scene_path}/{table_prim}，无法抬高桌面")

    # 桌面当前高度与拉伸基准点，都取世界坐标（env 被平移时同样成立）
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    top_z = bbox_cache.ComputeWorldBound(table).ComputeAlignedRange().GetMax()[2]
    pivot_z = UsdGeom.XformCache().GetLocalToWorldTransform(table).ExtractTranslation()[2]

    table_xform = UsdGeom.Xformable(table)
    scale_op = _find_xform_op(table_xform, UsdGeom.XformOp.TypeScale)
    if scale_op is None:
        scale_op = table_xform.AddScaleOp()
    scale_x, scale_y, scale_z = scale_op.Get()
    scale_op.Set(Gf.Vec3f(scale_x, scale_y, scale_z * z_scale))

    lift = float(top_z - pivot_z) * (z_scale - 1.0)
    for name in lifted_prims:
        prim = stage.GetPrimAtPath(f"{scene_path}/{name}")
        if not prim.IsValid():
            raise ValueError(f"场景中没有 {scene_path}/{name}，无法随桌面抬高")
        translate_op = _find_xform_op(UsdGeom.Xformable(prim), UsdGeom.XformOp.TypeTranslate)
        if translate_op is None:
            raise ValueError(f"{scene_path}/{name} 没有 xformOp:translate，无法抬高")
        x, y, z = translate_op.Get()
        translate_op.Set(Gf.Vec3d(x, y, z + lift))
    # 仿真里没开 Python logging（sim_runner.py 也是 print），这里跟它保持一致
    print(
        f"[INFO] {scene_path}: table z×{z_scale:.2f}，桌面 {float(top_z):.4f} → "
        f"{float(top_z) + lift:.4f} m；{list(lifted_prims)} 上移 {lift:.4f} m",
        flush=True,
    )
    return lift


def spawn_scene_with_table_lift(
    prim_path: str,
    cfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    *,
    table_z_scale: float = 1.0,
    table_prim: str = "table",
    lifted_prims: Sequence[str] = (),
    **kwargs,
) -> Usd.Prim:
    """``UsdFileCfg`` 的 spawn 函数：加载场景后按需抬高桌面。

    用法是把它塞进 ``sim_utils.UsdFileCfg(func=...)``（见各任务的 env_cfg）。
    注意 ``spawn_from_usd`` 自己会展开 ``{ENV_REGEX_NS}`` 并克隆到其余 env，
    这里必须用同样的方式先展开父级路径，才能对每个 env 都打上覆盖。
    """
    scene_parent, scene_name = str(prim_path).rsplit("/", 1)
    env_roots = sim_utils.find_matching_prim_paths(scene_parent) or [scene_parent]

    prim = sim_utils.spawn_from_usd(
        prim_path, cfg, translation=translation, orientation=orientation, **kwargs
    )
    if table_z_scale != 1.0:
        for env_root in env_roots:
            lift_table(f"{env_root}/{scene_name}", table_z_scale, table_prim, lifted_prims)
    return prim
