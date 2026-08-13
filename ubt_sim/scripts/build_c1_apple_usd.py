#!/usr/bin/env python3
"""Build the C1 task apple with the original visual and a stable sphere collider."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


DEFAULT_SOURCE = Path("/ubt_sim/assets/scenes/parlor/scene_v2.usd")
DEFAULT_OUTPUT = Path("/ubt_sim/assets/robots/walker_c1/c1_task_apple.usda")
SOURCE_APPLE_PRIM = Sdf.Path("/World/apple")
APPLE_RADIUS_M = 0.040
APPLE_MASS_KG = 0.10
APPLE_FRICTION = 1.20


def _remove_physics_from_visual(visual_prim: Usd.Prim) -> None:
    for prim in Usd.PrimRange.AllPrims(visual_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI(prim)
            collision_api.GetCollisionEnabledAttr().Set(False)
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI)


def _center_and_scale_visual(stage: Usd.Stage, visual_prim: Usd.Prim) -> None:
    xformable = UsdGeom.Xformable(visual_prim)
    xformable.ClearXformOpOrder()

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=False,
    )
    bbox = bbox_cache.ComputeLocalBound(visual_prim).ComputeAlignedBox()
    source_size = bbox.GetSize()
    source_center = bbox.GetMidpoint()
    max_size = max(source_size)
    if max_size <= 0.0:
        raise RuntimeError(f"invalid source apple bounds: {source_size}")

    scale = (2.0 * APPLE_RADIUS_M) / max_size
    transform = Gf.Matrix4d(1.0)
    transform.SetScale(Gf.Vec3d(scale))
    transform.SetTranslateOnly(Gf.Vec3d(*(float(-value * scale) for value in source_center)))
    xformable.AddTransformOp(opSuffix="fitToCollider").Set(transform)
    stage.GetRootLayer().Save()


def build(source_path: Path, output_path: Path) -> None:
    source_path = source_path.resolve()
    output_path = output_path.resolve()
    if not source_path.is_file():
        raise FileNotFoundError(source_path)

    source_stage = Usd.Stage.Open(str(source_path))
    if not source_stage or not source_stage.GetPrimAtPath(SOURCE_APPLE_PRIM):
        raise RuntimeError(f"missing source apple prim {SOURCE_APPLE_PRIM} in {source_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()

    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/C1Apple").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(APPLE_MASS_KG)

    collision = UsdGeom.Sphere.Define(stage, "/C1Apple/Collision")
    collision.CreateRadiusAttr(APPLE_RADIUS_M)
    collision.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.1, 0.1)])
    UsdGeom.Imageable(collision).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(collision.GetPrim())

    physics_material = UsdShade.Material.Define(
        stage, "/C1Apple/PhysicsMaterials/AppleGripMaterial"
    )
    material_api = UsdPhysics.MaterialAPI.Apply(physics_material.GetPrim())
    material_api.CreateStaticFrictionAttr(APPLE_FRICTION)
    material_api.CreateDynamicFrictionAttr(APPLE_FRICTION)
    material_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(collision.GetPrim()).Bind(
        physics_material, materialPurpose="physics"
    )

    visual = UsdGeom.Xform.Define(stage, "/C1Apple/Visual").GetPrim()
    relative_source = os.path.relpath(source_path, output_path.parent)
    visual.GetReferences().AddReference(relative_source, SOURCE_APPLE_PRIM)
    stage.GetRootLayer().Save()
    stage.Reload()

    visual = stage.GetPrimAtPath("/C1Apple/Visual")
    _remove_physics_from_visual(visual)
    _center_and_scale_visual(stage, visual)

    check_stage = Usd.Stage.Open(str(output_path))
    check_root = check_stage.GetPrimAtPath("/C1Apple")
    check_visual = check_stage.GetPrimAtPath("/C1Apple/Visual")
    collisions = [
        prim
        for prim in Usd.PrimRange.AllPrims(check_root)
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    visual_collisions = [
        prim
        for prim in Usd.PrimRange.AllPrims(check_visual)
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if [str(prim.GetPath()) for prim in collisions] != ["/C1Apple/Collision"]:
        raise RuntimeError(f"unexpected collision prims: {[prim.GetPath() for prim in collisions]}")
    if visual_collisions:
        raise RuntimeError(f"visual still has collision APIs: {[prim.GetPath() for prim in visual_collisions]}")

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=False,
    )
    visual_bbox = bbox_cache.ComputeLocalBound(check_visual).ComputeAlignedBox()
    visual_size = visual_bbox.GetSize()
    visual_center = visual_bbox.GetMidpoint()
    if abs(max(visual_size) - 2.0 * APPLE_RADIUS_M) > 1.0e-4:
        raise RuntimeError(f"visual diameter mismatch: {visual_size}")
    if max(abs(value) for value in visual_center) > 1.0e-4:
        raise RuntimeError(f"visual is not centered: {visual_center}")

    print(f"[OK] wrote {output_path}")
    print(
        "[OK] visual size "
        f"[{visual_size[0]:.4f}, {visual_size[1]:.4f}, {visual_size[2]:.4f}] m; "
        f"sphere collider radius={APPLE_RADIUS_M:.3f} m"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    build(args.source, args.output)


if __name__ == "__main__":
    main()
