#!/usr/bin/env python3
"""把 ``read_dex_blender_colors.py`` 导出的 JSON 写回 tienkung_dex 的 USD。

在 Isaac Sim 容器内运行：

    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/apply_dex_colors_json.py --variant inspire --colors /tmp/dex_colors.json

普通 link：每个 link 的 (颜色, 金属度, 粗糙度) 相同的会复用同一个 ``Custom_*`` 材质，
绑定写进 ``configuration/<variant>_base.usd`` 的 ``/visuals/<link>/<link>``。

被拆成零件的 link（JSON 里带 ``faces``，见 ``split_blend_link_parts.py``）：
在 ``/visuals/<link>/<link>/mesh`` 上按零件面号建 GeomSubset，每个 subset 单独绑材质；
原来的强绑定从 Xform 挪到 mesh 上，避免盖住 subset 绑定。几何/物理/关节不变。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from recolor_tienkung_dex_usd import (  # noqa: E402
    ASSET_ROOT,
    USD_DIR,
    VARIANTS,
    _base_usd_path,
    _find_template_material,
    _iter_link_visuals,
    _robot_root,
    _set_input,
    ensure_pxr,
)


def _decal_texture_asset(texture: str) -> str:
    """贴图统一用 ``tienkung_dex/textures`` 下的副本，按相对 base 层的路径引用。"""
    name = Path(texture).name
    if (ASSET_ROOT / "textures" / name).is_file():
        return f"../../textures/{name}"
    return texture


def _decal_short_name(texture: str) -> str:
    stem = Path(texture).stem
    return stem[len("logo_"):] if stem.startswith("logo_") else stem


def _paint_key(paint: dict) -> tuple:
    color = tuple(round(float(c), 5) for c in paint["color"][:3])
    return (
        color,
        round(float(paint.get("metallic", 0.0)), 3),
        round(float(paint.get("roughness", 0.5)), 3),
    )


def _material_name(key: tuple, index: int) -> str:
    color, metallic, roughness = key
    hexcode = "".join(f"{int(round(min(max(c, 0.0), 1.0) * 255)):02X}" for c in color)
    return f"Custom_{hexcode}_m{int(round(metallic * 100))}_r{int(round(roughness * 100))}"


def _resolve_visual(name: str, link_prims: dict[str, list]) -> tuple[str, int | None]:
    """把 Blender 物体名还原成 (link, 第几个 visual)。

    一个 link 有多个 ``<visual>`` 时，``build_dex_blender_scene.py`` 把第 2 个命名成
    ``<link>_1``、第 3 个 ``<link>_2``…；USD 里对应的内层 Xform 按同样顺序排列。
    """
    if name in link_prims:
        return name, 0
    base, _, tail = name.rpartition("_")
    if tail.isdigit() and base in link_prims:
        return base, int(tail)
    return name, None


def _classify(colors: dict):
    """拆成普通 link 与带零件的 link。"""
    plain: dict[str, dict] = {}
    parts: dict[str, list[tuple[str, list[int], dict]]] = {}
    for name, entry in colors.items():
        link = entry.get("link", name)
        faces = entry.get("faces")
        if faces:
            parts.setdefault(link, []).append((name, faces, entry))
        else:
            plain.setdefault(link, entry)
    return plain, parts


def _check_faces(link: str, entries, n_faces: int) -> bool:
    cover = bytearray(n_faces)
    for name, faces, _ in entries:
        for f in faces:
            if not 0 <= f < n_faces:
                print(f"[ERROR] {link}/{name} 面号越界: {f} (总面数 {n_faces})")
                return False
            cover[f] += 1
    missing = cover.count(0)
    overlap = sum(1 for c in cover if c > 1)
    if missing or overlap:
        print(f"[ERROR] {link} 零件面号覆盖不完整: 缺 {missing} 面, 重叠 {overlap} 面")
        return False
    return True


def apply_colors(usd_path: str | Path, colors: dict, dry_run: bool = False) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    base_path = _base_usd_path(Path(usd_path))
    if not base_path.is_file():
        raise FileNotFoundError(f"找不到 base 层 USD: {base_path}")

    stage = Usd.Stage.Open(str(base_path))
    if stage is None:
        raise RuntimeError(f"无法打开 USD: {base_path}")

    root = _robot_root(stage)
    # 一个 link 可能有多个 <visual>（内层 Xform 名 = mesh 文件名），按顺序收集
    link_prims: dict[str, list] = {}
    for link, prim in _iter_link_visuals(stage):
        link_prims.setdefault(link, []).append(prim)

    decals = {name: entry for name, entry in colors.items() if entry.get("decal")}
    colors = {name: entry for name, entry in colors.items() if not entry.get("decal")}

    plain, parts = _classify(colors)

    missing = sorted(
        name
        for name in (set(plain) | set(parts))
        if _resolve_visual(name, link_prims)[1] is None
    )
    if missing:
        print(f"[WARN] JSON 里的 link 在 USD 中不存在（忽略）: {', '.join(missing[:6])}")

    template = _find_template_material(stage, root)
    template_spec = template.GetPrimStack()[0]
    root_layer = stage.GetRootLayer()
    looks_path = root.AppendChild("Looks")
    stage.DefinePrim(looks_path)

    material_cache: dict[tuple, UsdShade.Material] = {}

    def material_for(paint: dict) -> UsdShade.Material:
        key = _paint_key(paint)
        material = material_cache.get(key)
        if material is not None:
            return material
        name = _material_name(key, len(material_cache))
        mat_path = looks_path.AppendChild(name)
        if not stage.GetPrimAtPath(mat_path):
            Sdf.CopySpec(template_spec.layer, template_spec.path, root_layer, mat_path)
        mat_prim = stage.GetPrimAtPath(mat_path)
        shader = next(c for c in mat_prim.GetChildren() if c.GetTypeName() == "Shader")
        color, metallic, roughness = key
        _set_input(
            shader, "inputs:diffuse_color_constant", Gf.Vec3f(*color), Sdf.ValueTypeNames.Color3f
        )
        _set_input(shader, "inputs:metallic_constant", metallic, Sdf.ValueTypeNames.Float)
        _set_input(
            shader,
            "inputs:reflection_roughness_constant",
            roughness,
            Sdf.ValueTypeNames.Float,
        )
        material = UsdShade.Material(mat_prim)
        material_cache[key] = material
        return material

    decal_materials: dict[str, UsdShade.Material] = {}

    def decal_material_for(texture: str) -> UsdShade.Material:
        """OmniPBR + 贴图：漫反射贴图用上面的 PNG，透明度取同一张图的 alpha。"""
        cached = decal_materials.get(texture)
        if cached is not None:
            return cached
        mat_path = looks_path.AppendChild(f"Decal_{_decal_short_name(texture)}")
        if not stage.GetPrimAtPath(mat_path):
            Sdf.CopySpec(template_spec.layer, template_spec.path, root_layer, mat_path)
        mat_prim = stage.GetPrimAtPath(mat_path)
        shader = next(c for c in mat_prim.GetChildren() if c.GetTypeName() == "Shader")
        asset = Sdf.AssetPath(_decal_texture_asset(texture))
        _set_input(
            shader,
            "inputs:diffuse_color_constant",
            Gf.Vec3f(1.0, 1.0, 1.0),
            Sdf.ValueTypeNames.Color3f,
        )
        _set_input(shader, "inputs:diffuse_texture", asset, Sdf.ValueTypeNames.Asset)
        _set_input(shader, "inputs:enable_opacity", True, Sdf.ValueTypeNames.Bool)
        _set_input(shader, "inputs:enable_opacity_texture", True, Sdf.ValueTypeNames.Bool)
        _set_input(shader, "inputs:opacity_texture", asset, Sdf.ValueTypeNames.Asset)
        _set_input(shader, "inputs:opacity_mode", 0, Sdf.ValueTypeNames.Int)  # mono_alpha
        _set_input(shader, "inputs:opacity_threshold", 0.0, Sdf.ValueTypeNames.Float)
        _set_input(shader, "inputs:metallic_constant", 0.0, Sdf.ValueTypeNames.Float)
        _set_input(
            shader, "inputs:reflection_roughness_constant", 0.5, Sdf.ValueTypeNames.Float
        )
        material = UsdShade.Material(mat_prim)
        decal_materials[texture] = material
        return material

    # ---- 普通 link --------------------------------------------------------
    plain_targets = []
    covered: dict[str, set[int]] = {}
    link_material: dict[str, UsdShade.Material] = {}
    for name, paint in plain.items():
        link, index = _resolve_visual(name, link_prims)
        prims = link_prims.get(link)
        if not prims or index is None or index >= len(prims):
            continue
        material = material_for(paint)
        plain_targets.append((prims[index], material))
        covered.setdefault(link, set()).add(index)
        if index == 0:
            link_material[link] = material

    # JSON 只给了一个 link 一项、但 USD 里有多个 visual（如 left_base_link）：
    # 其余 visual 沿用同一颜色，否则会留着 importer 的默认灰
    for link, material in link_material.items():
        for index, prim in enumerate(link_prims[link]):
            if index not in covered.get(link, set()):
                plain_targets.append((prim, material))

    # ---- 含零件的 link ----------------------------------------------------
    part_targets = []
    for link, entries in parts.items():
        prims = link_prims.get(link)
        if not prims:
            print(f"[WARN] 零件所属 link 不存在: {link}")
            continue
        xform = next(
            (
                prim
                for prim in prims
                if stage.GetPrimAtPath(prim.GetPath().AppendChild("mesh"))
            ),
            prims[0],
        )
        mesh_prim = stage.GetPrimAtPath(xform.GetPath().AppendChild("mesh"))
        if not mesh_prim or mesh_prim.GetTypeName() != "Mesh":
            print(f"[WARN] {link} 下没有 mesh prim，跳过零件配色")
            continue
        mesh = UsdGeom.Mesh(mesh_prim)
        n_faces = len(mesh.GetFaceVertexCountsAttr().Get() or [])
        if not _check_faces(link, entries, n_faces):
            continue
        subsets = [(name, faces, material_for(entry)) for name, faces, entry in entries]
        default_mat = material_for(max(entries, key=lambda item: len(item[1]))[2])
        part_targets.append((link, xform, mesh, subsets, default_mat))

    # ---- 贴花 ------------------------------------------------------------
    decal_targets = []
    for obj_name, entry in decals.items():
        link = entry.get("link", "")
        corners = entry.get("corners") or []
        texture = entry.get("texture", "")
        if len(corners) != 4:
            print(f"[WARN] 贴花 {obj_name} 角点数不是 4，跳过")
            continue
        prims = link_prims.get(link)
        if not prims:
            print(f"[WARN] 贴花 {obj_name} 的 link {link!r} 在 USD 中不存在，跳过")
            continue
        # 角点是 link 局部坐标：优先用与 link 同名的内层 Xform（视觉原点为单位阵）
        xform = next((prim for prim in prims if prim.GetName() == link), prims[0])
        if not texture:
            print(f"[WARN] 贴花 {obj_name} 没有贴图，跳过")
            continue
        decal_targets.append((obj_name, link, xform, corners, texture))

    # ---- 报告 -------------------------------------------------------------
    assignments: dict[str, list[str]] = {}
    for prim, material in plain_targets:
        assignments.setdefault(material.GetPath().name, []).append(prim.GetName())
    for link, _xform, _mesh, subsets, _default in part_targets:
        per_mat: dict[str, int] = {}
        for name, faces, material in subsets:
            per_mat[material.GetPath().name] = per_mat.get(material.GetPath().name, 0) + len(faces)
        for mat_name, face_count in sorted(per_mat.items()):
            assignments.setdefault(mat_name, []).append(f"{link}[{face_count}面]")

    for name, items in sorted(assignments.items()):
        preview = ", ".join(items[:4]) + (" ..." if len(items) > 4 else "")
        print(f"  {name:32s} {len(items):3d} 项  {preview}")
    if part_targets:
        print(f"[INFO] 零件配色 link: {', '.join(link for link, *_ in part_targets)}")
    for obj_name, link, _xform, _corners, texture in decal_targets:
        print(f"  {('Decal_' + _decal_short_name(texture)):32s}  贴花  {obj_name} -> /visuals/{link}/decals/")

    if dry_run:
        print("[INFO] --dry-run：未写入 USD")
        return

    # ---- 写入 -------------------------------------------------------------
    for prim, material in plain_targets:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.strongerThanDescendants
        )

    for link, xform, mesh, subsets, default_mat in part_targets:
        # 1) 去掉 Xform 上的强绑定，否则会盖住 subset 绑定
        for prop in ("material:binding", "material:binding:strength"):
            if xform.HasProperty(prop):
                xform.RemoveProperty(prop)

        # 2) 清掉根层里已存在的 subset，保证重复执行结果一致
        for child in mesh.GetPrim().GetChildren():
            if child.GetTypeName() == "GeomSubset" and root_layer.GetPrimAtPath(child.GetPath()):
                stage.RemovePrim(child.GetPath())

        # 3) mesh 上绑默认材质（兜底），再按零件建 subset
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(default_mat)
        for name, faces, material in subsets:
            subset = UsdGeom.Subset.CreateGeomSubset(
                mesh,
                name,
                UsdGeom.Tokens.face,
                Vt.IntArray(faces),
                UsdShade.Tokens.materialBind,
            )
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)

    # ---- 贴花 -------------------------------------------------------------
    for obj_name, link, xform, corners, texture in decal_targets:
        # 挂在 /visuals/<link> 下、与视觉 Xform 平级：这样既继承 link 的变换，
        # 又不会继承视觉 Xform 上的 material:binding（普通 link 是
        # strongerThanDescendants，挂进去会盖住贴花自己的材质）。
        outer_path = xform.GetPath().GetParentPath()
        inner_matrix = UsdGeom.Xformable(xform).GetLocalTransformation()
        decals_path = outer_path.AppendChild("decals")
        if not stage.GetPrimAtPath(decals_path):
            UsdGeom.Xform.Define(stage, decals_path)

        mesh_path = decals_path.AppendChild(f"decal_{_decal_short_name(texture)}")
        if stage.GetPrimAtPath(mesh_path) and root_layer.GetPrimAtPath(mesh_path):
            stage.RemovePrim(mesh_path)

        points = [
            Gf.Vec3f(*inner_matrix.Transform(Gf.Vec3d(*corner))) for corner in corners
        ]
        mesh = UsdGeom.Mesh.Define(stage, mesh_path)
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        gprim = UsdGeom.Gprim(mesh.GetPrim())
        if hasattr(gprim, "CreateDoubleSidedAttr"):
            gprim.CreateDoubleSidedAttr(True)
        st = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        )
        st.Set(
            [Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 0.0), Gf.Vec2f(1.0, 1.0), Gf.Vec2f(0.0, 1.0)]
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(decal_material_for(texture))

    if not root_layer.Save():
        raise RuntimeError(f"保存 USD 失败: {base_path}")
    print(f"[INFO] 已写入 {base_path}")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply Blender-edited per-link colors to tienkung_dex USDs.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--colors", required=True, help="read_dex_blender_colors.py 导出的 JSON")
    parser.add_argument(
        "--variant",
        default="inspire",
        choices=list(VARIANTS),
        help="要写回的变体。",
    )
    parser.add_argument("--usd-dir", default=str(USD_DIR), help="顶层 USD 所在目录。")
    parser.add_argument("--dry-run", action="store_true", help="只打印分组，不写 USD。")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    ensure_pxr()

    with open(args.colors, encoding="utf-8") as fh:
        colors = json.load(fh)

    usd_path = Path(args.usd_dir) / VARIANTS[args.variant]
    apply_colors(usd_path, colors, dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
