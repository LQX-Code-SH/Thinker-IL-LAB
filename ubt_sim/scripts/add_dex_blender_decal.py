#!/usr/bin/env python3
"""生成一块带贴图的贴花平面（logo），供在 Blender 里手动摆位。

dex 的 STL 网格没有 UV，无法直接复用 ``tienkung_pro`` 的 logo 贴图材质，
所以在胸口/腰部这类近似平面的位置放一块独立的小平面（decal）来贴图。

脚本读取调色场景（不需要保存，只借它的 link 位姿和网格范围），在目标 link
正前方生成一块竖直平面，单独存成一个 .blend：

    blender -b --factory-startup --python scripts/add_dex_blender_decal.py -- \
        --blend ~/dex_recolor/tienkung_dex_inspire.blend \
        --texture assets/robots/tienkung_pro/textures/logo_xingzhe.png \
        --name xingzhe --link waist_pitch_link \
        --out ~/dex_recolor/decal_xingzhe.blend

然后在调色场景里 File -> Append -> 选中该 .blend 的 Object
-> ``decal_waist_pitch_link_xingzhe`` 追加进来，用 G/S/R 手动对位。

平面的世界变换与调色场景一致（同一个 GROUND_Z 抬高、同一个 link 位姿），
所以追加进来就落在机器人胸前的正确位置。物体名固定为
``decal_<link>_<name>``，自定义属性 ``source_link`` / ``texture`` 供后续写回 USD。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _link_objects(link: str):
    """目标 link 的网格物体；link 被拆成零件时按 ``source_link`` 收集。

    已追加的贴花也带 ``source_link``，必须排除，否则会被当成网格的一部分。
    """
    return [
        obj
        for obj in bpy.data.objects
        if obj.type == "MESH"
        and not obj.get("decal")
        and not obj.name.startswith("decal_")
        and (obj.name == link or obj.get("source_link") == link)
    ]


def _local_vertices(objs) -> np.ndarray:
    chunks = []
    for obj in objs:
        mesh = obj.data
        co = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
        mesh.vertices.foreach_get("co", co)
        chunks.append(co.reshape(-1, 3))
    return np.concatenate(chunks)


def _image_size(path: Path) -> tuple[int, int]:
    import struct

    header = path.read_bytes()[:33]
    return struct.unpack(">II", header[16:24])


def _make_material(name: str, texture: Path):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    tex = nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(str(texture), check_existing=True)
    tex.image.colorspace_settings.name = "sRGB"
    tex.interpolation = "Cubic"
    links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    if tex.image.depth == 32:  # RGBA：logo 通常带透明底
        links.new(tex.outputs["Alpha"], bsdf.inputs["Alpha"])
        mat.blend_method = "BLEND"
        if hasattr(mat, "shadow_method"):
            mat.shadow_method = "NONE"
    bsdf.inputs["Roughness"].default_value = 0.5
    bsdf.inputs["Metallic"].default_value = 0.0
    mat.use_backface_culling = False
    return mat


def _make_plane(name: str, width: float, tex_wh: tuple[int, int]):
    """在 yz 平面建一个朝 +x 的矩形（平面局部坐标 x=0）。"""
    img_w, img_h = tex_wh
    height = width * img_h / img_w
    hw, hh = width / 2.0, height / 2.0
    verts = [(0.0, -hw, -hh), (0.0, hw, -hh), (0.0, hw, hh), (0.0, -hw, hh)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], [(0, 1, 2, 3)])
    mesh.update()
    uv = mesh.uv_layers.new(name="UVMap")
    for loop, (u, v) in zip(mesh.loops, [(0, 0), (1, 0), (1, 1), (0, 1)]):
        uv.data[loop.index].uv = (u, v)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj, height


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--blend", required=True, help="调色场景 .blend（只读，不会保存）")
    parser.add_argument("--texture", required=True, help="贴图 PNG")
    parser.add_argument("--name", required=True, help="贴花短名，如 xingzhe / zuhe")
    parser.add_argument("--link", default="waist_pitch_link", help="贴花所属 link")
    parser.add_argument("--out", default="", help="输出的贴花 .blend，默认 ~/dex_recolor/decal_<name>.blend")
    parser.add_argument("--width", type=float, default=0.10, help="贴花宽度（米），高度按图片比例")
    parser.add_argument("--z", type=float, default=0.22, help="贴花中心在 link 局部坐标的 z")
    parser.add_argument("--y", type=float, default=0.0, help="贴花中心在 link 局部坐标的 y（左右）")
    parser.add_argument("--gap", type=float, default=0.004, help="与网格表面的间隙（米）")
    args = parser.parse_args(argv)

    texture = Path(args.texture).expanduser().resolve()
    if not texture.is_file():
        raise SystemExit(f"找不到贴图: {texture}")
    out_path = (
        Path(args.out).expanduser()
        if args.out
        else Path.home() / "dex_recolor" / f"decal_{args.name}.blend"
    )

    bpy.ops.wm.open_mainfile(filepath=str(Path(args.blend).expanduser()))
    objs = _link_objects(args.link)
    if not objs:
        raise SystemExit(f"找不到 link 网格: {args.link}")

    tex_wh = _image_size(texture)
    height = args.width * tex_wh[1] / tex_wh[0]
    verts = _local_vertices(objs)

    # 在贴花覆盖的 z 区间、y 区间里找最靠前的表面，保证整块平面都在表面之外
    z_lo, z_hi = args.z - height / 2.0, args.z + height / 2.0
    y_lo, y_hi = args.y - args.width / 2.0, args.y + args.width / 2.0
    region = verts[
        (verts[:, 2] >= z_lo - 0.005)
        & (verts[:, 2] <= z_hi + 0.005)
        & (verts[:, 1] >= y_lo - 0.005)
        & (verts[:, 1] <= y_hi + 0.005)
    ]
    if len(region) < 10:
        raise SystemExit("该位置没有网格顶点，检查 --z / --y")
    x_front = float(region[:, 0].max())
    local_pos = (x_front + args.gap, args.y, args.z)
    world = objs[0].matrix_world @ Matrix.Translation(local_pos)
    world_pos = world.translation

    print(
        f"[INFO] {args.link}: {len(objs)} 个物体, 局部范围 z={verts[:,2].min():.3f}..{verts[:,2].max():.3f}"
    )
    print(
        f"[INFO] 表面 x_max={x_front:.4f}, 贴花局部中心={tuple(round(v, 4) for v in local_pos)}, "
        f"世界中心={tuple(round(v, 4) for v in world_pos)}"
    )
    print(f"[INFO] 尺寸 {args.width:.3f} x {height:.3f} m")

    # 只保留贴花平面的干净场景
    bpy.ops.wm.read_factory_settings(use_empty=True)
    obj, _ = _make_plane(f"decal_{args.link}_{args.name}", args.width, tex_wh)
    obj.matrix_world = world
    obj.data.materials.append(_make_material(f"decal_{args.name}", texture))
    obj["decal"] = True
    obj["source_link"] = args.link
    obj["texture"] = str(texture)
    obj["width"] = args.width

    note = bpy.data.texts.new("README")
    note.write(
        "贴花平面\n"
        f"  物体: {obj.name}\n"
        f"  link: {args.link}\n"
        f"  贴图: {texture}\n"
        f"  尺寸: {args.width:.3f} x {height:.3f} m\n\n"
        "用法：在调色场景里 File -> Append -> 本文件 -> Object -> "
        f"{obj.name}，\n"
        "然后用 G（移动）/ S（缩放）/ R（旋转）对位；贴花是独立物体，不影响机器人网格。\n"
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out_path))
    print(f"[INFO] 已保存 {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
