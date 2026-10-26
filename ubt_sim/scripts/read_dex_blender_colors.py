#!/usr/bin/env python3
"""从 Blender 调色场景导出每个 link 的材质颜色（JSON）。

在宿主机运行（Blender 自带 Python，不需要 Isaac Sim）：

    blender -b --factory-startup --python scripts/read_dex_blender_colors.py -- \
        --blend ~/dex_recolor/tienkung_dex_inspire.blend \
        --out /tmp/dex_colors.json

之后用 ``apply_dex_colors_json.py`` 把 JSON 写回 USD。
"""

from __future__ import annotations

import argparse
import json
import sys

import bpy
import numpy as np


def _parse_args() -> argparse.Namespace:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--blend", required=True, help="调色用 .blend 文件")
    parser.add_argument("--out", default="dex_colors.json", help="输出 JSON")
    return parser.parse_args(argv)


def _material_paint(mat):
    color = (0.5, 0.5, 0.5)
    metallic = 0.0
    roughness = 0.5
    if mat.use_nodes and mat.node_tree:
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf is not None:
            color = tuple(bsdf.inputs["Base Color"].default_value)[:3]
            metallic = float(bsdf.inputs["Metallic"].default_value)
            roughness = float(bsdf.inputs["Roughness"].default_value)
    return color, metallic, roughness


def _is_decal(obj) -> bool:
    """``add_dex_blender_decal.py`` 生成的贴花平面。"""
    return bool(obj.get("decal")) or obj.name.startswith("decal_")


def _nearest_link(center, link_meshes: dict[str, list], limit: float = 0.08):
    """按到网格表面的最近距离找贴花所属 link（摆位后可能已离开创建时的 link）。"""
    best_dist, best_link = limit, None
    for link, objs in link_meshes.items():
        for obj in objs:
            inv = obj.matrix_world.inverted()
            local_center = np.array(inv @ center)
            co = np.empty(len(obj.data.vertices) * 3, dtype=np.float32)
            obj.data.vertices.foreach_get("co", co)
            verts = co.reshape(-1, 3)
            dist = float(np.sqrt(np.min(np.sum((verts - local_center) ** 2, axis=1))))
            if dist < best_dist:
                best_dist, best_link = dist, link
    return best_link, best_dist


def _decal_row(obj, link_world: dict[str, object], link_meshes: dict[str, list]):
    """贴花按 link 局部坐标导出四个角点，写回 USD 时直接建四边形。"""
    declared = obj.get("source_link", "")
    link, dist = _nearest_link(obj.matrix_world.translation, link_meshes)
    if link is None:
        link = declared
    elif link != declared:
        print(
            f"[INFO] 贴花 {obj.name}: 按位置改挂到 {link}"
            f"（创建时是 {declared}，距表面 {dist * 1000:.1f} mm）"
        )
    world = link_world.get(link)
    if world is None:
        print(f"[WARN] 贴花 {obj.name} 找不到所属 link {link!r}，跳过")
        return None
    mesh = obj.data
    if len(mesh.vertices) != 4:
        print(f"[WARN] 贴花 {obj.name} 不是四边形（{len(mesh.vertices)} 顶点），跳过")
        return None
    matrix_local = world.inverted() @ obj.matrix_world
    corners = [
        [round(v, 6) for v in (matrix_local @ vert.co)]
        for vert in mesh.vertices
    ]
    texture = obj.get("texture") or _image_path(obj.active_material)
    if not texture:
        print(f"[WARN] 贴花 {obj.name} 找不到贴图，跳过")
        return None
    return {
        "decal": True,
        "link": link,
        "texture": texture,
        "corners": corners,
    }


def _image_path(mat) -> str:
    if mat is None or not mat.use_nodes:
        return ""
    for node in mat.node_tree.nodes:
        if node.type == "TEX_IMAGE" and node.image is not None:
            return bpy.path.abspath(node.image.filepath)
    return ""


def _load_part_faces() -> dict[str, tuple[str, list[int]]]:
    """读取 ``split_blend_link_parts.py`` 写进 .blend 的面号映射：零件名 -> (link, faces)。"""
    text = bpy.data.texts.get("part_faces")
    if text is None:
        return {}
    raw = text.as_string().strip()
    if not raw:
        return {}
    mapping: dict[str, tuple[str, list[int]]] = {}
    for link, parts in json.loads(raw).items():
        for part_name, faces in parts.items():
            mapping[part_name] = (link, faces)
    return mapping


def main() -> int:
    args = _parse_args()
    bpy.ops.wm.open_mainfile(filepath=args.blend)

    part_faces = _load_part_faces()

    # link -> 世界变换 / 网格物体（拆分后的零件用 custom prop source_link 归到原 link）
    link_world: dict[str, object] = {}
    link_meshes: dict[str, list] = {}
    for obj in bpy.data.objects:
        if obj.type != "MESH" or obj.name == "Ground" or _is_decal(obj):
            continue
        link = obj.get("source_link") or obj.name
        link_world.setdefault(link, obj.matrix_world)
        link_meshes.setdefault(link, []).append(obj)

    rows = {}
    part_count = 0
    decal_count = 0
    for obj in bpy.data.objects:
        if obj.type != "MESH" or obj.name == "Ground":
            continue
        if _is_decal(obj):
            row = _decal_row(obj, link_world, link_meshes)
            if row is not None:
                rows[obj.name] = row
                decal_count += 1
            continue
        mat = obj.active_material
        if mat is None:
            print(f"[WARN] {obj.name} 没有材质，跳过")
            continue
        color, metallic, roughness = _material_paint(mat)
        row = {
            "material": mat.name,
            "color": [round(c, 6) for c in color],
            "metallic": round(metallic, 4),
            "roughness": round(roughness, 4),
        }
        part = part_faces.get(obj.name)
        if part is not None:
            link, faces = part
            row["link"] = link
            row["faces"] = faces
            part_count += 1
        rows[obj.name] = row

    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(rows, fh, indent=2, ensure_ascii=False)
    print(
        f"[INFO] 导出 {len(rows)} 个物体（{part_count} 个零件，{decal_count} 个贴花）-> {args.out}"
    )
    if part_count:
        links = sorted({row["link"] for row in rows.values() if "faces" in row})
        print(f"[INFO] 含零件的 link: {', '.join(links)}")
    if decal_count:
        names = sorted(name for name, row in rows.items() if row.get("decal"))
        print(f"[INFO] 贴花: {', '.join(names)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
