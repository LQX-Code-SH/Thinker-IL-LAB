#!/usr/bin/env python3
"""把 .blend 里某个 link 的网格按不连通壳体（loose parts）拆成多个物体。

跑在宿主机（不是 Isaac Sim 容器）：

    blender -b --factory-startup --python scripts/split_blend_link_parts.py -- \
        --blend ~/work/dex_recolor/tienkung_dex_inspire.blend \
        --link waist_pitch_link

拆分后每个零件一个物体、一个独立材质副本，命名 ``<link>_pNN``（按三角形数从多到少），
零件之间的面号映射写进 .blend 的 ``part_faces`` 文本块，供
``read_dex_blender_colors.py`` / ``apply_dex_colors_json.py`` 按面写回 USD（GeomSubset）。

因为点/面顺序与 URDF 的 STL 一致（USD 的 mesh 也是同一顺序），面号可以直接当 USD 的
face index 用。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import bpy
import numpy as np


def _weld_components(vertex_co: np.ndarray, triangles: np.ndarray, eps: float):
    """按焊接后的顶点求三角形连通分量，返回每个三角形所属分量 id。"""
    key = np.round(vertex_co / eps).astype(np.int64)
    _, inverse = np.unique(key, axis=0, return_inverse=True)
    weld = inverse.reshape(-1)[triangles]  # (n_tris, 3)

    parent = list(range(int(weld.max()) + 1))

    def find(x: int) -> int:
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    for a, b, c in weld.tolist():
        ra, rb, rc = find(a), find(b), find(c)
        if ra != rb:
            parent[rb] = ra
        ra = find(a)
        if ra != rc:
            parent[rc] = ra

    roots = np.fromiter((find(i) for i in range(len(parent))), dtype=np.int64, count=len(parent))
    return roots[weld[:, 0]]


def _make_mesh(name: str, vertex_co: np.ndarray, triangles: np.ndarray):
    """用 (m, 3) 顶点索引数组快速建三角网格（每面独立顶点，不做焊接）。"""
    tri_verts = vertex_co[triangles.reshape(-1)]
    n_verts = len(tri_verts)
    mesh = bpy.data.meshes.new(name)
    mesh.vertices.add(n_verts)
    mesh.vertices.foreach_set("co", np.ascontiguousarray(tri_verts, dtype=np.float32).ravel())
    mesh.loops.add(n_verts)
    mesh.loops.foreach_set("vertex_index", np.arange(n_verts, dtype=np.int32))
    mesh.polygons.add(len(triangles))
    mesh.polygons.foreach_set("loop_start", np.arange(0, n_verts, 3, dtype=np.int32))
    mesh.polygons.foreach_set("loop_total", np.full(len(triangles), 3, dtype=np.int32))
    mesh.update(calc_edges=False)
    return mesh


def _collection(name: str, parent):
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
        parent.children.link(coll)
    return coll


def split_link(obj, eps: float, min_tris: int, log) -> dict[str, list[int]]:
    mesh = obj.data
    n_polys = len(mesh.polygons)
    if n_polys == 0:
        return {}

    vertex_co = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
    mesh.vertices.foreach_get("co", vertex_co)
    vertex_co = vertex_co.reshape(-1, 3)

    loop_vertex = np.empty(len(mesh.loops), dtype=np.int32)
    mesh.loops.foreach_get("vertex_index", loop_vertex)
    if len(mesh.loops) != n_polys * 3:
        raise RuntimeError(f"{obj.name} 不是纯三角网格（loops={len(mesh.loops)} faces={n_polys}）")
    triangles = loop_vertex.reshape(-1, 3)

    face_comp = _weld_components(vertex_co, triangles, eps)
    comps, counts = np.unique(face_comp, return_counts=True)
    order = np.argsort(-counts)  # 大零件在前
    log(f"[INFO] {obj.name}: {n_polys} 面 -> {len(comps)} 个零件")

    parent_coll = obj.users_collection[0] if obj.users_collection else bpy.context.scene.collection
    big_coll = _collection(f"{obj.name}_parts", parent_coll)
    small_coll = _collection(f"{obj.name}_parts_small", big_coll)

    base_mat = mesh.materials[0] if mesh.materials else None
    matrix = obj.matrix_world.copy()
    faces_by_part: dict[str, list[int]] = {}

    for part_index, comp_pos in enumerate(order):
        comp = comps[comp_pos]
        faces = np.nonzero(face_comp == comp)[0]
        part_name = f"{obj.name}_p{part_index:02d}"
        part_mesh = _make_mesh(part_name, vertex_co, triangles[faces])
        part_obj = bpy.data.objects.new(part_name, part_mesh)
        part_obj.matrix_world = matrix
        (big_coll if len(faces) >= min_tris else small_coll).objects.link(part_obj)

        if base_mat is not None:
            mat = base_mat.copy()
            mat.name = f"{base_mat.name}__p{part_index:02d}"
            part_mesh.materials.append(mat)

        part_obj["source_link"] = obj.name
        part_obj["part_index"] = part_index
        part_obj["tri_count"] = int(len(faces))
        part_obj["weld_eps"] = eps
        faces_by_part[part_name] = faces.tolist()

        bbox_min = vertex_co[triangles[faces].reshape(-1)].min(axis=0)
        bbox_max = vertex_co[triangles[faces].reshape(-1)].max(axis=0)
        log(
            f"  {part_name:24s} tris={len(faces):6d} "
            f"bbox={np.round(bbox_min, 3).tolist()}..{np.round(bbox_max, 3).tolist()}"
        )

    mesh_name = mesh.name
    bpy.data.objects.remove(obj, do_unlink=True)
    if mesh.users == 0:
        bpy.data.meshes.remove(mesh)
    log(f"[INFO] 已删除原物体 {mesh_name}，拆出 {len(faces_by_part)} 个零件")
    return faces_by_part


def _load_part_faces() -> dict:
    text = bpy.data.texts.get("part_faces")
    if text is None:
        return {}
    raw = text.as_string().strip()
    return json.loads(raw) if raw else {}


def _save_part_faces(data: dict) -> None:
    text = bpy.data.texts.get("part_faces") or bpy.data.texts.new("part_faces")
    text.clear()
    text.write(json.dumps(data))
    text.use_fake_user = True


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--blend", required=True)
    parser.add_argument("--link", action="append", required=True, help="要拆的 link 名，可重复")
    parser.add_argument("--out", default="", help="输出 .blend，默认原地保存")
    parser.add_argument("--eps", type=float, default=1e-6, help="焊接容差（米）")
    parser.add_argument(
        "--min-tris", type=int, default=1000,
        help="小于该面数的零件放进 <link>_parts_small 集合（仍独立成物体）",
    )
    args = parser.parse_args(argv)

    blend_path = Path(args.blend).expanduser()
    out_path = Path(args.out).expanduser() if args.out else blend_path
    bpy.ops.wm.open_mainfile(filepath=str(blend_path))

    logs: list[str] = []

    def log(msg: str) -> None:
        print(msg, flush=True)
        logs.append(msg)

    part_faces = _load_part_faces()
    for link in args.link:
        obj = bpy.data.objects.get(link)
        if obj is None or obj.type != "MESH":
            raise SystemExit(f"找不到网格物体: {link}")
        part_faces[link] = split_link(obj, args.eps, args.min_tris, log)

    _save_part_faces(part_faces)

    note = bpy.data.texts.get("README")
    if note is not None:
        note.write(
            "\n零件拆分说明：\n"
            f"  {', '.join(args.link)} 已拆成 <link>_pNN 多个物体，每个物体独立材质。\n"
            "  小零件在 <link>_parts_small 集合里，可整组隐藏。\n"
            "  面号映射保存在文本块 part_faces 中，写回 USD 时按 GeomSubset 绑定颜色。\n"
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out_path))
    log(f"[INFO] 已保存 {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
