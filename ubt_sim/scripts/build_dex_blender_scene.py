#!/usr/bin/env python3
"""用 URDF + STL 生成一个 Blender 场景，供人工给 tienkung_dex 调色。

跑在宿主机（不是 Isaac Sim 容器）：

    blender -b --factory-startup --python scripts/build_dex_blender_scene.py -- \
        --variant inspire --out ~/dex_recolor/tienkung_dex_inspire.blend

每个 link 一个物体（物体名 = URDF link 名），材质沿用
``recolor_tienkung_dex_usd.py`` 的 pro 方案（ProBlack / ProGrey / ProDarkGrey /
ProMetal / ProWhite）。在 Blender 里改材质的 Base Color，或把物体换到别的材质槽，
保存后可用 ``read_dex_blender_colors.py`` 读回每个 link 的颜色并写回 USD。

注意：Blender 4.0.2 的发行包没有 USD 导入，所以这里直接用 URDF 的 STL，
几何与 Isaac Sim 里的 USD 同源。
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "assets" / "robots" / "tienkung_dex"
URDF_DIR = ASSET_ROOT / "urdf"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from recolor_tienkung_dex_usd import (  # noqa: E402
    PRO_MATERIAL_NAMES,
    PRO_PALETTE,
    pro_palette_for_link,
)
from split_blend_link_parts import _save_part_faces, split_link  # noqa: E402

VARIANTS = {
    "base": "tienkung_dex_base.urdf",
    "inspire": "tienkung_dex_inspire.urdf",
    "brainco": "tienkung_dex_brainco.urdf",
}

# 与场景 yaml 的 init_state.pos[2] 一致：URDF 以 pelvis 为原点，抬高后脚掌落在地面
GROUND_Z = 0.998


def _rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _origin_matrix(element) -> np.ndarray:
    from xml.etree import ElementTree as ET

    T = np.eye(4)
    if element is None:
        return T
    xyz = np.array([float(v) for v in (element.get("xyz") or "0 0 0").split()])
    rpy = [float(v) for v in (element.get("rpy") or "0 0 0").split()]
    T[:3, :3] = _rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _load_stl(path: Path) -> np.ndarray:
    """读取二进制 STL，返回 (n, 3, 3) 的三角形顶点。"""
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL 文件过小: {path}")
    count = struct.unpack("<I", data[80:84])[0]
    if 84 + count * 50 != len(data):
        raise ValueError(f"只支持二进制 STL: {path}")
    record = np.frombuffer(data[84:], dtype=np.uint8).reshape(count, 50)
    return record[:, 12:48].copy().view("<f4").reshape(count, 3, 3).astype(np.float32)


def _mesh_from_triangles(name: str, tris: np.ndarray):
    verts = tris.reshape(-1, 3)
    n_verts = len(verts)
    mesh = bpy.data.meshes.new(name)
    mesh.vertices.add(n_verts)
    mesh.vertices.foreach_set("co", verts.ravel())
    mesh.loops.add(n_verts)
    mesh.loops.foreach_set("vertex_index", np.arange(n_verts, dtype=np.int32))
    mesh.polygons.add(n_verts // 3)
    mesh.polygons.foreach_set("loop_start", np.arange(0, n_verts, 3, dtype=np.int32))
    mesh.polygons.foreach_set("loop_total", np.full(n_verts // 3, 3, dtype=np.int32))
    mesh.update(calc_edges=False)
    return mesh


def _make_material(key: str, paint):
    mat = bpy.data.materials.new(key)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (*paint.color, 1.0)
        bsdf.inputs["Metallic"].default_value = paint.metallic or 0.0
        bsdf.inputs["Roughness"].default_value = (
            paint.roughness if paint.roughness is not None else 0.5
        )
    # Solid 视图 / 大纲里也能看到颜色
    mat.diffuse_color = (*paint.color, 1.0)
    mat.metallic = paint.metallic or 0.0
    mat.roughness = paint.roughness if paint.roughness is not None else 0.5
    return mat


def _collection(name: str, parent):
    coll = bpy.data.collections.new(name)
    parent.children.link(coll)
    return coll


_HAND_TOKENS_ALL = (
    "hand", "thumb", "index", "middle", "ring", "pinky", "palm", "casing",
    "finger", "flange", "touch", "proximal", "distal", "metacarpal", "base_link",
)
_LEG_TOKENS = ("hip_", "knee_", "ankle_")


def _is_hand_link(link: str) -> bool:
    return any(tok in link for tok in _HAND_TOKENS_ALL)


def _region_for_link(link: str) -> str:
    """按区域分组材质：头、躯干、左右臂、左右腿、左右手各自独立。"""
    if "head" in link:
        return "Head"
    if _is_hand_link(link):
        if link.startswith(("left", "L_")):
            return "LeftHand"
        if link.startswith(("right", "R_")):
            return "RightHand"
    if link.endswith("_l_link") or link.startswith("left"):
        return "LeftLeg" if any(tok in link for tok in _LEG_TOKENS) else "LeftArm"
    if link.endswith("_r_link") or link.startswith("right"):
        return "RightLeg" if any(tok in link for tok in _LEG_TOKENS) else "RightArm"
    return "Torso"


def _link_collection(link: str, body, left, right):
    if _is_hand_link(link):
        if link.startswith(("left", "L_")):
            return left
        if link.startswith(("right", "R_")):
            return right
    return body


def build(
    variant: str,
    out_path: Path,
    split_links: tuple[str, ...] = (),
    split_eps: float = 1e-6,
    split_min_tris: int = 1000,
) -> None:
    from xml.etree import ElementTree as ET

    urdf_path = URDF_DIR / VARIANTS[variant]
    root = ET.parse(urdf_path).getroot()
    links = {link.get("name"): link for link in root.findall("link")}

    parent_of: dict[str, str] = {}
    joint_of: dict[str, object] = {}
    for joint in root.findall("joint"):
        child = joint.find("child").get("link")
        parent_of[child] = joint.find("parent").get("link")
        joint_of[child] = joint

    world_cache: dict[str, np.ndarray] = {}

    def world_matrix(name: str) -> np.ndarray:
        if name not in world_cache:
            if name not in parent_of:
                world_cache[name] = np.eye(4)
            else:
                world_cache[name] = world_matrix(parent_of[name]) @ _origin_matrix(
                    joint_of[name].find("origin")
                )
        return world_cache[name]

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"

    root_coll = scene.collection
    body = _collection("body", root_coll)
    left = _collection("left_hand", root_coll)
    right = _collection("right_hand", root_coll)

    # 每个区域一套材质（Head_ProBlack / Torso_ProBlack / LeftLeg_ProGrey ...），
    # 初始颜色与 pro 方案一致，但头、躯干、四肢、双手互不影响。
    materials: dict[str, object] = {}
    used_keys: set[str] = set()

    def material_for_link(link: str):
        key = pro_palette_for_link(link)
        used_keys.add(key)
        name = f"{_region_for_link(link)}_{PRO_MATERIAL_NAMES[key]}"
        if name not in materials:
            materials[name] = _make_material(name, PRO_PALETTE[key])
        return materials[name]

    ground_mesh = bpy.data.meshes.new("Ground")
    ground_mesh.from_pydata(
        [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)], [], [(0, 1, 2, 3)]
    )
    ground_mesh.update()
    ground = bpy.data.objects.new("Ground", ground_mesh)
    ground_mat = bpy.data.materials.new("GroundMat")
    ground_mat.use_nodes = True
    ground_bsdf = ground_mat.node_tree.nodes.get("Principled BSDF")
    if ground_bsdf is not None:
        ground_bsdf.inputs["Base Color"].default_value = (0.32, 0.33, 0.34, 1.0)
        ground_bsdf.inputs["Roughness"].default_value = 0.85
    ground_mat.diffuse_color = (0.32, 0.33, 0.34, 1.0)
    ground.data.materials.append(ground_mat)
    root_coll.objects.link(ground)

    T_ground = np.eye(4)
    T_ground[2, 3] = GROUND_Z

    created = 0
    skipped: list[str] = []
    for link_name, link in links.items():
        visuals = link.findall("visual")
        if not visuals:
            skipped.append(link_name)
            continue
        T_link = world_matrix(link_name)
        coll = _link_collection(link_name, body, left, right)
        for idx, visual in enumerate(visuals):
            mesh_el = visual.find("geometry/mesh")
            if mesh_el is None:
                continue
            mesh_path = (URDF_DIR / mesh_el.get("filename")).resolve()
            if not mesh_path.is_file():
                print(f"[WARN] 缺少网格: {mesh_path}")
                continue
            tris = _load_stl(mesh_path)
            suffix = f"_{idx}" if idx else ""
            mesh = _mesh_from_triangles(link_name + suffix, tris)
            obj = bpy.data.objects.new(link_name + suffix, mesh)
            coll.objects.link(obj)
            T_visual = T_ground @ T_link @ _origin_matrix(visual.find("origin"))
            obj.matrix_world = Matrix(T_visual.tolist())
            obj.data.materials.append(material_for_link(link_name))
            created += 1

    # 没被任何 link 用到的调色板材质也建出来（pro_blue / pro_stop_red 备用）；
    # 没有用户会被 Blender 在保存时丢掉，需要 fake user 保活
    for key, paint in PRO_PALETTE.items():
        if key in used_keys:
            continue
        name = PRO_MATERIAL_NAMES[key]
        mat = _make_material(name, paint)
        mat.use_fake_user = True
        materials[name] = mat

    # 灯光 / 相机
    sun_data = bpy.data.lights.new("Sun", type="SUN")
    sun_data.energy = 3.0
    sun = bpy.data.objects.new("Sun", sun_data)
    sun.rotation_euler = (0.5, 0.15, 0.8)
    root_coll.objects.link(sun)

    cam_data = bpy.data.cameras.new("Camera")
    cam = bpy.data.objects.new("Camera", cam_data)
    cam.location = (3.2, -0.6, 1.15)
    target = Vector((0.0, 0.0, 0.9))
    cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
    root_coll.objects.link(cam)
    scene.camera = cam

    # 视口默认用 Material Preview，打开就能看到颜色
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type != "VIEW_3D":
                continue
            for space in area.spaces:
                if space.type == "VIEW_3D":
                    space.shading.type = "MATERIAL"

    note = bpy.data.texts.new("README")
    note.write(
        "tienkung_dex 调色场景\n"
        f"variant: {variant}\n\n"
        "材质按区域拆分：Head_* / Torso_* / LeftArm_* / RightArm_* / "
        "LeftLeg_* / RightLeg_* / LeftHand_* / RightHand_*，\n"
        "后缀为 pro 方案色名（ProBlack / ProGrey / ProDarkGrey / ProMetal / ProWhite）。\n"
        "同一区域内同色零件仍共用一个材质，不同区域互不影响。\n"
        "1. 选中物体（物体名 = URDF link 名），在 Material Properties 里改 Base Color；\n"
        "2. 某一区域内还要再细分（比如单独改某个 link），复制材质再指派即可；\n"
        "3. 改完保存 .blend，再运行 read_dex_blender_colors.py 把颜色写回 USD。\n"
    )

    # 可选：建场景时直接按 loose parts 拆分，保证一次保存就是最终状态
    if split_links:
        part_faces: dict[str, dict[str, list[int]]] = {}
        for link in split_links:
            obj = bpy.data.objects.get(link)
            if obj is None or obj.type != "MESH":
                print(f"[WARN] 找不到要拆分的网格: {link}")
                continue
            part_faces[link] = split_link(obj, split_eps, split_min_tris, print)
        _save_part_faces(part_faces)
        note.write(
            "\n零件拆分说明：\n"
            f"  {', '.join(split_links)} 已拆成 <link>_pNN 多个物体，每个物体独立材质。\n"
            "  小零件在 <link>_parts_small 集合里，可整组隐藏。\n"
            "  面号映射保存在文本块 part_faces 中，写回 USD 时按 GeomSubset 绑定颜色。\n"
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out_path))
    print(f"[INFO] variant={variant} 物体={created} 跳过无网格 link={len(skipped)}")
    if skipped:
        print("[INFO] 跳过:", ", ".join(sorted(skipped)[:12]))
    print(f"[INFO] 已保存 {out_path}")


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", default="inspire", choices=list(VARIANTS))
    parser.add_argument(
        "--out",
        default=str(Path.home() / "dex_recolor" / "tienkung_dex_inspire.blend"),
    )
    parser.add_argument(
        "--split-links",
        action="append",
        default=[],
        help="建场景时顺带按 loose parts 拆分的 link，可重复（如 waist_pitch_link）。",
    )
    parser.add_argument("--split-eps", type=float, default=1e-6, help="拆分焊接容差（米）。")
    parser.add_argument(
        "--split-min-tris",
        type=int,
        default=1000,
        help="拆分后小于该面数的零件放进 <link>_parts_small 集合。",
    )
    args = parser.parse_args(argv)
    build(
        args.variant,
        Path(args.out).expanduser(),
        tuple(args.split_links),
        args.split_eps,
        args.split_min_tris,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
