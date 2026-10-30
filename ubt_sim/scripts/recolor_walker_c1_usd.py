#!/usr/bin/env python3
"""给 walker_c1 合并版 USD 上色（复用 tienkung_dex 的方案）。

与 ``recolor_tienkung_dex_usd.py`` 同构，差异：
1. 颜色不是写死的调色板，而是从 ``~/c1_mat_edit/assignments.csv``
   （Blender ``Untitled.blend`` 导出的 83 条映射）读取；
2. 头部 ``head_pitch_link`` 是独立 mesh + GeomSubset（沿用 dex 的细粒度做法）；
3. 胸口 logo 走 ``/visuals/<link>/decals/``，与视觉 Xform 平级。

核心机制（都来自 dex 脚本）：
- 转换器把 ``material:binding`` 写在 ``/visuals/<link>/<visual>`` 的 Xform 上；
- 用 ``strongerThanDescendants`` 绑定，所以**有 subset 时必须先删掉 Xform 上的绑定**；
- 材质靠克隆 importer 生成的 OmniPBR 模板，只改颜色/金属度/粗糙度。

容器内运行：

    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/recolor_walker_c1_usd.py --force
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "assets" / "robots" / "walker_c1"
USD_DIR = ASSET_ROOT / "usd"
TOP_USD = USD_DIR / "walker_c1_merged.usd"
TEXTURES = ASSET_ROOT / "textures"

COLORS_CSV = ASSET_ROOT / "colors.csv"
HEAD_CSV = ASSET_ROOT / "head_parts.csv"

INTERNAL_GREY = ("#969696", 0.5, 0.0)
DECAL_LINK = "waist_roll_link"
DECAL_TEXTURE = "logubt_warm_white.png"
DECAL_W, DECAL_H = 0.115, 0.046
DECAL_Z_OFFSET = 0.03  # 胸口 logo 整体上移 3cm

# merge_fixed_joints 把 hand_base / palm 的几何并进了 wrist_roll：
#   官方 3588 / 5620 / 30050  ->  merged 39258
# 这里按世界位置把 39258 面重新切回三组，各绑不同颜色（白 / 银 / 蓝灰交替）。
WRIST_SPLIT = {
    "L_wrist_roll_link": ("L_wrist_roll_link", "L_hand_base_link", "L_palm_link"),
    "R_wrist_roll_link": ("R_wrist_roll_link", "R_hand_base_link", "R_palm_Link"),
}
OFFICIAL_USD = USD_DIR / "walker_c1_with_hands.usd"


# SimulationApp 需在进程内保活，否则 pxr 绑定会被回收
_SIM_APP = None


def ensure_pxr() -> None:
    """/isaac-sim/python.sh 只提供解释器，pxr 由 Kit 启动时注入 sys.path。"""
    try:
        import pxr  # noqa: F401

        return
    except ModuleNotFoundError:
        pass

    print("[INFO] pxr 未就绪，无头启动 Isaac Sim 加载 USD 运行时...")
    from isaacsim import SimulationApp

    original_argv = sys.argv
    sys.argv = [original_argv[0]]
    try:
        global _SIM_APP
        _SIM_APP = SimulationApp({"headless": True, "enable_cameras": False})
    finally:
        sys.argv = original_argv


# ---------------------------------------------------------------- 基础工具


def hex_to_linear(h: str) -> tuple[float, float, float]:
    h = h.lstrip("#")
    srgb = [int(h[i : i + 2], 16) / 255.0 for i in (0, 2, 4)]

    def f(c):
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    return tuple(f(c) for c in srgb)


def _base_usd_path(top_usd: Path) -> Path:
    """顶层 instanceable USD -> 同目录 configuration 下的 base 层。"""
    if "configuration" in top_usd.parts:
        return top_usd
    return top_usd.parent / "configuration" / f"{top_usd.stem}_base.usd"


def _robot_root(stage):
    default_prim = stage.GetDefaultPrim()
    return default_prim.GetPath() if default_prim else None


def _iter_link_visuals(stage):
    """遍历 ``/visuals/<link>/<visual>``；内层 Xform 才是 importer 写绑定的位置。"""
    prefix = "/visuals/"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(prefix):
            continue
        parts = path[len(prefix) :].split("/")
        if len(parts) == 2 and parts[1] != "decals":
            yield parts[0], parts[1], prim


def _find_template_material(stage, root):
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Material":
            continue
        if not str(prim.GetPath()).startswith(f"{root}/Looks/"):
            continue
        for child in prim.GetChildren():
            if child.GetTypeName() != "Shader":
                continue
            source = child.GetAttribute("info:mdl:sourceAsset")
            if source and "OmniPBR" in str(source.Get()):
                return prim
    raise RuntimeError(f"{root}/Looks 下没有找到 OmniPBR 材质模板")


def _set_input(shader, name, value, value_type):
    attr = shader.GetAttribute(name)
    if not attr:
        attr = shader.CreateAttribute(name, value_type)
    attr.Set(value)


# ---------------------------------------------------------------- 颜色方案


def load_colors() -> tuple[dict[str, tuple], dict[str, tuple]]:
    """返回 (link -> spec, HP 零件名 -> spec)。spec = (hex, roughness, metallic)。"""
    body, head = {}, {}
    if not COLORS_CSV.is_file():
        raise FileNotFoundError(f"缺少配色表: {COLORS_CSV}")
    with open(COLORS_CSV, encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            mat = row["material"]
            spec = (
                INTERNAL_GREY
                if mat.startswith("C1_")
                else (row["hex_srgb"], float(row["roughness"]), float(row["metallic"]))
            )
            parts = row["prim_path_in_blender"].split("/")
            if len(parts) < 2:
                continue
            if "HP_" in row["prim_path_in_blender"]:
                head[parts[-1]] = spec
            else:
                body[parts[1]] = spec
    return body, head


def load_head_parts() -> dict[str, tuple[int, tuple[float, float, float]]]:
    import re

    num = re.compile(r"[-+]?\d+\.\d+")
    out = {}
    if not HEAD_CSV.is_file():
        return out
    with open(HEAD_CSV, encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if not row["object"].startswith("HP_"):
                continue
            v = [float(x) for x in num.findall(row["bbox_world"])]
            if len(v) < 6:
                continue
            out[row["object"]] = (
                int(row["faces"]),
                ((v[0] + v[1]) / 2, (v[2] + v[3]) / 2, (v[4] + v[5]) / 2),
            )
    return out


# ---------------------------------------------------------------- 材质


def _material_name(spec) -> str:
    color, roughness, metallic = spec
    hexcode = color.lstrip("#").upper()
    return f"Custom_{hexcode}_m{int(round(metallic * 100))}_r{int(round(roughness * 100))}"


def _ensure_materials(stage, root, specs):
    """按 spec 建 OmniPBR 材质，返回 {spec: UsdShade.Material}。"""
    from pxr import Gf, Sdf, UsdShade

    template = _find_template_material(stage, root)
    template_spec = template.GetPrimStack()[0]
    root_layer = stage.GetRootLayer()
    looks_path = root.AppendChild("Looks")
    stage.DefinePrim(looks_path)

    materials = {}
    for spec in specs:
        name = _material_name(spec)
        mat_path = looks_path.AppendChild(name)
        if not stage.GetPrimAtPath(mat_path):
            Sdf.CopySpec(template_spec.layer, template_spec.path, root_layer, mat_path)
        mat_prim = stage.GetPrimAtPath(mat_path)
        shader = next(c for c in mat_prim.GetChildren() if c.GetTypeName() == "Shader")
        color, roughness, metallic = spec
        _set_input(
            shader,
            "inputs:diffuse_color_constant",
            Gf.Vec3f(*hex_to_linear(color)),
            Sdf.ValueTypeNames.Color3f,
        )
        _set_input(shader, "inputs:metallic_constant", float(metallic), Sdf.ValueTypeNames.Float)
        _set_input(
            shader, "inputs:reflection_roughness_constant", float(roughness), Sdf.ValueTypeNames.Float
        )
        materials[spec] = UsdShade.Material(mat_prim)
    return materials


def _ensure_decal_material(stage, root):
    from pxr import Sdf, UsdShade

    template = _find_template_material(stage, root)
    template_spec = template.GetPrimStack()[0]
    root_layer = stage.GetRootLayer()
    looks_path = root.AppendChild("Looks")
    stage.DefinePrim(looks_path)

    mat_path = looks_path.AppendChild("Decal_logubt")
    if not stage.GetPrimAtPath(mat_path):
        Sdf.CopySpec(template_spec.layer, template_spec.path, root_layer, mat_path)
    mat_prim = stage.GetPrimAtPath(mat_path)
    shader = next(c for c in mat_prim.GetChildren() if c.GetTypeName() == "Shader")
    # 相对 base 层（configuration/）的贴图路径
    asset = Sdf.AssetPath(f"../../textures/{DECAL_TEXTURE}")
    _set_input(shader, "inputs:diffuse_texture", asset, Sdf.ValueTypeNames.Asset)
    _set_input(shader, "inputs:enable_opacity", True, Sdf.ValueTypeNames.Bool)
    _set_input(shader, "inputs:opacity_texture", asset, Sdf.ValueTypeNames.Asset)
    _set_input(shader, "inputs:opacity_threshold", 0.5, Sdf.ValueTypeNames.Float)
    return UsdShade.Material(mat_prim)


# ---------------------------------------------------------------- 头部


def head_components(mesh_prim, world_matrix):
    from pxr import Gf, UsdGeom

    mesh = UsdGeom.Mesh(mesh_prim)
    pts = mesh.GetPointsAttr().Get()
    pts = [world_matrix.Transform(Gf.Vec3d(q[0], q[1], q[2])) for q in pts]
    counts = list(mesh.GetFaceVertexCountsAttr().Get())
    idx = list(mesh.GetFaceVertexIndicesAttr().Get())

    canon, vmap = {}, []
    for p in pts:
        key = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
        vmap.append(canon.setdefault(key, len(canon)))

    parent = list(range(len(canon)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    off = 0
    for c in counts:
        v0 = vmap[idx[off]]
        for k in range(1, c):
            union(v0, vmap[idx[off + k]])
        off += c

    groups = defaultdict(list)
    off = 0
    for fi, c in enumerate(counts):
        groups[find(vmap[idx[off]])].append(fi)
        off += c

    keys = list(canon.keys())
    out = []
    for faces in groups.values():
        vs, fs, off = set(), set(faces), 0
        for fi, c in enumerate(counts):
            if fi in fs:
                vs.update(vmap[i] for i in idx[off : off + c])
            off += c
        px = [keys[v][0] for v in vs]
        py = [keys[v][1] for v in vs]
        pz = [keys[v][2] for v in vs]
        out.append((faces, ((min(px) + max(px)) / 2, (min(py) + max(py)) / 2, (min(pz) + max(pz)) / 2)))
    return out


# ---------------------------------------------------------------- 主流程


def apply_palette(top_usd: Path, dry_run: bool = False) -> dict:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    base_path = _base_usd_path(Path(top_usd))
    if not base_path.is_file():
        raise FileNotFoundError(f"找不到 base 层: {base_path}")

    stage = Usd.Stage.Open(str(base_path))
    root = _robot_root(stage)
    root_layer = stage.GetRootLayer()
    main_stage = Usd.Stage.Open(str(top_usd))
    xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    body, head = load_colors()
    parts = load_head_parts()

    specs = set(body.values()) | set(head.values()) | {INTERNAL_GREY}
    materials = _ensure_materials(stage, root, specs)
    print(f"[INFO] 材质 {len(materials)} 个（OmniPBR 模板克隆）")

    # ---- 机身：绑在视觉 Xform 上（strongerThanDescendants） ----
    bound = 0
    for link, visual, prim in _iter_link_visuals(stage):
        # 合并后的 link（如 wrist_roll）下面挂着 hand_base / palm 的独立 visual，
        # 优先按 visual 名取色，取不到再回退到 link 名
        spec = body.get(visual) or body.get(link)
        if spec is None:
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            materials[spec], UsdShade.Tokens.strongerThanDescendants
        )
        bound += 1
    print(f"[INFO] 机身绑定 {bound} 个 link")

    # ---- 头部：删掉 Xform 强绑，改用 mesh + GeomSubset ----
    head_link = "head_pitch_link"
    mesh_path = f"/visuals/{head_link}/{head_link}/mesh"
    hp_mesh = stage.GetPrimAtPath(mesh_path)
    subset_count = 0
    if hp_mesh:
        xform = stage.GetPrimAtPath(f"/visuals/{head_link}/{head_link}")
        if xform:
            for prop in ("material:binding", "material:binding:strength"):
                if xform.HasProperty(prop):
                    xform.RemoveProperty(prop)

        proxy = None
        for q in Usd.PrimRange.Stage(main_stage, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if q.GetPath().pathString.endswith(f"{head_link}/visuals/{head_link}/mesh"):
                proxy = q
                break
        world = xf_cache.GetLocalToWorldTransform(proxy) if proxy else Gf.Matrix4d(1.0)

        comps = head_components(hp_mesh, world)
        mesh = UsdGeom.Mesh(hp_mesh)

        # 兜底材质 + 清旧 subset
        UsdShade.MaterialBindingAPI.Apply(hp_mesh).Bind(materials[INTERNAL_GREY])
        for child in hp_mesh.GetChildren():
            if child.GetTypeName() == "GeomSubset" and root_layer.GetPrimAtPath(child.GetPath()):
                stage.RemovePrim(child.GetPath())

        used = set()
        for name in sorted(parts):
            _, cb = parts[name]
            best, bd = None, 1e9
            for i, (_, cu) in enumerate(comps):
                if i in used:
                    continue
                d = math.dist(cb, cu)
                if d < bd:
                    best, bd = i, d
            if best is None or bd > 0.05:
                continue
            used.add(best)
            sub = UsdGeom.Subset.CreateGeomSubset(
                mesh, f"part_{subset_count:02d}", UsdGeom.Tokens.face,
                Vt.IntArray(comps[best][0]), "materialBind",
            )
            UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(materials[head[name]])
            subset_count += 1
        for i, (faces, _) in enumerate(comps):
            if i in used:
                continue
            sub = UsdGeom.Subset.CreateGeomSubset(
                mesh, f"part_{subset_count:02d}", UsdGeom.Tokens.face,
                Vt.IntArray(faces), "materialBind",
            )
            UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(materials[INTERNAL_GREY])
            subset_count += 1
    print(f"[INFO] 头部 GeomSubset {subset_count} 个（Xform 强绑已移除）")


    # ---- 胸口 decal：挂在 /visuals/<link>/decals/，与视觉 Xform 平级 ----
    decal_mat = _ensure_decal_material(stage, root)
    chest = UsdGeom.Mesh(stage.GetPrimAtPath(f"/visuals/{DECAL_LINK}/{DECAL_LINK}/mesh"))
    if chest:
        pts = chest.GetPointsAttr().Get()
        counts = list(chest.GetFaceVertexCountsAttr().Get())
        idx = list(chest.GetFaceVertexIndicesAttr().Get())
        zs = [q[2] for q in pts]
        # 0.68 是基准比例，+0.03 m 是用户要求的整体上移 3cm
        z0 = min(zs) + 0.68 * (max(zs) - min(zs)) - DECAL_H / 2 + DECAL_Z_OFFSET
        z1 = z0 + DECAL_H
        x_front = max(q[0] for q in pts)

        picked, off = [], 0
        for c in counts:
            vs = idx[off : off + c]
            off += c
            cx = sum(pts[v][0] for v in vs) / c
            cy = sum(pts[v][1] for v in vs) / c
            cz = sum(pts[v][2] for v in vs) / c
            if cx > x_front - 0.05 and abs(cy) <= DECAL_W / 2 and z0 <= cz <= z1:
                picked.append(vs)

        if picked:
            remap, npts, nst, nidx = {}, [], [], []
            for vs in picked:
                for v in vs:
                    if v not in remap:
                        remap[v] = len(npts)
                        npts.append(Gf.Vec3f(pts[v][0] + 0.0015, pts[v][1], pts[v][2]))
                        nst.append(Gf.Vec2f(0.5 + pts[v][1] / DECAL_W, (pts[v][2] - z0) / DECAL_H))
                    nidx.append(remap[v])

            decals_path = f"/visuals/{DECAL_LINK}/decals"
            if not stage.GetPrimAtPath(decals_path):
                UsdGeom.Xform.Define(stage, decals_path)
            decals_xform = stage.GetPrimAtPath(decals_path)
            for prop in ("material:binding", "material:binding:strength"):
                if decals_xform.HasProperty(prop):
                    decals_xform.RemoveProperty(prop)

            mesh_path_d = f"{decals_path}/decal_logubt"
            if stage.GetPrimAtPath(mesh_path_d) and root_layer.GetPrimAtPath(mesh_path_d):
                stage.RemovePrim(mesh_path_d)
            dec = UsdGeom.Mesh.Define(stage, mesh_path_d)
            dec.CreatePointsAttr(Vt.Vec3fArray(npts))
            dec.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(picked)))
            dec.CreateFaceVertexIndicesAttr(Vt.IntArray(nidx))
            dec.CreateSubdivisionSchemeAttr("none")
            dec.CreateOrientationAttr("rightHanded")
            pv = UsdGeom.PrimvarsAPI(dec.GetPrim()).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
            )
            pv.Set(Vt.Vec2fArray(nst))
            UsdShade.MaterialBindingAPI.Apply(dec.GetPrim()).Bind(decal_mat)
            print(f"[INFO] decal {len(picked)} 面 -> {mesh_path_d}")

    if dry_run:
        print("[INFO] --dry-run：未写盘")
        return {}

    if not root_layer.Save():
        raise RuntimeError(f"保存失败: {base_path}")
    print(f"[INFO] 已写入 {base_path}")
    return {}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Recolor walker_c1 USD (dex scheme).")
    parser.add_argument("--usd", default=str(TOP_USD))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force", action="store_true", help="保留参数，兼容 --force 习惯")
    return parser.parse_args()


def main() -> int:
    global _SIM_APP
    args = _parse_args()
    ensure_pxr()
    try:
        apply_palette(Path(args.usd), dry_run=args.dry_run)
    finally:
        if _SIM_APP is not None:
            _SIM_APP.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
