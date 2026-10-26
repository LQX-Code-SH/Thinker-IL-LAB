#!/usr/bin/env python3
"""给 tienkung_dex 的 USD 设置外观配色。

两套方案（``--scheme``）：

- ``pro``（默认）：复用 ``tienkung_pro`` 已调好的材质取值（黑 / 灰 / 深灰 /
  金属 / 白），按 link 角色映射到 dex，观感与 pro 场景一致；
- ``reference``：早期按参考照片手工分的五色方案（亮银 / 枪灰 / 石墨 / 蓝 / 青）。

URDF 里机身所有 link 共用同一个 0.753 灰，USD 转换后整体是一块均匀灰模。
本脚本在 **USD 层**按 link 名重新绑定 OmniPBR 材质，只改材质的颜色、金属度、
粗糙度，不改几何、质量、关节或碰撞体。

USD 转换默认 ``make_instanceable=True``，顶层 USD 里的 link 都是 instance proxy，
无法在顶层写覆盖，所以配色直接写进转换生成的 ``configuration/<variant>_base.usd``
（该层同时放着 ``/visuals`` 网格和 ``Looks`` 材质），physics/robot/sensor 分层不动。
``import_tienkung_dex_usd.py`` 转换结束后会自动调用本脚本；单独调整配色时：

    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/recolor_tienkung_dex_usd.py --variant inspire

    # 换回早期五色方案
    ... scripts/recolor_tienkung_dex_usd.py --variant inspire --scheme reference
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "assets" / "robots" / "tienkung_dex"
USD_DIR = ASSET_ROOT / "usd"

# variant -> 顶层 USD 文件名（sim 实际加载的就是这一层）
VARIANTS: dict[str, str] = {
    "base": "tienkung_dex_base.usd",
    "inspire": "tienkung_dex_inspire.usd",
    "brainco": "tienkung_dex_brainco.usd",
}


@dataclass(frozen=True)
class Paint:
    """一个 OmniPBR 材质的取值；metallic/roughness 为 None 时保留模板默认。"""

    color: tuple[float, float, float]
    metallic: float | None = None
    roughness: float | None = None


# ---------------------------------------------------------------------------
# 方案一：pro —— 取自 assets/robots/tienkung_pro/tienkung_pro_v2.usd
# ---------------------------------------------------------------------------
# pro 的机身是纯色材质按网格/子网格绑定，没有贴图（只有胸前 logo 用 PNG）。
# 这里按面数占比取每个 link 的实际主色：
#   主体结构/胸甲/骨盆/脚掌    black      (0.0037, UsdPreviewSurface)
#   大腿外壳/小腿/头部壳体      grey       (0.23,0.24,0.30, OmniSurface)
#   肩 roll/yaw、肘、腕 roll    dark_grey  (0.098,0.106,0.125)
#   踝 pitch                   metal      (0.56, metalness 0.8)
#   手掌/手背外壳               white      (1.0,0.96,0.91)
PRO_PALETTE: dict[str, Paint] = {
    "pro_black": Paint((0.003735, 0.003735, 0.003735), metallic=0.02, roughness=0.65),
    "pro_grey": Paint((0.229573, 0.243125, 0.297297), metallic=0.22, roughness=0.35),
    "pro_dark_grey": Paint((0.098039, 0.105882, 0.125490), metallic=0.0, roughness=0.40),
    "pro_metal": Paint((0.563707, 0.559354, 0.559354), metallic=0.80, roughness=0.30),
    "pro_white": Paint((1.0, 0.960570, 0.911197), metallic=0.0, roughness=0.10),
    # pro: OmniSurface 全黑 + specular_reflection_roughness=0 -> 镜面高光黑（头部外壳/摄像头）
    "pro_head_black": Paint((0.000001, 0.000001, 0.000001), metallic=0.0, roughness=0.0),
    # pro: 摄像头模组指示灯（dex 无同款灯，仅备用；roughness 用 MDL 默认近似值）
    "pro_blue": Paint((0.248834, 0.819603, 0.826255), metallic=0.42, roughness=0.30),
    "pro_stop_red": Paint((0.992278, 0.0, 0.0), metallic=0.50, roughness=0.30),
}

PRO_MATERIAL_NAMES: dict[str, str] = {
    "pro_black": "ProBlack",
    "pro_grey": "ProGrey",
    "pro_dark_grey": "ProDarkGrey",
    "pro_metal": "ProMetal",
    "pro_white": "ProWhite",
    "pro_head_black": "ProHeadBlack",
    "pro_blue": "ProBlue",
    "pro_stop_red": "ProStopRed",
}

# pro 侧 link -> 材质的对应（按面数占比取主色），同名 link 直接沿用；
# dex 的 waist_*/hip_yaw_*/wrist_pitch_* 角色与 pro 不同，按角色映射。
_PRO_GREY_PREFIXES = ("hip_yaw_", "knee_pitch_")  # 大腿外壳、小腿
_PRO_GREY_LINKS = {"head_pitch_link"}  # 头部壳体
# pro 的 head_black 用在 head_roll 外壳和 cam_black 摄像头子网格上（镜面黑）
_PRO_HEAD_BLACK_LINKS = {"head_yaw_link", "camera_head_link"}
_PRO_METAL_PREFIXES = ("ankle_pitch_",)  # 踝关节金属件
_PRO_DARK_GREY_PREFIXES = (
    "shoulder_roll_",
    "shoulder_yaw_",
    "elbow_pitch_",
    "elbow_yaw_",
    "wrist_roll_",
)

_HAND_TOKENS = ("hand", "thumb", "index", "middle", "ring", "pinky", "palm", "finger")
# pro 双手是白色外壳 + 黑色指节：掌心/指根/法兰用白，指尖与传感器用黑
_HAND_WHITE_TOKENS = ("casing", "palm", "base", "flange", "mcp", "pip", "proximal", "metacarpal")
_HAND_BLACK_TOKENS = ("force_sensor", "touch")


def pro_palette_for_link(link: str) -> str:
    """按 role 把 dex 的 link 映射到 pro 调色板；未命中规则的统一黑色主体。"""
    if any(tok in link for tok in _HAND_TOKENS):
        if any(tok in link for tok in _HAND_BLACK_TOKENS):
            return "pro_black"
        if any(tok in link for tok in _HAND_WHITE_TOKENS):
            return "pro_white"
        return "pro_black"
    # BrainCo 手部安装基座不带 hand 字样
    if link in {"left_base_link", "right_base_link"}:
        return "pro_white"
    if link in _PRO_HEAD_BLACK_LINKS:
        return "pro_head_black"
    if link in _PRO_GREY_LINKS or link.startswith(_PRO_GREY_PREFIXES):
        return "pro_grey"
    if link.startswith(_PRO_METAL_PREFIXES):
        return "pro_metal"
    if link.startswith(_PRO_DARK_GREY_PREFIXES):
        return "pro_dark_grey"
    return "pro_black"


# ---------------------------------------------------------------------------
# 方案二：reference —— 早期按参考照片手工分的五色
# ---------------------------------------------------------------------------
REFERENCE_PALETTE: dict[str, Paint] = {
    "silver": Paint((0.72, 0.73, 0.75)),  # 亮银外壳：胸甲、大腿
    "gunmetal": Paint((0.30, 0.31, 0.33)),  # 枪灰结构件：上臂、前臂、小腿、双手
    "graphite": Paint((0.09, 0.095, 0.105)),  # 石墨黑：头、腰腹、骨盆、关节、脚掌
    "accent_blue": Paint((0.12, 0.45, 0.62)),  # 蓝色饰圈：腕关节、脚踝正面
    "accent_cyan": Paint((0.18, 0.68, 0.78)),  # 青色：面部相机/面罩
}

REFERENCE_MATERIAL_NAMES: dict[str, str] = {
    "silver": "DexSilver",
    "gunmetal": "DexGunmetal",
    "graphite": "DexGraphite",
    "accent_blue": "DexAccentBlue",
    "accent_cyan": "DexAccentCyan",
}

_REFERENCE_SILVER_LINKS = {"waist_pitch_link"}
_REFERENCE_SILVER_PREFIXES = ("hip_yaw_",)
_REFERENCE_GUNMETAL_PREFIXES = ("shoulder_yaw_", "wrist_pitch_", "knee_pitch_")
_REFERENCE_BLUE_PREFIXES = ("wrist_roll_", "ankle_pitch_")
_REFERENCE_CYAN_LINKS = {"camera_head_link"}


def reference_palette_for_link(link: str) -> str:
    """返回 link 对应的调色板 key；未命中规则的 link 统一石墨黑。"""
    if link in _REFERENCE_CYAN_LINKS:
        return "accent_cyan"
    if link in _REFERENCE_SILVER_LINKS or link.startswith(_REFERENCE_SILVER_PREFIXES):
        return "silver"
    if link.startswith(_REFERENCE_BLUE_PREFIXES):
        return "accent_blue"
    if link.startswith(_REFERENCE_GUNMETAL_PREFIXES) or any(
        tok in link for tok in _HAND_TOKENS
    ):
        return "gunmetal"
    return "graphite"


SCHEMES = {
    "pro": (PRO_PALETTE, PRO_MATERIAL_NAMES, pro_palette_for_link),
    "reference": (REFERENCE_PALETTE, REFERENCE_MATERIAL_NAMES, reference_palette_for_link),
}

# 兼容旧调用方（import 脚本 / 文档里的示例）
PALETTE = REFERENCE_PALETTE
MATERIAL_NAMES = REFERENCE_MATERIAL_NAMES
palette_for_link = reference_palette_for_link


# SimulationApp 需要在进程内保持存活，否则 pxr 绑定会被回收
_SIM_APP = None


def ensure_pxr() -> None:
    """确保 ``pxr`` 可用。

    ``/isaac-sim/python.sh`` 仅提供 Python 解释器，USD 的 pxr 绑定由 Kit 在启动时
    注入 sys.path，因此未启动 SimulationApp 时需先无头启动一次（约 20-40 秒）。
    """
    try:
        import pxr  # noqa: F401

        return
    except ModuleNotFoundError:
        pass

    print("[INFO] pxr 未就绪，启动无头 Isaac Sim 以加载 USD 运行时...")
    from isaacsim import SimulationApp

    # SimulationApp 会把 sys.argv 透传给 Kit，避免本脚本的参数被当成 Kit 参数
    original_argv = sys.argv
    sys.argv = [original_argv[0]]
    try:
        global _SIM_APP
        _SIM_APP = SimulationApp({"headless": True, "enable_cameras": False})
    finally:
        sys.argv = original_argv


def _base_usd_path(top_usd: Path) -> Path:
    """顶层 USD -> 同目录 configuration 下的 base 层；传入 base 层则原样返回。

    不能按 ``_base.usd`` 后缀判断：base 变体的顶层文件本身就叫
    ``tienkung_dex_base.usd``，会被误认成 base 层。
    """
    if top_usd.parent.name == "configuration":
        return top_usd
    return top_usd.parent / "configuration" / f"{top_usd.stem}_base.usd"


def _robot_root(stage):
    """USD 默认 prim 即机器人根（``/tienkung_dex_inspire`` / ``..._brainco``）。"""
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        raise RuntimeError("USD 没有 defaultPrim，无法确定机器人根节点")
    return default_prim.GetPath()


def _iter_link_visuals(stage):
    """遍历 base 层里 ``/visuals/<link>/<visual>`` 这一层。

    内层 Xform 是 URDF importer 写入 ``material:binding`` 的位置；
    其下 mesh 上还有一条指向 DefaultMaterial 的弱绑定，不需要单独处理。

    内层 Xform 名不一定等于 link 名：一个 link 有多个 ``<visual>`` 时，
    名称为 mesh 文件名（如 ``left_base_link/left_base_visual_link``），
    所以按路径层级判断，不能要求两段同名。
    """
    prefix = "/visuals/"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(prefix):
            continue
        parts = path[len(prefix):].split("/")
        if len(parts) == 2:
            yield parts[0], prim


def _find_template_material(stage, root):
    """找一个由 URDF importer 生成的 OmniPBR 材质，作为新材质的模板。"""
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
    if not attr:  # 不存在的 attribute 是 invalid 对象，不是 None
        attr = shader.CreateAttribute(name, value_type)
    attr.Set(value)


def _ensure_palette_materials(stage, root, palette, material_names):
    """在根层创建/更新调色板材质，返回 {key: UsdShade.Material}。"""
    from pxr import Gf, Sdf, UsdShade

    template = _find_template_material(stage, root)
    template_spec = template.GetPrimStack()[0]
    root_layer = stage.GetRootLayer()
    looks_path = root.AppendChild("Looks")
    stage.DefinePrim(looks_path)  # 根层里建一个 over，保证父级 spec 存在

    materials = {}
    for key, paint in palette.items():
        mat_path = looks_path.AppendChild(material_names[key])
        if not stage.GetPrimAtPath(mat_path):
            Sdf.CopySpec(template_spec.layer, template_spec.path, root_layer, mat_path)
        mat_prim = stage.GetPrimAtPath(mat_path)
        shader = next(c for c in mat_prim.GetChildren() if c.GetTypeName() == "Shader")
        _set_input(
            shader,
            "inputs:diffuse_color_constant",
            Gf.Vec3f(*paint.color),
            Sdf.ValueTypeNames.Color3f,
        )
        if paint.metallic is not None:
            _set_input(
                shader,
                "inputs:metallic_constant",
                paint.metallic,
                Sdf.ValueTypeNames.Float,
            )
        if paint.roughness is not None:
            _set_input(
                shader,
                "inputs:reflection_roughness_constant",
                paint.roughness,
                Sdf.ValueTypeNames.Float,
            )
        materials[key] = UsdShade.Material(mat_prim)
    return materials


def apply_palette(
    usd_path: str | Path, dry_run: bool = False, scheme: str = "pro"
) -> dict[str, list[str]]:
    """把指定方案的调色板应用到 ``usd_path``（顶层 USD），返回 {材质 key: [link...]}。

    顶层 USD 是 instanceable 组合，实际写入的是同目录
    ``configuration/<stem>_base.usd``；传入 base 文件本身也可以。
    """
    from pxr import Usd, UsdShade

    if scheme not in SCHEMES:
        raise ValueError(f"未知配色方案: {scheme}")
    palette, material_names, link_painter = SCHEMES[scheme]

    usd_path = Path(usd_path)
    base_path = _base_usd_path(usd_path)
    if not base_path.is_file():
        raise FileNotFoundError(f"找不到 base 层 USD: {base_path}")

    stage = Usd.Stage.Open(str(base_path))
    if stage is None:
        raise RuntimeError(f"无法打开 USD: {base_path}")

    root = _robot_root(stage)
    assignments: dict[str, list[str]] = {key: [] for key in palette}
    targets = []
    for link, prim in _iter_link_visuals(stage):
        key = link_painter(link)
        assignments[key].append(link)
        targets.append((link, prim, key))

    if not targets:
        raise RuntimeError(f"{usd_path} 中未找到 /<link>/visuals/<link> 节点")

    print(f"[INFO] {base_path.name}: {len(targets)} 个 link 待配色（方案 {scheme}）")
    for key, links in assignments.items():
        if links:
            sample = ", ".join(sorted(links)[:4])
            more = " ..." if len(links) > 4 else ""
            print(f"  {key:14s} {len(links):3d}  {sample}{more}")

    if dry_run:
        print("[INFO] --dry-run：未写入 USD")
        return assignments

    materials = _ensure_palette_materials(stage, root, palette, material_names)
    for _link, prim, key in targets:
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        api.Bind(materials[key], UsdShade.Tokens.strongerThanDescendants)

    root_layer = stage.GetRootLayer()
    if not root_layer.Save():
        raise RuntimeError(f"保存 USD 失败: {base_path}")
    print(f"[INFO] 已写入 {base_path}")
    return assignments


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply a color scheme to tienkung_dex USDs.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--variant",
        default="all",
        choices=[*VARIANTS, "all"],
        help="要配色的模型变体；all 依次处理 base/inspire/brainco，缺失的文件跳过。",
    )
    parser.add_argument(
        "--scheme",
        default="pro",
        choices=list(SCHEMES),
        help="配色方案：pro 复用 tienkung_pro 材质；reference 为早期五色方案。",
    )
    parser.add_argument("--usd-dir", default=str(USD_DIR), help="顶层 USD 所在目录。")
    parser.add_argument(
        "--usd",
        action="append",
        default=[],
        help="直接指定顶层 USD 路径（可重复），优先于 --variant。",
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印配色分组，不写 USD。")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    ensure_pxr()

    if args.usd:
        paths = [Path(p) for p in args.usd]
    else:
        names = list(VARIANTS.values()) if args.variant == "all" else [VARIANTS[args.variant]]
        paths = [Path(args.usd_dir) / name for name in names]

    existing = [p for p in paths if p.is_file()]
    missing = [p for p in paths if not p.is_file()]
    for path in missing:
        print(f"[WARN] 跳过不存在的 USD: {path}")
    if not existing:
        print("[ERROR] 没有可处理的 USD")
        return 1

    for path in existing:
        apply_palette(path, dry_run=args.dry_run, scheme=args.scheme)
    return 0


if __name__ == "__main__":
    sys.exit(main())
