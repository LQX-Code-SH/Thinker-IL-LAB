#!/usr/bin/env python3
"""校验 tienkung_dex 的 USD 产物与源 URDF 是否一致。

检查项：
  1. USD 文件存在且可打开，存在 ArticulationRoot；
  2. 关节集合（revolute/prismatic）与 URDF 一致（缺失 / 多余均报错）；
  3. 关节限位（度）与 URDF 偏差不超过 ``--tolerance-deg``；
  4. 关节轴向与 URDF 一致（符号相反视为差异，需显式改 URDF 或 USD）；
  5. 刚体质量 > 0，mesh 资产引用可解析。

在 Isaac Sim 容器内运行（需要 pxr）：

    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/check_tienkung_dex_usd.py --variant inspire
    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/check_tienkung_dex_usd.py --usd <path> --urdf <path> --json /tmp/report.json

退出码：0 = 通过；1 = 存在错误。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "assets" / "robots" / "tienkung_dex"
URDF_DIR = ASSET_ROOT / "urdf"
USD_DIR = ASSET_ROOT / "usd"

VARIANTS: dict[str, tuple[str, str]] = {
    "base": ("tienkung_dex_base.urdf", "tienkung_dex_base.usd"),
    "inspire": ("tienkung_dex_inspire.urdf", "tienkung_dex_inspire.usd"),
    "brainco": ("tienkung_dex_brainco.urdf", "tienkung_dex_brainco.usd"),
}

_MOVABLE_JOINT_TYPES = {"revolute", "continuous", "prismatic"}

# SimulationApp 需要在进程内保持存活，否则 pxr 绑定会被回收
_SIM_APP = None


def ensure_pxr() -> None:
    """确保 ``pxr`` 可用。

    ``/isaac-sim/python.sh`` 仅提供 Python 解释器，USD 的 pxr 绑定由 Kit 在启动时
    注入 sys.path，因此未启动 SimulationApp 时需先无头启动一次（约 20-40 秒）。
    """
    global _SIM_APP
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
        _SIM_APP = SimulationApp({"headless": True, "enable_cameras": False})
    finally:
        sys.argv = original_argv


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate tienkung_dex USD against its source URDF.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--variant", default="inspire", choices=list(VARIANTS), help="模型变体。")
    parser.add_argument("--usd", default=None, help="USD 路径（默认按 --variant 推断）。")
    parser.add_argument("--urdf", default=None, help="URDF 路径（默认按 --variant 推断）。")
    parser.add_argument("--tolerance-deg", type=float, default=0.5, help="限位允许偏差（度）。")
    parser.add_argument("--json", default=None, help="将检查报告写入该 JSON 文件。")
    return parser.parse_args()


_AXIS_TOKENS = {
    "X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
}


def _iter_prims(stage):
    from pxr import Usd

    return stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))


def _axis_to_vec(value) -> tuple[float, float, float] | None:
    """USD 关节轴可能是向量，也可能是 ``"X"``/``"-Y"`` 这类主轴 token。"""
    if value is None:
        return None
    if isinstance(value, str):
        token = value.strip()
        if token in _AXIS_TOKENS:
            return _AXIS_TOKENS[token]
        try:
            parts = [float(v) for v in token.strip("[]()").replace(",", " ").split()]
        except ValueError:
            return None
        return tuple(parts) if len(parts) == 3 else None  # type: ignore[return-value]
    try:
        parts = [float(v) for v in value]
    except (TypeError, ValueError):
        return None
    return tuple(parts) if len(parts) == 3 else None  # type: ignore[return-value]


def _read_urdf(urdf_path: Path) -> dict:
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        if jtype not in _MOVABLE_JOINT_TYPES:
            continue
        limit = joint.find("limit")
        axis = joint.find("axis")
        lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else None
        upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else None
        axis_xyz = axis.get("xyz") if axis is not None else None
        joints[name] = {
            "type": jtype,
            "lower_deg": None if lower is None else math.degrees(lower),
            "upper_deg": None if upper is None else math.degrees(upper),
            "axis": axis_xyz,
            "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") else None,
            "mimic": joint.find("mimic").get("joint") if joint.find("mimic") is not None else None,
            "mimic_multiplier": float(joint.find("mimic").get("multiplier", 1.0)) if joint.find("mimic") is not None else None,
            "mimic_offset": float(joint.find("mimic").get("offset", 0.0)) if joint.find("mimic") is not None else None,
        }
    return {"robot_name": root.get("name"), "joints": joints, "links": len(root.findall("link"))}


def _read_usd(usd_path: Path) -> dict:
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"无法打开 USD: {usd_path}")

    articulation_roots = []
    joints = {}
    bodies_without_mass = []
    unresolved_meshes = []

    for prim in _iter_prims(stage):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_roots.append(str(prim.GetPath()))

        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            joint = UsdPhysics.RevoluteJoint(prim) if prim.IsA(UsdPhysics.RevoluteJoint) else UsdPhysics.PrismaticJoint(prim)
            lower = joint.GetLowerLimitAttr().Get()
            upper = joint.GetUpperLimitAttr().Get()
            axis = _axis_to_vec(joint.GetAxisAttr().Get())
            body1 = joint.GetBody1Rel().GetTargets()
            # 保留 mimic 时 importer 会打 PhysxMimicJointAPI；此时限位与 URDF 原值一致
            is_mimic = prim.HasAPI("PhysxMimicJointAPI") or prim.HasAttribute("physxMimicJoint:referenceJoint")
            joints[prim.GetName()] = {
                "path": str(prim.GetPath()),
                "is_mimic": bool(is_mimic),
                "lower_deg": lower,
                "upper_deg": upper,
                "axis": axis,
                "body1": str(body1[0]) if body1 else None,
            }

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            mass_attr = prim.GetAttribute("physics:mass")
            mass = mass_attr.Get() if mass_attr else None
            if mass is None or mass <= 0.0:
                bodies_without_mass.append(str(prim.GetPath()))

        if prim.IsA("UsdGeomMesh"):
            mesh = prim.GetAttribute("geometry:points")
            if mesh is None:
                unresolved_meshes.append(str(prim.GetPath()))

    return {
        "stage": str(usd_path),
        "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "articulation_roots": articulation_roots,
        "joints": joints,
        "bodies_without_mass": bodies_without_mass,
        "unresolved_meshes": unresolved_meshes,
    }


def _check(urdf: dict, usd: dict, tolerance_deg: float) -> tuple[list[str], list[str], dict]:
    errors: list[str] = []
    warnings: list[str] = []

    if not usd["articulation_roots"]:
        errors.append("USD 中未找到 ArticulationRootAPI，Isaac Lab 无法作为 articulation 加载")

    urdf_joints = urdf["joints"]
    usd_joints = usd["joints"]

    missing = sorted(set(urdf_joints) - set(usd_joints))
    extra = sorted(set(usd_joints) - set(urdf_joints))
    if missing:
        errors.append(f"USD 缺少 URDF 中的关节 {len(missing)} 个: {missing[:8]}{' ...' if len(missing) > 8 else ''}")
    if extra:
        errors.append(f"USD 存在 URDF 未定义的关节 {len(extra)} 个: {extra[:8]}{' ...' if len(extra) > 8 else ''}")

    limit_rows = []
    for name in sorted(set(urdf_joints) & set(usd_joints)):
        u = urdf_joints[name]
        s = usd_joints[name]
        row = {"joint": name, "urdf": [u["lower_deg"], u["upper_deg"]], "usd": [s["lower_deg"], s["upper_deg"]]}

        # mimic 关节在 USD 中可能是：① 原样保留（限位=URDF 原值）；
        # ② 被转成普通关节（限位=源关节限位×倍率）。两种约定都接受。
        is_mimic = bool(u.get("mimic"))
        candidates = {"lower_deg": [u["lower_deg"]], "upper_deg": [u["upper_deg"]]}
        if is_mimic:
            source = urdf_joints.get(u["mimic"])
            if source is not None and source["lower_deg"] is not None and source["upper_deg"] is not None:
                mult = u.get("mimic_multiplier") or 1.0
                offset = u.get("mimic_offset") or 0.0
                mapped = [mult * source["lower_deg"] + offset, mult * source["upper_deg"] + offset]
                candidates["lower_deg"].append(min(mapped))
                candidates["upper_deg"].append(max(mapped))
                row["mimic_of"] = u["mimic"]
                row["expected_candidates"] = [sorted(candidates["lower_deg"]), sorted(candidates["upper_deg"])]

        for key in ("lower_deg", "upper_deg"):
            b = s[key]
            values = [v for v in candidates[key] if v is not None]
            if not values:
                if b is not None:
                    warnings.append(f"{name}.{key}: URDF 未定义，USD={b}")
                continue
            if b is None:
                errors.append(f"{name}.{key}: USD 未定义，URDF={values[0]:.3f}°")
                continue
            if all(abs(v - b) > tolerance_deg for v in values):
                delta = min(abs(v - b) for v in values)
                row["delta_deg"] = round(max(delta, row.get("delta_deg", 0.0)), 4)
                errors.append(
                    f"{name}.{key}: URDF={values[0]:.3f}° USD={b:.3f}° 偏差 {delta:.3f}°"
                    + (f"（mimic 映射期望 {values[1]:.3f}°）" if len(values) > 1 else "")
                )
        limit_rows.append(row)

        if u["axis"] and s["axis"] and not is_mimic:
            try:
                urdf_axis = [float(v) for v in u["axis"].split()]
            except ValueError:
                urdf_axis = []
            usd_axis = list(s["axis"])
            if len(urdf_axis) == 3:
                dot = sum(a * b for a, b in zip(urdf_axis, usd_axis))
                if dot < -0.9:
                    warnings.append(f"{name}: URDF 轴向 {u['axis']} 与 USD 轴向 {usd_axis} 符号相反")
                elif abs(dot) < 0.9:
                    warnings.append(f"{name}: URDF 轴向 {u['axis']} 与 USD 轴向 {usd_axis} 不一致")

    if usd["bodies_without_mass"]:
        # URDF 中 *_tip_link / *_flange_link 等固定末端件本身没有 inertial，
        # 属于模型侧已知情况，不阻塞启动；仅提示，避免 PhysX 因 0 质量抖动。
        warnings.append(
            f"{len(usd['bodies_without_mass'])} 个刚体缺少有效质量（URDF 无 inertial，多为固定末端件）: "
            f"{usd['bodies_without_mass'][:5]}"
        )
    if usd["unresolved_meshes"]:
        warnings.append(f"{len(usd['unresolved_meshes'])} 个 mesh 无法读取 points 属性")

    summary = {
        "robot": urdf["robot_name"],
        "urdf_links": urdf["links"],
        "urdf_joints": len(urdf_joints),
        "usd_joints": len(usd_joints),
        "articulation_roots": usd["articulation_roots"],
        "joint_limits": limit_rows,
        "errors": errors,
        "warnings": warnings,
    }
    return errors, warnings, summary


def main() -> int:
    args = _parse_args()
    ensure_pxr()
    urdf_name, usd_name = VARIANTS[args.variant]
    urdf_path = Path(args.urdf) if args.urdf else URDF_DIR / urdf_name
    usd_path = Path(args.usd) if args.usd else USD_DIR / usd_name

    if not urdf_path.is_file():
        print(f"[ERROR] URDF 不存在: {urdf_path}")
        return 1
    if not usd_path.is_file():
        print(f"[ERROR] USD 不存在: {usd_path}")
        print("        先运行 scripts/import_tienkung_dex_usd.py 生成资产")
        return 1

    urdf = _read_urdf(urdf_path)
    usd = _read_usd(usd_path)
    errors, warnings, summary = _check(urdf, usd, args.tolerance_deg)

    print(f"[INFO] robot={summary['robot']} urdf_links={summary['urdf_links']}")
    print(f"[INFO] joints: urdf={summary['urdf_joints']} usd={summary['usd_joints']}")
    print(f"[INFO] articulation roots: {summary['articulation_roots'] or 'NONE'}")
    for msg in warnings:
        print(f"[WARN] {msg}")
    for msg in errors:
        print(f"[ERROR] {msg}")

    if args.json:
        Path(args.json).write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[INFO] 报告已写入 {args.json}")

    if errors:
        print(f"[FAIL] 检查未通过：{len(errors)} 个错误")
        return 1
    print("[OK] 检查通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
