#!/usr/bin/env python3
"""把 walker_c1 URDF 转换为 Isaac Sim 可加载的 USD。

复用 ``scripts/import_tienkung_dex_usd.py`` 的结构，差异只有一处：
**``merge_fixed_joints=True``** —— C1 的 URDF 里有 12 个固定关节
（肩基座、palm、hand_mount 等），不合并会在 Isaac 里被当作独立刚体，导致上肢"爆炸"。
C1 没有 mimic 关节、也没有 ``*_tcp_link``，所以合并是安全的。

容器内运行（需要 GPU / Omniverse Kit）：

    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/import_walker_c1_usd.py --force

转换完成后会自动调用 ``recolor_walker_c1_usd.py`` 上色；不需要时加 ``--no-recolor``。
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "assets" / "robots" / "walker_c1"
URDF_DIR = ASSET_ROOT / "urdf"
USD_DIR = ASSET_ROOT / "usd"

# variant -> (urdf 文件名, usd 文件名)
VARIANTS: dict[str, tuple[str, str]] = {
    "hands": ("walker_c1_with_hands.urdf", "walker_c1_merged.usd"),
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert walker_c1 URDF to USD (fixed joints merged).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--variant", default="hands", choices=[*VARIANTS, "all"])
    parser.add_argument("--urdf-dir", default=str(URDF_DIR))
    parser.add_argument("--usd-dir", default=str(USD_DIR))
    parser.add_argument("--force", action="store_true", help="即使 USD 已存在也重新转换。")
    parser.add_argument(
        "--keep-fixed-joints",
        action="store_true",
        help="保留固定关节（默认合并，合并后上肢才不会散架）。",
    )
    parser.add_argument(
        "--collision-type",
        default="convex_hull",
        choices=["none", "convex_hull", "convex_decomposition", "bounding_sphere", "bounding_cube"],
    )
    parser.add_argument("--stiffness", type=float, default=0.0)
    parser.add_argument("--damping", type=float, default=1.0)
    parser.add_argument("--joint-target-type", default="position", choices=["position", "velocity", "none"])
    parser.add_argument("--no-recolor", action="store_true", help="转换后不套用配色。")

    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def _build_cfg(args: argparse.Namespace, urdf_path: Path, usd_file_name: str):
    from isaaclab.sim.converters import UrdfConverterCfg

    cfg_cls = UrdfConverterCfg
    joint_drive = None
    if "joint_drive" in cfg_cls.__dataclass_fields__:
        gains = cfg_cls.JointDriveCfg.PDGainsCfg(stiffness=args.stiffness, damping=args.damping)
        drive_cls = cfg_cls.JointDriveCfg
        drive_kwargs = {"gains": gains, "target_type": args.joint_target_type}
        if "drive_type" in drive_cls.__dataclass_fields__:
            drive_kwargs["drive_type"] = "force"
        joint_drive = drive_cls(**drive_kwargs)

    candidates = {
        "asset_path": str(urdf_path),
        "usd_dir": str(Path(args.usd_dir)),
        "usd_file_name": usd_file_name,
        "force_usd_conversion": bool(args.force),
        "make_instanceable": True,
        "fix_base": False,
        # C1 的关键差异：合并 12 个固定关节
        "merge_fixed_joints": not args.keep_fixed_joints,
        "convert_mimic_joints_to_normal_joints": True,
        "collision_type": args.collision_type,
        "self_collision": False,
        "joint_drive": joint_drive,
    }
    fields = set(cfg_cls.__dataclass_fields__.keys())
    cfg_kwargs = {k: v for k, v in candidates.items() if k in fields and v is not None}
    skipped = sorted(set(candidates) - set(cfg_kwargs))
    if skipped:
        print(f"[WARN] 当前 Isaac Lab 版本不支持以下参数，已忽略: {skipped}")
    return cfg_cls(**cfg_kwargs)


def _convert(args: argparse.Namespace, variant: str) -> Path:
    from isaaclab.sim.converters import UrdfConverter

    urdf_name, usd_name = VARIANTS[variant]
    urdf_path = Path(args.urdf_dir) / urdf_name
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF 不存在: {urdf_path}")

    usd_path = Path(args.usd_dir) / usd_name
    print(f"[INFO] ({variant}) {urdf_path} -> {usd_path}")
    converter = UrdfConverter(_build_cfg(args, urdf_path, usd_name))
    produced = Path(converter.usd_path)
    if not produced.is_file():
        raise RuntimeError(f"转换结束但未找到 USD 产物: {produced}")
    print(f"[INFO] ({variant}) USD 生成完成: {produced} ({produced.stat().st_size / 1048576:.1f} MB)")
    return produced


def main() -> int:
    args = _parse_args()

    from isaaclab.app import AppLauncher

    simulation_app = AppLauncher(args).app
    exit_code = 0
    try:
        try:
            import omni.kit.app

            omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
                "isaacsim.asset.importer.urdf", True
            )
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] 启用 URDF importer 扩展失败（继续）: {exc}")

        Path(args.usd_dir).mkdir(parents=True, exist_ok=True)
        variants = list(VARIANTS) if args.variant == "all" else [args.variant]

        for variant in variants:
            try:
                produced = _convert(args, variant)
                if not args.no_recolor:
                    try:
                        sys.path.insert(0, str(Path(__file__).resolve().parent))
                        from recolor_walker_c1_usd import apply_palette

                        apply_palette(produced)
                    except Exception as exc:  # noqa: BLE001
                        print(f"[WARN] 配色失败（USD 已生成，可稍后单独跑）: {exc}")
            except Exception:
                exit_code = 1
                print(f"[ERROR] ({variant}) 转换失败:")
                traceback.print_exc()
    finally:
        simulation_app.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
