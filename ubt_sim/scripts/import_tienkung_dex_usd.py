#!/usr/bin/env python3
"""把 tienkung_dex 的 URDF 转换为 Isaac Sim 可加载的 USD。

依赖 Isaac Lab 的 ``UrdfConverter``（内部调用 Isaac Sim URDF Importer），
必须在 Isaac Sim 容器内运行（需要 GPU / Omniverse Kit）：

    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/import_tienkung_dex_usd.py --variant inspire --force

产物默认写入 ``assets/robots/tienkung_dex/usd/``，随后用
``scripts/check_tienkung_dex_usd.py`` 做关节名/限位/质量自检。
转换完成后会自动调用 ``recolor_tienkung_dex_usd.py`` 按参考照片配色；
不需要配色时加 ``--no-recolor``。
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "assets" / "robots" / "tienkung_dex"
URDF_DIR = ASSET_ROOT / "urdf"
USD_DIR = ASSET_ROOT / "usd"

# 同目录脚本，转换为 USD 后套用外观配色
sys.path.insert(0, str(Path(__file__).resolve().parent))
from recolor_tienkung_dex_usd import apply_palette  # noqa: E402

# variant -> (urdf 文件名, usd 文件名)
VARIANTS: dict[str, tuple[str, str]] = {
    "base": ("tienkung_dex_base.urdf", "tienkung_dex_base.usd"),
    "inspire": ("tienkung_dex_inspire.urdf", "tienkung_dex_inspire.usd"),
    "brainco": ("tienkung_dex_brainco.urdf", "tienkung_dex_brainco.usd"),
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert tienkung_dex URDF assets to USD.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--variant",
        default="all",
        choices=[*VARIANTS, "all"],
        help="要转换的模型变体；all 依次转换 base/inspire/brainco。",
    )
    parser.add_argument("--urdf-dir", default=str(URDF_DIR), help="URDF 目录。")
    parser.add_argument("--usd-dir", default=str(USD_DIR), help="USD 输出目录。")
    parser.add_argument("--force", action="store_true", help="即使 USD 已存在也重新转换。")
    parser.add_argument(
        "--keep-mimic",
        action="store_true",
        help="保留 URDF mimic 关节；默认转换成普通关节（由驱动层按倍率下发）。",
    )
    parser.add_argument(
        "--collision-type",
        default="convex_hull",
        choices=["none", "convex_hull", "convex_decomposition", "bounding_sphere", "bounding_cube"],
        help="碰撞体近似方式。convex_decomposition 更精确但转换慢。",
    )
    parser.add_argument("--stiffness", type=float, default=0.0, help="关节驱动刚度默认值。")
    parser.add_argument("--damping", type=float, default=1.0, help="关节驱动阻尼默认值。")
    parser.add_argument(
        "--joint-target-type",
        default="position",
        choices=["position", "velocity", "none"],
        help="关节驱动目标类型。",
    )
    parser.add_argument(
        "--no-recolor",
        action="store_true",
        help="转换后不套用参考图配色（保留 URDF 自带的统一灰色）。",
    )

    # AppLauncher 需要在此处注册 --headless / --device / --enable_cameras 等参数
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def _get_urdf_converter_cfg_cls():
    from isaaclab.sim.converters import UrdfConverterCfg

    return UrdfConverterCfg


def _build_cfg(args: argparse.Namespace, urdf_path: Path, usd_file_name: str):
    """构造 UrdfConverterCfg，自动过滤不同 Isaac Lab 版本不支持的字段。"""
    cfg_cls = _get_urdf_converter_cfg_cls()

    joint_drive = None
    if "joint_drive" in cfg_cls.__dataclass_fields__:
        pd_gains_cls = cfg_cls.JointDriveCfg.PDGainsCfg
        drive_cls = cfg_cls.JointDriveCfg
        gains = pd_gains_cls(stiffness=args.stiffness, damping=args.damping)
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
        "merge_fixed_joints": False,
        "convert_mimic_joints_to_normal_joints": not args.keep_mimic,
        "collision_type": args.collision_type,
        "self_collision": False,
        "joint_drive": joint_drive,
    }
    fields = set(cfg_cls.__dataclass_fields__.keys())
    cfg_kwargs = {k: v for k, v in candidates.items() if k in fields and v is not None}

    skipped = sorted(set(candidates) - set(cfg_kwargs))
    if skipped:
        print(f"[WARN] 当前 Isaac Lab 版本不支持以下转换参数，已忽略: {skipped}")

    return cfg_cls(**cfg_kwargs)


def _convert(args: argparse.Namespace, variant: str) -> Path:
    from isaaclab.sim.converters import UrdfConverter

    urdf_name, usd_name = VARIANTS[variant]
    urdf_path = Path(args.urdf_dir) / urdf_name
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF 不存在: {urdf_path}")

    usd_path = Path(args.usd_dir) / usd_name
    print(f"[INFO] ({variant}) {urdf_path} -> {usd_path}")
    cfg = _build_cfg(args, urdf_path, usd_name)
    converter = UrdfConverter(cfg)

    produced = Path(converter.usd_path)
    if not produced.is_file():
        raise RuntimeError(f"转换结束但未找到 USD 产物: {produced}")
    size_mb = produced.stat().st_size / (1024 * 1024)
    print(f"[INFO] ({variant}) USD 生成完成: {produced} ({size_mb:.1f} MB)")
    return produced


def main() -> int:
    args = _parse_args()

    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    exit_code = 0
    try:
        # URDF Importer 扩展在部分镜像中默认未启用
        try:
            import omni.kit.app

            ext_manager = omni.kit.app.get_app().get_extension_manager()
            ext_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        except Exception as exc:  # noqa: BLE001 - 扩展名在不同版本可能不同
            print(f"[WARN] 启用 isaacsim.asset.importer.urdf 扩展失败（继续尝试转换）: {exc}")

        Path(args.usd_dir).mkdir(parents=True, exist_ok=True)
        variants = list(VARIANTS) if args.variant == "all" else [args.variant]

        for variant in variants:
            try:
                produced = _convert(args, variant)
                if not args.no_recolor:
                    apply_palette(produced)
            except Exception:
                exit_code = 1
                print(f"[ERROR] ({variant}) 转换失败:")
                traceback.print_exc()
    finally:
        simulation_app.close()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
