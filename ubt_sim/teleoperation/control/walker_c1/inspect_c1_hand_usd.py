#!/usr/bin/env python3
"""Audit the actual Walker C1 hand joints in the runtime main USD.

Run with an Isaac/pxr Python environment.  The script also parses the control
list from ``devices/walker_c1/config.py`` without importing Isaac Lab, then
checks the standalone retarget config against both sources.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
from pathlib import Path
import sys
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[4]
DEFAULT_RETARGET_CONFIG = SCRIPT_PATH.with_name("pico_hand_retarget_config.json")
CONTROL_CONFIG = REPO_ROOT / "ubt_sim/source/ubt_sim/devices/walker_c1/config.py"


def _literal_string_list(source_path: Path, variable: str) -> list[str]:
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if not any(isinstance(target, ast.Name) and target.id == variable for target in targets):
            continue
        value = ast.literal_eval(node.value)
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            break
        return value
    raise RuntimeError(f"could not read literal {variable} from {source_path}")


def _target_name(targets: list[Any]) -> str | None:
    if len(targets) != 1:
        return None
    return Path(str(targets[0])).name


def audit_main_usd(
    retarget_config_path: Path = DEFAULT_RETARGET_CONFIG,
    usd_path: Path | None = None,
) -> dict[str, Any]:
    try:
        from pxr import Usd
    except ImportError as exc:
        raise RuntimeError(
            "pxr is unavailable; run with Isaac Sim python or install usd-core in a temporary environment"
        ) from exc

    config = json.loads(retarget_config_path.read_text(encoding="utf-8"))
    main_usd = usd_path or REPO_ROOT / config["asset"]["usd_path"]
    stage = Usd.Stage.Open(str(main_usd.resolve()), Usd.Stage.LoadAll)
    if stage is None:
        raise RuntimeError(f"failed to open main USD: {main_usd}")

    control_order = {
        "left": _literal_string_list(CONTROL_CONFIG, "WALKER_C1_LEFT_HAND_JOINTS"),
        "right": _literal_string_list(CONTROL_CONFIG, "WALKER_C1_RIGHT_HAND_JOINTS"),
    }
    semantic_order = config["joint_order"]
    joint_root = config["asset"]["joint_prim_path"].rstrip("/")
    records: list[dict[str, Any]] = []
    errors: list[str] = []

    for side in ("left", "right"):
        configured_names = [config["joints"][key]["joint_name"][side] for key in semantic_order]
        if configured_names != control_order[side]:
            errors.append(f"{side} retarget joint order differs from ActionsCfg control order")
        for order_index, semantic in enumerate(semantic_order):
            expected = config["joints"][semantic]
            name = expected["joint_name"][side]
            prim = stage.GetPrimAtPath(f"{joint_root}/{name}")
            if not prim or not prim.IsValid():
                errors.append(f"missing USD prim for {name}")
                continue
            lower_deg = prim.GetAttribute("physics:lowerLimit").Get()
            upper_deg = prim.GetAttribute("physics:upperLimit").Get()
            velocity_deg_s = prim.GetAttribute("physxJoint:maxJointVelocity").Get()
            lower_rad = math.radians(float(lower_deg))
            upper_rad = math.radians(float(upper_deg))
            velocity_rad_s = math.radians(float(velocity_deg_s))
            body0 = _target_name(prim.GetRelationship("physics:body0").GetTargets())
            body1 = _target_name(prim.GetRelationship("physics:body1").GetTargets())
            applied_schemas = list(prim.GetAppliedSchemas())
            checks = {
                "type": prim.GetTypeName() == "PhysicsRevoluteJoint",
                "enabled": prim.GetAttribute("physics:jointEnabled").Get() is True,
                "in_articulation": prim.GetAttribute("physics:excludeFromArticulation").Get() is False,
                "angular_drive": "PhysicsDriveAPI:angular" in applied_schemas,
                "independent_no_mimic": not any(
                    "mimic" in str(schema).lower() for schema in applied_schemas
                ),
                "parent": body0 == expected["parent_link"][side],
                "child": body1 == expected["child_link"][side],
                "q_min": math.isclose(lower_rad, float(expected["q_min_rad"][side]), abs_tol=2e-6),
                "q_max": math.isclose(upper_rad, float(expected["q_max_rad"][side]), abs_tol=2e-6),
                "velocity": math.isclose(
                    velocity_rad_s,
                    float(expected["velocity_limit_rad_s"]),
                    abs_tol=2e-6,
                ),
            }
            failed = [key for key, passed in checks.items() if not passed]
            if failed:
                errors.append(f"{name} failed: {', '.join(failed)}")
            records.append(
                {
                    "side": side,
                    "order": order_index,
                    "semantic": semantic,
                    "joint_name": name,
                    "type": prim.GetTypeName(),
                    "parent_link": body0,
                    "child_link": body1,
                    "axis": str(prim.GetAttribute("physics:axis").Get()),
                    "q_min_rad": lower_rad,
                    "q_max_rad": upper_rad,
                    "velocity_limit_rad_s": velocity_rad_s,
                    "drive_type": str(prim.GetAttribute("drive:angular:physics:type").Get()),
                    "checks": checks,
                }
            )

    return {
        "usd_path": str(main_usd.resolve()),
        "default_prim": str(stage.GetDefaultPrim().GetPath()),
        "meters_per_unit": float(stage.GetMetadata("metersPerUnit")),
        "up_axis": str(stage.GetMetadata("upAxis")),
        "control_order": control_order,
        "joints": records,
        "errors": errors,
        "ok": not errors and len(records) == 22,
    }


def _print_report(report: dict[str, Any]) -> None:
    print(f"main USD: {report['usd_path']}")
    print(
        f"defaultPrim={report['default_prim']} upAxis={report['up_axis']} "
        f"metersPerUnit={report['meters_per_unit']}"
    )
    for side in ("left", "right"):
        print(f"\n{side.upper()} control order:")
        print(" idx joint_name                 q_min(rad) q_max(rad) vel(rad/s) role")
        for item in report["joints"]:
            if item["side"] != side:
                continue
            print(
                f" {item['order']:>2}  {item['joint_name']:<26} "
                f"{item['q_min_rad']:>10.4f} {item['q_max_rad']:>10.4f} "
                f"{item['velocity_limit_rad_s']:>10.4f} {item['semantic']}"
            )
    if report["errors"]:
        print("\nFAILED:", file=sys.stderr)
        for error in report["errors"]:
            print(f"- {error}", file=sys.stderr)
    else:
        print("\nOK: 22 driven revolute joints match the retarget config and control order.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_RETARGET_CONFIG)
    parser.add_argument("--usd", type=Path, default=None)
    parser.add_argument("--json", action="store_true", help="print machine-readable audit output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report = audit_main_usd(args.config, args.usd)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        _print_report(report)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
