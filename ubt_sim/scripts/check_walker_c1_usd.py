#!/usr/bin/env python3
"""walker_c1 USD 自检：把当前配色/关节方案作为基线做回归校验。

容器内运行：

    docker exec -w /ubt_sim ubt-sim /isaac-sim/python.sh \
        scripts/check_walker_c1_usd.py

检查项：
1. articulation root 存在且唯一
2. revolute = 53、fixed = 0（未合并固定关节会让上肢散架）
3. 相机 5 路
4. 每个 visual Xform 都有材质绑定，且不是转换器默认材质
5. 头部 `head_pitch_link` 有 22 个 GeomSubset
6. 胸口 decal 存在且有贴图材质
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
USD_DIR = PROJECT_ROOT / "assets" / "robots" / "walker_c1" / "usd"
TOP_USD = USD_DIR / "walker_c1_merged.usd"
BASE_USD = USD_DIR / "configuration" / "walker_c1_merged_base.usd"

EXPECT_REVOLUTE = 53
EXPECT_FIXED = 0
# merged 版不自带相机 prim；采集相机由 task/walker_c1_parlor 运行时新建
EXPECT_CAMERAS = 0
# 22 个 HP 零件 + 1 个兜底 subset
EXPECT_HEAD_SUBSETS = 23
DECAL_PATH = "/visuals/waist_roll_link/decals/decal_logubt"

_SIM_APP = None


def ensure_pxr() -> None:
    try:
        import pxr  # noqa: F401

        return
    except ModuleNotFoundError:
        pass
    print("[INFO] pxr 未就绪，无头启动 Isaac Sim ...")
    from isaacsim import SimulationApp

    global _SIM_APP
    argv = sys.argv
    sys.argv = [argv[0]]
    try:
        _SIM_APP = SimulationApp({"headless": True, "enable_cameras": False})
    finally:
        sys.argv = argv


def main() -> int:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    if not TOP_USD.is_file():
        print(f"[FAIL] 缺少 {TOP_USD}")
        return 1

    stage = Usd.Stage.Open(str(TOP_USD))
    base = Usd.Stage.Open(str(BASE_USD)) if BASE_USD.is_file() else None
    failures: list[str] = []

    arts = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print(f"articulation root : {len(arts)}  ({arts[0].GetPath() if arts else '-'})")
    if len(arts) != 1:
        failures.append(f"articulation root 应为 1，实际 {len(arts)}")

    rev = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    fix = [p for p in stage.Traverse() if p.IsA(UsdPhysics.FixedJoint)]
    print(f"joints            : revolute={len(rev)} fixed={len(fix)}")
    if len(rev) != EXPECT_REVOLUTE:
        failures.append(f"revolute 应为 {EXPECT_REVOLUTE}，实际 {len(rev)}")
    if len(fix) != EXPECT_FIXED:
        failures.append(f"fixed 应为 {EXPECT_FIXED}，实际 {len(fix)}（未合并会散架）")

    cams = [p for p in stage.Traverse() if p.IsA(UsdGeom.Camera)]
    print(f"cameras           : {len(cams)}")
    if len(cams) != EXPECT_CAMERAS:
        failures.append(f"相机应为 {EXPECT_CAMERAS}，实际 {len(cams)}")

    if base is not None:
        unbound, defaulted = [], []
        for link in base.GetPrimAtPath("/visuals").GetChildren():
            for vis in link.GetChildren():
                if vis.GetTypeName() != "Xform" or vis.GetName() == "decals":
                    continue
                # head_pitch_link 走 GeomSubset，Xform 上故意不绑材质
                if link.GetName() == "head_pitch_link":
                    continue
                mat, _ = UsdShade.MaterialBindingAPI(vis).ComputeBoundMaterial()
                name = mat.GetPath().name if mat else None
                if name is None:
                    # 头部原整块 mesh 已设 invisible（被 23 个 subset 取代），不参与统计
                    if vis.GetName().startswith("mesh"):
                        continue
                    unbound.append(f"{link.GetName()}/{vis.GetName()}")
                elif not name.startswith(("Custom_", "Decal_")):
                    defaulted.append(f"{link.GetName()}/{vis.GetName()} -> {name}")
        print(f"visual 绑定       : 未绑 {len(unbound)}，默认材质 {len(defaulted)}")
        if unbound:
            failures.append(f"{len(unbound)} 个 visual 无材质，如 {unbound[:3]}")
        if defaulted:
            failures.append(f"{len(defaulted)} 个 visual 仍是默认材质，如 {defaulted[:3]}")

        head = base.GetPrimAtPath("/visuals/head_pitch_link/head_pitch_link/mesh")
        subs = [c for c in head.GetChildren() if c.GetTypeName() == "GeomSubset"] if head else []
        print(f"头部 GeomSubset   : {len(subs)}")
        if len(subs) != EXPECT_HEAD_SUBSETS:
            failures.append(f"头部 subset 应为 {EXPECT_HEAD_SUBSETS}，实际 {len(subs)}")

        decal = base.GetPrimAtPath(DECAL_PATH)
        decal_mat = None
        if decal:
            decal_mat, _ = UsdShade.MaterialBindingAPI(decal).ComputeBoundMaterial()
        decal_name = decal_mat.GetPath().name if decal_mat else None
        print(f"胸口 decal        : {bool(decal)}  material={decal_name}")
        if not decal:
            failures.append(f"缺少 decal: {DECAL_PATH}")
        elif not (decal_name and decal_name.startswith("Decal_")):
            failures.append(f"decal 材质异常: {decal_name}")
    else:
        failures.append(f"缺少 base 层: {BASE_USD}")

    print()
    if failures:
        print("[FAIL] 发现以下问题：")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("[PASS] 关节 / 相机 / 配色 / decal 全部符合基线")
    return 0


if __name__ == "__main__":
    ensure_pxr()
    code = main()
    if _SIM_APP is not None:
        _SIM_APP.close()
    raise SystemExit(code)
