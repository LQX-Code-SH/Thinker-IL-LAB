#!/usr/bin/env python3
"""Walker C1 开发者模式 / 运控权限（half_sdk）管理。

为什么需要它
    C1 的机械臂只有在下面两个条件同时成立时才接收 ROS2 关节指令：
      1. 工作模式 ``workMode = develop``（开发者模式，机器人会语音播报）；
      2. 运控权限 ``developModeState = HalfSdkActive``（half_sdk，上半身电机由 SDK 接管）。
    此外 half_sdk 的前置条件是 ``legStatus = standing``（腿部由内置算法托管平衡）。

    VLA（rollout / 推理服务）只下发双臂 + 双手，因此只允许 half_sdk：
      - ``normal``    ：内部算法控制全身，手臂关节指令会被忽略；
      - ``half_sdk``  ：手臂/头/腰由 SDK 控制，腿部仍由系统保持平衡（VLA 唯一安全档位）；
      - ``full_sdk``  ：全身电机交给 SDK，内置平衡退出，必须悬吊/扶持（本工具需 --yes）。

用法
    python3 sdk_mode.py status                  # 查看工作模式 / 运控权限 / 腿部 / 急停
    python3 sdk_mode.py check                   # 校验"是否可部署 VLA"，可部署返回 0
    python3 sdk_mode.py half_sdk                # develop -> 腿部 STAND -> half_sdk（幂等）
    python3 sdk_mode.py normal                  # 回到 normal（保留开发者模式）
    python3 sdk_mode.py develop                 # 只切工作模式到 develop
    python3 sdk_mode.py work-mode joystick      # 切任意工作模式（退出开发者模式用）
    python3 sdk_mode.py full_sdk --yes          # 全身 SDK（危险，需悬吊）

退出码
    0  成功 / 状态满足
    1  执行或连接失败
    2  check 未通过（当前不是 half_sdk 等），供脚本做部署门禁

连接
    机器内（开发者容器）默认走 ``/tmp/cc_api.sock``，只需 api_id；
    跨机调试才需要 ``--address 0.0.0.0:51000 --token <token>``。
    Astron 上部分接口需要 root，请尽量用 root 运行。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time

EXIT_OK = 0
EXIT_ERR = 1
EXIT_CHECK_FAIL = 2

MODES = ("normal", "half_sdk", "full_sdk")
COMMANDS = ("status", "check", "develop", "work-mode", "half_sdk", "normal", "full_sdk")

# 目标模式 -> 期望的 developModeState
EXPECTED_STATE = {
    "normal": "NormalActive",
    "half_sdk": "HalfSdkActive",
    "full_sdk": "FullSdkActive",
}

MODE_DESC = {
    "normal_active": "内部算法控制全身，仅可调用 CC-API 技能（手臂指令被忽略）",
    "half_sdk_active": "SDK 控制上半身（头/臂/腰），腿部由系统托管（VLA 部署档位）",
    "full_sdk_active": "SDK 控制全身电机，内部平衡已退出（危险，需悬吊）",
    "sdk_error": "SDK 程序运行异常，需退出开发者模式或重启运控",
    "error": "运控/SDK 异常（真机实测值）：查 fault.current.get，必要时退出开发者模式或重启运控",
    "estop": "急停已触发，运控已停止",
    "wait_mc_start": "等待运控启动",
    "sdk_exiting": "SDK 正在退出（过渡态，稍候）",
    "sdk_starting": "SDK 正在启动（过渡态，稍候）",
    "sdk_entering": "SDK 正在进入（过渡态，稍候）",
}

STATUS_KEYS = ["developModeState", "workMode", "legStatus", "mainState",
               "isMotionUp", "eStop", "servoEStop"]

# 连接对象由 connect() 写入，供 Skill 调用使用
ubt_robot = None


# ── 小工具 ────────────────────────────────────────────────────────────────────


def to_snake(name) -> str:
    """HalfSdkActive -> half_sdk_active"""
    return re.sub(r"(?<!^)(?=[A-Z])", "_", str(name)).lower()


def norm(value) -> str:
    return str(value).strip().lower() if value is not None else ""


def as_bool(value):
    """把状态字段统一成 bool；无法判断时返回 None。

    真机实测 eStop/servoEStop 返回的是 "released" 字符串（按下时可能是
    "pressed"/"triggered"），因此不能只认 "true"/"false"。
    """
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "pressed", "triggered", "down", "engaged"):
        return True
    if text in ("0", "false", "no", "off", "released", "up", "normal", ""):
        return False
    return None


def unwrap(result):
    """兼容 rsp.result 直接是 dict、JSON 字符串或 {"xxx": {...}} 包装。"""
    if isinstance(result, str):
        try:
            result = json.loads(result)
        except json.JSONDecodeError:
            return {}
    if isinstance(result, dict):
        inner = result.get("result")
        if isinstance(inner, dict) and len(result) == 1:
            return inner
    return result if isinstance(result, dict) else {}


# ── 连接与状态 ────────────────────────────────────────────────────────────────


def connect(args) -> tuple:
    """返回 (api, error_message)。"""
    global ubt_robot
    if ubt_robot is None:
        try:
            import ubt_robot as _ubt_robot  # noqa: PLC0415 - 依赖可选，延迟导入
            ubt_robot = _ubt_robot
        except Exception as exc:  # pragma: no cover - 环境相关
            return None, (
                "未检测到 ubt_robot（CC API Python SDK）：%s\n"
                "  在机器人开发者容器内运行本脚本，或在当前环境安装 wheel：\n"
                "    pip3 install <C1_SDK>/cc_api_package/cc_api_client_python_lib/"
                "<amd64|arm64>/ubt_robot-*.whl\n"
                "  ubt_IL 容器内可执行：bash install_ccapi.sh" % exc
            )

    api = ubt_robot.Api()
    cfg = ubt_robot.Config()
    cfg.api_id = args.api_id
    address = args.address or os.environ.get("WALKER_C1_CC_ADDRESS", "")
    token = args.token or os.environ.get("WALKER_C1_CC_TOKEN", "")
    if address:
        cfg.address = address
    if token:
        cfg.token = token

    try:
        api.initialize(cfg)
        api.start()
    except Exception as exc:
        return None, f"CC API 初始化失败：{exc}"

    try:
        connected = api.wait_until_connected()
    except Exception as exc:
        api.stop()
        return None, f"连接 CC 服务异常：{exc}"

    if not connected:
        api.stop()
        return None, (
            "连接 CC 服务超时。请检查：\n"
            "  1) CC 服务是否启动（/tmp/cc_api.sock 是否存在，或 --address 是否正确）；\n"
            "  2) 是否与其它 SDK 程序使用相同 api_id（CC 客户端存在连接互斥）；\n"
            "  3) Astron 上部分接口需要 root 权限。"
        )
    return api, ""


def _status_request(api, keys):
    try:
        rsp = api.request_sync("cc.api.status.get", {"list": list(keys)})
    except Exception as exc:
        return None, str(exc)
    if not rsp.is_success:
        return None, f"{rsp.error_code} {rsp.error_message}"
    data = unwrap(rsp.result)
    if not data:
        return None, "status.get 返回空结果"
    return data, None


def get_status(api) -> dict:
    """取状态；某些固件不认识个别字段时退化为逐项查询。"""
    status, err = _status_request(api, STATUS_KEYS)
    if status is not None:
        return status

    merged = {}
    for key in STATUS_KEYS:
        part, _ = _status_request(api, [key])
        if isinstance(part, dict):
            merged.update(part)
    if merged:
        return merged
    raise RuntimeError(f"status.get 失败：{err}")


def print_status(status: dict) -> None:
    state = str(status.get("developModeState", "?"))
    desc = MODE_DESC.get(to_snake(state), "未知状态")
    print(f"  工作模式   : {status.get('workMode')}  （develop = 开发者模式）")
    print(f"  运控权限   : {state}  ({desc})")
    print(f"  腿部模式   : {status.get('legStatus')}")
    print(f"  主状态     : {status.get('mainState')}   运控上电: {status.get('isMotionUp')}")
    print(f"  急停       : eStop={status.get('eStop')} servoEStop={status.get('servoEStop')}")


def wait_for_state(api, expected: str, timeout: float, interval: float) -> tuple:
    """模式切换异步经过 SdkExiting/SdkStarting 等过渡态，轮询到目标或超时。"""
    deadline = time.time() + timeout
    status = get_status(api)
    while norm(status.get("developModeState")) != norm(expected) and time.time() < deadline:
        time.sleep(interval)
        status = get_status(api)
    return status, norm(status.get("developModeState")) == norm(expected)


def check_ready(status: dict) -> list:
    """返回阻止 VLA 部署的问题列表（空列表=可部署）。"""
    problems = []

    state = str(status.get("developModeState", ""))
    if state != EXPECTED_STATE["half_sdk"]:
        problems.append(f"developModeState={state or '?'}，需要 HalfSdkActive（half_sdk）")

    work_mode = norm(status.get("workMode"))
    if work_mode and work_mode != "develop":
        problems.append(f"workMode={status.get('workMode')}，需要 develop（开发者模式）")

    leg = norm(status.get("legStatus"))
    if leg and leg != "standing":
        problems.append(f"legStatus={status.get('legStatus')}，需要 standing")

    for key in ("eStop", "servoEStop"):
        if as_bool(status.get(key)) is True:
            problems.append(f"{key}=true，急停已触发")

    if as_bool(status.get("isMotionUp")) is False:
        problems.append("isMotionUp=false，运控未上电")
    return problems


# ── 动作 ──────────────────────────────────────────────────────────────────────


def ensure_develop(api, args) -> bool:
    status = get_status(api)
    work_mode = status.get("workMode")
    # 某些固件不返回 workMode：无法轮询确认，发完请求后继续，由 half_sdk 切换结果兜底
    work_mode_known = work_mode not in (None, "")
    if work_mode_known and norm(work_mode) == "develop":
        print("- 已处于开发者模式（workMode=develop）")
        return True

    print(f"- 工作模式 {work_mode} -> develop（机器人会语音播报）")
    try:
        rsp = api.request_sync("cc.api.work_mode.switch", {"workMode": "develop"})
    except Exception as exc:
        print(f"x 切换工作模式异常：{exc}")
        return False
    if not rsp.is_success:
        print(f"x 切换工作模式失败：{rsp.error_code} {rsp.error_message}")
        return False

    if not work_mode_known:
        print("- 状态不含 workMode，跳过轮询确认（后续 half_sdk 切换会暴露问题）")
        return True

    deadline = time.time() + args.timeout
    while time.time() < deadline:
        if norm(get_status(api).get("workMode")) == "develop":
            print("- 已进入开发者模式")
            return True
        time.sleep(args.interval)
    print(f"x {args.timeout:.0f}s 内未进入开发者模式，请检查急停/运控是否就绪")
    return False


def ensure_leg_stand(api, auto: bool, timeout: float, interval: float) -> bool:
    """half_sdk 前置条件：腿部需处于 standing（A000004 技能切 STAND）。"""
    status = get_status(api)
    leg = norm(status.get("legStatus"))
    if leg == "standing":
        print("- 腿部已处于 standing，满足 half_sdk 前置条件")
        return True
    if not auto:
        print(f"! 腿部当前为 {status.get('legStatus')}，half_sdk 需要 standing；"
              f"去掉 --no-leg-stand 可自动切换")
        return False

    print(f"- 腿部当前为 {status.get('legStatus')}，自动切换为 STAND ...")
    try:
        skill = ubt_robot.Skill(api)
        work = skill.launch("A000004", {"mode": "STAND"})
        if not work:
            print("x 技能 A000004 启动失败")
            return False
        result = work.wait_until_finish()
        ok = result.type == ubt_robot.WorkResultType.SUCCESS
    except Exception as exc:
        print(f"x 切换腿部模式异常：{exc}（也可用遥控器/Web 把腿部切到站立）")
        return False

    deadline = time.time() + timeout
    leg_now = norm(get_status(api).get("legStatus"))
    while leg_now != "standing" and time.time() < deadline:
        time.sleep(interval)
        leg_now = norm(get_status(api).get("legStatus"))
    print(f"- 腿部切换结果: {'成功' if ok else '失败'}，当前 legStatus={leg_now or '?'}")
    return ok and leg_now == "standing"


def request_switch_mc_mode(api, mode: str, args) -> bool:
    print(f"== 切换到 {mode} ==")
    try:
        rsp = api.request_sync("cc.api.sdk.switch_mc_mode", {"mode": mode})
    except Exception as exc:
        print(f"x 切换请求异常：{exc}")
        return False
    if not rsp.is_success:
        print(f"x 切换失败：{rsp.error_code} {rsp.error_message}")
        return False
    print(f"- 请求结果: {rsp.result}")

    expected = EXPECTED_STATE[mode]
    status, ok = wait_for_state(api, expected, args.timeout, args.interval)
    print("== 切换后状态 ==")
    print_status(status)
    print(f"- 目标状态 {expected}: {'已达成' if ok else '未达成，请检查运控'}")
    return ok


def request_work_mode(api, work_mode: str, args) -> bool:
    print(f"== 切换工作模式 -> {work_mode} ==")
    try:
        rsp = api.request_sync("cc.api.work_mode.switch", {"workMode": work_mode})
    except Exception as exc:
        print(f"x 切换工作模式异常：{exc}")
        return False
    if not rsp.is_success:
        print(f"x 切换工作模式失败：{rsp.error_code} {rsp.error_message}")
        return False
    print(f"- 请求结果: {rsp.result}")

    deadline = time.time() + args.timeout
    current = norm(get_status(api).get("workMode"))
    while current != norm(work_mode) and time.time() < deadline:
        time.sleep(args.interval)
        current = norm(get_status(api).get("workMode"))
    print_status(get_status(api))
    ok = current == norm(work_mode)
    print(f"- workMode={work_mode}: {'已达成' if ok else '未达成'}")
    return ok


def confirm_full_sdk(skip: bool) -> bool:
    if skip:
        return True
    print("! full_sdk 会让内部平衡算法退出，机器人将失去自平衡。")
    print("! 请确认机器人已悬吊或有人员扶持，且急停按钮在可及范围内。")
    if not sys.stdin.isatty():
        print("x 非交互环境，如需继续请显式加 --yes")
        return False
    try:
        answer = input("  确认切换请输入 full_sdk（其他任意输入取消）: ").strip()
    except EOFError:
        return False
    return answer == "full_sdk"


# ── CLI ───────────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Walker C1 开发者模式 / 运控权限（half_sdk）管理",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("command", nargs="?", default="status", choices=COMMANDS,
                        help="status | check | half_sdk | normal | develop | work-mode | full_sdk")
    parser.add_argument("name", nargs="?", default=None,
                        help="work-mode 的目标工作模式，如 joystick / develop")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出状态")
    parser.add_argument("--yes", action="store_true", help="full_sdk 跳过交互确认")
    parser.add_argument("--no-leg-stand", action="store_true",
                        help="切 half_sdk 时不自动把腿部切到 STAND，仅检查")
    parser.add_argument("--no-develop", action="store_true",
                        help="half_sdk 不自动进入开发者模式（已手动进入时用）")
    parser.add_argument("--timeout", type=float, default=20.0, help="状态轮询超时（秒，默认 20）")
    parser.add_argument("--interval", type=float, default=0.5, help="状态轮询间隔（秒，默认 0.5）")
    parser.add_argument("--api-id", default=os.environ.get("WALKER_C1_API_ID", "walker_il_sdk_mode"),
                        help="CC API 客户端 id（默认 walker_il_sdk_mode）")
    parser.add_argument("--address", default="", help="跨机连接地址，如 0.0.0.0:51000")
    parser.add_argument("--token", default="", help="跨机连接 token")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if os.geteuid() != 0 and not args.address:
        print("[warn] 当前非 root；Astron 上部分 CC 接口需要 root，若连接/切换失败请用 root 重试",
              file=sys.stderr)

    api, err = connect(args)
    if api is None:
        print(f"x {err}", file=sys.stderr)
        return EXIT_ERR

    try:
        status = get_status(api)

        if args.command == "status":
            if args.json:
                print(json.dumps(status, ensure_ascii=False, indent=2))
            else:
                print("== 当前状态 ==")
                print_status(status)
            return EXIT_OK

        if args.command == "check":
            problems = check_ready(status)
            if args.json:
                print(json.dumps({"ready": not problems, "problems": problems,
                                  "status": status}, ensure_ascii=False, indent=2))
            else:
                print("== 当前状态 ==")
                print_status(status)
                if problems:
                    print("== 不可部署 VLA ==")
                    for item in problems:
                        print(f"  x {item}")
                else:
                    print("== 可部署 VLA ==")
                    print("  OK: develop + half_sdk + standing，急停未触发")
            return EXIT_OK if not problems else EXIT_CHECK_FAIL

        if args.command == "develop":
            ok = ensure_develop(api, args)
            print_status(get_status(api))
            return EXIT_OK if ok else EXIT_ERR

        if args.command == "work-mode":
            if not args.name:
                print("x work-mode 需要目标模式，例如：work-mode joystick", file=sys.stderr)
                return EXIT_ERR
            ok = request_work_mode(api, args.name, args)
            return EXIT_OK if ok else EXIT_ERR

        if args.command == "half_sdk":
            print("== 准备 half_sdk（VLA 部署档位） ==")
            print_status(status)
            if not args.no_develop and not ensure_develop(api, args):
                print("x 进入开发者模式失败，已中止")
                return EXIT_ERR
            if not ensure_leg_stand(api, not args.no_leg_stand, args.timeout, args.interval):
                print("x half_sdk 前置条件不满足（腿部需 standing），已中止")
                return EXIT_ERR

            current = str(get_status(api).get("developModeState", ""))
            if current == EXPECTED_STATE["half_sdk"]:
                print("- 已处于 half_sdk，无需重复切换")
                print_status(get_status(api))
                return EXIT_OK
            ok = request_switch_mc_mode(api, "half_sdk", args)
            if not ok:
                return EXIT_ERR
            problems = check_ready(get_status(api))
            if problems:
                for item in problems:
                    print(f"  ! {item}")
                return EXIT_ERR
            print("== half_sdk 就绪：可以部署 VLA ==")
            return EXIT_OK

        if args.command == "normal":
            ok = request_switch_mc_mode(api, "normal", args)
            return EXIT_OK if ok else EXIT_ERR

        if args.command == "full_sdk":
            if not confirm_full_sdk(args.yes):
                print("- 已取消")
                return EXIT_ERR
            if not ensure_leg_stand(api, not args.no_leg_stand, args.timeout, args.interval):
                print("x 腿部未 standing，已中止")
                return EXIT_ERR
            ok = request_switch_mc_mode(api, "full_sdk", args)
            return EXIT_OK if ok else EXIT_ERR

        print(f"x 未知命令：{args.command}", file=sys.stderr)
        return EXIT_ERR
    except Exception as exc:
        print(f"x {exc}", file=sys.stderr)
        return EXIT_ERR
    finally:
        try:
            api.stop()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
