#!/usr/bin/env python3
"""Walker S2 统一控制器 CLI 入口（合并原 6 个 controller 家族脚本）。

子命令（各子命令保留原有 argparse，直接转发，行为与原脚本一致）：

  state      关节/夹爪/末端状态、单关节移动、预备姿态、夹爪控制（原 walker_s2_controller.py）
  joint      关节/手部调试（原 walker_s2_joint_test.py）
  endpoint   末端/TCP 位姿测试（原 walker_s2_endpoint_pose_test.py）
  home       分段回 home 全零位 + 张开双手（原 walker_s2_reset.py）
  analyze    关节阶跃/正弦响应分析 + CSV（原 joint_analysis.py）
  camera     相机话题信息/预览/保存（原 walker_s2_camera.py，现 utils/camera.py）

无子命令或首参为 flag 时默认走 ``state``，兼容旧调用；``--help``/``help`` 查看子命令清单，
各子命令自己的 ``--help``（如 ``joint --help``）由转发后的 argparse 提供：

    python3 walker_s2_controller.py --print-state
    python3 walker_s2_controller.py joint --print
    python3 walker_s2_controller.py home
    python3 walker_s2_controller.py --help

类实现见 ``utils.controller``（``WalkerS2Controller`` / ``RobotController``）；
消费方（carry_box / pick_part 等）应 ``from utils.controller import ...``。
"""

import importlib
import os
import sys

# 以脚本真实路径（解析符号链接）定位 utils 包。注意："utils" 是通用包名，
# insert 到 0 位会遮蔽环境中同名包——本仓库的 walker_s2/utils 即唯一 utils，属有意为之。
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

# (module, entry_func, 简介, 常用参数提示) — 同一模块可有多个入口函数
_SUBCOMMANDS = {
    "state":    ("utils.controller", "main",          "关节/夹爪/末端状态、单关节移动、预备姿态、夹爪控制",
                 "--print-state | --init | --grip-open | --move-joint --joint <名> --pos <rad>"),
    "joint":    ("utils.controller", "main_joint",    "关节/手部调试",
                 "--print | --move JOINT=ANGLE | --hand-open | --monitor"),
    "endpoint": ("utils.controller", "main_endpoint", "末端/TCP 位姿测试",
                 "--no-move | --side left"),
    "home":     ("utils.controller", "main_home",     "分段回 home 全零位 + 张开双手",
                 ""),
    "analyze":  ("utils.joint_analyzer", "main",      "关节阶跃/正弦响应分析 + CSV",
                 "--joint <名> --step <rad> | --sine | --listen | --csv <路径>"),
    "camera":   ("utils.camera", "main",              "相机话题信息/预览/保存",
                 "--preview | --save --count 5 | --topic <话题>"),
}

# (命令, 说明) — 帮助末尾的使用示例
_EXAMPLES = [
    ("state --print-state",                 "查看关节/夹爪/末端状态"),
    ("state --init",                        "分段移动到预备姿态"),
    ("joint --move R_elbow_yaw_joint=0.5",  "移动单关节到 0.5 rad（回车确认）"),
    ("camera --preview",                    "相机实时预览（按 Q/ESC 退出）"),
    ("camera --save --count 5",             "保存 5 帧相机图像为 PNG"),
    ("home",                                "分段回 home 全零位 + 张开双手"),
    ("--print-state",                       "旧式调用：无子命令默认走 state"),
]


def _print_help():
    """打印分发层帮助（子命令清单）。子命令自身的 --help 仍由各自 argparse 提供。"""
    print("Walker S2 统一控制器 — 子命令清单\n")
    print("用法: python3 walker_s2_controller.py [子命令] [选项]")
    print("      无子命令或首参为 flag 时默认走 state，兼容旧调用")
    print("      子命令详细参数: python3 walker_s2_controller.py <子命令> --help\n")
    print("子命令:")
    for name, (_, _, desc, egs) in _SUBCOMMANDS.items():
        print(f"  {name:<10} {desc}")
        if egs:
            print(f"  {'':<10} 常用: {egs}")
    print("\n示例:")
    for cmd, comment in _EXAMPLES:
        print(f"  python3 walker_s2_controller.py {cmd:<38} # {comment}")


def main():
    argv = sys.argv[1:]

    # 分发层帮助（-h/--help/help）；子命令自己的 --help 由转发后的 argparse 处理
    if argv and argv[0] in ("-h", "--help", "help"):
        _print_help()
        return

    if argv and not argv[0].startswith("-") and argv[0] in _SUBCOMMANDS:
        sub, rest = argv[0], argv[1:]
    elif not argv or argv[0].startswith("-"):
        sub, rest = "state", argv  # 默认 state，兼容旧 --print-state 等无子命令调用
    else:
        valid = ", ".join(_SUBCOMMANDS)
        print(f"未知子命令 '{argv[0]}'，可用: {valid}（--help 查看子命令清单）", file=sys.stderr)
        sys.exit(2)

    mod_name, func_name, _, _ = _SUBCOMMANDS[sub]
    module = importlib.import_module(mod_name)
    entry = getattr(module, func_name, None)
    if entry is None:
        print(f"[BUG] 入口缺失: {mod_name}.{func_name}", file=sys.stderr)
        sys.exit(1)
    entry(rest)


if __name__ == "__main__":
    main()
