#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 统一控制器 CLI 入口。

子命令（各自保留独立 argparse，直接转发给实现）：

  state      关节/手/末端状态、单关节移动、预备位、回站立位
  joint      关节调试、手部开合、连续监视
  endpoint   末端位姿读取 / 笛卡尔 IK / EE 相对位移
  hand       灵巧手开合、指定角度、闭合度百分比、单指正弦
  home       分段回站立位 + 张开双手
  play       播放预备位 / 回站立位，定时抓拍全程
  analyze    关节阶跃/正弦响应分析 + CSV
  camera     相机 RGB/深度 信息、预览、存盘
  teleop     键盘末端遥操作

无子命令或首参是 flag 时默认走 ``state``，兼容旧式调用。
话题与具身天工 3.0 真机一致；仿真和真机只靠 ``ROS_DOMAIN_ID`` 区分
（默认 0（与真机同域），隔离时用 146），脚本不用改。

    export ROS_DOMAIN_ID=0            # 默认与真机同域
    python3 tienkung_dex_controller.py --print-state
    python3 tienkung_dex_controller.py state --print-hand
    python3 tienkung_dex_controller.py endpoint --side right --delta-xyz 0.05 0 0
    python3 tienkung_dex_controller.py hand --side right --close
    python3 tienkung_dex_controller.py home
    python3 tienkung_dex_controller.py camera --preview
    python3 tienkung_dex_controller.py --help

类实现见 ``dex_controller.py``（``DexController``，别名
``TienkungDexController``）；上层脚本应 ``from dex_controller import DexController``。
"""

import importlib
import os
import sys

# 用脚本真实路径（解析符号链接）定位同目录模块与 utils 包
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

# (module, entry_func, 简介, 常用参数提示)
_SUBCOMMANDS = {
    "state": ("dex_controller", "main",
              "状态、单关节移动、预备位、回站立位",
              "--print-state | --print-hand | --print-ee | --init | --home"),
    "joint": ("dex_controller", "main_joint",
              "关节调试 / 手部开合 / 连续监视",
              "--print | --move JOINT=ANGLE | --shift JOINT=DELTA | --monitor"),
    "endpoint": ("dex_controller", "main_endpoint",
                 "末端位姿 / 笛卡尔 IK / EE 相对位移",
                 "--side right --no-move | --delta-xyz 0.05 0 0 | --local | --world"),
    "hand": ("dex_controller", "main_hand",
             "灵巧手开合 / 指定角度 / 闭合度 / 单指正弦",
             "--side right --close | --open | --percent 0.5 | --pose JOINT=RAD"),
    "home": ("dex_controller", "main_home",
             "分段回站立位 + 张开双手",
             "--duration 10 | --no-staged | --keep-hands"),
    "play": ("utils.pose_play", "main",
             "播放预备位 / 回站立位（可重复、可抓拍）",
             "ready | home [--from-home] [--rounds 2] [--snap-interval 0.4]"),
    "analyze": ("utils.joint_analyzer", "main",
                "关节阶跃/正弦响应分析 + CSV",
                "--joint <名> --step <rad> | --sine | --listen | --csv <路径>"),
    "camera": ("utils.camera", "main",
               "相机 RGB/深度 信息、预览、存盘",
               "--info | --preview | --save --count 5"),
    "teleop": ("keyboard_ee_control", "main",
               "键盘末端遥操作（需 X11）",
               "--side right | --translation-step 0.012 | --input-hz 1"),
}

_EXAMPLES = [
    ("state --print-state",                        "查看 31 个身体关节角度"),
    ("state --print-hand",                         "查看双手"),
    ("state --init",                               "移动到预备抓取位（READY_POSE + 预备位手型：拇指侧摆内收 850 = 90°）"),
    ("joint --move elbow_pitch_r_joint=-0.5",      "单关节绝对移动"),
    ("endpoint --side right --no-move",            "读取右臂末端位姿（自动初始化 IK）"),
    ("endpoint --side right --delta-xyz 0.05 0 0", "末端前移 5 cm（局部系加 --local）"),
    ("hand --side right --close",                  "右手握拳"),
    ("home",                                       "分段回站立位 + 张开双手"),
    ("play ready",                                 "直接从当前位置播一遍预备动作并抓拍（/tmp/ready_seq）"),
    ("play home --rounds 2",                       "回站立位，来两遍（中间自动回预备位）"),
    ("analyze --joint elbow_pitch_r_joint --step -0.6 --duration 5 --csv /tmp/e.csv", "阶跃响应辨识"),
    ("camera --preview",                           "相机预览（q 退出，s 存盘）"),
    ("teleop",                                     "键盘末端遥操作"),
    ("--print-state",                              "旧式调用：默认走 state"),
]


def _print_help():
    print("TienKung Dex 统一控制器 — 子命令清单\n")
    print("用法: python3 tienkung_dex_controller.py [子命令] [选项]")
    print("      无子命令或首参为 flag 时默认走 state")
    print("      子命令详细参数: python3 tienkung_dex_controller.py <子命令> --help")
    print("      通道: 默认 ROS_DOMAIN_ID=0（与真机同域，话题完全相同）\n")
    print("子命令:")
    for name, (_, _, desc, egs) in _SUBCOMMANDS.items():
        print(f"  {name:<10} {desc}")
        if egs:
            print(f"  {'':<10} 常用: {egs}")
    print("\n示例:")
    for cmd, comment in _EXAMPLES:
        print(f"  python3 tienkung_dex_controller.py {cmd:<58} # {comment}")


def main():
    argv = sys.argv[1:]

    if argv and argv[0] in ("-h", "--help", "help"):
        _print_help()
        return

    if argv and not argv[0].startswith("-") and argv[0] in _SUBCOMMANDS:
        sub, rest = argv[0], argv[1:]
    elif not argv or argv[0].startswith("-"):
        sub, rest = "state", argv
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
