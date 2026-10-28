#!/usr/bin/env python3
"""TienKung Dex 推理服务客户端。

通过 ZMQ 与 inference_server 交互：start/stop/home/status/shutdown 指令，以及
launch（拉起服务进程并指定模型参数）和 watch（订阅状态流）。

子命令：
    launch   拉起 inference_server.sh（本地或 --remote 经 SSH+docker-exec），预热指定模型，
             轮询 status 至 READY。
    start    开始推理。
    stop     结束推理（hold 当前位姿）。
    home     回 home_position（右臂 7 维站立位，不含手指）。
    status   查询当前服务状态 + telemetry。
    shutdown 关服务（kill 桥接 + 退出；**不回 home**，要归位先单独发 home——
             插件默认 return_home_on_disconnect=False，与 tienkung_pro 一致）。
    watch    订阅状态 PUB 流（Ctrl-C 退）。

**没有 load（热切换模型）**：服务端重建机器人时写死 `--robot.type=walker`，且不带
`--robot.cameras`，对 Dex 会重建出一个错误的机器人（无相机）。换模型请 shutdown + launch。

用法示例：
    # 本地拉起（仿真）
    python inference_client.py launch --policy-path /ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model

    # 远程拉起（SSH 到跑容器的机器，再 docker exec 进去）
    python inference_client.py launch --policy-path /ubt_IL/model/.../pretrained_model \\
        --remote --host 192.168.41.2 --ssh-user nvidia

    # ZMQ 指令（host 默认 127.0.0.1）
    python inference_client.py status
    python inference_client.py start
    python inference_client.py stop
    python inference_client.py home
    python inference_client.py shutdown
    python inference_client.py watch
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from typing import Any

try:
    import zmq
except ImportError:
    zmq = None  # launch 子命令不需要 zmq；ZMQ 指令时会报错


# ── 默认值 ────────────────────────────────────────────────────────────────────

DEFAULT_HOST = os.environ.get("SERVER_HOST", "127.0.0.1")
DEFAULT_CMD_PORT = int(os.environ.get("SERVER_CMD_PORT", "5570"))
DEFAULT_STATUS_PORT = int(os.environ.get("SERVER_STATUS_PORT", "5571"))
DEFAULT_CONTAINER = os.environ.get("DEPLOY_CONTAINER", "lerobot-tienkung")
DEFAULT_SSH_USER = os.environ.get("DEPLOY_SSH_USER", os.environ.get("USER", "ubt"))
DEFAULT_SSH_PORT = os.environ.get("DEPLOY_SSH_PORT", "22")
SCRIPT_PATH = "/ubt_IL/scripts/deploy/tienkung_dex/inference_server.sh"
LOG_FILE = "/tmp/tienkung_dex_inference_server.log"
LAUNCH_POLL_TIMEOUT_S = 180  # 预热（policy 加载 + 连机器人）最长等待


# ── ZMQ 通信 ──────────────────────────────────────────────────────────────────


def _require_zmq() -> None:
    if zmq is None:
        print("[ERROR] pyzmq 未安装。ZMQ 指令需要在客户端环境安装 pyzmq（pip install pyzmq）。",
              file=sys.stderr)
        sys.exit(1)


def send_cmd(host: str, port: int, cmd: str, params: dict | None = None,
             timeout_ms: int = 8000) -> dict[str, Any]:
    """发一条 ZMQ REP/REQ 指令，返回响应 dict。"""
    _require_zmq()
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{host}:{port}")
    req: dict[str, Any] = {"cmd": cmd}
    if params:
        req["params"] = params
    try:
        sock.send_json(req, flags=zmq.NOBLOCK)
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        if not poller.poll(timeout=timeout_ms):
            return {"ok": False, "error": f"timeout ({timeout_ms}ms) connecting to {host}:{port}"}
        return sock.recv_json()
    except Exception as e:
        return {"ok": False, "error": str(e)}
    finally:
        sock.close()


def watch_status(host: str, port: int) -> None:
    """订阅状态 PUB 流，持续打印。"""
    _require_zmq()
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{host}:{port}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    print(f"[watch] 订阅 {host}:{port} 状态流（Ctrl-C 退出）...")
    try:
        while True:
            msg = sock.recv_json()
            print(f"[{time.strftime('%H:%M:%S')}] state={msg.get('state')} "
                  f"model={msg.get('model')} loop_hz={msg.get('loop_hz')} "
                  f"robot={msg.get('robot_connected')} uptime={msg.get('uptime_s')}s",
                  flush=True)
    except KeyboardInterrupt:
        print("\n[watch] stopped")
    finally:
        sock.close()


# ── 参数 <-> env 映射 ─────────────────────────────────────────────────────────


# (cli attr, env var)
_PARAM_MAP = [
    ("policy_path", "POLICY_PATH"),
    ("joint_config", "JOINT_CONFIG"),
    ("hand_variant", "HAND_VARIANT"),
    ("fps", "FPS"),
    ("task", "TASK"),
    ("camera_port", "CAMERA_PORT"),
    ("image_enabled", "IMAGE_ENABLED"),
    ("camera_display", "CAMERA_DISPLAY"),
    ("max_relative_target", "MAX_RELATIVE_TARGET"),
]


def _params_to_env(args: argparse.Namespace) -> dict[str, str]:
    """launch 用：把 params 转成 inference_server.sh 识别的环境变量。"""
    env: dict[str, str] = {}
    for attr, env_key in _PARAM_MAP:
        val = getattr(args, attr, None)
        if val is not None:
            env[env_key] = str(val)
    if getattr(args, "cmd_port", None):
        env["SERVER_CMD_PORT"] = str(args.cmd_port)
    if getattr(args, "status_port", None):
        env["SERVER_STATUS_PORT"] = str(args.status_port)
    return env


# ── launch（拉起服务进程）─────────────────────────────────────────────────────


def _launch_local(env: dict[str, str]) -> None:
    env_full = os.environ.copy()
    env_full.update(env)
    print(f"[launch] 本地拉起: bash {SCRIPT_PATH}")
    subprocess.run(["bash", SCRIPT_PATH], env=env_full, check=False)


def _launch_remote(env: dict[str, str], host: str, ssh_user: str, ssh_port: str,
                   container: str) -> None:
    """SSH 到 host，sudo docker exec container 跑 inference_server.sh。"""
    env_assignments = " ".join(f"{k}={shlex.quote(v)}" for k, v in env.items())
    inner = f"cd /ubt_IL && {env_assignments} bash {SCRIPT_PATH}"
    docker_cmd = ["sudo", "docker", "exec", container, "bash", "-lc", inner]
    remote_cmd = " ".join(shlex.quote(c) for c in docker_cmd)
    ssh_target = f"{ssh_user}@{host}"
    print(f"[launch] 远程拉起: ssh -p {ssh_port} {ssh_target} -> {remote_cmd}")
    subprocess.run(["ssh", "-p", str(ssh_port), ssh_target, remote_cmd], check=False)


def cmd_launch(args: argparse.Namespace) -> int:
    if not args.policy_path:
        print("[ERROR] launch 需要 --policy-path", file=sys.stderr)
        return 1
    env = _params_to_env(args)
    if args.remote:
        _launch_remote(env, args.host, args.ssh_user, args.ssh_port, args.container)
    else:
        _launch_local(env)
    # 轮询 status 至 READY / ERROR
    print(f"[launch] 轮询状态 {args.host}:{args.cmd_port} 至 READY（超时 {LAUNCH_POLL_TIMEOUT_S}s）...")
    deadline = time.time() + LAUNCH_POLL_TIMEOUT_S
    last_state = None
    while time.time() < deadline:
        resp = send_cmd(args.host, args.cmd_port, "status", timeout_ms=3000)
        state = resp.get("state", "?")
        if state != last_state:
            print(f"  state={state} {resp.get('model', '')} "
                  f"{'err=' + resp.get('load_error', '') if resp.get('load_error') else ''}")
            last_state = state
        if state == "READY":
            print(f"[OK] 服务就绪（READY）。模型={resp.get('model')} policy={resp.get('policy_path')}")
            print("     start: bash inference_client.sh start")
            return 0
        if state == "ERROR":
            print(f"[ERROR] 服务进入 ERROR 态：{resp.get('load_error')}", file=sys.stderr)
            return 1
        time.sleep(0.5)
    print(f"[ERROR] 等待 READY 超时。检查 {LOG_FILE}。", file=sys.stderr)
    return 1


# ── 子命令分发 ────────────────────────────────────────────────────────────────


def _print_resp(resp: dict[str, Any]) -> int:
    print(json.dumps(resp, indent=2, ensure_ascii=False))
    return 0 if resp.get("ok") else 1


def cmd_simple(args: argparse.Namespace) -> int:
    resp = send_cmd(args.host, args.cmd_port, args.command, timeout_ms=8000)
    return _print_resp(resp)


def cmd_load(args: argparse.Namespace) -> int:
    """Dex 不支持热切换（服务端重建路径写死了 walker 的 robot.type 且不带 cameras）。"""
    print(
        "[ERROR] tienkung_dex 不支持 load（运行中热切换模型）。\n"
        "        服务端重建机器人时写死 --robot.type=walker 且不带 --robot.cameras，\n"
        "        对 Dex 会重建出错误的机器人（无相机）。\n"
        "        换模型请：\n"
        "          bash inference_client.sh shutdown\n"
        f"          bash inference_client.sh launch --policy-path <新模型>",
        file=sys.stderr,
    )
    return 1


def cmd_watch(args: argparse.Namespace) -> int:
    watch_status(args.host, args.status_port)
    return 0


# ── argparse ──────────────────────────────────────────────────────────────────


def _add_param_args(p: argparse.ArgumentParser) -> None:
    """launch 的模型参数（对应 inference_server.sh 的环境变量）。"""
    p.add_argument("--policy-path", dest="policy_path", default=None,
                   help="pretrained_model 目录路径（launch 必填）")
    p.add_argument("--joint-config", dest="joint_config", default=None,
                   help="关节配置：tienkung_dex_20 | tienkung_dex_brainco_13")
    p.add_argument("--hand-variant", dest="hand_variant", default=None,
                   help="手型 inspire|brainco（留空跟随 joint-config）")
    p.add_argument("--fps", type=int, default=None, help="控制环 FPS（默认 15）")
    p.add_argument("--task", default=None, help="任务描述字符串")
    p.add_argument("--camera-port", dest="camera_port", type=int, default=None,
                   help="相机 ZMQ 端口（仿真 5758 / 真机 5558）")
    p.add_argument("--image-enabled", dest="image_enabled", default=None,
                   help="1=由 Bridge2 把 ROS Image 转 JPEG 发 5558（真机）")
    p.add_argument("--camera-display", dest="camera_display", default=None,
                   help="true/false 是否弹预览窗（服务默认 false）")
    p.add_argument("--max-relative-target", dest="max_relative_target", default=None,
                   help="单步相对目标限幅 rad（真机首轮建议 0.01；留空=不限）")


def _add_zmq_args(p: argparse.ArgumentParser, with_status: bool = False) -> None:
    p.add_argument("--host", default=DEFAULT_HOST, help=f"服务器主机（默认 {DEFAULT_HOST}）")
    p.add_argument("--cmd-port", dest="cmd_port", type=int, default=DEFAULT_CMD_PORT,
                   help=f"REP 指令端口（默认 {DEFAULT_CMD_PORT}）")
    if with_status:
        p.add_argument("--status-port", dest="status_port", type=int, default=DEFAULT_STATUS_PORT,
                       help=f"PUB 状态端口（默认 {DEFAULT_STATUS_PORT}）")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="TienKung Dex 推理服务客户端（ZMQ 指令 + launch 拉起）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # launch
    p = sub.add_parser("launch", help="拉起 inference_server 并预热指定模型")
    _add_param_args(p)
    _add_zmq_args(p, with_status=True)
    p.add_argument("--remote", action="store_true",
                   help="经 SSH+docker-exec 在远端容器内拉起")
    p.add_argument("--ssh-user", dest="ssh_user", default=DEFAULT_SSH_USER,
                   help=f"SSH 用户（默认 {DEFAULT_SSH_USER}）")
    p.add_argument("--ssh-port", dest="ssh_port", default=DEFAULT_SSH_PORT,
                   help=f"SSH 端口（默认 {DEFAULT_SSH_PORT}）")
    p.add_argument("--container", default=DEFAULT_CONTAINER,
                   help=f"docker 容器名（默认 {DEFAULT_CONTAINER}）")
    p.set_defaults(func=cmd_launch)

    # 简单指令
    for cmd in ("start", "stop", "home", "status", "shutdown"):
        p = sub.add_parser(cmd, help=f"{cmd} 指令")
        _add_zmq_args(p)
        p.set_defaults(func=cmd_simple, command=cmd)

    # load：占位，明确报"不支持"（见 cmd_load）
    p = sub.add_parser("load", help="Dex 不支持热切换，会报错提示改用 shutdown+launch")
    _add_param_args(p)
    _add_zmq_args(p)
    p.set_defaults(func=cmd_load)

    # watch
    p = sub.add_parser("watch", help="订阅状态 PUB 流")
    _add_zmq_args(p, with_status=True)
    p.set_defaults(func=cmd_watch)

    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
