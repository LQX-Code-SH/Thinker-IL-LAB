#!/usr/bin/env python3
"""Walker S2 推理服务客户端。

通过 ZMQ 与 inference_server 交互：start/stop/home/status/shutdown/load 指令，以及
launch（远程拉起服务进程并指定模型参数）和 --watch（订阅状态流）。

子命令：
    launch   拉起 inference_server.sh（本地或 --remote 经 SSH+docker-exec），预热指定模型，
             轮询 status 至 READY。
    load     运行中热切换模型（整体重建，含机器人重连）。
    start    开始推理（engine reset+resume）。
    stop     结束推理（engine pause，hold 当前位姿）。
    home     回 home_position（canonical，先 pause 再 _send_home_chunk，避双源）。
    status   查询当前模型状态 + telemetry。
    shutdown 关服务（回 home + kill 桥接 + 退出）。
    watch    订阅状态 PUB 流（Ctrl-C 退）。

用法示例：
    # 远程拉起（SSH+docker-exec 到跑容器的 Jetson，预热 walker_s2_10d 模型）
    # --host = 跑 lerobot-tienkung 容器的机器（本部署=Jetson 192.168.11.3），
    #          不是机器人主控 PC(192.168.11.2)。--ssh-user 默认 ubt，Jetson 用 walker。
    python inference_client.py launch --policy-path /ubt_IL/model/.../pretrained_model \\
        --robot-model walker_s2_10d --inference-hz 2 --fps 15 \\
        --remote --host 192.168.11.3 --ssh-user walker

    # 本地拉起
    python inference_client.py launch --policy-path ... --robot-model walker_s2_10d

    # ZMQ 指令（host 默认 127.0.0.1）
    python inference_client.py status
    python inference_client.py start
    python inference_client.py stop
    python inference_client.py home
    python inference_client.py load --policy-path /ubt_IL/model/<other>/.../pretrained_model
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
SCRIPT_PATH = "/ubt_IL/scripts/deploy/walker_s2/inference_server.sh"
LAUNCH_POLL_TIMEOUT_S = 180  # 预热（policy 加载+连机器人）最长等待


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
                  f"chunk_id={msg.get('chunk_id')} robot={msg.get('robot_connected')} "
                  f"uptime={msg.get('uptime_s')}s", flush=True)
    except KeyboardInterrupt:
        print("\n[watch] stopped")
    finally:
        sock.close()


# ── 参数 <-> env/dict 映射 ────────────────────────────────────────────────────

# (cli attr, env var for launch, dict key for load)
_PARAM_MAP = [
    ("policy_path", "POLICY_PATH", "policy_path"),
    ("robot_model", "ROBOT_MODEL", "robot_model"),
    ("robot_config", "ROBOT_CONFIG", "robot_config"),
    ("inference_type", "INFERENCE_TYPE", "inference_type"),
    ("inference_hz", "INFERENCE_HZ", "inference_hz"),
    ("execution_horizon", "EXECUTION_HORIZON", "execution_horizon"),
    ("fps", "FPS", "fps"),
    ("task", "TASK", "task"),
    ("blend_horizon", "BLEND_HORIZON", "blend_horizon"),
    ("body_publish_hz", "BODY_PUBLISH_HZ", "body_publish_hz"),
    ("body_v_max", "BODY_V_MAX", "body_v_max"),
]


def _params_to_env(args: argparse.Namespace) -> dict[str, str]:
    """launch 用：把 params 转成 inference_server.sh 识别的环境变量。"""
    env: dict[str, str] = {}
    for attr, env_key, _ in _PARAM_MAP:
        val = getattr(args, attr, None)
        if val is not None:
            env[env_key] = str(val)
    if getattr(args, "cmd_port", None):
        env["SERVER_CMD_PORT"] = str(args.cmd_port)
    if getattr(args, "status_port", None):
        env["SERVER_STATUS_PORT"] = str(args.status_port)
    return env


def _params_to_dict(args: argparse.Namespace) -> dict[str, Any]:
    """load 用：把 params 转成服务端 load 指令的 params dict。"""
    params: dict[str, Any] = {}
    for attr, _, dict_key in _PARAM_MAP:
        val = getattr(args, attr, None)
        if val is not None:
            params[dict_key] = val
    return params


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
            print(f"  state={state} {resp.get('model', '')} {'err=' + resp.get('load_error', '') if resp.get('load_error') else ''}")
            last_state = state
        if state == "READY":
            print(f"[OK] 服务就绪（READY）。模型={resp.get('model')} policy={resp.get('policy_path')}")
            print(f"     start: python inference_client.py start  (或 bash inference_client.sh start)")
            return 0
        if state == "ERROR":
            print(f"[ERROR] 服务进入 ERROR 态：{resp.get('load_error')}", file=sys.stderr)
            return 1
        time.sleep(0.5)
    print("[ERROR] 等待 READY 超时。检查 /tmp/walker_inference_server.log。", file=sys.stderr)
    return 1


# ── 子命令分发 ────────────────────────────────────────────────────────────────


def _print_resp(resp: dict[str, Any]) -> int:
    print(json.dumps(resp, indent=2, ensure_ascii=False))
    return 0 if resp.get("ok") else 1


def cmd_load(args: argparse.Namespace) -> int:
    if not args.policy_path:
        print("[ERROR] load 需要 --policy-path", file=sys.stderr)
        return 1
    params = _params_to_dict(args)
    resp = send_cmd(args.host, args.cmd_port, "load", params=params, timeout_ms=8000)
    return _print_resp(resp)


def cmd_simple(args: argparse.Namespace) -> int:
    resp = send_cmd(args.host, args.cmd_port, args.command, timeout_ms=8000)
    return _print_resp(resp)


def cmd_watch(args: argparse.Namespace) -> int:
    watch_status(args.host, args.status_port)
    return 0


# ── argparse ──────────────────────────────────────────────────────────────────


def _add_param_args(p: argparse.ArgumentParser) -> None:
    """launch / load 共用的模型参数。"""
    p.add_argument("--policy-path", dest="policy_path", default=None,
                   help="pretrained_model 目录路径（launch/load 必填）")
    p.add_argument("--robot-model", dest="robot_model", default=None,
                   help="ROBOT_MODELS 注册表键，如 walker_s2_10d")
    p.add_argument("--robot-config", dest="robot_config", default=None,
                   help="自定义 robot config JSON 路径（可选，覆盖 ROBOT_MODEL）")
    p.add_argument("--inference-type", dest="inference_type", default=None,
                   help="act_async（默认）| sync")
    p.add_argument("--inference-hz", dest="inference_hz", type=float, default=None,
                   help="act_async 重规划频率")
    p.add_argument("--execution-horizon", dest="execution_horizon", type=int, default=None,
                   help="act_async chunk 截断步数（0=不截断）")
    p.add_argument("--fps", type=int, default=None, help="控制环 FPS")
    p.add_argument("--task", default=None, help="任务描述字符串")
    p.add_argument("--blend-horizon", dest="blend_horizon", type=int, default=None,
                   help="桥接动作块融合重叠点数")
    p.add_argument("--body-publish-hz", dest="body_publish_hz", type=float, default=None,
                   help="桥接 300Hz body 发布频率")
    p.add_argument("--body-v-max", dest="body_v_max", type=float, default=None,
                   help="桥接 rate_limit 单关节最大速度 rad/s")


def _add_zmq_args(p: argparse.ArgumentParser, with_status: bool = False) -> None:
    p.add_argument("--host", default=DEFAULT_HOST, help=f"服务器主机（默认 {DEFAULT_HOST}）")
    p.add_argument("--cmd-port", dest="cmd_port", type=int, default=DEFAULT_CMD_PORT,
                   help=f"REP 指令端口（默认 {DEFAULT_CMD_PORT}）")
    if with_status:
        p.add_argument("--status-port", dest="status_port", type=int, default=DEFAULT_STATUS_PORT,
                       help=f"PUB 状态端口（默认 {DEFAULT_STATUS_PORT}）")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Walker S2 推理服务客户端（ZMQ 指令 + launch 拉起）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # launch
    p = sub.add_parser("launch", help="拉起 inference_server 并预热指定模型")
    _add_param_args(p)
    _add_zmq_args(p, with_status=True)
    p.add_argument("--remote", action="store_true",
                   help="经 SSH+docker-exec 在远端 Jetson 容器内拉起")
    p.add_argument("--ssh-user", dest="ssh_user", default=DEFAULT_SSH_USER,
                   help=f"SSH 用户（默认 {DEFAULT_SSH_USER}）")
    p.add_argument("--ssh-port", dest="ssh_port", default=DEFAULT_SSH_PORT,
                   help=f"SSH 端口（默认 {DEFAULT_SSH_PORT}）")
    p.add_argument("--container", default=DEFAULT_CONTAINER,
                   help=f"docker 容器名（默认 {DEFAULT_CONTAINER}）")
    p.set_defaults(func=cmd_launch)

    # load
    p = sub.add_parser("load", help="运行中热切换模型（整体重建含重连）")
    _add_param_args(p)
    _add_zmq_args(p)
    p.set_defaults(func=cmd_load)

    # simple commands
    for cmd in ("start", "stop", "home", "status", "shutdown"):
        p = sub.add_parser(cmd, help=f"{cmd} 指令")
        _add_zmq_args(p)
        p.set_defaults(func=cmd_simple, command=cmd)

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
