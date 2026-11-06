#!/bin/bash
# Walker C1 SDK 模式（开发者模式 / half_sdk）切换包装脚本。
#
# 背景：C1 手臂关节只在下述条件同时成立时接收 ROS2 指令
#   workMode=develop（开发者模式） + developModeState=HalfSdkActive（half_sdk）
# 因此 VLA 部署前必须切到 half_sdk；本脚本负责切换/校验，rollout.sh 等通过
# sdk_guard.sh 强制门禁。
#
# 传输方式（默认 auto）
#   本地：当前环境能 import ubt_robot 且能访问 /tmp/cc_api.sock 时直接执行；
#   远程：否则 SSH 到机器人开发者容器（默认 ubt@192.168.11.3:2222）执行，
#         脚本内容经 stdin 送入远端 python3，无需拷贝文件、无需改动远端环境。
#
# 用法
#   bash sdk_mode.sh status                  # 查看工作模式 / 运控权限 / 腿部 / 急停
#   bash sdk_mode.sh check                   # 校验可部署性（0=可部署，2=不满足）
#   bash sdk_mode.sh half_sdk                # 进入开发者模式 -> 腿部 STAND -> half_sdk
#   bash sdk_mode.sh normal                  # 回到 normal（保留开发者模式）
#   bash sdk_mode.sh develop                 # 只切工作模式到 develop
#   bash sdk_mode.sh work-mode joystick      # 退出开发者模式等
#
# 密码
#   默认交互输入（ssh 会提示 Password:）。非交互场景（CI/脚本）可：
#     export WALKER_C1_SDK_PASSWORD='<密码>'     # 有 sshpass 用 sshpass，否则自动用 SSH_ASKPASS
#   或写入密码文件（推荐，免 export/免交互）：
#     printf '%s\n' '<密码>' > ~/.walker_c1_sdk_password && chmod 600 ~/.walker_c1_sdk_password
#
# 可选环境变量
#   WALKER_C1_SDK_HOST       SSH 主机（默认 192.168.11.3）
#   WALKER_C1_SDK_PORT       SSH 端口（默认 2222）
#   WALKER_C1_SDK_USER       SSH 用户（默认 ubt）
#   WALKER_C1_SDK_PASSWORD   SSH 密码（优先于密码文件；不设则读密码文件，再不行交互输入）
#   WALKER_C1_SDK_PASSWORD_FILE  密码文件路径（默认 ~/.walker_c1_sdk_password）
#   WALKER_C1_SDK_PYTHON     远端/本地解释器（默认 python3）
#   WALKER_C1_SDK_SSH_OPTS   追加 ssh 参数（空格分隔）
#   WALKER_C1_SDK_CONTAINER  非空：SSH 到宿主机后 sudo -n docker exec 进该容器
#   WALKER_C1_API_ID / WALKER_C1_CC_ADDRESS / WALKER_C1_CC_TOKEN  透传给 sdk_mode.py
#
# 示例
#   bash sdk_mode.sh half_sdk
#   WALKER_C1_SDK_PASSWORD='***' bash sdk_mode.sh status
#   bash sdk_mode.sh --local status                      # 就在 CC 容器内执行
#   bash sdk_mode.sh --ssh-host 192.168.11.3 --ssh-user ubt --ssh-port 2222 check
#   bash sdk_mode.sh --ssh-container ubt_ros2_demo half_sdk   # ssh 落到宿主机时
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_SCRIPT="${SCRIPT_DIR}/sdk_mode.py"

SSH_HOST="${WALKER_C1_SDK_HOST:-192.168.11.3}"
SSH_PORT="${WALKER_C1_SDK_PORT:-2222}"
SSH_USER="${WALKER_C1_SDK_USER:-ubt}"
SSH_PASSWORD="${WALKER_C1_SDK_PASSWORD:-}"
# 未显式设密码时，自动读取密码文件（默认 ~/.walker_c1_sdk_password，权限应为 600），
# 这样日常部署无需每次 export / 交互输入。可用 WALKER_C1_SDK_PASSWORD_FILE 覆盖路径。
if [[ -z "$SSH_PASSWORD" ]]; then
    _PW_FILE="${WALKER_C1_SDK_PASSWORD_FILE:-${HOME:-/tmp}/.walker_c1_sdk_password}"
    if [[ -r "$_PW_FILE" ]]; then
        SSH_PASSWORD="$(head -n 1 "$_PW_FILE" 2>/dev/null | tr -d '\r\n')"
    fi
fi
SSH_CONTAINER="${WALKER_C1_SDK_CONTAINER:-}"
REMOTE_PY="${WALKER_C1_SDK_PYTHON:-python3}"
LOCAL_PY="${WALKER_C1_LOCAL_PYTHON:-python3}"
USE_SUDO=0
TRANSPORT="auto"

PY_ARGS=()

usage() {
    sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

# ── 参数解析：包装层选项与 python 侧参数分离 ──────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)        usage 0 ;;
        --local)          TRANSPORT="local"; shift ;;
        --remote)         TRANSPORT="remote"; shift ;;
        --ssh-host)       SSH_HOST="$2"; shift 2 ;;
        --ssh-port)       SSH_PORT="$2"; shift 2 ;;
        --ssh-user)       SSH_USER="$2"; shift 2 ;;
        --ssh-container)  SSH_CONTAINER="$2"; shift 2 ;;
        --sudo)           USE_SUDO=1; shift ;;
        --python)         REMOTE_PY="$2"; shift 2 ;;
        *)                PY_ARGS+=("$1"); shift ;;
    esac
done

if [[ ! -f "$PY_SCRIPT" ]]; then
    echo "[error] 找不到 $PY_SCRIPT" >&2
    exit 1
fi

if ! command -v "$LOCAL_PY" >/dev/null 2>&1 && [[ ! -x /usr/bin/python3 ]]; then
    echo "[error] 未找到本地 python3（可用 WALKER_C1_LOCAL_PYTHON 指定）" >&2
    exit 1
fi

# ── 选择 transport ────────────────────────────────────────────────────────────
# 优先用 PATH 上的 python3；若 wheel 装在系统 python（install_ccapi.sh 默认）而
# PATH 上的 python3 是 venv 解释器，则回退到 /usr/bin/python3。
resolve_local_py() {
    if [[ -n "${WALKER_C1_LOCAL_PYTHON:-}" ]]; then
        return 0
    fi
    if [[ -x /usr/bin/python3 ]] && /usr/bin/python3 -c "import ubt_robot" >/dev/null 2>&1; then
        LOCAL_PY=/usr/bin/python3
    fi
}

if [[ "$TRANSPORT" == "auto" ]]; then
    resolve_local_py
    # 显式给了跨机 address 时只能本地执行；否则以"本地有 SDK 且能连 socket"为准
    if [[ " ${PY_ARGS[*]-} " == *" --address "* ]]; then
        TRANSPORT="local"
    elif [[ -S /tmp/cc_api.sock ]] && "$LOCAL_PY" -c "import ubt_robot" >/dev/null 2>&1; then
        TRANSPORT="local"
    else
        TRANSPORT="remote"
    fi
fi

# ── 本地执行 ──────────────────────────────────────────────────────────────────
if [[ "$TRANSPORT" == "local" ]]; then
    resolve_local_py
    exec "$LOCAL_PY" "$PY_SCRIPT" "${PY_ARGS[@]-}"
fi

# ── 远程执行（SSH + stdin 送入脚本）───────────────────────────────────────────
if ! command -v ssh >/dev/null 2>&1; then
    cat >&2 <<'EOF'
[error] 当前环境没有 ssh 客户端，无法远程到机器人开发者容器执行。三选一：
  1) 在宿主机（有 ssh）执行本脚本：
       bash <宿主机路径>/ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh half_sdk
  2) 在容器内安装 ssh 客户端（需 root）：
       apt-get update && apt-get install -y openssh-client
  3) 在 CC 服务所在容器内执行（--local），需先安装 ubt_robot：
       bash <C1_SDK>/... 或 bash install_ccapi.sh
EOF
    exit 1
fi

SSH_CMD=(ssh -T -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new -p "$SSH_PORT")
if [[ -n "${WALKER_C1_SDK_SSH_OPTS:-}" ]]; then
    # shellcheck disable=SC2206  # 需要按空格拆分
    SSH_EXTRA=(${WALKER_C1_SDK_SSH_OPTS})
    SSH_CMD+=("${SSH_EXTRA[@]}")
fi

SSHPASS_BIN=""
ASKPASS_FILE=""
# 非交互密码：优先 sshpass；没有 sshpass 时用 SSH_ASKPASS（OpenSSH 8.4+ 的
# SSH_ASKPASS_REQUIRE=force，部署容器里通常没有 sshpass，但一定有 ssh）。
cleanup_askpass() {
    if [[ -n "$ASKPASS_FILE" ]]; then
        rm -f "$ASKPASS_FILE"
    fi
}
trap cleanup_askpass EXIT

if [[ -n "$SSH_PASSWORD" ]]; then
    if command -v sshpass >/dev/null 2>&1; then
        SSHPASS_BIN="sshpass"
    else
        ASKPASS_FILE="$(mktemp "${TMPDIR:-/tmp}/walker_c1_askpass.XXXXXX" 2>/dev/null || true)"
        if [[ -n "$ASKPASS_FILE" ]]; then
            cat > "$ASKPASS_FILE" <<'ASK'
#!/bin/sh
printf '%s\n' "${WALKER_C1_SDK_PASSWORD:-}"
ASK
            chmod 700 "$ASKPASS_FILE"
            export WALKER_C1_SDK_PASSWORD="$SSH_PASSWORD"
            export SSH_ASKPASS="$ASKPASS_FILE"
            export SSH_ASKPASS_REQUIRE=force
            echo "[sdk_mode] 未安装 sshpass，使用 SSH_ASKPASS 非交互登录" >&2
        else
            echo "[warn] 无法创建临时 askpass 脚本，改为交互输入密码" >&2
        fi
    fi
fi

quote_args() {
    local out="" arg
    for arg in "$@"; do
        out+=" $(printf '%q' "$arg")"
    done
    printf '%s' "$out"
}

quoted="$(quote_args "${PY_ARGS[@]-}")"
if [[ -n "$SSH_CONTAINER" ]]; then
    remote_cmd="sudo -n docker exec -i ${SSH_CONTAINER} ${REMOTE_PY} -${quoted}"
elif [[ "$USE_SUDO" -eq 1 ]]; then
    remote_cmd="sudo -n ${REMOTE_PY} -${quoted}"
else
    remote_cmd="${REMOTE_PY} -${quoted}"
fi

target="${SSH_USER}@${SSH_HOST}"
echo "[sdk_mode] remote: ssh -p ${SSH_PORT} ${target} -> ${remote_cmd}" >&2
echo "[sdk_mode] 密码来源：WALKER_C1_SDK_PASSWORD 或 ~/.walker_c1_sdk_password（sshpass / SSH_ASKPASS 自动处理）" >&2

if [[ -n "$SSHPASS_BIN" ]]; then
    # sshpass 通过环境变量传密码，避免出现在 ps 命令行里
    SSHPASS="$SSH_PASSWORD" "${SSHPASS_BIN}" -e "${SSH_CMD[@]}" "$target" "$remote_cmd" < "$PY_SCRIPT"
else
    "${SSH_CMD[@]}" "$target" "$remote_cmd" < "$PY_SCRIPT"
fi
rc=$?

case "$rc" in
    255) echo "[sdk_mode] SSH 连接失败（主机/端口/密码/网络），见上方输出" >&2 ;;
esac
exit "$rc"
