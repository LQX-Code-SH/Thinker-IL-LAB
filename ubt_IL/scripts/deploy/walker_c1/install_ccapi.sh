#!/bin/bash
# 在 ubt_IL 容器内安装 Walker C1 CC API Python SDK（ubt_robot wheel）。
#
# 适用场景：希望用 sdk_mode.sh --local 在本容器内切换开发者模式 / half_sdk。
# 注意：CC 服务的 Unix socket（/tmp/cc_api.sock）默认只在机器人开发者容器里，
#       本容器即使装了 wheel，也仍需满足下面之一才能连接：
#         - 把 /tmp/cc_api.sock 挂进本容器；
#         - 用 --address <host:port> --token <token> 跨机连接。
#       不满足时请继续用 sdk_mode.sh 的默认远程模式（SSH 到开发者容器）。
#
# 用法
#   bash install_ccapi.sh                          # 自动搜索 walker_c1_sdk
#   bash install_ccapi.sh --sdk-dir /path/to/walker_c1_sdk
#   bash install_ccapi.sh --python /usr/bin/python3
#   bash install_ccapi.sh --check                  # 只检查，不安装
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SDK_DIR="${WALKER_C1_SDK_DIR:-}"
PY="/usr/bin/python3"
CHECK_ONLY=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sdk-dir) SDK_DIR="$2"; shift 2 ;;
        --python)  PY="$2"; shift 2 ;;
        --check)   CHECK_ONLY=1; shift ;;
        -h|--help) sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "[error] 未知参数：$1" >&2; exit 1 ;;
    esac
done

find_sdk_dir() {
    local candidate
    for candidate in \
        "$SDK_DIR" \
        "/ubt_IL/walker_c1_sdk" \
        "/home/walker/work/walker_c1_sdk" \
        "${HOME}/walker_c1_sdk" \
        "/opt/walker_c1_sdk"; do
        [[ -n "$candidate" ]] || continue
        if [[ -d "$candidate/cc_api_package/cc_api_client_python_lib" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}

if [[ -z "$SDK_DIR" ]]; then
    SDK_DIR="$(find_sdk_dir || true)"
fi
if [[ -z "$SDK_DIR" ]]; then
    cat >&2 <<'EOF'
[error] 未找到 walker_c1_sdk 目录（需含 cc_api_package/cc_api_client_python_lib）。
  宿主机 SDK 默认在 /home/walker/work/walker_c1_sdk，但该目录不在容器挂载范围内。
  处理方式（任选）：
    1) 把 SDK 拷进仓库挂载目录，例如：
         cp -r /home/walker/work/walker_c1_sdk <repo>/ubt_IL/walker_c1_sdk
       然后重新执行本脚本（容器内即 /ubt_IL/walker_c1_sdk）；
    2) 用 --sdk-dir 指向容器内可见的路径；
    3) 直接用 sdk_mode.sh 的默认远程模式（SSH 到机器人开发者容器），无需安装。
EOF
    exit 1
fi

case "$(uname -m)" in
    aarch64|arm64) WHL_ARCH="arm64" ;;
    x86_64|amd64)  WHL_ARCH="amd64" ;;
    *) echo "[error] 不支持的架构：$(uname -m)" >&2; exit 1 ;;
esac

WHL_DIR="$SDK_DIR/cc_api_package/cc_api_client_python_lib/$WHL_ARCH"
WHL="$(ls "$WHL_DIR"/ubt_robot-*.whl 2>/dev/null | head -1 || true)"

if [[ -z "$WHL" ]]; then
    echo "[error] 未找到 wheel：$WHL_DIR/ubt_robot-*.whl" >&2
    exit 1
fi

echo "[ccapi] SDK    : $SDK_DIR"
echo "[ccapi] 架构   : $(uname -m) -> $WHL_ARCH"
echo "[ccapi] wheel  : $WHL"
echo "[ccapi] python : $PY"

if [[ "$CHECK_ONLY" -eq 1 ]]; then
    "$PY" -c "import ubt_robot; print('[ccapi] 已安装 ubt_robot:', ubt_robot.__file__)" 2>/dev/null \
        || echo "[ccapi] ubt_robot 尚未安装（--check 模式，不做安装）"
    exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
    echo "[warn] 当前非 root；安装到系统 python 需要 root，必要时加 sudo" >&2
fi

"$PY" -m pip install --no-deps --force-reinstall "$WHL"

"$PY" -c "import ubt_robot; print('[ccapi] 安装成功:', ubt_robot.__file__)"

if [[ -S /tmp/cc_api.sock ]]; then
    echo "[ccapi] 检测到 /tmp/cc_api.sock，可用：bash $SCRIPT_DIR/sdk_mode.sh --local status"
else
    cat <<EOF
[ccapi] 未检测到 /tmp/cc_api.sock（CC 服务不在本容器）。
        本容器仍可用远程模式：
          bash $SCRIPT_DIR/sdk_mode.sh status
        若要在本容器内直连 CC，需要挂载 socket 或使用 --address/--token 跨机连接。
EOF
fi
