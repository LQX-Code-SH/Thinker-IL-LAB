#!/bin/bash
# Walker C1 VLA 部署门禁：只在 half_sdk（开发者模式 + 上半身 SDK 权限）下放行。
#
# 被 rollout.sh / robot_ready.sh / inference_server.sh source 后调用：
#   walker_c1_require_half_sdk <caller>
#
# 环境变量
#   SDK_GUARD=auto|1|0    auto（默认）=检测到真机才校验；1=强制校验；0=跳过
#   SDK_AUTO_SWITCH=1     校验不通过时自动执行 sdk_mode.sh half_sdk（默认只提示）
#   WALKER_C1_REAL=1      强制按真机处理（auto 模式下生效）
#   WALKER_C1_SDK_HOST / _PORT / _USER / _PASSWORD / _CONTAINER
#                         透传给 sdk_mode.sh（远程执行用）
#
# auto 判定为真机的条件（任一）：WALKER_C1_REAL=1、显式给了 SDK SSH 主机、
# CAMERA_MSG_TYPE=shm_msgs/*（真机相机）、或本地有 ubt_robot 且 /tmp/cc_api.sock 存在。

walker_c1_guard_enabled() {
    case "${SDK_GUARD:-auto}" in
        0|off|false|no) return 1 ;;
        1|on|true|yes|strict) return 0 ;;
        auto|"")
            [[ "${WALKER_C1_REAL:-0}" == "1" ]] && return 0
            [[ -n "${WALKER_C1_SDK_HOST:-}${WALKER_C1_SDK_CONTAINER:-}" ]] && return 0
            [[ "${CAMERA_MSG_TYPE:-}" == shm_msgs/* ]] && return 0
            # shellcheck disable=SC2317  # python3 可能不存在时静默跳过
            if [[ -S /tmp/cc_api.sock ]] && python3 -c "import ubt_robot" >/dev/null 2>&1; then
                return 0
            fi
            return 1
            ;;
        *)
            echo "[sdk-guard] 未知 SDK_GUARD=${SDK_GUARD}（按 auto 处理）" >&2
            return 1
            ;;
    esac
}

walker_c1_require_half_sdk() {
    local caller="${1:-deploy}"
    local guard_dir sdk rc
    guard_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    sdk="${guard_dir}/sdk_mode.sh"

    case "${SDK_GUARD:-auto}" in
        0|off|false|no)
            echo "[sdk-guard] SDK_GUARD=${SDK_GUARD}，跳过 half_sdk 校验"
            return 0
            ;;
    esac

    if ! walker_c1_guard_enabled; then
        echo "[sdk-guard] 未检测到真机（SDK_GUARD=auto），跳过 half_sdk 校验；真机部署请设 WALKER_C1_REAL=1"
        return 0
    fi

    if [[ ! -f "$sdk" ]]; then
        echo "[sdk-guard] 警告：$sdk 不存在，跳过 half_sdk 校验" >&2
        return 0
    fi

    # 注意：调用方多为 set -e，必须用 `|| rc=$?` 方式取退出码，避免直接中止
    rc=0
    bash "$sdk" check || rc=$?
    if [[ $rc -eq 0 ]]; then
        echo "[sdk-guard] OK：已处于 half_sdk（$caller 可部署）"
        return 0
    fi

    if [[ $rc -ne 2 ]]; then
        cat >&2 <<EOF
[sdk-guard] 拒绝部署（$caller）：无法读取机器人 SDK 状态（连接/SSH 失败）。
  - 确认机器人已上电、开发者容器可达，默认 ubt@192.168.11.3:2222；
  - 非交互环境设置 WALKER_C1_SDK_PASSWORD（sshpass 或 SSH_ASKPASS），或改用 ssh 免密；
  - 若这是仿真/离线环境，设 SDK_GUARD=0 跳过本检查。
EOF
        return 1
    fi

    if [[ "${SDK_AUTO_SWITCH:-0}" == "1" ]]; then
        echo "[sdk-guard] 当前不满足 half_sdk，尝试自动切换 ..."
        if bash "$sdk" half_sdk --yes && bash "$sdk" check; then
            echo "[sdk-guard] OK：已自动切到 half_sdk"
            return 0
        fi
        echo "[sdk-guard] 自动切换失败" >&2
    fi

    cat >&2 <<EOF
[sdk-guard] 拒绝部署（$caller）：当前不是半身 SDK 模式。
  C1 只有 half_sdk 才接收手臂关节指令：normal 下指令被忽略，full_sdk 会交出全身平衡（危险）。
  手动切换（进入开发者模式 + 腿部 STAND + half_sdk）：
    bash ${guard_dir}/sdk_mode.sh half_sdk
  或让部署脚本自动切换：
    SDK_AUTO_SWITCH=1 <原命令>
  只想跳过检查（仿真/调试）：
    SDK_GUARD=0 <原命令>
EOF
    return 1
}
