#!/usr/bin/env bash
# 纯 C++ 单 node 启动脚本：通过不同 yaml 切换机型
#
# 用法:
#   ./scripts/run_cpp_hand.sh rh56dfx
#   ./scripts/run_cpp_hand.sh rh5dg2
#   ./scripts/run_cpp_hand.sh rh56f1
#   ./scripts/run_cpp_hand.sh rh56h1
#   ./scripts/run_cpp_hand.sh rh56h1_canfd
#   ./scripts/run_cpp_hand.sh config/device_protocol_rh56dfx_example.yaml
#   ./scripts/run_cpp_hand.sh rh56dfx --angles 1000,1000,1000,1000,1200,1800
#   ./scripts/run_cpp_hand.sh rh56dfx --read-only
#
# 可选环境变量:
#   INSPIRE_CORE_DIR  默认: <仓库根>/src/inspire_serial_core

set -euo pipefail

PROFILE="${1:-}"
if [[ -z "$PROFILE" ]]; then
    echo "用法: $0 <rh56dfx|rh5dg2|rh56f1|rh56h1|rh56h1_canfd|yaml路径> [serial_hand_control_node 额外参数...]" >&2
    exit 1
fi
shift || true
EXTRA_ARGS=("$@")

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CORE_DIR="${INSPIRE_CORE_DIR:-$ROOT/src/inspire_serial_core}"
BUILD_DIR="$CORE_DIR/build"
CONFIG_DIR="$CORE_DIR/config"
BIN="$BUILD_DIR/serial_hand_control_node"

case "$PROFILE" in
    rh56dfx) CONFIG="$CONFIG_DIR/device_protocol_rh56dfx_example.yaml" ;;
    rh5dg2)  CONFIG="$CONFIG_DIR/device_protocol_rh5dg2_example.yaml" ;;
    rh56f1)  CONFIG="$CONFIG_DIR/device_protocol_rh56f1_example.yaml" ;;
    rh56h1)  CONFIG="$CONFIG_DIR/device_protocol_rh56h1_example.yaml" ;;
    rh56h1_canfd) CONFIG="$CONFIG_DIR/device_protocol_rh56h1_canfd_example.yaml" ;;
    eg5cd1)  CONFIG="$CONFIG_DIR/device_protocol_eg5cd1_example.yaml" ;;
    *)
        if [[ -f "$PROFILE" ]]; then
            CONFIG="$PROFILE"
        elif [[ -f "$CONFIG_DIR/$PROFILE" ]]; then
            CONFIG="$CONFIG_DIR/$PROFILE"
        else
            echo "未知机型或配置文件不存在: $PROFILE" >&2
            exit 1
        fi
        ;;
esac

if [[ ! -x "$BIN" ]]; then
    echo "未找到可执行文件: $BIN" >&2
    echo "请先编译: cd $CORE_DIR && cmake -S . -B build && cmake --build build -j" >&2
    exit 1
fi

echo "程序:     $BIN"
echo "配置:     $CONFIG"
echo "工作目录: $CORE_DIR"
echo "按 Ctrl+C 退出"
echo ""

cd "$CORE_DIR"
exec "$BIN" --config "$CONFIG" "${EXTRA_ARGS[@]}"
