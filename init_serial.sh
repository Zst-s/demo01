#!/usr/bin/env bash
# Prepare access only. trans_node2 opens/configures 115200 baud when launched.
set -eo pipefail
check_only=false
if [[ ${1:-} == --help ]]; then
    echo "用法: $0 [--check] [/dev/ttyUSB0 或 /dev/serial/by-id/...]"
    echo '只准备当前用户访问权限；115200 串口初始化由 trans_node2 完成。'
    exit 0
fi
if [[ ${1:-} == --check ]]; then check_only=true; shift; fi
if (( $# > 1 )); then echo '参数过多' >&2; exit 2; fi
port=${1:-/dev/ttyUSB0}
device=$(readlink -f -- "$port") || exit 2
case "$device" in
    /dev/ttyUSB[0-9]*|/dev/ttyACM[0-9]*) ;;
    *) echo "不是USB串口设备: $port" >&2; exit 2 ;;
esac
if [[ ! -c "$device" ]]; then echo "串口不存在: $port" >&2; exit 2; fi
if [[ ! -r "$device" || ! -w "$device" ]]; then
    if "$check_only"; then echo "当前用户没有串口读写权限: $device" >&2; exit 3; fi
    command -v setfacl >/dev/null || { echo '缺少setfacl，请安装acl软件包后重试' >&2; exit 3; }
    # Give this user access to this device, without world-writable permissions.
    sudo setfacl -m "u:$(id -un):rw" -- "$device"
fi
[[ -r "$device" && -w "$device" ]] || { echo '串口权限初始化失败' >&2; exit 3; }
echo "串口访问就绪: $port -> $device；trans_node2将使用115200 baud。拔插后需重新检查。"
