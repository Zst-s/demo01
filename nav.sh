#!/usr/bin/env bash
# Saved-map navigation with formal ROGMap memory + direct Nav2 -> trans_node2.
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
if [[ ${1:-} == --help || $# == 0 ]]; then
    echo "用法: $0 地图名 [串口=/dev/ttyACM0] [导航记忆文件] [定位=icp]"
    echo "示例: $0 my_map11 /dev/ttyACM0"
    echo '地图名不带.yaml；icp需要同名YAML、图像和PCD。'
    echo '导航记忆文件可使用保存地图后handover导出的文件；省略时使用该地图独立文件。'
    exit 0
fi
if (( $# > 4 )); then echo '参数过多' >&2; exit 2; fi
world=$1
port=${2:-/dev/ttyACM0}
localization=${4:-icp}
[[ "$world" =~ ^[[:alnum:]_-][[:alnum:]_.-]*$ && "$world" != .* ]] || { echo '地图名不合法' >&2; exit 2; }
case "$localization" in icp|amcl|slam_toolbox) ;; *) echo '定位方式应为icp、amcl或slam_toolbox' >&2; exit 2 ;; esac
record=${3:-$HOME/.ros/passage_memory/nav-${world}-${localization}.json}
cd -- "$workspace_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"
# Validate the selected installed map before requesting permissions or opening hardware.
python3 - "$world" "$localization" <<'PY'
import importlib.util,sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
base=Path(get_package_share_directory('rm_nav_bringup'))
p=Path(get_package_share_directory('sentry_passage_fusion_test'))/'launch/passage_real.launch.py'
spec=importlib.util.spec_from_file_location('passage_launch',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
print('加载地图身份:',m.saved_map_identity(str(base/'map'),sys.argv[1],sys.argv[2],str(base/'PCD')))
PY
"$workspace_dir/init_serial.sh" "$port"
if command -v fuser >/dev/null && fuser "$port" >/dev/null 2>&1; then
    echo "串口已被其他进程占用: $port；请先关闭已有串口节点。" >&2
    exit 3
fi
printf '导航: map=%s localization=%s port=%s\n速度: Nav2 -> trans_node2；旧gate仅诊断。Ctrl+C停止全部launch节点。\n' "$world" "$localization" "$port"
exec ros2 launch sentry_passage_fusion_test passage_real.launch.py \
    mode:=nav lio:=pointlio world:="$world" localization:="$localization" \
    enable_motion:=true serial_port:="$port" record_path:="$record" rviz:=true
