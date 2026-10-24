#!/usr/bin/env bash
# Save a new Point-LIO / SLAM map bundle; preserve all existing maps.
set -euo pipefail
if [[ $# -ne 1 || ! "$1" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    echo 'Usage: ./save_passage_map.sh NEW_MAP_NAME' >&2
    exit 2
fi
workspace_dir="$(dirname -- "$(realpath -- "${BASH_SOURCE[0]}")")"
set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"
exec ros2 run sentry_passage_fusion_test export_pointlio_map.py "$workspace_dir" "$1"
