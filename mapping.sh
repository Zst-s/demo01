#!/usr/bin/env bash
# Mapping + passage memory + Nav2 planning; no chassis or serial output.
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
gnome-terminal -- bash -c '
    cd "$1" || exit 1
    source /opt/ros/humble/setup.bash || exit 1
    source install/setup.bash || exit 1
    export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
    export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"
    ros2 launch sentry_passage_fusion_test passage_real.launch.py \
        mode:=mapping lio:=pointlio enable_motion:=false rviz:=true
    exec bash
' bash "$workspace_dir"
