#!/usr/bin/env bash
# 独立构建/启动感知实验，不 source 或覆盖原实车 install。
set -eo pipefail
passage_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
passage_build_root="$passage_workspace/.passage_lab"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "$passage_build_root/log" build \
  --base-paths "$passage_workspace/src/rm_perception/rog_map" \
    "$passage_workspace/src/rm_perception/sentry_terrain_bridge" \
    "$passage_workspace/src/rm_perception/sentry_passage_sim" \
    "$passage_workspace/src/rm_navigation/rm_navigation" \
  --build-base "$passage_build_root/build" --install-base "$passage_build_root/install" \
  --executor sequential --event-handlers desktop_notification- --cmake-args -DBUILD_TESTING=ON
source "$passage_build_root/install/setup.bash"
ctest --test-dir "$passage_build_root/build/rog_map" --output-on-failure
ctest --test-dir "$passage_build_root/build/sentry_terrain_bridge" --output-on-failure
ctest --test-dir "$passage_build_root/build/sentry_passage_sim" --output-on-failure
if [[ "${1:-}" == "--build-only" ]]; then
  echo "隔离构建与单元测试完成：$passage_build_root"
  exit 0
fi
exec ros2 launch sentry_passage_sim perception_lab.launch.py "$@"
