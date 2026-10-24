cmake_minimum_required(VERSION 3.8)
project(sentry_passage_fusion_test)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
# 新插件仅注册，不加入任何现有实车启动配置。
add_library(static_correction_test_layer SHARED src/static_correction_layer.cpp)
ament_target_dependencies(static_correction_test_layer
  rclcpp nav2_costmap_2d nav_msgs pluginlib tf2_ros)
pluginlib_export_plugin_description_file(nav2_costmap_2d correction_plugins.xml)
add_executable(fusion_harness src/fusion_harness.cpp)
ament_target_dependencies(fusion_harness
  rclcpp nav2_util nav2_costmap_2d nav_msgs nav2_msgs std_msgs pluginlib tf2_ros)
install(TARGETS static_correction_test_layer LIBRARY DESTINATION lib)
install(TARGETS fusion_harness DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fusion_scenario.py scripts/global_passage_memory.py
  scripts/live_memory_shadow.py scripts/passage_entry_manager.py
  scripts/passage_memory_v2.py scripts/isolated_entry_gate.py
  scripts/passage_record_io.py scripts/continuous_passage_memory.py
  scripts/passage_region.py scripts/continuous_wall_region.py
  scripts/controller_command_adapter.py
  scripts/passage_runtime.py
  scripts/export_pointlio_map.py
  scripts/passage_record_handover.py
  scripts/mobile_lab_bridge.py
  scripts/static_route_guard.py
  scripts/grid_fingerprint.py
  scripts/structure_observation.py
  scripts/rigid_transform.py
  scripts/progressive_probe.py scripts/progressive_probe_coordinator.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(continuous_memory
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_continuous_memory.py
    TIMEOUT 15 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(structure_observation
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_structure_observation.py
    TIMEOUT 15 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(structure_tf
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_structure_tf.py
    TIMEOUT 15 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(structure_tf_ros
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_structure_observation.py --ros --tilted
    TIMEOUT 15 ENV "ROS_DOMAIN_ID=232" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(structure_observation_ros
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_structure_observation.py --ros
    TIMEOUT 15 ENV "ROS_DOMAIN_ID=231" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(runtime_io
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_runtime_io.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(runtime_chain
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_runtime_chain.py
    TIMEOUT 30
    ENV "ROS_DOMAIN_ID=230" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(passage_memory
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_global_passage_memory.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(passage_entry_manager
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_passage_entry_manager.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(static_route_guard
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_static_route_guard.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(grid_fingerprint
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_grid_fingerprint.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(passage_memory_v2
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_passage_memory_v2.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(progressive_probe
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_progressive_probe.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(progressive_probe_ros
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_progressive_probe_ros.py
    TIMEOUT 20
    ENV "ROS_DOMAIN_ID=222" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(live_memory_adapter
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_live_memory_shadow.py
    TIMEOUT 10 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(live_memory_ros
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_live_memory_ros.py
    TIMEOUT 25
    ENV "ROS_DOMAIN_ID=220" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(memory_fusion_ros
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_memory_fusion_ros.py
      $<TARGET_FILE:fusion_harness>
    TIMEOUT 65
    ENV "ROS_DOMAIN_ID=221" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(nav2_planner_history
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_nav2_planner_history.py
    TIMEOUT 45
    ENV "ROS_DOMAIN_ID=223" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(nav2_controller_gate
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_controller_gate_ros.py
    TIMEOUT 90
    ENV "ROS_DOMAIN_ID=225" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(visualization_config
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_visualization_config.py
    TIMEOUT 10
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(fusion_integration
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fusion_scenario.py
      --harness $<TARGET_FILE:fusion_harness>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/fusion_test.yaml
    TIMEOUT 90
    ENV "ROS_DOMAIN_ID=218" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(production_correction_isolated
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_production_correction.py
      $<TARGET_FILE:fusion_harness>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/production_preview.yaml
    TIMEOUT 45
    ENV "ROS_DOMAIN_ID=227" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(virtual_preview_wiring
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_virtual_preview_wiring.py
      $<TARGET_FILE:fusion_harness>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/production_preview.yaml
    TIMEOUT 35
    ENV "ROS_DOMAIN_ID=228" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()
ament_package()
