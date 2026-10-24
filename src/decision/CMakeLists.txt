cmake_minimum_required(VERSION 3.8)
project(decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(rm_decision_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(trans_node src/trans_node.cpp)
add_executable(trans_node2 src/trans_node2.cpp)
add_executable(decision src/decision.cpp)
add_executable(rosout_node src/rosout.cpp)
add_executable(rosout_trans src/rosout_trans.cpp)
add_executable(pub_super_ele src/super_ele.cpp)


target_include_directories(trans_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trans_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(trans_node2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trans_node2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  trans_node
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "base_interfaces_demo"
  "rm_decision_interfaces"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "serial"
)

ament_target_dependencies(
  trans_node2
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "base_interfaces_demo"
  "rm_decision_interfaces"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "serial"
)

ament_target_dependencies(
  decision
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
  "rm_decision_interfaces"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "tf2"
  "serial"
)

ament_target_dependencies(
  rosout_node
  "rclcpp"
  "std_msgs"
  "rcl_interfaces"
)

ament_target_dependencies(
  rosout_trans
  "rclcpp"
  "rcl_interfaces"
)

ament_target_dependencies(
  pub_super_ele
  "rclcpp"
  "std_msgs"
)


install(TARGETS 
  trans_node
  trans_node2
  decision
  rosout_node
  rosout_trans
  pub_super_ele
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(serial_watchdog_pty
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test_serial_watchdog.py
      $<TARGET_FILE:trans_node2>
    TIMEOUT 20
    ENV "ROS_DOMAIN_ID=231" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()
ament_package()
