cmake_minimum_required(VERSION 3.8)
project(rm_bt)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(PACKAGE_INCLUDE_DEPENDS
  behaviortree_cpp
  behaviortree_ros2
  rm_decision_interfaces
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav2_msgs
  geometry_msgs
)

# 定义宏函数 add_target_dependencies
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${PACKAGE_INCLUDE_DEPENDS})
    target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

add_executable(rm_bt
  src/tree_exec_node.cpp
)
ament_target_dependencies(rm_bt ${PACKAGE_INCLUDE_DEPENDS})
target_include_directories(rm_bt PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(rm_bt nav2pose)
install(TARGETS rm_bt DESTINATION lib/${PROJECT_NAME})

ament_auto_add_executable(goal_pose_bridge
  src/goal_pose_bridge.cpp
)
ament_target_dependencies(goal_pose_bridge
  rclcpp rclcpp_action geometry_msgs nav2_msgs
)
install(TARGETS goal_pose_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_auto_add_executable(bt_monitor
  src/tree_out.cpp
)
ament_target_dependencies(bt_monitor
  rclcpp rcl_interfaces
)
install(TARGETS bt_monitor
  RUNTIME DESTINATION lib/${PROJECT_NAME})

#Action — 使用 add_library 而非 ament_auto_add_library，避免插件库互相链接导致符号冲突
add_library(sub_BothCampHp SHARED
  plugins/action/sub_BothCampHp.cpp
)
add_target_dependencies(sub_BothCampHp)
install(TARGETS sub_BothCampHp LIBRARY DESTINATION lib)           #sub_BothCampHp

add_library(sub_game_status SHARED
  plugins/action/sub_game_status.cpp
)
add_target_dependencies(sub_game_status)
install(TARGETS sub_game_status LIBRARY DESTINATION lib)          #sub_game_status

add_library(sub_current_stance SHARED
  plugins/action/sub_current_stance.cpp
)
add_target_dependencies(sub_current_stance)
install(TARGETS sub_current_stance LIBRARY DESTINATION lib)       #sub_current_stance

# add_library(sub_mid_if SHARED
#   plugins/action/sub_mid_if.cpp
# )
# add_target_dependencies(sub_mid_if)
# install(TARGETS sub_mid_if LIBRARY DESTINATION lib)

add_library(send_goal SHARED
  plugins/action/send_goal.cpp
)
add_target_dependencies(send_goal)
install(TARGETS send_goal LIBRARY DESTINATION lib)                #send_goal

add_library(pub_nav2_goal SHARED
  plugins/action/pub_nav2_goal.cpp
)
add_target_dependencies(pub_nav2_goal)
install(TARGETS pub_nav2_goal LIBRARY DESTINATION lib)            #pub_nav2_goal

add_library(pub_current_location SHARED
  plugins/action/pub_current_location.cpp
)
add_target_dependencies(pub_current_location)
install(TARGETS pub_current_location LIBRARY DESTINATION lib)     #pub_current_location

add_library(pub_super_ele SHARED
  plugins/action/pub_super_ele.cpp
)
add_target_dependencies(pub_super_ele)
install(TARGETS pub_super_ele LIBRARY DESTINATION lib)             #pub_super_ele

add_library(pub_intensify SHARED
  plugins/action/pub_intensify.cpp
)
add_target_dependencies(pub_intensify)
install(TARGETS pub_intensify LIBRARY DESTINATION lib)             #pub_intensify

add_library(pub_if_qifu SHARED
  plugins/action/pub_if_qifu.cpp
)
add_target_dependencies(pub_if_qifu)
install(TARGETS pub_if_qifu LIBRARY DESTINATION lib)             #pub_if_qifu

add_library(nav2pose SHARED
  plugins/action/nav2pose.cpp
)
add_target_dependencies(nav2pose)
install(TARGETS nav2pose LIBRARY DESTINATION lib)                   #nav2pose

add_library(sub_enemydetected SHARED
  plugins/action/sub_enemydetected.cpp
)
add_target_dependencies(sub_enemydetected)
install(TARGETS sub_enemydetected LIBRARY DESTINATION lib)        #sub_enemydetected

add_library(sub_RobotStatus SHARED
  plugins/action/sub_RobotStatus.cpp
)
add_target_dependencies(sub_RobotStatus)
install(TARGETS sub_RobotStatus LIBRARY DESTINATION lib)          #sub_RobotStatus  

add_library(sub_fort_if SHARED
  plugins/action/sub_fort_if.cpp
)
add_target_dependencies(sub_fort_if)
install(TARGETS sub_fort_if LIBRARY DESTINATION lib)              #sub_fort_if

add_library(sub_midhigh_if SHARED
  plugins/action/sub_midhigh_if.cpp
)
add_target_dependencies(sub_midhigh_if)
install(TARGETS sub_midhigh_if LIBRARY DESTINATION lib)              #sub_midhigh_if

add_library(sub_outpost_if SHARED
  plugins/action/sub_outpost_if.cpp
)
add_target_dependencies(sub_outpost_if)
install(TARGETS sub_outpost_if LIBRARY DESTINATION lib)              #sub_outpost_if

add_library(sub_remain_time SHARED
  plugins/action/sub_remain_time.cpp
)
add_target_dependencies(sub_remain_time)
install(TARGETS sub_remain_time LIBRARY DESTINATION lib)              #sub_remain_time

#Control
add_library(randomselector SHARED
  plugins/control/randomselector.cpp
)
add_target_dependencies(randomselector)
install(TARGETS randomselector LIBRARY DESTINATION lib)           #randomselector

#Condition
add_library(restores_health SHARED
  plugins/condition/restores_health.cpp
)
add_target_dependencies(restores_health)
install(TARGETS restores_health LIBRARY DESTINATION lib)          #restores_health

add_library(check_hp SHARED
  plugins/condition/check_hp.cpp
)
add_target_dependencies(check_hp)
install(TARGETS check_hp LIBRARY DESTINATION lib)                 #check_hp

add_library(check_pill SHARED
  plugins/condition/check_pill.cpp
)
add_target_dependencies(check_pill)
install(TARGETS check_pill LIBRARY DESTINATION lib)               #check_pill

add_library(check_remain_time SHARED
  plugins/condition/check_remain_time.cpp
)
add_target_dependencies(check_remain_time)
install(TARGETS check_remain_time LIBRARY DESTINATION lib)        #check_remain_time

add_library(check_elapsed_time SHARED
  plugins/condition/check_elapsed_time.cpp
)
add_target_dependencies(check_elapsed_time)
install(TARGETS check_elapsed_time LIBRARY DESTINATION lib)       #check_elapsed_time

#Action - Timer
add_library(timer_action SHARED
  plugins/action/timer_action.cpp
)
add_target_dependencies(timer_action)
install(TARGETS timer_action LIBRARY DESTINATION lib)             #timer_action

ament_auto_package(
    INSTALL_TO_SHARE
    config
    launch
    behavior_tree
)
