cmake_minimum_required(VERSION 3.8)
project(sentry_passage_sim)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/passages.world
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_world.py ${CMAKE_CURRENT_BINARY_DIR}/passages.world
  DEPENDS scripts/generate_world.py)
add_custom_target(passage_world ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/passages.world)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/passages_dense.world
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_world.py ${CMAKE_CURRENT_BINARY_DIR}/passages_dense.world --dense
  DEPENDS scripts/generate_world.py)
add_custom_target(passage_dense_world ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/passages_dense.world)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/passages_mobile_dense.world
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_world.py ${CMAKE_CURRENT_BINARY_DIR}/passages_mobile_dense.world --dense --mobile
  DEPENDS scripts/generate_world.py)
add_custom_target(passage_mobile_dense_world ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/passages_mobile_dense.world)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/passages.world DESTINATION share/${PROJECT_NAME}/worlds)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/passages_dense.world DESTINATION share/${PROJECT_NAME}/worlds)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/passages_mobile_dense.world DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/scene_observer.py scripts/check_lab.py scripts/multiview_scan.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_test(NAME test_world_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_world.py)
endif()
ament_package()
