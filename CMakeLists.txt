cmake_minimum_required(VERSION 3.8)
project(offboard_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(boomboom_common REQUIRED)

add_library(mission_controller src/mission_controller.cpp)
target_include_directories(mission_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_controller PUBLIC boomboom_common::boomboom_common_core)

add_library(px4_interface src/px4_interface.cpp)
target_include_directories(px4_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(px4_interface rclcpp px4_msgs)

add_library(offboard_mission_node_lib src/offboard_mission_node.cpp)
target_include_directories(offboard_mission_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(offboard_mission_node_lib
  mission_controller
  px4_interface
  boomboom_common::boomboom_common_ros2
)
ament_target_dependencies(offboard_mission_node_lib rclcpp px4_msgs boomboom_common)

add_executable(offboard_mission_node src/main.cpp)
target_link_libraries(offboard_mission_node offboard_mission_node_lib)

install(TARGETS mission_controller px4_interface offboard_mission_node_lib offboard_mission_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_controller test/test_mission_controller.cpp)
  target_link_libraries(test_mission_controller mission_controller)
endif()

ament_package()
