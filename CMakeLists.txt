cmake_minimum_required(VERSION 3.8)
project(offboard_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(boomboom_common REQUIRED)

add_library(mission_executor
  src/mission_mode.cpp
  src/mission_executor.cpp
)
target_include_directories(mission_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_executor PUBLIC boomboom_common::boomboom_common_core)

add_library(px4_interface src/px4_interface.cpp)
target_include_directories(px4_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(px4_interface rclcpp px4_msgs)

add_library(gateway_executor src/gateway_executor.cpp)
target_include_directories(gateway_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gateway_executor PUBLIC mission_executor boomboom_common::boomboom_common_core)

add_library(gateway_state_mapping src/gateway_state_mapping.cpp)
target_include_directories(gateway_state_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gateway_state_mapping boomboom_common)

add_library(offboard_gateway_node_lib src/offboard_gateway_node.cpp)
target_include_directories(offboard_gateway_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(offboard_gateway_node_lib gateway_executor gateway_state_mapping px4_interface)
ament_target_dependencies(offboard_gateway_node_lib rclcpp rclcpp_action px4_msgs boomboom_common)
target_link_libraries(
  offboard_gateway_node_lib
  boomboom_common::boomboom_common__rosidl_typesupport_cpp)

add_executable(offboard_gateway_node src/gateway_main.cpp)
target_link_libraries(offboard_gateway_node offboard_gateway_node_lib)

add_library(offboard_mission_node_lib src/offboard_mission_node.cpp)
target_include_directories(offboard_mission_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(offboard_mission_node_lib
  mission_executor
  px4_interface
  boomboom_common::boomboom_common_ros2
)
ament_target_dependencies(offboard_mission_node_lib rclcpp std_srvs px4_msgs boomboom_common)

add_executable(offboard_mission_node src/main.cpp)
target_link_libraries(offboard_mission_node offboard_mission_node_lib)

install(TARGETS mission_executor px4_interface gateway_executor gateway_state_mapping offboard_gateway_node_lib
  offboard_mission_node offboard_gateway_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES config/mission_common.yaml DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_executor test/test_mission_executor.cpp)
  target_link_libraries(test_mission_executor mission_executor)
  ament_add_gtest(test_mission_mode test/test_mission_mode.cpp)
  target_link_libraries(test_mission_mode mission_executor)
  ament_add_gtest(test_px4_interface test/test_px4_interface.cpp)
  target_link_libraries(test_px4_interface px4_interface)
  ament_add_gtest(test_gateway_executor test/test_gateway_executor.cpp)
  target_link_libraries(test_gateway_executor gateway_executor)
  ament_add_gtest(test_gateway_state_mapping test/test_gateway_state_mapping.cpp)
  target_link_libraries(test_gateway_state_mapping gateway_state_mapping)
endif()

ament_package()
