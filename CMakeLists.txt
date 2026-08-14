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
find_package(px4_msgs REQUIRED)
find_package(boomboom_common REQUIRED)

add_library(px4_interface src/px4/px4_interface.cpp)
target_include_directories(px4_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(px4_interface rclcpp px4_msgs)

add_library(gateway_executor src/gateway/gateway_executor.cpp)
target_include_directories(gateway_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gateway_executor PUBLIC boomboom_common::boomboom_common_core)

add_library(offboard_gateway_node_lib src/gateway/offboard_gateway_node.cpp)
target_include_directories(offboard_gateway_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(offboard_gateway_node_lib gateway_executor px4_interface)
ament_target_dependencies(offboard_gateway_node_lib rclcpp rclcpp_action px4_msgs boomboom_common)
target_link_libraries(
  offboard_gateway_node_lib
  boomboom_common::boomboom_common__rosidl_typesupport_cpp)

add_executable(offboard_gateway_node src/gateway/main.cpp)
target_link_libraries(offboard_gateway_node offboard_gateway_node_lib)

install(TARGETS px4_interface gateway_executor offboard_gateway_node_lib
  offboard_gateway_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES config/mission_common.yaml DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_px4_interface test/test_px4_interface.cpp)
  target_link_libraries(test_px4_interface px4_interface)
  ament_add_gtest(test_gateway_executor test/test_gateway_executor.cpp)
  target_link_libraries(test_gateway_executor gateway_executor offboard_gateway_node_lib)
endif()

ament_package()
