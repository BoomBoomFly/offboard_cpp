cmake_minimum_required(VERSION 3.5)
project(offboard_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# 包含头文件目录
include_directories(
  include
)

# Production has no mock-RC compilation path.  Mock inputs belong only in an
# isolated test domain and are not part of this package target.
# The former direct-publisher state machine is intentionally removed from the
# build; SafetyGateAdapter is now the only control-input writer.
add_library(px4_offboard_lib
  src/safety_gate.cpp
  src/timestamp_gate.cpp
  src/safety_gate_adapter.cpp
  src/graph_guard.cpp
  src/landing_monitor.cpp
  src/flight_sequence.cpp
  src/rc_operator_adapter.cpp
  src/mission_start_gate.cpp
)

target_include_directories(px4_offboard_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_offboard_lib
  rclcpp
  std_msgs
  px4_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_topic_contract
    test/test_topic_contract.cpp
  )
  target_link_libraries(test_topic_contract px4_offboard_lib)
  ament_target_dependencies(test_topic_contract rclcpp px4_msgs)

  add_executable(test_gate_runtime test/test_gate_runtime.cpp)
  target_link_libraries(test_gate_runtime px4_offboard_lib gtest)
  ament_target_dependencies(test_gate_runtime rclcpp px4_msgs)
  add_test(NAME test_gate_runtime COMMAND test_gate_runtime)

  add_executable(test_safety_gate
    test/test_safety_gate.cpp
  )
  target_link_libraries(test_safety_gate px4_offboard_lib)
  add_test(NAME test_safety_gate COMMAND test_safety_gate)

  add_executable(test_landing_monitor test/test_landing_monitor.cpp)
  target_link_libraries(test_landing_monitor px4_offboard_lib)
  add_test(NAME test_landing_monitor COMMAND test_landing_monitor)

  add_executable(test_flight_sequence test/test_flight_sequence.cpp)
  target_link_libraries(test_flight_sequence px4_offboard_lib)
  add_test(NAME test_flight_sequence COMMAND test_flight_sequence)

  add_executable(test_contract_guards test/test_contract_guards.cpp)
  target_link_libraries(test_contract_guards px4_offboard_lib)
  add_test(NAME test_contract_guards COMMAND test_contract_guards)

  add_test(
    NAME test_start_source_contract
    COMMAND ${CMAKE_COMMAND}
      -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/flight_sequence_node.cpp
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_start_contract.cmake)
endif()

# ---------------------------------------------
# 控制节点：offboard_node
add_executable(offboard_node
  src/node.cpp
)

target_link_libraries(offboard_node
  px4_offboard_lib
)

ament_target_dependencies(offboard_node
  rclcpp
  std_msgs
  px4_msgs
)

add_executable(flight_sequence_node
  src/flight_sequence_node.cpp
)
target_link_libraries(flight_sequence_node px4_offboard_lib)
ament_target_dependencies(flight_sequence_node rclcpp std_msgs px4_msgs)

add_executable(rc_operator_adapter_node
  src/rc_operator_adapter_node.cpp
)
target_link_libraries(rc_operator_adapter_node px4_offboard_lib)
ament_target_dependencies(rc_operator_adapter_node rclcpp std_msgs px4_msgs)

add_executable(offboard_authority_node
  src/authority_publisher_node.cpp
)
target_link_libraries(offboard_authority_node px4_offboard_lib)
ament_target_dependencies(offboard_authority_node rclcpp std_msgs px4_msgs)

# 安装可执行文件
install(TARGETS
  offboard_node
  flight_sequence_node
  rc_operator_adapter_node
  offboard_authority_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "offboard_control.launch.py"
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
