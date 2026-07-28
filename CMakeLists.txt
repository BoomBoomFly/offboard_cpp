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

add_library(offboard_safety
  src/safety_gate.cpp
  src/timestamp_gate.cpp
  src/safety_gate_adapter.cpp
)

target_include_directories(offboard_safety PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(offboard_safety
  rclcpp
  std_msgs
  px4_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_safety_gate test/test_safety_gate.cpp)
  target_link_libraries(test_safety_gate offboard_safety)

  ament_add_gtest(test_topic_contract
    test/test_topic_contract.cpp
  )
  target_compile_definitions(test_topic_contract PRIVATE
    OFFBOARD_NODE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/src/node.cpp"
  )
  target_link_libraries(test_topic_contract offboard_safety)
endif()

# ---------------------------------------------
# 控制节点：offboard_node
add_executable(offboard_node
  src/node.cpp
)

target_link_libraries(offboard_node
  offboard_safety
)

ament_target_dependencies(offboard_node
  rclcpp
  std_msgs
  px4_msgs
)

# 安装可执行文件
install(TARGETS
  offboard_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
