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
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# 接口定义依赖包
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# 包含头文件目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Production has no mock-RC compilation path.  Mock inputs belong only in an
# isolated test domain and are not part of this package target.
# The former direct-publisher state machine is intentionally removed from the
# build; SafetyGateAdapter is now the only control-input writer.
add_library(px4_offboard_lib
  src/lib/param.cpp
  src/lib/input.cpp
  src/safety_gate.cpp
  src/timestamp_gate.cpp
  src/safety_gate_adapter.cpp
)

target_include_directories(px4_offboard_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_offboard_lib
  rclcpp
  rcl_interfaces
  std_msgs
  px4_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(px4_offboard_lib
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rc_input
    test/test_rc_input.cpp
  )
  target_link_libraries(test_rc_input
    px4_offboard_lib
  )
  ament_target_dependencies(test_rc_input
    rclcpp
    px4_msgs
  )

  ament_add_gtest(test_topic_contract
    test/test_topic_contract.cpp
  )
  target_link_libraries(test_topic_contract px4_offboard_lib)

  add_executable(test_safety_gate
    test/test_safety_gate.cpp
  )
  target_link_libraries(test_safety_gate px4_offboard_lib)
  add_test(NAME test_safety_gate COMMAND test_safety_gate)
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
  rcl_interfaces
  std_msgs
  px4_msgs
)

# ---------------------------------------------
# Demo 节点：发布简单轨迹给 offboard_node
add_executable(offboard_demo
  src/examples/offboard_demo.cpp
)

ament_target_dependencies(offboard_demo
  rclcpp
  std_msgs
  px4_msgs
)

# ---------------------------------------------
# Animal Testing 节点
add_executable(animal_testing
  src/examples/animal_testing.cpp
)

ament_target_dependencies(animal_testing
  rclcpp
  std_msgs
  px4_msgs
)

# 安装可执行文件
install(TARGETS
  offboard_node
  offboard_demo
  animal_testing
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
