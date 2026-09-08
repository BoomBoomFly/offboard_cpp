#include <gtest/gtest.h>

#include <limits>
#include <chrono>
#include <cmath>
#include <thread>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "offboard_cpp/px4/px4_interface.hpp"

namespace
{
px4_msgs::msg::VehicleCommandAck offboard_ack()
{
  px4_msgs::msg::VehicleCommandAck ack{};
  ack.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  ack.target_system = offboard_cpp::detail::kMissionSourceSystem;
  ack.target_component = offboard_cpp::detail::kMissionSourceComponent;
  return ack;
}

TEST(Px4InterfaceAck, AcceptsAckAddressedToMissionSource)
{
  EXPECT_TRUE(offboard_cpp::detail::is_mission_offboard_ack(offboard_ack()));
}

TEST(Px4InterfaceAck, RejectsAckAddressedToAnotherTarget)
{
  auto ack = offboard_ack();
  ack.target_system = offboard_cpp::detail::kMissionSourceSystem + 1;
  EXPECT_FALSE(offboard_cpp::detail::is_mission_offboard_ack(ack));

  ack = offboard_ack();
  ack.target_component = offboard_cpp::detail::kMissionSourceComponent + 1;
  EXPECT_FALSE(offboard_cpp::detail::is_mission_offboard_ack(ack));
}

TEST(Px4InterfacePosition, RequiresFiniteControllableHeading)
{
  px4_msgs::msg::VehicleLocalPosition position{};
  position.xy_valid = true;
  position.z_valid = true;
  position.v_xy_valid = true;
  position.v_z_valid = true;
  position.heading_good_for_control = true;
  position.x = 1.0F;
  position.y = 2.0F;
  position.z = -1.0F;
  position.vx = 0.0F;
  position.vy = 0.0F;
  position.vz = 0.0F;
  position.heading = 0.5F;
  EXPECT_TRUE(offboard_cpp::detail::is_local_position_healthy(position));

  position.v_xy_valid = false;
  EXPECT_FALSE(offboard_cpp::detail::is_local_position_healthy(position));
  position.v_xy_valid = true;
  position.v_z_valid = false;
  EXPECT_FALSE(offboard_cpp::detail::is_local_position_healthy(position));
  position.v_z_valid = true;
  position.heading_good_for_control = false;
  EXPECT_FALSE(offboard_cpp::detail::is_local_position_healthy(position));
  position.heading_good_for_control = true;
  position.heading = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(offboard_cpp::detail::is_local_position_healthy(position));
}
}  // namespace

TEST(Px4InterfaceAck, LandAckRequiresMatchingCommandAndSource)
{
  auto ack = offboard_ack();
  EXPECT_FALSE(offboard_cpp::detail::is_mission_land_ack(ack));
  ack.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  EXPECT_TRUE(offboard_cpp::detail::is_mission_land_ack(ack));
  ++ack.target_component;
  EXPECT_FALSE(offboard_cpp::detail::is_mission_land_ack(ack));
}

TEST(Px4InterfacePublish, RequiresSyncAndPublishesPositionOnlyAndInPlaceLand)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("px4_interface_test");
    offboard_cpp::Px4Interface interface(*node);
    const auto now = []() {
      return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    offboard_cpp::MissionActions actions;
    actions.publish_setpoint = true;
    actions.position_ned = {1.0, 2.0, -3.0};
    actions.yaw_rad = 0.4;
    EXPECT_FALSE(interface.publish(actions, now()));
    const auto qos = rclcpp::SensorDataQoS();
    auto sync = node->create_publisher<px4_msgs::msg::TimesyncStatus>(
      "/fmu/out/timesync_status", qos);
    size_t mode_count = 0, setpoint_count = 0, command_count = 0;
    auto mode_sub = node->create_subscription<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos,
      [&](px4_msgs::msg::OffboardControlMode::ConstSharedPtr mode) {
        ++mode_count;
        EXPECT_TRUE(mode->position);
        EXPECT_FALSE(mode->velocity);
        EXPECT_FALSE(mode->acceleration);
      });
    auto setpoint_sub = node->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos,
      [&](px4_msgs::msg::TrajectorySetpoint::ConstSharedPtr point) {
        ++setpoint_count;
        EXPECT_FLOAT_EQ(point->position[2], -3.0F);
        EXPECT_FLOAT_EQ(point->yaw, 0.4F);
        EXPECT_TRUE(std::isnan(point->velocity[0]));
        EXPECT_TRUE(std::isnan(point->acceleration[2]));
        EXPECT_TRUE(std::isnan(point->yawspeed));
        EXPECT_GE(point->timestamp, 1000000U);
      });
    auto command_sub = node->create_subscription<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos,
      [&](px4_msgs::msg::VehicleCommand::ConstSharedPtr command) {
        ++command_count;
        EXPECT_EQ(command->command, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        EXPECT_TRUE(command->from_external);
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto spin = [&executor]() {
      const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
      while (std::chrono::steady_clock::now() < end) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    };
    spin();
    px4_msgs::msg::TimesyncStatus reference;
    reference.timestamp = 1000000;
    sync->publish(reference);
    spin();
    EXPECT_TRUE(interface.publish(actions, now()));
    spin();
    actions = {};
    actions.request_land = true;
    EXPECT_TRUE(interface.publish(actions, now()));
    spin();
    EXPECT_EQ(mode_count, 1U);
    EXPECT_EQ(setpoint_count, 1U);
    EXPECT_EQ(command_count, 1U);
    EXPECT_FALSE(interface.publish(actions, now() + 3000000));
  }
  rclcpp::shutdown();
}
