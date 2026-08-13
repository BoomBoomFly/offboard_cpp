#include <gtest/gtest.h>

#include <limits>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "offboard_cpp/px4_interface.hpp"

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
  position.heading_good_for_control = true;
  position.x = 1.0F;
  position.y = 2.0F;
  position.z = -1.0F;
  position.vx = 0.0F;
  position.vy = 0.0F;
  position.vz = 0.0F;
  position.heading = 0.5F;
  EXPECT_TRUE(offboard_cpp::detail::is_local_position_healthy(position));

  position.heading_good_for_control = false;
  EXPECT_FALSE(offboard_cpp::detail::is_local_position_healthy(position));
  position.heading_good_for_control = true;
  position.heading = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(offboard_cpp::detail::is_local_position_healthy(position));
}
}  // namespace
