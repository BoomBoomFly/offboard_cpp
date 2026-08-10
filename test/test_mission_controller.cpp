#include <gtest/gtest.h>

#include "offboard_cpp/mission_controller.hpp"

namespace
{
offboard_cpp::MissionInputs ready_inputs(std::int64_t now_us)
{
  offboard_cpp::MissionInputs inputs;
  inputs.now_us = now_us;
  inputs.status_fresh = true;
  inputs.local_position_fresh = true;
  inputs.local_position_healthy = true;
  inputs.position_ned = {10.0, 20.0, -0.1};
  inputs.heading_rad = 0.3;
  return inputs;
}

void boot_to_ready(offboard_cpp::MissionController & controller)
{
  auto inputs = ready_inputs(0);
  controller.tick(inputs);
  controller.tick(inputs);
  controller.tick(inputs);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::READY);
}

TEST(MissionController, StartRequiresDisarmedRcEdge)
{
  offboard_cpp::MissionController controller;
  auto armed_at_boot = ready_inputs(0);
  armed_at_boot.armed = true;
  armed_at_boot.latest_arming_reason = 2;
  controller.tick(armed_at_boot);
  controller.tick(armed_at_boot);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::WAIT_DISARMED);

  armed_at_boot.armed = false;
  armed_at_boot.now_us = 1;
  controller.tick(armed_at_boot);
  controller.tick(armed_at_boot);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::READY);

  armed_at_boot.armed = true;
  armed_at_boot.latest_arming_reason = 4;  // external command: never accepted.
  armed_at_boot.now_us = 2;
  controller.tick(armed_at_boot);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::READY);

  armed_at_boot.armed = false;
  armed_at_boot.now_us = 3;
  controller.tick(armed_at_boot);
  armed_at_boot.armed = true;
  armed_at_boot.latest_arming_reason = 2;
  armed_at_boot.now_us = 4;
  controller.tick(armed_at_boot);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::OFFBOARD_PRESTREAM);
}

TEST(MissionController, PrestreamAckAndOffboardGate)
{
  offboard_cpp::MissionConfig config;
  config.prestream_duration_s = 1.0;
  offboard_cpp::MissionController controller(config);
  boot_to_ready(controller);
  auto inputs = ready_inputs(10);
  inputs.armed = true;
  inputs.latest_arming_reason = 1;
  controller.tick(inputs);
  inputs.now_us += 1000000;
  auto actions = controller.tick(inputs);
  EXPECT_TRUE(actions.request_offboard);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::REQUEST_OFFBOARD);
  inputs.ack_sequence = 1;
  inputs.offboard_ack = offboard_cpp::AckResult::ACCEPTED;
  inputs.now_us += 1;
  controller.tick(inputs);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::REQUEST_OFFBOARD);
  inputs.offboard = true;
  inputs.now_us += 1;
  controller.tick(inputs);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::TAKEOFF);
}

TEST(MissionController, LocalizationLossLandsInPlaceAndResetMovesHome)
{
  offboard_cpp::MissionController controller;
  boot_to_ready(controller);
  auto inputs = ready_inputs(10);
  inputs.armed = true;
  inputs.latest_arming_reason = 2;
  controller.tick(inputs);
  inputs.now_us += 1000000;
  controller.tick(inputs);
  inputs.ack_sequence = 1;
  inputs.offboard_ack = offboard_cpp::AckResult::ACCEPTED;
  inputs.offboard = true;
  ++inputs.now_us;
  controller.tick(inputs);
  inputs.reset.xy_counter = 1;
  inputs.reset.delta_x = 2.0F;
  ++inputs.now_us;
  const auto corrected = controller.tick(inputs);
  EXPECT_TRUE(corrected.publish_setpoint);
  EXPECT_DOUBLE_EQ(corrected.position_ned[0], 12.0);
  inputs.local_position_healthy = false;
  ++inputs.now_us;
  const auto land = controller.tick(inputs);
  EXPECT_TRUE(land.request_land);
  EXPECT_EQ(controller.state(), offboard_cpp::MissionState::WAIT_LANDED);
}
}  // namespace
