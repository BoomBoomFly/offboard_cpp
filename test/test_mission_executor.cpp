#include <gtest/gtest.h>

#include "offboard_cpp/mission_executor.hpp"

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

void boot_to_ready(offboard_cpp::MissionExecutor & executor)
{
  auto inputs = ready_inputs(0);
  executor.tick(inputs);
  executor.tick(inputs);
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::READY);
}

TEST(MissionExecutor, StartRequiresDisarmedRcEdge)
{
  offboard_cpp::MissionExecutor executor;
  auto armed_at_boot = ready_inputs(0);
  armed_at_boot.armed = true;
  armed_at_boot.latest_arming_reason = 2;
  executor.tick(armed_at_boot);
  executor.tick(armed_at_boot);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::WAIT_DISARMED);

  armed_at_boot.armed = false;
  armed_at_boot.now_us = 1;
  executor.tick(armed_at_boot);
  executor.tick(armed_at_boot);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::READY);

  armed_at_boot.armed = true;
  armed_at_boot.latest_arming_reason = 4;
  armed_at_boot.now_us = 2;
  executor.tick(armed_at_boot);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::READY);

  armed_at_boot.armed = false;
  armed_at_boot.now_us = 3;
  executor.tick(armed_at_boot);
  armed_at_boot.armed = true;
  armed_at_boot.latest_arming_reason = 2;
  armed_at_boot.now_us = 4;
  executor.tick(armed_at_boot);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::OFFBOARD_PRESTREAM);
}

TEST(MissionExecutor, PrestreamAckAndOffboardGate)
{
  offboard_cpp::MissionConfig config;
  config.prestream_duration_s = 1.0;
  offboard_cpp::MissionExecutor executor(config);
  boot_to_ready(executor);
  auto inputs = ready_inputs(10);
  inputs.armed = true;
  inputs.latest_arming_reason = 1;
  executor.tick(inputs);
  inputs.now_us += 1000000;
  auto actions = executor.tick(inputs);
  EXPECT_TRUE(actions.request_offboard);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::REQUEST_OFFBOARD);
  inputs.ack_sequence = 1;
  inputs.offboard_ack = offboard_cpp::AckResult::ACCEPTED;
  inputs.now_us += 1;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::REQUEST_OFFBOARD);
  inputs.offboard = true;
  inputs.now_us += 1;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::TAKEOFF);
}

TEST(MissionExecutor, AcceptedAckGetsFullStateTimeout)
{
  offboard_cpp::MissionConfig config;
  config.prestream_duration_s = 1.0;
  config.offboard_ack_timeout_s = 2.0;
  config.offboard_state_timeout_s = 2.0;
  offboard_cpp::MissionExecutor executor(config);
  boot_to_ready(executor);
  auto inputs = ready_inputs(10);
  inputs.armed = true;
  inputs.latest_arming_reason = 1;
  executor.tick(inputs);
  inputs.now_us += 1000000;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::REQUEST_OFFBOARD);

  inputs.now_us += 1900000;
  inputs.ack_sequence = 1;
  inputs.offboard_ack = offboard_cpp::AckResult::ACCEPTED;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::REQUEST_OFFBOARD);

  inputs.now_us += 1999999;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::REQUEST_OFFBOARD);

  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::COMPLETE);
}

TEST(MissionExecutor, LocalizationLossLandsInPlaceAndResetMovesHome)
{
  offboard_cpp::MissionExecutor executor;
  boot_to_ready(executor);
  auto inputs = ready_inputs(10);
  inputs.armed = true;
  inputs.latest_arming_reason = 2;
  executor.tick(inputs);
  inputs.now_us += 1000000;
  executor.tick(inputs);
  inputs.ack_sequence = 1;
  inputs.offboard_ack = offboard_cpp::AckResult::ACCEPTED;
  inputs.offboard = true;
  ++inputs.now_us;
  executor.tick(inputs);
  inputs.reset.xy_counter = 1;
  inputs.reset.delta_x = 2.0F;
  ++inputs.now_us;
  const auto corrected = executor.tick(inputs);
  EXPECT_TRUE(corrected.publish_setpoint);
  EXPECT_DOUBLE_EQ(corrected.position_ned[0], 12.0);
  inputs.landed = true;
  inputs.landed_sequence = 1;
  inputs.local_position_healthy = false;
  ++inputs.now_us;
  const auto land = executor.tick(inputs);
  EXPECT_TRUE(land.request_land);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::WAIT_LANDED);

  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::WAIT_LANDED);

  ++inputs.landed_sequence;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::COMPLETE);
}

TEST(MissionExecutor, CancelDuringHoverReturnsLocalHome)
{
  offboard_cpp::MissionConfig config;
  config.stable_duration_s = 0.0;
  config.hover_duration_s = 60.0;
  offboard_cpp::MissionExecutor executor(config);
  boot_to_ready(executor);
  auto inputs = ready_inputs(10);
  inputs.armed = true;
  inputs.latest_arming_reason = 2;
  executor.tick(inputs);
  inputs.now_us += 1000000;
  executor.tick(inputs);
  inputs.ack_sequence = 1;
  inputs.offboard_ack = offboard_cpp::AckResult::ACCEPTED;
  inputs.offboard = true;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::TAKEOFF);

  inputs.position_ned[2] = -1.6;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::HOVER);

  inputs.cancel_requested = true;
  ++inputs.now_us;
  const auto actions = executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::MissionState::RETURN_LOCAL);
  EXPECT_EQ(executor.state_reason(), BOOMBOOM_STATE_REASON_CANCELLED);
  EXPECT_TRUE(actions.publish_setpoint);
  EXPECT_DOUBLE_EQ(actions.position_ned[0], 10.0);
  EXPECT_DOUBLE_EQ(actions.position_ned[1], 20.0);
  EXPECT_DOUBLE_EQ(actions.position_ned[2], -1.6);
}
}  // namespace
