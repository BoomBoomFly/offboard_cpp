#include <gtest/gtest.h>

#include <limits>

#include "offboard_cpp/gateway_executor.hpp"

namespace
{
offboard_cpp::MissionInputs inputs_at(std::int64_t now_us)
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

offboard_cpp::GatewayGoal takeoff_goal()
{
  offboard_cpp::GatewayGoal goal;
  goal.command = offboard_cpp::GatewayCommand::TAKEOFF;
  return goal;
}

void boot_to_ready(offboard_cpp::GatewayExecutor & executor)
{
  auto inputs = inputs_at(0);
  executor.tick(inputs);
  executor.tick(inputs);
  ASSERT_EQ(executor.state(), offboard_cpp::GatewayState::READY);
}

void takeoff_to_idle(offboard_cpp::GatewayExecutor & executor)
{
  boot_to_ready(executor);
  ASSERT_TRUE(executor.start(takeoff_goal(), 1));
  auto inputs = inputs_at(1);
  executor.tick(inputs);
  inputs.armed = true;
  inputs.latest_arming_reason = 1;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::OFFBOARD_PRESTREAM);
  inputs.now_us += 1000000;
  EXPECT_TRUE(executor.tick(inputs).request_offboard);
  inputs.ack_sequence = 1;
  inputs.offboard_ack = offboard_cpp::AckResult::ACCEPTED;
  inputs.offboard = true;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::EXECUTING);
  inputs.position_ned[2] = -1.6;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::IDLE);
}

TEST(GatewayExecutor, ValidatesCommandsAndOnlyAcceptsTakeoffBeforeOffboard)
{
  offboard_cpp::GatewayExecutor executor;
  boot_to_ready(executor);
  auto goto_goal = takeoff_goal();
  goto_goal.command = offboard_cpp::GatewayCommand::GOTO;
  EXPECT_FALSE(executor.can_accept(goto_goal));
  EXPECT_TRUE(executor.can_accept(takeoff_goal()));
  auto invalid = takeoff_goal();
  invalid.duration_s = -1.0;
  EXPECT_FALSE(offboard_cpp::GatewayExecutor::valid_goal(invalid));
  invalid = takeoff_goal();
  invalid.command = static_cast<offboard_cpp::GatewayCommand>(99);
  EXPECT_FALSE(offboard_cpp::GatewayExecutor::valid_goal(invalid));
  invalid = takeoff_goal();
  invalid.target_ned[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(offboard_cpp::GatewayExecutor::valid_goal(invalid));
}

TEST(GatewayExecutor, TakeoffUsesObservedDisarmedRcEdgeAndOneCommandAtATime)
{
  offboard_cpp::GatewayExecutor executor;
  boot_to_ready(executor);
  ASSERT_TRUE(executor.start(takeoff_goal(), 10));
  EXPECT_FALSE(executor.can_accept(takeoff_goal()));
  auto inputs = inputs_at(10);
  inputs.armed = true;
  inputs.latest_arming_reason = 4;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::WAIT_RC_ARM);
  inputs.armed = false;
  ++inputs.now_us;
  executor.tick(inputs);
  inputs.armed = true;
  inputs.latest_arming_reason = 2;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::OFFBOARD_PRESTREAM);
}

TEST(GatewayExecutor, CancelGotoBecomesHoldWithoutReturnHome)
{
  offboard_cpp::MissionConfig config;
  config.stable_duration_s = 0.0;
  offboard_cpp::GatewayExecutor executor(config);
  takeoff_to_idle(executor);
  auto goto_goal = takeoff_goal();
  goto_goal.command = offboard_cpp::GatewayCommand::GOTO;
  goto_goal.target_ned = {50.0, 60.0, -2.0};
  goto_goal.yaw_rad = 1.0;
  ASSERT_TRUE(executor.start(goto_goal, 2000000));
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::EXECUTING);
  executor.request_cancel();
  auto inputs = inputs_at(2000001);
  inputs.armed = true;
  inputs.offboard = true;
  inputs.position_ned = {12.0, 21.0, -1.5};
  inputs.heading_rad = 0.4;
  const auto actions = executor.tick(inputs);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::IDLE);
  EXPECT_TRUE(actions.publish_setpoint);
  EXPECT_EQ(actions.position_ned, inputs.position_ned);
  EXPECT_TRUE(executor.result().cancelled);
}

TEST(GatewayExecutor, LocalizationLossOverridesCancelAndRequestsInPlaceLand)
{
  offboard_cpp::MissionConfig config;
  config.stable_duration_s = 0.0;
  offboard_cpp::GatewayExecutor executor(config);
  takeoff_to_idle(executor);
  auto hold_goal = takeoff_goal();
  hold_goal.command = offboard_cpp::GatewayCommand::HOLD;
  hold_goal.duration_s = 10.0;
  ASSERT_TRUE(executor.start(hold_goal, 2000000));
  executor.request_cancel();
  auto inputs = inputs_at(2000001);
  inputs.armed = true;
  inputs.offboard = true;
  inputs.local_position_healthy = false;
  const auto actions = executor.tick(inputs);
  EXPECT_TRUE(actions.request_land);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::WAIT_LANDED);
  EXPECT_FALSE(executor.result().cancelled);
  EXPECT_FALSE(executor.cancel_requested());
}

TEST(GatewayExecutor, LandCannotBeCancelledAndCompletesAfterNewLandingSample)
{
  offboard_cpp::MissionConfig config;
  config.stable_duration_s = 0.0;
  offboard_cpp::GatewayExecutor executor(config);
  takeoff_to_idle(executor);
  auto land_goal = takeoff_goal();
  land_goal.command = offboard_cpp::GatewayCommand::LAND;
  ASSERT_TRUE(executor.start(land_goal, 2000000));
  EXPECT_FALSE(executor.cancel_requested());
  auto inputs = inputs_at(2000001);
  inputs.armed = true;
  inputs.offboard = true;
  inputs.landed_sequence = 4;
  EXPECT_TRUE(executor.tick(inputs).request_land);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::WAIT_LANDED);
  inputs.landed = true;
  ++inputs.landed_sequence;
  ++inputs.now_us;
  executor.tick(inputs);
  EXPECT_TRUE(executor.result().succeeded);
  EXPECT_EQ(executor.state(), offboard_cpp::GatewayState::WAIT_DISARMED);
}
}  // namespace
