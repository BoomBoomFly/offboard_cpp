#include <gtest/gtest.h>

#include "offboard_cpp/mission_mode.hpp"

namespace
{
offboard_cpp::MissionInputs inputs_at(std::int64_t now_us, const std::array<double, 3> & position)
{
  offboard_cpp::MissionInputs inputs;
  inputs.now_us = now_us;
  inputs.position_ned = position;
  return inputs;
}

TEST(MissionMode, ReachPositionRequiresContinuousStability)
{
  offboard_cpp::ReachPositionMode mode(0.2, 0.15, 1.0);
  const offboard_cpp::PositionTarget target{{1.0, 2.0, -1.5}, 0.3};
  mode.on_activate(0);

  auto inputs = inputs_at(0, target.position_ned);
  auto update = mode.update_setpoint(inputs, target);
  EXPECT_EQ(update.result, offboard_cpp::ModeResult::RUNNING);
  EXPECT_TRUE(update.actions.publish_setpoint);
  EXPECT_EQ(update.actions.position_ned, target.position_ned);

  inputs.now_us = 500000;
  inputs.position_ned[0] += 1.0;
  update = mode.update_setpoint(inputs, target);
  EXPECT_EQ(update.result, offboard_cpp::ModeResult::RUNNING);

  inputs.now_us = 700000;
  inputs.position_ned = target.position_ned;
  mode.update_setpoint(inputs, target);
  inputs.now_us = 1699999;
  update = mode.update_setpoint(inputs, target);
  EXPECT_EQ(update.result, offboard_cpp::ModeResult::RUNNING);
  inputs.now_us = 1700000;
  update = mode.update_setpoint(inputs, target);
  EXPECT_EQ(update.result, offboard_cpp::ModeResult::SUCCEEDED);
}

TEST(MissionMode, HoldPositionCompletesAtDuration)
{
  offboard_cpp::HoldPositionMode mode(2.0);
  const offboard_cpp::PositionTarget target{{3.0, 4.0, -2.0}, -0.2};
  mode.on_activate(100);

  auto inputs = inputs_at(2000099, target.position_ned);
  auto update = mode.update_setpoint(inputs, target);
  EXPECT_EQ(update.result, offboard_cpp::ModeResult::RUNNING);
  inputs.now_us = 2000100;
  update = mode.update_setpoint(inputs, target);
  EXPECT_EQ(update.result, offboard_cpp::ModeResult::SUCCEEDED);
  EXPECT_DOUBLE_EQ(update.actions.yaw_rad, target.yaw_rad);
}
}  // namespace
