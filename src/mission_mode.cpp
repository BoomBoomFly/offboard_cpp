#include "offboard_cpp/mission_mode.hpp"

#include <cmath>

namespace offboard_cpp
{
namespace
{
constexpr std::int64_t kUsPerSecond = 1000000;
}

MissionActions make_position_setpoint(const PositionTarget & target)
{
  MissionActions actions;
  actions.publish_setpoint = true;
  actions.position_ned = target.position_ned;
  actions.yaw_rad = target.yaw_rad;
  return actions;
}

ReachPositionMode::ReachPositionMode(
  double position_tolerance_m, double velocity_tolerance_mps, double stable_duration_s)
: position_tolerance_m_(position_tolerance_m),
  velocity_tolerance_mps_(velocity_tolerance_mps),
  stable_duration_us_(static_cast<std::int64_t>(stable_duration_s * kUsPerSecond))
{}

void ReachPositionMode::on_activate(std::int64_t)
{
  stable_since_us_ = -1;
}

ModeUpdate ReachPositionMode::update_setpoint(
  const MissionInputs & inputs, const PositionTarget & target)
{
  ModeUpdate update;
  update.actions = make_position_setpoint(target);

  const double dx = inputs.position_ned[0] - target.position_ned[0];
  const double dy = inputs.position_ned[1] - target.position_ned[1];
  const double dz = inputs.position_ned[2] - target.position_ned[2];
  const double speed = std::sqrt(
    inputs.velocity_ned[0] * inputs.velocity_ned[0] +
    inputs.velocity_ned[1] * inputs.velocity_ned[1] +
    inputs.velocity_ned[2] * inputs.velocity_ned[2]);
  const bool stable = std::sqrt(dx * dx + dy * dy + dz * dz) <= position_tolerance_m_ &&
    speed <= velocity_tolerance_mps_;

  if (!stable) {
    stable_since_us_ = -1;
  } else {
    if (stable_since_us_ < 0) {
      stable_since_us_ = inputs.now_us;
    }
    if (inputs.now_us - stable_since_us_ >= stable_duration_us_) {
      update.result = ModeResult::SUCCEEDED;
    }
  }
  return update;
}

HoldPositionMode::HoldPositionMode(double duration_s)
: duration_us_(static_cast<std::int64_t>(duration_s * kUsPerSecond))
{}

void HoldPositionMode::on_activate(std::int64_t now_us)
{
  activated_at_us_ = now_us;
}

ModeUpdate HoldPositionMode::update_setpoint(
  const MissionInputs & inputs, const PositionTarget & target)
{
  ModeUpdate update;
  update.actions = make_position_setpoint(target);
  if (inputs.now_us - activated_at_us_ >= duration_us_) {
    update.result = ModeResult::SUCCEEDED;
  }
  return update;
}

}  // namespace offboard_cpp
