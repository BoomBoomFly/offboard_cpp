#include "offboard_cpp/gateway_executor.hpp"

#include <cmath>

extern "C" {
#include <boomboom_common/state.h>
#include <boomboom_common/status.h>
}

namespace offboard_cpp
{
namespace
{
constexpr std::int64_t kUsPerSecond = 1000000;
constexpr std::uint8_t kStickGesture = 1;
constexpr std::uint8_t kRcSwitch = 2;

bool finite(const GatewayGoal & goal)
{
  return std::isfinite(goal.target_ned[0]) && std::isfinite(goal.target_ned[1]) &&
         std::isfinite(goal.target_ned[2]) && std::isfinite(goal.yaw_rad) &&
         std::isfinite(goal.duration_s) && std::isfinite(goal.timeout_s);
}
}  // namespace

GatewayExecutor::GatewayExecutor(MissionConfig config)
: config_(config),
  reach_mode_(config.position_tolerance_m, config.velocity_tolerance_mps, config.stable_duration_s)
{}

bool GatewayExecutor::valid_goal(const GatewayGoal & goal)
{
  return static_cast<std::uint8_t>(goal.command) <= static_cast<std::uint8_t>(GatewayCommand::LAND) &&
         finite(goal) && goal.duration_s >= 0.0 && goal.timeout_s >= 0.0;
}

bool GatewayExecutor::can_accept(const GatewayGoal & goal) const
{
  if (!valid_goal(goal) || command_active_ || land_irreversible() ||
      state_ == GatewayState::TAKEOVER || state_ == GatewayState::FAILED) {
    return false;
  }
  if (goal.command == GatewayCommand::TAKEOFF) {
    return state_ == GatewayState::WAIT_DISARMED || state_ == GatewayState::READY;
  }
  return state_ == GatewayState::IDLE;
}

bool GatewayExecutor::start(const GatewayGoal & goal, std::int64_t now_us)
{
  if (!can_accept(goal)) { return false; }
  goal_ = goal;
  command_active_ = true;
  cancel_pending_ = false;
  result_ = {};
  goal_started_at_us_ = now_us;
  if (goal.command == GatewayCommand::TAKEOFF) {
    enter(GatewayState::WAIT_RC_ARM, now_us, BOOMBOOM_STATE_REASON_EKF_READY);
  } else if (goal.command == GatewayCommand::LAND) {
    enter(GatewayState::LAND_REQUEST, now_us);
  } else {
    if (goal.command == GatewayCommand::GOTO) {
      target_ = goal.target_ned;
      yaw_ = goal.yaw_rad;
    } else if (goal.command == GatewayCommand::RETURN_HOME) {
      target_[0] = home_[0];
      target_[1] = home_[1];
    }
    active_mode_ = make_active_mode(goal);
    active_mode_->on_activate(now_us);
    enter(GatewayState::EXECUTING, now_us);
  }
  return true;
}

bool GatewayExecutor::cancel_requested() const
{
  return command_active_ && !land_irreversible();
}

void GatewayExecutor::request_cancel()
{
  if (cancel_requested()) { cancel_pending_ = true; }
}

bool GatewayExecutor::land_irreversible() const
{
  return state_ == GatewayState::LAND_REQUEST || state_ == GatewayState::WAIT_LANDED;
}

void GatewayExecutor::enter(GatewayState state, std::int64_t now_us, std::uint16_t reason)
{
  state_ = state;
  entered_at_us_ = now_us;
  state_reason_ = reason;
}

void GatewayExecutor::finish(
  bool succeeded, bool cancelled, std::uint16_t condition, std::uint16_t reason)
{
  command_active_ = false;
  cancel_pending_ = false;
  result_ = {true, succeeded, cancelled, reason, condition};
  ++completion_sequence_;
}

bool GatewayExecutor::source_is_rc(std::uint8_t reason) const
{
  return reason == kStickGesture || reason == kRcSwitch;
}

bool GatewayExecutor::controls_offboard() const
{
  return state_ == GatewayState::OFFBOARD_PRESTREAM || state_ == GatewayState::REQUEST_OFFBOARD ||
         state_ == GatewayState::EXECUTING || state_ == GatewayState::IDLE;
}

void GatewayExecutor::apply_reset(const PositionReset & reset)
{
  if (!have_reset_) {
    reset_ = reset;
    have_reset_ = true;
    return;
  }
  if (reset.xy_counter != reset_.xy_counter) {
    home_[0] += reset.delta_x;
    home_[1] += reset.delta_y;
    target_[0] += reset.delta_x;
    target_[1] += reset.delta_y;
  }
  if (reset.z_counter != reset_.z_counter) {
    home_[2] += reset.delta_z;
    target_[2] += reset.delta_z;
  }
  if (reset.heading_counter != reset_.heading_counter) { yaw_ += reset.delta_heading; }
  reset_ = reset;
}

ModeBase * GatewayExecutor::make_active_mode(const GatewayGoal & goal)
{
  if (goal.command == GatewayCommand::HOLD) {
    hold_mode_ = HoldPositionMode(goal.duration_s);
    return &hold_mode_;
  }
  return &reach_mode_;
}

MissionActions GatewayExecutor::update_mode(const MissionInputs & inputs, bool * complete)
{
  const auto update = active_mode_->update_setpoint(inputs, PositionTarget{target_, yaw_});
  *complete = update.result == ModeResult::SUCCEEDED;
  return update.actions;
}

void GatewayExecutor::hold_current(const MissionInputs & inputs)
{
  target_ = inputs.position_ned;
  yaw_ = inputs.heading_rad;
  active_mode_ = nullptr;
}

MissionActions GatewayExecutor::tick(const MissionInputs & inputs)
{
  const bool arm_rising_edge = inputs.armed && !previous_armed_;
  previous_armed_ = inputs.armed;
  apply_reset(inputs.reset);
  if (inputs.status_fresh && !inputs.armed) { observed_disarmed_ = true; }

  // PX4/RC authority and localization have priority over action cancellation and timeout.
  if (controls_offboard() && (!inputs.status_fresh || inputs.failsafe ||
      ((state_ == GatewayState::EXECUTING || state_ == GatewayState::IDLE) && !inputs.offboard))) {
    if (command_active_) {
      finish(false, false, inputs.failsafe ? BOOMBOOM_STATUS_CONDITION_FAILSAFE :
        BOOMBOOM_STATUS_CONDITION_LOST, inputs.failsafe ? BOOMBOOM_STATE_REASON_FAILSAFE :
        BOOMBOOM_STATE_REASON_AUTHORITY_LOST);
    }
    enter(GatewayState::TAKEOVER, inputs.now_us, inputs.failsafe ?
      BOOMBOOM_STATE_REASON_FAILSAFE : BOOMBOOM_STATE_REASON_AUTHORITY_LOST);
    return {};
  }
  if (controls_offboard() && (!inputs.local_position_fresh || !inputs.local_position_healthy)) {
    if (command_active_) {
      finish(false, false, BOOMBOOM_STATUS_CONDITION_INVALID,
        BOOMBOOM_STATE_REASON_LOCALIZATION_INVALID);
    }
    enter(GatewayState::LAND_REQUEST, inputs.now_us, BOOMBOOM_STATE_REASON_LOCALIZATION_INVALID);
  }

  if (command_active_ && goal_.timeout_s > 0.0 && state_ != GatewayState::LAND_REQUEST &&
      state_ != GatewayState::WAIT_LANDED &&
      inputs.now_us - goal_started_at_us_ >= static_cast<std::int64_t>(goal_.timeout_s * kUsPerSecond)) {
    if (controls_offboard() && inputs.offboard) {
      hold_current(inputs);
      finish(false, false, BOOMBOOM_STATUS_CONDITION_TIMEOUT, BOOMBOOM_STATE_REASON_NONE);
      enter(GatewayState::IDLE, inputs.now_us);
    } else {
      finish(false, false, BOOMBOOM_STATUS_CONDITION_TIMEOUT, BOOMBOOM_STATE_REASON_NONE);
      enter(GatewayState::TAKEOVER, inputs.now_us);
    }
  }

  if (cancel_pending_) {
    if (state_ == GatewayState::EXECUTING || state_ == GatewayState::IDLE) {
      hold_current(inputs);
      finish(false, true, BOOMBOOM_STATUS_CONDITION_CANCELLED, BOOMBOOM_STATE_REASON_CANCELLED);
      enter(GatewayState::IDLE, inputs.now_us, BOOMBOOM_STATE_REASON_CANCELLED);
    } else {
      finish(false, true, BOOMBOOM_STATUS_CONDITION_CANCELLED, BOOMBOOM_STATE_REASON_CANCELLED);
      enter(inputs.armed ? GatewayState::TAKEOVER : GatewayState::READY, inputs.now_us,
        BOOMBOOM_STATE_REASON_CANCELLED);
    }
  }

  MissionActions actions;
  switch (state_) {
    case GatewayState::WAIT_DISARMED:
      if (observed_disarmed_) { enter(GatewayState::READY, inputs.now_us, BOOMBOOM_STATE_REASON_EKF_READY); }
      break;
    case GatewayState::READY:
      break;
    case GatewayState::WAIT_RC_ARM:
      if (!inputs.status_fresh || !inputs.local_position_fresh || !inputs.local_position_healthy) {
        break;
      }
      if (observed_disarmed_ && arm_rising_edge && source_is_rc(inputs.latest_arming_reason)) {
        home_ = inputs.position_ned;
        target_ = home_;
        target_[2] -= config_.takeoff_height_m;
        yaw_ = inputs.heading_rad;
        enter(GatewayState::OFFBOARD_PRESTREAM, inputs.now_us, BOOMBOOM_STATE_REASON_RC_ARM_EDGE);
        // Count the arm-edge tick as the first 20 Hz prestream sample.
        actions = make_position_setpoint(PositionTarget{target_, yaw_});
      }
      break;
    case GatewayState::OFFBOARD_PRESTREAM:
      actions = make_position_setpoint(PositionTarget{target_, yaw_});
      if (!inputs.armed) {
        finish(false, false, BOOMBOOM_STATUS_CONDITION_LOST, BOOMBOOM_STATE_REASON_AUTHORITY_LOST);
        enter(GatewayState::WAIT_DISARMED, inputs.now_us);
      } else if (inputs.now_us - entered_at_us_ >=
        static_cast<std::int64_t>(config_.prestream_duration_s * kUsPerSecond)) {
        request_ack_sequence_ = inputs.ack_sequence;
        offboard_ack_accepted_ = false;
        offboard_ack_accepted_at_us_ = 0;
        enter(GatewayState::REQUEST_OFFBOARD, inputs.now_us, BOOMBOOM_STATE_REASON_PRESTREAM_READY);
        actions.request_offboard = true;
      }
      break;
    case GatewayState::REQUEST_OFFBOARD:
      actions = make_position_setpoint(PositionTarget{target_, yaw_});
      if (inputs.ack_sequence > request_ack_sequence_) {
        if (inputs.offboard_ack == AckResult::ACCEPTED && !offboard_ack_accepted_) {
          offboard_ack_accepted_ = true;
          offboard_ack_accepted_at_us_ = inputs.now_us;
        } else if (inputs.offboard_ack == AckResult::REJECTED) {
          finish(false, false, BOOMBOOM_STATUS_CONDITION_REJECTED, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
          enter(GatewayState::FAILED, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
        }
      }
      if (state_ == GatewayState::REQUEST_OFFBOARD && offboard_ack_accepted_ && inputs.offboard) {
        active_mode_ = make_active_mode(goal_);
        active_mode_->on_activate(inputs.now_us);
        enter(GatewayState::EXECUTING, inputs.now_us, BOOMBOOM_STATE_REASON_OFFBOARD_ACCEPTED);
      } else if (state_ == GatewayState::REQUEST_OFFBOARD &&
        inputs.now_us - (offboard_ack_accepted_ ? offboard_ack_accepted_at_us_ : entered_at_us_) >=
        static_cast<std::int64_t>((offboard_ack_accepted_ ? config_.offboard_state_timeout_s :
          config_.offboard_ack_timeout_s) * kUsPerSecond)) {
        finish(false, false, BOOMBOOM_STATUS_CONDITION_TIMEOUT, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
        enter(GatewayState::FAILED, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
      }
      break;
    case GatewayState::EXECUTING: {
      bool complete = false;
      actions = update_mode(inputs, &complete);
      if (complete) {
        finish(true, false, BOOMBOOM_STATUS_CONDITION_COMPLETE, BOOMBOOM_STATE_REASON_TARGET_REACHED);
        active_mode_ = nullptr;
        enter(GatewayState::IDLE, inputs.now_us, BOOMBOOM_STATE_REASON_TARGET_REACHED);
      }
      break;
    }
    case GatewayState::IDLE:
      actions = make_position_setpoint(PositionTarget{target_, yaw_});
      break;
    case GatewayState::LAND_REQUEST:
      actions.request_land = true;
      land_request_sequence_ = inputs.landed_sequence;
      enter(GatewayState::WAIT_LANDED, inputs.now_us);
      break;
    case GatewayState::WAIT_LANDED:
      if (inputs.landed_sequence > land_request_sequence_ && inputs.landed) {
        if (command_active_) {
          finish(true, false, BOOMBOOM_STATUS_CONDITION_COMPLETE, BOOMBOOM_STATE_REASON_LANDED);
        }
        enter(GatewayState::WAIT_DISARMED, inputs.now_us, BOOMBOOM_STATE_REASON_LANDED);
      }
      break;
    case GatewayState::TAKEOVER:
    case GatewayState::FAILED:
      break;
  }
  return actions;
}

}  // namespace offboard_cpp
