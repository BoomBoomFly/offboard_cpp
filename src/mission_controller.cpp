#include "offboard_cpp/mission_controller.hpp"

#include <cmath>

namespace offboard_cpp
{
namespace
{
constexpr std::uint8_t kStickGesture = 1;
constexpr std::uint8_t kRcSwitch = 2;
constexpr std::int64_t kUsPerSecond = 1000000;
}

MissionController::MissionController(MissionConfig config) : config_(config) {}

void MissionController::enter(MissionState state, std::int64_t now_us, std::uint16_t reason)
{
  state_ = state;
  entered_at_us_ = now_us;
  state_reason_ = reason;
  stable_since_us_ = -1;
}

bool MissionController::source_is_rc(std::uint8_t reason) const
{
  return reason == kStickGesture || reason == kRcSwitch;
}

void MissionController::apply_reset(const PositionReset & reset)
{
  if (!have_reset_) {
    reset_ = reset;
    have_reset_ = true;
    return;
  }
  if (reset.xy_counter != reset_.xy_counter) {
    faults_ |= BOOMBOOM_FAULT_STATE_RESET;
    home_[0] += reset.delta_x;
    home_[1] += reset.delta_y;
    target_[0] += reset.delta_x;
    target_[1] += reset.delta_y;
  }
  if (reset.z_counter != reset_.z_counter) {
    faults_ |= BOOMBOOM_FAULT_STATE_RESET;
    home_[2] += reset.delta_z;
    target_[2] += reset.delta_z;
  }
  if (reset.heading_counter != reset_.heading_counter) {
    faults_ |= BOOMBOOM_FAULT_STATE_RESET;
    yaw_ += reset.delta_heading;
  }
  reset_ = reset;
}

bool MissionController::position_stable(
  const MissionInputs & inputs, const std::array<double, 3> & target) const
{
  const double dx = inputs.position_ned[0] - target[0];
  const double dy = inputs.position_ned[1] - target[1];
  const double dz = inputs.position_ned[2] - target[2];
  const double speed = std::sqrt(
    inputs.velocity_ned[0] * inputs.velocity_ned[0] +
    inputs.velocity_ned[1] * inputs.velocity_ned[1] +
    inputs.velocity_ned[2] * inputs.velocity_ned[2]);
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= config_.position_tolerance_m &&
         speed <= config_.velocity_tolerance_mps;
}

MissionActions MissionController::setpoint() const
{
  MissionActions actions;
  actions.publish_setpoint = true;
  actions.position_ned = target_;
  actions.yaw_rad = yaw_;
  return actions;
}

MissionActions MissionController::tick(const MissionInputs & inputs)
{
  const bool arm_rising_edge = inputs.armed && !previous_armed_;
  previous_armed_ = inputs.armed;
  apply_reset(inputs.reset);
  if (inputs.status_fresh && !inputs.armed) {
    observed_disarmed_ = true;
  }

  if ((state_ == MissionState::OFFBOARD_PRESTREAM || state_ == MissionState::REQUEST_OFFBOARD ||
       state_ == MissionState::TAKEOFF || state_ == MissionState::HOVER ||
       state_ == MissionState::RETURN_LOCAL) &&
      (!inputs.status_fresh || inputs.failsafe ||
       ((state_ == MissionState::TAKEOFF || state_ == MissionState::HOVER ||
         state_ == MissionState::RETURN_LOCAL) && !inputs.offboard))) {
    faults_ |= !inputs.status_fresh ? BOOMBOOM_FAULT_INPUT_STALE :
      (inputs.failsafe ? BOOMBOOM_FAULT_PX4_FAILSAFE : BOOMBOOM_FAULT_AUTHORITY_LOST);
    enter(MissionState::COMPLETE, inputs.now_us, !inputs.status_fresh ?
      BOOMBOOM_STATE_REASON_INPUT_STALE : (inputs.failsafe ? BOOMBOOM_STATE_REASON_FAILSAFE :
      BOOMBOOM_STATE_REASON_AUTHORITY_LOST));
  }
  if ((state_ == MissionState::OFFBOARD_PRESTREAM || state_ == MissionState::REQUEST_OFFBOARD ||
       state_ == MissionState::TAKEOFF || state_ == MissionState::HOVER ||
       state_ == MissionState::RETURN_LOCAL) &&
      (!inputs.local_position_fresh || !inputs.local_position_healthy)) {
    faults_ |= !inputs.local_position_fresh ? BOOMBOOM_FAULT_INPUT_STALE :
      BOOMBOOM_FAULT_LOCALIZATION_INVALID;
    enter(MissionState::LAND_REQUEST, inputs.now_us, BOOMBOOM_STATE_REASON_LOCALIZATION_INVALID);
  }

  MissionActions actions;
  switch (state_) {
    case MissionState::BOOT:
      enter(MissionState::WAIT_DISARMED, inputs.now_us, BOOMBOOM_STATE_REASON_STARTUP);
      break;
    case MissionState::WAIT_DISARMED:
      if (observed_disarmed_) { enter(MissionState::WAIT_LOCALIZATION, inputs.now_us); }
      break;
    case MissionState::WAIT_LOCALIZATION:
      if (!inputs.armed && inputs.status_fresh && inputs.local_position_fresh &&
          inputs.local_position_healthy) { enter(MissionState::READY, inputs.now_us, BOOMBOOM_STATE_REASON_EKF_READY); }
      break;
    case MissionState::READY:
      if (!inputs.status_fresh || !inputs.local_position_fresh || !inputs.local_position_healthy) {
        enter(MissionState::WAIT_LOCALIZATION, inputs.now_us);
      } else if (arm_rising_edge && source_is_rc(inputs.latest_arming_reason)) {
        home_ = inputs.position_ned;
        target_ = home_;
        target_[2] -= config_.takeoff_height_m;
        yaw_ = inputs.heading_rad;
        enter(MissionState::OFFBOARD_PRESTREAM, inputs.now_us, BOOMBOOM_STATE_REASON_RC_ARM_EDGE);
      }
      break;
    case MissionState::OFFBOARD_PRESTREAM:
      actions = setpoint();
      if (!inputs.armed) {
        enter(MissionState::WAIT_DISARMED, inputs.now_us);
      } else if (inputs.now_us - entered_at_us_ >=
        static_cast<std::int64_t>(config_.prestream_duration_s * kUsPerSecond)) {
        request_ack_sequence_ = inputs.ack_sequence;
        offboard_ack_accepted_ = false;
        enter(MissionState::REQUEST_OFFBOARD, inputs.now_us, BOOMBOOM_STATE_REASON_PRESTREAM_READY);
        actions = setpoint();
        actions.request_offboard = true;
      }
      break;
    case MissionState::REQUEST_OFFBOARD:
      actions = setpoint();
      if (inputs.ack_sequence > request_ack_sequence_) {
        if (inputs.offboard_ack == AckResult::ACCEPTED) {
          offboard_ack_accepted_ = true;
        } else if (inputs.offboard_ack == AckResult::REJECTED) {
          faults_ |= BOOMBOOM_FAULT_COMMAND_REJECTED;
          enter(MissionState::COMPLETE, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
          break;
        }
      }
      if (offboard_ack_accepted_ && inputs.offboard) {
        enter(MissionState::TAKEOFF, inputs.now_us, BOOMBOOM_STATE_REASON_OFFBOARD_ACCEPTED);
      } else if (inputs.now_us - entered_at_us_ >= static_cast<std::int64_t>(
          (offboard_ack_accepted_ ? config_.offboard_state_timeout_s :
          config_.offboard_ack_timeout_s) * kUsPerSecond)) {
        faults_ |= BOOMBOOM_FAULT_COMMAND_REJECTED;
        enter(MissionState::COMPLETE, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
      }
      break;
    case MissionState::TAKEOFF:
      actions = setpoint();
      if (position_stable(inputs, target_)) {
        if (stable_since_us_ < 0) { stable_since_us_ = inputs.now_us; }
        if (inputs.now_us - stable_since_us_ >=
          static_cast<std::int64_t>(config_.stable_duration_s * kUsPerSecond)) {
          enter(MissionState::HOVER, inputs.now_us, BOOMBOOM_STATE_REASON_TARGET_REACHED);
        }
      } else { stable_since_us_ = -1; }
      break;
    case MissionState::HOVER:
      actions = setpoint();
      if (inputs.cancel_requested) {
        target_[0] = home_[0];
        target_[1] = home_[1];
        enter(MissionState::RETURN_LOCAL, inputs.now_us, BOOMBOOM_STATE_REASON_CANCELLED);
        actions = setpoint();
      } else if (inputs.now_us - entered_at_us_ >=
        static_cast<std::int64_t>(config_.hover_duration_s * kUsPerSecond)) {
        target_[0] = home_[0];
        target_[1] = home_[1];
        enter(MissionState::RETURN_LOCAL, inputs.now_us, BOOMBOOM_STATE_REASON_HOVER_COMPLETE);
        actions = setpoint();
      }
      break;
    case MissionState::RETURN_LOCAL:
      actions = setpoint();
      if (position_stable(inputs, target_)) {
        enter(MissionState::LAND_REQUEST, inputs.now_us, BOOMBOOM_STATE_REASON_TARGET_REACHED);
      }
      break;
    case MissionState::LAND_REQUEST:
      actions.request_land = true;
      land_request_sequence_ = inputs.landed_sequence;
      enter(MissionState::WAIT_LANDED, inputs.now_us);
      break;
    case MissionState::WAIT_LANDED:
      if (inputs.landed_sequence > land_request_sequence_ && inputs.landed) {
        enter(MissionState::COMPLETE, inputs.now_us, BOOMBOOM_STATE_REASON_LANDED);
      }
      break;
    case MissionState::COMPLETE:
      if (inputs.status_fresh && !inputs.armed) { enter(MissionState::WAIT_DISARMED, inputs.now_us); }
      break;
  }
  return actions;
}

const char * mission_state_name(MissionState state)
{
  switch (state) {
    case MissionState::BOOT: return "BOOT";
    case MissionState::WAIT_DISARMED: return "WAIT_DISARMED";
    case MissionState::WAIT_LOCALIZATION: return "WAIT_LOCALIZATION";
    case MissionState::READY: return "READY";
    case MissionState::OFFBOARD_PRESTREAM: return "OFFBOARD_PRESTREAM";
    case MissionState::REQUEST_OFFBOARD: return "REQUEST_OFFBOARD";
    case MissionState::TAKEOFF: return "TAKEOFF";
    case MissionState::HOVER: return "HOVER";
    case MissionState::RETURN_LOCAL: return "RETURN_LOCAL";
    case MissionState::LAND_REQUEST: return "LAND_REQUEST";
    case MissionState::WAIT_LANDED: return "WAIT_LANDED";
    case MissionState::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}
}  // namespace offboard_cpp
