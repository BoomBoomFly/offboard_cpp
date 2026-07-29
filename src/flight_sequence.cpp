#include <flight_sequence.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace offboard_cpp
{

FlightSequence::FlightSequence(FlightConfig config)
: config_(std::move(config))
{
}

void FlightSequence::enter(MissionState next, std::int64_t now_ns)
{
  state_ = next;
  entered_ns_ = now_ns;
  last_request_ = MissionRequest::NONE;
}

bool FlightSequence::close_to(
  const std::array<double, 3> & a, const std::array<double, 3> & b) const
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= config_.position_tolerance;
}

void FlightSequence::move_toward(
  std::array<double, 3> target, double speed, double dt)
{
  const double dx = target[0] - command_[0];
  const double dy = target[1] - command_[1];
  const double dz = target[2] - command_[2];
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double step = std::max(0.0, speed) * std::max(0.0, dt);
  if (distance <= step || distance == 0.0) {
    command_ = target;
    return;
  }
  command_[0] += dx * step / distance;
  command_[1] += dy * step / distance;
  command_[2] += dz * step / distance;
}

MissionRequest FlightSequence::once(MissionRequest request)
{
  if (last_request_ == request) {
    return MissionRequest::NONE;
  }
  last_request_ = request;
  return request;
}

FlightOutput FlightSequence::tick(const FlightInputs & inputs)
{
  double dt = 0.0;
  if (previous_ns_ >= 0 && inputs.now_ns >= previous_ns_) {
    dt = std::min(0.1, static_cast<double>(inputs.now_ns - previous_ns_) / 1.0e9);
  }
  previous_ns_ = inputs.now_ns;

  if (inputs.car_target_fresh) {
    last_car_target_ = inputs.car_target;
    have_car_target_ = true;
  }
  if (state_ != MissionState::WAIT_START &&
    (!inputs.odometry_fresh || !inputs.vehicle_status_fresh))
  {
    return FlightOutput{state_, false, command_, config_.yaw, MissionRequest::NONE};
  }

  MissionRequest request = MissionRequest::NONE;
  switch (state_) {
    case MissionState::WAIT_START:
      start_authorized_ = start_authorized_ || inputs.start;
      if (inputs.odometry_fresh) {
        command_ = inputs.position;
      }
      if (start_authorized_ && inputs.armed && inputs.odometry_fresh &&
        inputs.vehicle_status_fresh) {
        home_ = inputs.position;
        command_ = inputs.position;
        have_home_ = true;
        enter(MissionState::TAKEOFF, inputs.now_ns);
      }
      break;
    case MissionState::TAKEOFF: {
        auto target = home_;
        target[2] = config_.home_surface_z - config_.takeoff_height;
        move_toward(target, config_.takeoff_speed, dt);
        if (close_to(inputs.position, target)) {
          enter(
            config_.task == MissionTask::TASK2 ? MissionState::ACQUIRE_CAR :
            MissionState::HOVER_3S, inputs.now_ns);
        }
        break;
      }
    case MissionState::HOVER_3S:
      if (inputs.now_ns - entered_ns_ >=
        static_cast<std::int64_t>(config_.hover_seconds * 1.0e9))
      {
        enter(
          config_.task == MissionTask::VERTICAL_TEST ? MissionState::HOME_DESCEND :
          MissionState::ACQUIRE_CAR, inputs.now_ns);
      }
      break;
    case MissionState::ACQUIRE_CAR:
      if (have_car_target_) {
        enter(MissionState::FOLLOW, inputs.now_ns);
      }
      break;
    case MissionState::FOLLOW:
      if (have_car_target_) {
        auto target = last_car_target_;
        target[0] += config_.follow_offset_x;
        target[1] += config_.follow_offset_y;
        target[2] = config_.home_surface_z - config_.takeoff_height;
        move_toward(target, config_.cruise_speed, dt);
      }
      if (inputs.follow_complete) {
        enter(
          config_.task == MissionTask::TASK1 ? MissionState::DROP :
          MissionState::PLATFORM_ALIGN, inputs.now_ns);
      }
      break;
    case MissionState::DROP:
      if (inputs.drop_complete) {
        enter(MissionState::RETURN_HOME, inputs.now_ns);
      }
      break;
    case MissionState::RETURN_HOME: {
        auto target = home_;
        target[2] = config_.home_surface_z - config_.takeoff_height;
        move_toward(target, config_.cruise_speed, dt);
        if (close_to(inputs.position, target)) {
          enter(
            config_.task == MissionTask::TASK1 ? MissionState::HOME_DESCEND :
            MissionState::HOME_LAND, inputs.now_ns);
        }
        break;
      }
    case MissionState::HOME_DESCEND:
      move_toward(
        {home_[0], home_[1], config_.home_surface_z},
        config_.home_land_speed, dt);
      if (std::abs(inputs.position[2] - config_.home_surface_z) <= config_.position_tolerance) {
        request = once(MissionRequest::LAND_HOME);
      }
      if (inputs.landing_confirmed) {
        enter(MissionState::LAND_CONFIRMED, inputs.now_ns);
      }
      break;
    case MissionState::LAND_CONFIRMED:
      request = once(MissionRequest::DISARM);
      if (!inputs.armed) {
        enter(MissionState::DISARMED, inputs.now_ns);
      }
      break;
    case MissionState::DISARMED:
      enter(MissionState::COMPLETE, inputs.now_ns);
      break;
    case MissionState::PLATFORM_ALIGN: {
        if (have_car_target_) {
          auto target = last_car_target_;
          target[0] += config_.follow_offset_x;
          target[1] += config_.follow_offset_y;
          target[2] = config_.platform_surface_z - config_.takeoff_height;
          move_toward(target, config_.cruise_speed, dt);
          if (close_to(inputs.position, target)) {
            enter(MissionState::PLATFORM_DESCEND, inputs.now_ns);
          }
        }
        break;
      }
    case MissionState::PLATFORM_DESCEND:
      move_toward(
        {command_[0], command_[1], config_.platform_surface_z},
        config_.platform_land_speed, dt);
      if (std::abs(inputs.position[2] - config_.platform_surface_z) <=
        config_.position_tolerance)
      {
        request = once(MissionRequest::LAND_PLATFORM);
      }
      if (inputs.landing_confirmed) {
        enter(MissionState::PLATFORM_LANDED, inputs.now_ns);
      }
      break;
    case MissionState::PLATFORM_LANDED:
      request = once(MissionRequest::DISARM);
      if (!inputs.armed) {
        enter(MissionState::HOLD_5S, inputs.now_ns);
      }
      break;
    case MissionState::HOLD_5S:
      if (inputs.now_ns - entered_ns_ >=
        static_cast<std::int64_t>(config_.platform_hold_seconds * 1.0e9))
      {
        enter(MissionState::REARM, inputs.now_ns);
      }
      break;
    case MissionState::REARM:
      request = once(MissionRequest::REARM);
      if (inputs.armed) {
        enter(MissionState::PLATFORM_TAKEOFF, inputs.now_ns);
      }
      break;
    case MissionState::PLATFORM_TAKEOFF: {
        auto target = command_;
        target[2] = config_.platform_surface_z - config_.takeoff_height;
        move_toward(target, config_.takeoff_speed, dt);
        if (close_to(inputs.position, target)) {
          enter(MissionState::RETURN_HOME, inputs.now_ns);
        }
        break;
      }
    case MissionState::HOME_LAND:
      move_toward(
        {home_[0], home_[1], config_.home_surface_z},
        config_.home_land_speed, dt);
      if (!inputs.landing_confirmed) {
        if (std::abs(inputs.position[2] - config_.home_surface_z) <=
          config_.position_tolerance)
        {
          request = once(MissionRequest::LAND_HOME);
        }
      } else {
        request = once(MissionRequest::DISARM);
        if (!inputs.armed) {
          enter(MissionState::COMPLETE, inputs.now_ns);
        }
      }
      break;
    case MissionState::COMPLETE:
      break;
  }

  return FlightOutput{
    state_, inputs.odometry_fresh, command_, config_.yaw, request};
}

const char * mission_state_name(MissionState state)
{
  switch (state) {
    case MissionState::WAIT_START: return "WAIT_START";
    case MissionState::TAKEOFF: return "TAKEOFF";
    case MissionState::HOVER_3S: return "HOVER_3S";
    case MissionState::ACQUIRE_CAR: return "ACQUIRE_CAR";
    case MissionState::FOLLOW: return "FOLLOW";
    case MissionState::DROP: return "DROP";
    case MissionState::RETURN_HOME: return "RETURN_HOME";
    case MissionState::HOME_DESCEND: return "HOME_DESCEND";
    case MissionState::LAND_CONFIRMED: return "LAND_CONFIRMED";
    case MissionState::DISARMED: return "DISARMED";
    case MissionState::PLATFORM_ALIGN: return "PLATFORM_ALIGN";
    case MissionState::PLATFORM_DESCEND: return "PLATFORM_DESCEND";
    case MissionState::PLATFORM_LANDED: return "PLATFORM_LANDED";
    case MissionState::HOLD_5S: return "HOLD_5S";
    case MissionState::REARM: return "REARM";
    case MissionState::PLATFORM_TAKEOFF: return "PLATFORM_TAKEOFF";
    case MissionState::HOME_LAND: return "HOME_LAND";
    case MissionState::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}

}  // namespace offboard_cpp
