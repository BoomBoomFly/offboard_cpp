#pragma once

#include <array>
#include <cstdint>

namespace offboard_cpp
{

enum class MissionState : std::uint8_t {
  BOOT, WAIT_DISARMED, WAIT_LOCALIZATION, READY, OFFBOARD_PRESTREAM,
  REQUEST_OFFBOARD, TAKEOFF, HOVER, RETURN_LOCAL, LAND_REQUEST,
  WAIT_LANDED, COMPLETE
};

enum class AckResult : std::uint8_t { NONE, ACCEPTED, REJECTED };

struct PositionReset {
  std::uint8_t xy_counter{};
  std::uint8_t z_counter{};
  std::uint8_t heading_counter{};
  float delta_x{};
  float delta_y{};
  float delta_z{};
  float delta_heading{};
};

struct MissionConfig {
  double takeoff_height_m{1.5};
  double hover_duration_s{60.0};
  double prestream_duration_s{1.0};
  double offboard_ack_timeout_s{2.0};
  double offboard_state_timeout_s{2.0};
  double position_tolerance_m{0.20};
  double velocity_tolerance_mps{0.15};
  double stable_duration_s{1.0};
};

struct MissionInputs {
  std::int64_t now_us{};
  bool status_fresh{};
  bool local_position_fresh{};
  bool local_position_healthy{};
  bool armed{};
  bool offboard{};
  bool failsafe{};
  bool cancel_requested{};
  bool landed{};
  std::uint64_t landed_sequence{};
  std::uint8_t latest_arming_reason{};
  std::array<double, 3> position_ned{};
  std::array<double, 3> velocity_ned{};
  double heading_rad{};
  PositionReset reset{};
  std::uint64_t ack_sequence{};
  AckResult offboard_ack{AckResult::NONE};
};

struct MissionActions {
  bool publish_setpoint{};
  std::array<double, 3> position_ned{};
  double yaw_rad{};
  bool request_offboard{};
  bool request_land{};
};

}  // namespace offboard_cpp
