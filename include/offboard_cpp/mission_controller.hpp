#pragma once

#include "offboard_cpp/mission_types.hpp"

extern "C" {
#include <boomboom_common/fault.h>
#include <boomboom_common/state.h>
}

namespace offboard_cpp
{

class MissionController
{
public:
  explicit MissionController(MissionConfig config = {});
  MissionActions tick(const MissionInputs & inputs);
  MissionState state() const { return state_; }
  std::uint64_t faults() const { return faults_; }
  std::int64_t entered_at_us() const { return entered_at_us_; }
  std::uint16_t state_reason() const { return state_reason_; }

private:
  void enter(MissionState state, std::int64_t now_us, std::uint16_t reason = BOOMBOOM_STATE_REASON_NONE);
  void apply_reset(const PositionReset & reset);
  bool source_is_rc(std::uint8_t reason) const;
  bool position_stable(const MissionInputs & inputs, const std::array<double, 3> & target) const;
  MissionActions setpoint() const;

  MissionConfig config_;
  MissionState state_{MissionState::BOOT};
  std::int64_t entered_at_us_{};
  std::int64_t stable_since_us_{-1};
  bool observed_disarmed_{};
  bool previous_armed_{};
  bool have_reset_{};
  PositionReset reset_{};
  std::uint64_t request_ack_sequence_{};
  bool offboard_ack_accepted_{};
  std::array<double, 3> home_{};
  std::array<double, 3> target_{};
  double yaw_{};
  std::uint64_t faults_{};
  std::uint16_t state_reason_{BOOMBOOM_STATE_REASON_STARTUP};
};

const char * mission_state_name(MissionState state);

}  // namespace offboard_cpp
