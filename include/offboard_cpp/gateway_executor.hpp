#pragma once

#include <array>
#include <cstdint>

#include "offboard_cpp/mission_mode.hpp"
#include "offboard_cpp/mission_types.hpp"

namespace offboard_cpp
{

// This executor deliberately stays ROS-free so its command and safety policy can
// be exercised without a PX4 or an action server.
enum class GatewayCommand : std::uint8_t { TAKEOFF = 0, GOTO = 1, HOLD = 2, RETURN_HOME = 3, LAND = 4 };

enum class GatewayState : std::uint8_t {
  WAIT_DISARMED, READY, WAIT_RC_ARM, OFFBOARD_PRESTREAM, REQUEST_OFFBOARD,
  EXECUTING, IDLE, LAND_REQUEST, WAIT_LANDED, TAKEOVER, FAILED
};

struct GatewayGoal {
  GatewayCommand command{GatewayCommand::TAKEOFF};
  std::array<double, 3> target_ned{};
  double yaw_rad{};
  double duration_s{};
  double timeout_s{};
};

struct GatewayResult {
  bool complete{};
  bool succeeded{};
  bool cancelled{};
  std::uint16_t reason{};
  std::uint16_t condition{};
};

class GatewayExecutor
{
public:
  explicit GatewayExecutor(MissionConfig config = {});

  static bool valid_goal(const GatewayGoal & goal);
  bool can_accept(const GatewayGoal & goal) const;
  bool start(const GatewayGoal & goal, std::int64_t now_us);
  bool cancel_requested() const;
  void request_cancel();
  MissionActions tick(const MissionInputs & inputs);

  GatewayState state() const { return state_; }
  GatewayCommand active_command() const { return goal_.command; }
  bool land_irreversible() const;
  std::uint64_t completion_sequence() const { return completion_sequence_; }
  GatewayResult result() const { return result_; }
  std::uint16_t state_reason() const { return state_reason_; }

private:
  void enter(GatewayState state, std::int64_t now_us, std::uint16_t reason = 0);
  void finish(bool succeeded, bool cancelled, std::uint16_t condition, std::uint16_t reason);
  void apply_reset(const PositionReset & reset);
  bool source_is_rc(std::uint8_t reason) const;
  bool controls_offboard() const;
  ModeBase * make_active_mode(const GatewayGoal & goal);
  MissionActions update_mode(const MissionInputs & inputs, bool * complete);
  void hold_current(const MissionInputs & inputs);

  MissionConfig config_;
  ReachPositionMode reach_mode_;
  HoldPositionMode hold_mode_{0.0};
  ModeBase * active_mode_{};
  GatewayState state_{GatewayState::WAIT_DISARMED};
  GatewayGoal goal_{};
  GatewayResult result_{};
  std::int64_t entered_at_us_{};
  std::int64_t goal_started_at_us_{};
  bool observed_disarmed_{};
  bool previous_armed_{};
  bool command_active_{};
  bool cancel_pending_{};
  bool have_reset_{};
  PositionReset reset_{};
  std::array<double, 3> home_{};
  std::array<double, 3> target_{};
  double yaw_{};
  std::uint64_t request_ack_sequence_{};
  bool offboard_ack_accepted_{};
  std::int64_t offboard_ack_accepted_at_us_{};
  std::uint64_t land_request_sequence_{};
  std::uint64_t completion_sequence_{};
  std::uint16_t state_reason_{};
};

}  // namespace offboard_cpp
