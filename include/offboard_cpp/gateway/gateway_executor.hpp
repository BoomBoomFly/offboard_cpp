#pragma once

#include <array>
#include <cstdint>

#include "offboard_cpp/mission/mission_types.hpp"

namespace offboard_cpp
{

// 执行器刻意不依赖 ROS：它只把 PX4 状态快照归约为动作，因而可在没有 PX4 或 Action
// server 的情况下验证命令与安全策略。ROS 节点仅负责边界适配，不能绕过这里的状态机。
enum class GatewayCommand : std::uint8_t { TAKEOFF = 0, GOTO = 1, HOLD = 2, RETURN_HOME = 3, LAND = 4 };

// WAIT_* 与 REQUEST_OFFBOARD 是安全握手阶段，不是飞行模式；只有 ACK 被接受且 PX4
// 实际报告 OFFBOARD 后，才进入 EXECUTING。LAND_REQUEST 之后的落地流程不可取消。
enum class GatewayState : std::uint8_t {
  WAIT_DISARMED, READY, WAIT_RC_ARM, OFFBOARD_PRESTREAM, REQUEST_OFFBOARD,
  EXECUTING, IDLE, LAND_REQUEST, WAIT_LANDED, TAKEOVER, FAILED
};

struct GatewayGoal {
  GatewayCommand command{GatewayCommand::TAKEOFF};
  // PX4 本地 NED：north/east/down，单位 m；yaw 为弧度，duration/timeout 为秒。
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
  // 仅未发出 LAND 的活动目标可以取消；空中取消会冻结当前位置，而非自动返航。
  bool cancel_requested() const;
  void request_cancel();
  MissionActions tick(const MissionInputs & inputs);

  GatewayState state() const { return state_; }
  GatewayCommand active_command() const { return goal_.command; }
  bool land_irreversible() const;
  // 每次完成递增，供 ROS 边界层在 timer 中恰好结算一次 Action 结果。
  std::uint64_t completion_sequence() const { return completion_sequence_; }
  GatewayResult result() const { return result_; }
  std::uint16_t state_reason() const { return state_reason_; }

private:
  void enter(GatewayState state, std::int64_t now_us, std::uint16_t reason = 0);
  void finish(bool succeeded, bool cancelled, std::uint16_t condition, std::uint16_t reason);
  void apply_reset(const PositionReset & reset);
  bool source_is_rc(std::uint8_t reason) const;
  bool controls_offboard() const;
  void activate_goal(std::int64_t now_us);
  MissionActions update_setpoint(const MissionInputs & inputs, bool * complete);
  void hold_current(const MissionInputs & inputs);

  MissionConfig config_;
  GatewayState state_{GatewayState::WAIT_DISARMED};
  GatewayGoal goal_{};
  GatewayResult result_{};
  std::int64_t entered_at_us_{};
  std::int64_t goal_started_at_us_{};
  std::int64_t active_since_us_{};
  std::int64_t stable_since_us_{-1};
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
