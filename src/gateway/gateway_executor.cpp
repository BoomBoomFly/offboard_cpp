#include "offboard_cpp/gateway/gateway_executor.hpp"

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
: config_(config)
{}

bool GatewayExecutor::valid_goal(const GatewayGoal & goal)
{
  return static_cast<std::uint8_t>(goal.command) <= static_cast<std::uint8_t>(GatewayCommand::LAND) &&
         finite(goal) && goal.duration_s >= 0.0 && goal.timeout_s >= 0.0;
}

bool GatewayExecutor::can_accept(const GatewayGoal & goal) const
{
  // TAKEOFF 是唯一可从地面状态开始的命令；后续指令必须建立在已维持的 Offboard
  // 控制流上，且起飞、降落、接管和故障均不能与另一个 Action 交叠。
  if (!valid_goal(goal) || command_active_ || land_irreversible() ||
      state_ == GatewayState::TAKEOVER || state_ == GatewayState::FAILED)
  {
    return false;
  }
  if (goal.command == GatewayCommand::TAKEOFF)
  {
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
    activate_goal(now_us);
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
    // PX4 报告的是同一物理位置在新 NED 原点下的增量，home 与当前目标必须同行平移。
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

void GatewayExecutor::activate_goal(std::int64_t now_us)
{
  active_since_us_ = now_us;
  stable_since_us_ = -1;
}

MissionActions GatewayExecutor::update_setpoint(const MissionInputs & inputs, bool * complete)
{
  MissionActions actions;
  actions.publish_setpoint = true;
  actions.position_ned = target_;
  actions.yaw_rad = yaw_;
  if (goal_.command == GatewayCommand::HOLD) {
    *complete = inputs.now_us - active_since_us_ >=
      static_cast<std::int64_t>(goal_.duration_s * kUsPerSecond);
    return actions;
  }

  const double dx = inputs.position_ned[0] - target_[0];
  const double dy = inputs.position_ned[1] - target_[1];
  const double dz = inputs.position_ned[2] - target_[2];
  const double speed = std::sqrt(
    inputs.velocity_ned[0] * inputs.velocity_ned[0] +
    inputs.velocity_ned[1] * inputs.velocity_ned[1] +
    inputs.velocity_ned[2] * inputs.velocity_ned[2]);
  // GOTO/RETURN_HOME 需要位置和速度连续满足门限，瞬时穿越目标不会完成任务。
  const bool stable = std::sqrt(dx * dx + dy * dy + dz * dz) <= config_.position_tolerance_m &&
    speed <= config_.velocity_tolerance_mps;
  if (!stable) {
    stable_since_us_ = -1;
  } else if (stable_since_us_ < 0) {
    stable_since_us_ = inputs.now_us;
  }
  *complete = stable_since_us_ >= 0 && inputs.now_us - stable_since_us_ >=
    static_cast<std::int64_t>(config_.stable_duration_s * kUsPerSecond);
  return actions;
}

void GatewayExecutor::hold_current(const MissionInputs & inputs)
{
  target_ = inputs.position_ned;
  yaw_ = inputs.heading_rad;
}

MissionActions GatewayExecutor::tick(const MissionInputs & inputs)
{
  const bool arm_rising_edge = inputs.armed && !previous_armed_;
  previous_armed_ = inputs.armed;
  apply_reset(inputs.reset);
  if (inputs.status_fresh && !inputs.armed) { observed_disarmed_ = true; }

  // PX4/RC 控制权和定位安全门禁优先于 Action 取消和超时：失去控制权不自动夺回，
  // 定位失效则请求原地降落，避免在未知位置继续发送设定点。
  if (controls_offboard() && (!inputs.status_fresh || inputs.failsafe ||
      !inputs.armed ||
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
    if (controls_offboard() && inputs.offboard) {
      // 空中可取消命令的安全语义是原地 HOLD；此处明确不改写为 RETURN_HOME。
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
      if (!inputs.status_fresh || inputs.failsafe || !inputs.local_position_fresh ||
        !inputs.local_position_healthy) {
        break;
      }
      if (observed_disarmed_ && arm_rising_edge && source_is_rc(inputs.latest_arming_reason)) {
        // 只接受已观测 DISARMED 之后的 RC 解锁上升沿；程序从不发送 ARM/DISARM。
        home_ = inputs.position_ned;
        target_ = home_;
        // NED 的 down 轴向下为正，起飞需减小 z。
        target_[2] -= config_.takeoff_height_m;
        yaw_ = inputs.heading_rad;
        enter(GatewayState::OFFBOARD_PRESTREAM, inputs.now_us, BOOMBOOM_STATE_REASON_RC_ARM_EDGE);
        if (!inputs.timesync_fresh) { entered_at_us_ = -1; }
        // 有时间同步时，解锁沿的 tick 才是第一帧预流。
        actions.publish_setpoint = true;
        actions.position_ned = target_;
        actions.yaw_rad = yaw_;
      }
      break;
    case GatewayState::OFFBOARD_PRESTREAM:
      if (!inputs.timesync_fresh) {
        // 未实际发送的时间不能累计为预流；恢复同步后重新积累完整窗口。
        entered_at_us_ = -1;
        break;
      }
      if (entered_at_us_ < 0) { entered_at_us_ = inputs.now_us; }
      actions.publish_setpoint = true;
      actions.position_ned = target_;
      actions.yaw_rad = yaw_;
      if (inputs.now_us - entered_at_us_ >=
        static_cast<std::int64_t>(config_.prestream_duration_s * kUsPerSecond)) {
        request_ack_sequence_ = inputs.ack_sequence;
        offboard_ack_accepted_ = false;
        offboard_ack_accepted_at_us_ = 0;
        enter(GatewayState::REQUEST_OFFBOARD, inputs.now_us, BOOMBOOM_STATE_REASON_PRESTREAM_READY);
        actions.request_offboard = true;
      }
      break;
    case GatewayState::REQUEST_OFFBOARD:
      actions.publish_setpoint = true;
      actions.position_ned = target_;
      actions.yaw_rad = yaw_;
      if (inputs.ack_sequence > request_ack_sequence_)
      {
        if (inputs.offboard_ack == AckResult::ACCEPTED && !offboard_ack_accepted_)
        {
          offboard_ack_accepted_ = true;
          offboard_ack_accepted_at_us_ = inputs.now_us;
        }
        else if (inputs.offboard_ack == AckResult::REJECTED)
        {
          finish(false, false, BOOMBOOM_STATUS_CONDITION_REJECTED, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
          enter(GatewayState::FAILED, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
        }
      }
      if (state_ == GatewayState::REQUEST_OFFBOARD && offboard_ack_accepted_ && inputs.offboard)
      {
        // ACK 仅表示 PX4 接受命令；必须再观察到实际 OFFBOARD 状态才允许执行起飞。
        activate_goal(inputs.now_us);
        enter(GatewayState::EXECUTING, inputs.now_us, BOOMBOOM_STATE_REASON_OFFBOARD_ACCEPTED);
      }
      else if (state_ == GatewayState::REQUEST_OFFBOARD &&
        inputs.now_us - (offboard_ack_accepted_ ? offboard_ack_accepted_at_us_ : entered_at_us_) >=
        static_cast<std::int64_t>((offboard_ack_accepted_ ? config_.offboard_state_timeout_s :
          config_.offboard_ack_timeout_s) * kUsPerSecond))
      {
        finish(false, false, BOOMBOOM_STATUS_CONDITION_TIMEOUT, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
        enter(GatewayState::FAILED, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
      }
      break;

    case GatewayState::EXECUTING:
    {
      bool complete = false;
      actions = update_setpoint(inputs, &complete);
      if (complete)
      {
        finish(true, false, BOOMBOOM_STATUS_CONDITION_COMPLETE, BOOMBOOM_STATE_REASON_TARGET_REACHED);
        enter(GatewayState::IDLE, inputs.now_us, BOOMBOOM_STATE_REASON_TARGET_REACHED);
      }
      break;
    }
    case GatewayState::IDLE:
      actions.publish_setpoint = true;
      actions.position_ned = target_;
      actions.yaw_rad = yaw_;
      break;
    case GatewayState::LAND_REQUEST:
    case GatewayState::WAIT_LANDED:
    {
      const bool sent = state_ == GatewayState::WAIT_LANDED;
      const bool accepted = sent && inputs.land_ack_sequence > land_ack_sequence_ &&
        inputs.land_ack == AckResult::ACCEPTED;
      if (!inputs.status_fresh || inputs.failsafe ||
        (!inputs.offboard && !inputs.auto_land && inputs.armed))
      {
        if (command_active_) {
          finish(false, false, BOOMBOOM_STATUS_CONDITION_LOST, BOOMBOOM_STATE_REASON_AUTHORITY_LOST);
        }
        enter(GatewayState::TAKEOVER, inputs.now_us, BOOMBOOM_STATE_REASON_AUTHORITY_LOST);
      } else if (sent && (accepted || inputs.auto_land) &&
        inputs.landed_sequence > land_request_sequence_ && inputs.landed)
      {
        if (command_active_) {
          finish(true, false, BOOMBOOM_STATUS_CONDITION_COMPLETE, BOOMBOOM_STATE_REASON_LANDED);
        }
        enter(GatewayState::WAIT_DISARMED, inputs.now_us, BOOMBOOM_STATE_REASON_LANDED);
      } else if (sent && inputs.land_ack_sequence > land_ack_sequence_ &&
        inputs.land_ack == AckResult::REJECTED)
      {
        if (command_active_) {
          finish(false, false, BOOMBOOM_STATUS_CONDITION_REJECTED, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
        }
        enter(GatewayState::FAILED, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
      } else if (!inputs.armed) {
        if (command_active_) {
          finish(false, false, BOOMBOOM_STATUS_CONDITION_LOST, BOOMBOOM_STATE_REASON_AUTHORITY_LOST);
        }
        enter(GatewayState::TAKEOVER, inputs.now_us, BOOMBOOM_STATE_REASON_AUTHORITY_LOST);
      } else if ((inputs.now_us - entered_at_us_ >= static_cast<std::int64_t>(
          (sent && (accepted || inputs.auto_land) ? config_.land_timeout_s :
          config_.land_ack_timeout_s) * kUsPerSecond)) ||
        (command_active_ && goal_.timeout_s > 0.0 && inputs.now_us - goal_started_at_us_ >=
          static_cast<std::int64_t>(goal_.timeout_s * kUsPerSecond)))
      {
        if (command_active_) {
          finish(false, false, BOOMBOOM_STATUS_CONDITION_TIMEOUT, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
        }
        enter(GatewayState::FAILED, inputs.now_us, BOOMBOOM_STATE_REASON_COMMAND_REJECTED);
      } else if (!sent) {
        actions.request_land = true;
      }
      break;
    }
    case GatewayState::TAKEOVER:
    case GatewayState::FAILED:
      break;
  }
  // 本 tick 内拒绝/超时后也立即停止预先构造的 setpoint。
  if (!controls_offboard()) { actions.publish_setpoint = false; }
  return actions;
}

void GatewayExecutor::land_sent(const MissionInputs & inputs)
{
  if (state_ != GatewayState::LAND_REQUEST) { return; }
  land_request_sequence_ = inputs.landed_sequence;
  land_ack_sequence_ = inputs.land_ack_sequence;
  enter(GatewayState::WAIT_LANDED, inputs.now_us, state_reason_);
}

}  // namespace offboard_cpp
