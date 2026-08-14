#include "offboard_cpp/gateway/offboard_gateway_node.hpp"

#include <chrono>

#include "offboard_cpp/gateway/gateway_executor.hpp"
#include "offboard_cpp/px4/px4_interface.hpp"

extern "C" {
#include <boomboom_common/state.h>
#include <boomboom_common/status.h>
}

namespace offboard_cpp
{
namespace
{
std::int64_t steady_now_us()
{
  // 状态机超时使用单调时钟，主机 NTP/WSL 校时不能缩短或延长安全窗口。
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

GatewayGoal to_gateway_goal(const boomboom_common::action::ExecuteFlight::Goal & goal)
{
  GatewayGoal converted;
  converted.command = static_cast<GatewayCommand>(goal.command);
  converted.target_ned = {goal.target.north_m, goal.target.east_m, goal.target.down_m};
  converted.yaw_rad = goal.target.yaw_rad;
  converted.duration_s = goal.duration_s;
  converted.timeout_s = goal.timeout_s;
  return converted;
}

}  // namespace

OffboardGatewayNode::OffboardGatewayNode() : Node("offboard_gateway_node")
{
  MissionConfig config;
  config.takeoff_height_m = declare_parameter<double>("mission.takeoff_height_m", 1.5);
  config.prestream_duration_s = declare_parameter<double>("mission.prestream_duration_s", 1.0);
  config.offboard_ack_timeout_s = declare_parameter<double>("mission.offboard_ack_timeout_s", 2.0);
  config.offboard_state_timeout_s = declare_parameter<double>("mission.offboard_state_timeout_s", 2.0);
  config.position_tolerance_m = declare_parameter<double>("mission.position_tolerance_m", 0.20);
  config.velocity_tolerance_mps = declare_parameter<double>("mission.velocity_tolerance_mps", 0.15);
  config.stable_duration_s = declare_parameter<double>("mission.stable_duration_s", 1.0);
  executor_ = std::make_unique<GatewayExecutor>(config);
  px4_ = std::make_unique<Px4Interface>(*this);
  state_pub_ = create_publisher<boomboom_common::msg::FlightState>("/boomboom/flight_state", 10);
  action_server_ = rclcpp_action::create_server<ExecuteFlight>(
    this, "/offboard/execute_flight",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteFlight::Goal> goal) {
      return handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandle> goal_handle) { return handle_cancel(goal_handle); },
    [this](const std::shared_ptr<GoalHandle> goal_handle) { handle_accepted(goal_handle); });
  // 50 ms 即 20 Hz；预流、状态机 tick 和 /fmu/in 发送在同一串行 timer 路径中闭环。
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { on_timer(); });
}

OffboardGatewayNode::~OffboardGatewayNode() = default;

std::uint8_t OffboardGatewayNode::flight_state_value(GatewayState state, GatewayCommand command)
{
  using FlightState = boomboom_common::msg::FlightState;
  switch (state) {
    case GatewayState::WAIT_DISARMED: return FlightState::IDLE;
    case GatewayState::READY:
    case GatewayState::WAIT_RC_ARM: return FlightState::READY;
    case GatewayState::OFFBOARD_PRESTREAM:
    case GatewayState::REQUEST_OFFBOARD:
    case GatewayState::EXECUTING:
      return command == GatewayCommand::HOLD ? FlightState::HOLDING : FlightState::EXECUTING;
    case GatewayState::IDLE: return FlightState::HOLDING;
    case GatewayState::LAND_REQUEST:
    case GatewayState::WAIT_LANDED: return FlightState::LANDING;
    case GatewayState::TAKEOVER:
    case GatewayState::FAILED: return FlightState::FAILED;
  }
  return FlightState::UNKNOWN;
}

rclcpp_action::GoalResponse OffboardGatewayNode::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteFlight::Goal> goal)
{
  if (action_slot_reserved_ || !executor_->can_accept(to_gateway_goal(*goal))) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  action_slot_reserved_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffboardGatewayNode::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  if (active_goal_ != goal_handle || !executor_->cancel_requested()) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  executor_->request_cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OffboardGatewayNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  if (!executor_->start(to_gateway_goal(*goal_handle->get_goal()), steady_now_us())) {
    auto result = std::make_shared<ExecuteFlight::Result>();
    result->status.domain = BOOMBOOM_STATUS_DOMAIN_COMMAND;
    result->status.condition = BOOMBOOM_STATUS_CONDITION_REJECTED;
    goal_handle->abort(result);
    action_slot_reserved_ = false;
    return;
  }
  active_goal_ = goal_handle;
}

boomboom_common::msg::FlightState OffboardGatewayNode::flight_state()
{
  boomboom_common::msg::FlightState state;
  state.stamp = get_clock()->now();
  state.state = flight_state_value(executor_->state(), executor_->active_command());
  state.reason = executor_->state_reason();
  state.armed = last_inputs_.armed;
  state.offboard = last_inputs_.offboard;
  state.localization_healthy = last_inputs_.local_position_fresh && last_inputs_.local_position_healthy;
  state.failsafe = last_inputs_.failsafe;
  state.pose.north_m = last_inputs_.position_ned[0];
  state.pose.east_m = last_inputs_.position_ned[1];
  state.pose.down_m = last_inputs_.position_ned[2];
  state.pose.yaw_rad = last_inputs_.heading_rad;
  return state;
}

void OffboardGatewayNode::finish_active_goal()
{
  if (!active_goal_) { return; }
  const auto gateway_result = executor_->result();
  auto result = std::make_shared<ExecuteFlight::Result>();
  result->status.domain = BOOMBOOM_STATUS_DOMAIN_COMMAND;
  result->status.condition = gateway_result.condition;
  result->status.detail = static_cast<std::uint32_t>(gateway_result.reason);
  result->final_state = flight_state();
  // Action 终态只在 completion_sequence 变化后结算，避免 timer 持续 tick 重复回调。
  if (gateway_result.cancelled) {
    active_goal_->canceled(result);
  } else if (gateway_result.succeeded) {
    active_goal_->succeed(result);
  } else {
    active_goal_->abort(result);
  }
  active_goal_.reset();
  action_slot_reserved_ = false;
}

void OffboardGatewayNode::on_timer()
{
  const auto now_us = steady_now_us();
  last_inputs_ = px4_->snapshot(now_us);
  const auto actions = executor_->tick(last_inputs_);
  const auto state = flight_state();
  state_pub_->publish(state);
  if (active_goal_) {
    auto feedback = std::make_shared<ExecuteFlight::Feedback>();
    feedback->state = state;
    active_goal_->publish_feedback(feedback);
  }
  if (executor_->completion_sequence() != observed_completion_sequence_) {
    observed_completion_sequence_ = executor_->completion_sequence();
    finish_active_goal();
  }
  // 先归约并发布可观测状态/Action 结果，再由唯一接口写入 PX4，保证写入决策可追溯。
  px4_->publish(actions, now_us);
}

}  // namespace offboard_cpp
