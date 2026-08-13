#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <boomboom_common/action/execute_flight.hpp>
#include <boomboom_common/msg/flight_state.hpp>

#include "offboard_cpp/gateway_executor.hpp"
#include "offboard_cpp/gateway_state_mapping.hpp"
#include "offboard_cpp/mission_types.hpp"

namespace offboard_cpp
{
class Px4Interface;

class OffboardGatewayNode : public rclcpp::Node
{
public:
  OffboardGatewayNode();
  ~OffboardGatewayNode() override;

private:
  using ExecuteFlight = boomboom_common::action::ExecuteFlight;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteFlight>;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteFlight::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void on_timer();
  boomboom_common::msg::FlightState flight_state();
  void finish_active_goal();

  std::unique_ptr<GatewayExecutor> executor_;
  std::unique_ptr<Px4Interface> px4_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<boomboom_common::msg::FlightState>::SharedPtr state_pub_;
  rclcpp_action::Server<ExecuteFlight>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  MissionInputs last_inputs_{};
  bool action_slot_reserved_{};
  std::uint64_t observed_completion_sequence_{};
};
}  // namespace offboard_cpp
