#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <boomboom_common/action/execute_flight.hpp>
#include <boomboom_common/msg/flight_state.hpp>

#include "offboard_cpp/gateway/gateway_executor.hpp"
#include "offboard_cpp/mission/mission_types.hpp"

namespace offboard_cpp
{
class Px4Interface;

class OffboardGatewayNode : public rclcpp::Node
{
public:
  OffboardGatewayNode();
  ~OffboardGatewayNode() override;

  static std::uint8_t flight_state_value(GatewayState state, GatewayCommand command);

private:
  using ExecuteFlight = boomboom_common::action::ExecuteFlight;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteFlight>;

  // Action 回调先预留唯一槽位，防止在 accepted 回调启动执行器前并发目标都通过检查。
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
  // 网关一次只驱动一个 PX4 设定点流，因此 Action 也严格一对一。
  std::shared_ptr<GoalHandle> active_goal_;
  MissionInputs last_inputs_{};
  bool action_slot_reserved_{};
  std::uint64_t observed_completion_sequence_{};
};
}  // namespace offboard_cpp
