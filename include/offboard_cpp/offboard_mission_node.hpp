#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <boomboom_common/msg/event.hpp>
#include <boomboom_common/msg/faults.hpp>
#include <boomboom_common/msg/state.hpp>
#include <boomboom_common/msg/status.hpp>

namespace offboard_cpp
{
class MissionController;
class Px4Interface;

class OffboardMissionNode : public rclcpp::Node
{
public:
  OffboardMissionNode();
  ~OffboardMissionNode() override;

private:
  void on_timer();
  std::unique_ptr<MissionController> controller_;
  std::unique_ptr<Px4Interface> px4_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<boomboom_common::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<boomboom_common::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<boomboom_common::msg::Faults>::SharedPtr faults_pub_;
  rclcpp::Publisher<boomboom_common::msg::Event>::SharedPtr event_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  bool cancel_requested_{};
  std::uint64_t event_sequence_{};
  int last_state_{-1};
};
}  // namespace offboard_cpp
