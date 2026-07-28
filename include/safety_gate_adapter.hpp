#ifndef OFFBOARD_CPP_SAFETY_GATE_ADAPTER_HPP
#define OFFBOARD_CPP_SAFETY_GATE_ADAPTER_HPP

#include <memory>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <safety_gate.hpp>

namespace offboard_cpp
{

// The only class in the package permitted to own PX4 control-input publishers.
class SafetyGateAdapter
{
public:
  SafetyGateAdapter(rclcpp::Node & node, const rclcpp::QoS & qos);
  void apply(
    const GateDecision & decision,
    const px4_msgs::msg::TrajectorySetpoint & setpoint,
    const px4_msgs::msg::OffboardControlMode & mode,
    std::int64_t timestamp_us);

private:
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
};

}  // namespace offboard_cpp

#endif  // OFFBOARD_CPP_SAFETY_GATE_ADAPTER_HPP
