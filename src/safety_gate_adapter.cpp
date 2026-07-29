#include <safety_gate_adapter.hpp>

namespace offboard_cpp
{
SafetyGateAdapter::SafetyGateAdapter(rclcpp::Node & node, const rclcpp::QoS & qos)
{
  setpoint_publisher_ = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "fmu/in/trajectory_setpoint", qos);
  mode_publisher_ = node.create_publisher<px4_msgs::msg::OffboardControlMode>(
    "fmu/in/offboard_control_mode", qos);
  command_publisher_ = node.create_publisher<px4_msgs::msg::VehicleCommand>(
    "fmu/in/vehicle_command", qos);
}

void SafetyGateAdapter::apply(
  const GateDecision & decision,
  const px4_msgs::msg::TrajectorySetpoint & setpoint,
  const px4_msgs::msg::OffboardControlMode & mode,
  std::int64_t timestamp_us)
{
  // Gate decisions are the single publication authority.  A rejected decision
  // is intentionally indistinguishable from no writer activity.
  if (decision.publish_setpoint) {
    auto message = setpoint;
    message.timestamp = timestamp_us;
    setpoint_publisher_->publish(message);
  }
  if (decision.publish_mode) {
    auto message = mode;
    message.timestamp = timestamp_us;
    mode_publisher_->publish(message);
  }
  if (decision.command == CommandKind::NONE) {
    return;
  }

  px4_msgs::msg::VehicleCommand command{};
  command.timestamp = timestamp_us;
  command.target_system = SafetyGate::kTargetSystem;
  command.target_component = SafetyGate::kTargetComponent;
  command.source_system = SafetyGate::kTargetSystem;
  command.source_component = SafetyGate::kTargetComponent;
  command.from_external = true;
  // This is the first transmission.  VehicleCommand.confirmation is a retry
  // counter, not an authority sequence or an ACK correlation field.
  command.confirmation = 0;
  if (decision.command == CommandKind::SET_MODE_OFFBOARD) {
    command.command = SafetyGate::kVehicleCmdDoSetMode;
    command.param1 = 1.0F;
    command.param2 = 6.0F;
  } else if (decision.command == CommandKind::LAND) {
    command.command = SafetyGate::kVehicleCmdNavLand;
  } else {
    command.command = SafetyGate::kVehicleCmdArmDisarm;
    command.param1 = decision.command == CommandKind::ARM ? 1.0F : 0.0F;
  }
  command_publisher_->publish(command);
}
}  // namespace offboard_cpp
