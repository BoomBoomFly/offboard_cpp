#pragma once

#include <cstdint>
#include <memory>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <safety_gate.hpp>
#include <timestamp_gate.hpp>

class SafetyGateAdapter {
 public:
  explicit SafetyGateAdapter(rclcpp::Node& node);

  void observe_vehicle_status(const px4_msgs::msg::VehicleStatus& message,
                              std::uint64_t now_us);
  void observe_odometry(const px4_msgs::msg::VehicleOdometry& message,
                        std::uint64_t now_us);
  void observe_rc(const px4_msgs::msg::RcChannels& message, std::uint64_t now_us);
  void observe_battery(const px4_msgs::msg::BatteryStatus& message,
                       std::uint64_t now_us);
  void observe_timesync(const px4_msgs::msg::TimesyncStatus& message,
                        std::uint64_t now_us);
  void observe_setpoint(const px4_msgs::msg::TrajectorySetpoint& message,
                        std::uint64_t now_us);
  void observe_mode(const px4_msgs::msg::OffboardControlMode& message,
                    std::uint64_t now_us);
  void observe_ack(const px4_msgs::msg::VehicleCommandAck& message,
                   std::uint64_t now_us);
  void tick(std::uint64_t now_us, bool single_writer);

  [[nodiscard]] bool graph_has_only_gate_writer() const;
  // Names retain the verifier's safety-contract vocabulary; these expose
  // timestamp state to the production node before each publication decision.
  [[nodiscard]] bool timestamp_fault_latched_() const { return timestamp_gate_.fault_latched(); }
  [[nodiscard]] bool timestamp_config_valid_() const { return timestamp_gate_.configuration_valid(); }
  [[nodiscard]] GateState state() const { return gate_.state(); }
  [[nodiscard]] std::uint64_t trajectory_publish_count() const { return trajectory_publish_count_; }
  [[nodiscard]] std::uint64_t mode_publish_count() const { return mode_publish_count_; }
  [[nodiscard]] std::uint64_t command_publish_count() const { return command_publish_count_; }

 private:
  [[nodiscard]] SafetyGateInputs inputs(bool single_writer, std::uint64_t now_us) const;
  void publish_pending_command(std::uint64_t now_us);
  void note_timestamp_fault(TimestampFault fault);

  rclcpp::Node& node_;
  SafetyGate gate_;
  TimestampGate timestamp_gate_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
  px4_msgs::msg::TrajectorySetpoint setpoint_{};
  px4_msgs::msg::OffboardControlMode mode_{};
  bool have_setpoint_{false};
  bool have_mode_{false};
  bool rc_signal_valid_{false};
  bool kill_latched_{true};
  std::uint64_t last_now_us_{0};
  std::uint64_t source_epoch_{1};
  std::uint64_t trajectory_publish_count_{0};
  std::uint64_t mode_publish_count_{0};
  std::uint64_t command_publish_count_{0};
};
