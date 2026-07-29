#ifndef OFFBOARD_CPP_NODE_HPP
#define OFFBOARD_CPP_NODE_HPP

#include <memory>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <landing_monitor.hpp>
#include <safety_gate.hpp>
#include <safety_gate_adapter.hpp>
#include <timestamp_gate.hpp>

class OffboardControlNode : public rclcpp::Node
{
public:
  OffboardControlNode();

private:
  struct AuthorityRecord {
    std::string owner;
    std::string lease;
    std::string epoch;
    std::uint64_t sequence{0};
    bool single_writer{false};
    bool single_owner{false};
    std::int64_t received_ns{-1};
  };

  std::int64_t steady_now_ns() const;
  bool fresh(std::int64_t received_ns, std::int64_t max_age_ns) const;
  bool finite_setpoint(const px4_msgs::msg::TrajectorySetpoint & message) const;
  bool accept_timesync_timestamp(std::uint64_t timestamp_us);
  bool accept_timestamp(offboard_cpp::TimestampStream stream, std::uint64_t timestamp_us);
  void reset_timestamp_epoch_inputs();
  bool graph_has_only_gate_writer() const;
  bool parse_authority(const std::string & value, AuthorityRecord * record) const;
  offboard_cpp::GateInputs inputs() const;
  void on_timer();
  void on_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr message);

  const std::int64_t odometry_freshness_ns_;
  const std::int64_t rc_freshness_ns_;
  const std::int64_t timesync_freshness_ns_;
  const std::int64_t vehicle_status_freshness_ns_;
  const std::int64_t land_detected_freshness_ns_;
  const std::int64_t setpoint_freshness_ns_;
  const std::int64_t mode_freshness_ns_;
  const std::int64_t operator_freshness_ns_;
  const std::int64_t authority_freshness_ns_;
  const std::int64_t timestamp_max_age_us_;
  const std::int64_t timestamp_max_future_us_;
  const double home_surface_z_;
  const double platform_surface_z_;
  std::string expected_owner_;
  std::string expected_lease_;
  std::string expected_epoch_;
  offboard_cpp::TimestampGate timestamp_gate_;
  bool timestamp_config_valid_{false};
  offboard_cpp::SafetyGate gate_;
  offboard_cpp::LandingMonitor landing_monitor_;
  std::unique_ptr<offboard_cpp::SafetyGateAdapter> adapter_;

  px4_msgs::msg::TrajectorySetpoint setpoint_{};
  px4_msgs::msg::OffboardControlMode mode_{};
  AuthorityRecord authority_{};
  std::int64_t status_received_ns_{-1};
  std::int64_t odom_received_ns_{-1};
  std::int64_t timesync_received_ns_{-1};
  std::int64_t rc_received_ns_{-1};
  std::int64_t setpoint_received_ns_{-1};
  std::int64_t mode_received_ns_{-1};
  std::int64_t land_received_ns_{-1};
  std::int64_t manual_arm_received_ns_{-1};
  std::int64_t kill_received_ns_{-1};
  std::int64_t last_ros_time_ns_{-1};
  std::uint64_t setpoint_sequence_{0};
  std::uint64_t mode_sequence_{0};
  bool vehicle_in_offboard_{false};
  bool vehicle_armed_{false};
  bool vehicle_landed_{false};
  bool landing_confirmed_{false};
  double altitude_z_{0.0};
  double vertical_speed_{0.0};
  double contact_surface_z_{0.0};
  std::uint64_t vehicle_status_generation_{0};
  std::uint64_t land_detected_timestamp_us_{0};
  offboard_cpp::MissionRequest mission_request_{offboard_cpp::MissionRequest::NONE};
  std::uint64_t mission_request_generation_{0};
  bool rc_valid_{false};
  bool odom_valid_{false};
  bool timestamp_fault_latched_{false};
  bool physical_kill_{false};
  bool manual_arm_enable_{false};
  bool recovery_requested_{false};
  bool activation_requested_{false};
  bool last_recovery_signal_{false};
  bool last_activation_signal_{false};

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr authority_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_arm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr command_request_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr landing_confirmed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flight_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_reason_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // OFFBOARD_CPP_NODE_HPP
