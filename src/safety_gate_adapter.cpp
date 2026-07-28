#include <safety_gate_adapter.hpp>

namespace {
constexpr char kTrajectoryTopic[] = "fmu/in/trajectory_setpoint";
constexpr char kModeTopic[] = "fmu/in/offboard_control_mode";
constexpr char kCommandTopic[] = "fmu/in/vehicle_command";
constexpr std::uint16_t kModeCommand = 176;
constexpr std::uint16_t kArmCommand = 400;
}  // namespace

SafetyGateAdapter::SafetyGateAdapter(rclcpp::Node& node) : node_(node) {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  trajectory_publisher_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      kTrajectoryTopic, qos);
  mode_publisher_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(kModeTopic, qos);
  command_publisher_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(kCommandTopic, qos);
}

void SafetyGateAdapter::note_timestamp_fault(TimestampFault fault) {
  if (fault == TimestampFault::BACKWARD || fault == TimestampFault::FROZEN ||
      fault == TimestampFault::ZERO || fault == TimestampFault::NO_TIMESYNC) {
    ++source_epoch_;
    gate_.source_epoch_changed();
  }
}

void SafetyGateAdapter::observe_vehicle_status(const px4_msgs::msg::VehicleStatus& message,
                                               std::uint64_t now_us) {
  note_timestamp_fault(timestamp_gate_.observe(TimestampStream::VEHICLE_STATUS,
                                               message.timestamp, now_us));
}

void SafetyGateAdapter::observe_odometry(const px4_msgs::msg::VehicleOdometry& message,
                                         std::uint64_t now_us) {
  note_timestamp_fault(timestamp_gate_.observe(TimestampStream::ODOMETRY, message.timestamp, now_us));
}

void SafetyGateAdapter::observe_rc(const px4_msgs::msg::RcChannels& message,
                                   std::uint64_t now_us) {
  rc_signal_valid_ = !message.signal_lost;
  note_timestamp_fault(timestamp_gate_.observe(TimestampStream::RC, message.timestamp, now_us));
  // No frozen production kill-switch mapping is available in the v1.16.2
  // default DDS profile.  Treat that absence as engaged, never as permission.
  kill_latched_ = true;
}

void SafetyGateAdapter::observe_battery(const px4_msgs::msg::BatteryStatus& message,
                                        std::uint64_t now_us) {
  note_timestamp_fault(timestamp_gate_.observe(TimestampStream::BATTERY, message.timestamp, now_us));
}

void SafetyGateAdapter::observe_timesync(const px4_msgs::msg::TimesyncStatus& message,
                                         std::uint64_t now_us) {
  timestamp_gate_.observe_timesync(message.timestamp, now_us);
  if (timestamp_gate_.fault_latched()) {
    ++source_epoch_;
    gate_.source_epoch_changed();
  }
}

void SafetyGateAdapter::observe_setpoint(const px4_msgs::msg::TrajectorySetpoint& message,
                                         std::uint64_t now_us) {
  setpoint_ = message;
  have_setpoint_ = timestamp_gate_.observe(TimestampStream::SETPOINT, message.timestamp, now_us) ==
                   TimestampFault::NONE;
}

void SafetyGateAdapter::observe_mode(const px4_msgs::msg::OffboardControlMode& message,
                                     std::uint64_t now_us) {
  mode_ = message;
  have_mode_ = timestamp_gate_.observe(TimestampStream::MODE, message.timestamp, now_us) ==
               TimestampFault::NONE;
}

void SafetyGateAdapter::observe_ack(const px4_msgs::msg::VehicleCommandAck& message,
                                    std::uint64_t now_us) {
  note_timestamp_fault(timestamp_gate_.observe(TimestampStream::COMMAND_ACK, message.timestamp, now_us));
  gate_.observe_ack(now_us, CommandAck{static_cast<std::uint16_t>(message.command),
                                       message.target_system, message.target_component,
                                       message.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED,
                                       message.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS});
}

SafetyGateInputs SafetyGateAdapter::inputs(bool single_writer, std::uint64_t now_us) const {
  SafetyGateInputs in;
  in.vehicle_status_fresh = timestamp_gate_.fresh(TimestampStream::VEHICLE_STATUS, now_us);
  in.odometry_fresh = timestamp_gate_.fresh(TimestampStream::ODOMETRY, now_us);
  in.battery_fresh = timestamp_gate_.fresh(TimestampStream::BATTERY, now_us);
  in.timesync_fresh = timestamp_gate_.timesync_fresh(now_us);
  in.rc_fresh = rc_signal_valid_ && timestamp_gate_.fresh(TimestampStream::RC, now_us);
  in.kill_fresh = in.rc_fresh;
  in.setpoint_fresh = have_setpoint_ && timestamp_gate_.fresh(TimestampStream::SETPOINT, now_us);
  in.mode_fresh = have_mode_ && timestamp_gate_.fresh(TimestampStream::MODE, now_us);
  in.setpoint_mode_paired = in.setpoint_fresh && in.mode_fresh;
  in.clock_monotonic = last_now_us_ == 0 || now_us >= last_now_us_;
  in.kill_latched = kill_latched_;
  in.manual_arm_enable = false;
  in.manual_activation = false;
  in.manual_reset = false;
  in.single_writer = single_writer;
  // Existing frozen /offboard messages do not carry owner_id, lease_id,
  // sequence, or source epoch.  Never infer these fields from a topic name.
  in.single_owner = false;
  in.owner_id = 0;
  in.lease_id = 0;
  in.epoch = source_epoch_;
  in.sequence = 0;
  return in;
}

bool SafetyGateAdapter::graph_has_only_gate_writer() const {
  return node_.count_publishers(kTrajectoryTopic) == 1 &&
         node_.count_publishers(kModeTopic) == 1 &&
         node_.count_publishers(kCommandTopic) == 1;
}

void SafetyGateAdapter::publish_pending_command(std::uint64_t now_us) {
  const auto pending = gate_.take_command_to_send();
  if (pending == PendingCommand::NONE) {
    return;
  }
  px4_msgs::msg::VehicleCommand command;
  command.command = pending == PendingCommand::MODE ? kModeCommand : kArmCommand;
  command.param1 = pending == PendingCommand::MODE ? 1.0F : 1.0F;
  command.param2 = pending == PendingCommand::MODE ? 6.0F : 0.0F;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 1;
  command.confirmation = 0;
  command.from_external = true;
  command.timestamp = now_us;
  command_publisher_->publish(command);
  ++command_publish_count_;
}

void SafetyGateAdapter::tick(std::uint64_t now_us, bool single_writer) {
  const bool clock_backwards = last_now_us_ != 0 && now_us < last_now_us_;
  last_now_us_ = now_us;
  if (clock_backwards) {
    gate_.source_epoch_changed();
    return;
  }
  if (timestamp_gate_.fault_latched() || !timestamp_gate_.configuration_valid()) {
    gate_.source_epoch_changed();
    return;
  }
  gate_.tick(now_us, inputs(single_writer, now_us));
  if (gate_.allows_control_payload()) {
    setpoint_.timestamp = now_us;
    mode_.timestamp = now_us;
    trajectory_publisher_->publish(setpoint_);
    mode_publisher_->publish(mode_);
    ++trajectory_publish_count_;
    ++mode_publish_count_;
  }
  publish_pending_command(now_us);
}
