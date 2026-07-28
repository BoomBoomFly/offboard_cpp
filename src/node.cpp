#include <node.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iterator>
#include <sstream>
#include <unordered_map>

namespace
{
constexpr std::int64_t kDefaultFreshnessNs = 500000000LL;
constexpr std::int64_t kPairingWindowNs = 100000000LL;
constexpr std::int64_t kMaxTimestampAgeUs = 500000LL;
constexpr std::int64_t kMaxTimestampFutureUs = 100000LL;
}

OffboardControlNode::OffboardControlNode()
: Node("offboard_control_node"),
  freshness_ns_(declare_parameter<std::int64_t>("safety.freshness_ns", kDefaultFreshnessNs)),
  timestamp_max_age_us_(declare_parameter<std::int64_t>("safety.timestamp_max_age_us", 500000)),
  timestamp_max_future_us_(declare_parameter<std::int64_t>("safety.timestamp_max_future_us", 100000)),
  expected_owner_(declare_parameter<std::string>("safety.expected_owner", "")),
  expected_lease_(declare_parameter<std::string>("safety.expected_lease", "")),
  expected_epoch_(declare_parameter<std::string>("safety.expected_epoch", "")),
  timestamp_gate_(
    static_cast<std::uint64_t>(std::max<std::int64_t>(0, timestamp_max_age_us_)),
    static_cast<std::uint64_t>(std::max<std::int64_t>(0, timestamp_max_future_us_))),
  timestamp_config_valid_(
    timestamp_max_age_us_ > 0 && timestamp_max_age_us_ <= kMaxTimestampAgeUs &&
    timestamp_max_future_us_ >= 0 && timestamp_max_future_us_ <= kMaxTimestampFutureUs),
  gate_(
    expected_owner_, expected_lease_, expected_epoch_,
    declare_parameter<bool>("takeoff_land.enable_arm", false))
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  adapter_ = std::make_unique<offboard_cpp::SafetyGateAdapter>(*this, qos);

  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "fmu/out/vehicle_status_v1", qos,
    [this](px4_msgs::msg::VehicleStatus::SharedPtr message) {
      if (!accept_timestamp(offboard_cpp::TimestampStream::VEHICLE_STATUS, message->timestamp)) {
        return;
      }
      status_received_ns_ = steady_now_ns();
      ++vehicle_status_generation_;
      vehicle_in_offboard_ = message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
      vehicle_armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    });
  odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "fmu/out/vehicle_odometry", qos,
    [this](px4_msgs::msg::VehicleOdometry::SharedPtr message) {
      if (!accept_timestamp(offboard_cpp::TimestampStream::ODOMETRY, message->timestamp)) {
        return;
      }
      const auto finite = [](const auto & values) {
        return std::all_of(std::begin(values), std::end(values),
          [](float value) { return std::isfinite(value); });
      };
      odom_valid_ = finite(message->position) && finite(message->velocity) &&
        finite(message->q) && finite(message->angular_velocity);
      odom_received_ns_ = steady_now_ns();
    });
  timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
    "fmu/out/timesync_status", qos,
    [this](px4_msgs::msg::TimesyncStatus::SharedPtr message) {
      if (accept_timesync_timestamp(message->timestamp)) {
        timesync_received_ns_ = steady_now_ns();
      }
    });
  rc_sub_ = create_subscription<px4_msgs::msg::RcChannels>(
    "fmu/out/rc_channels", qos,
    [this](px4_msgs::msg::RcChannels::SharedPtr message) {
      if (!accept_timestamp(offboard_cpp::TimestampStream::RC, message->timestamp)) {
        return;
      }
      rc_received_ns_ = steady_now_ns();
      rc_valid_ = !message->signal_lost && message->channel_count > 0 &&
        message->channel_count <= message->channels.size();
      for (std::size_t index = 0; rc_valid_ && index < message->channel_count; ++index) {
        rc_valid_ = std::isfinite(message->channels[index]) && message->channels[index] >= -1.0F &&
          message->channels[index] <= 1.0F;
      }
    });
  setpoint_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    "offboard/cmd", qos,
    [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr message) {
      if (!accept_timestamp(offboard_cpp::TimestampStream::SETPOINT, message->timestamp) ||
          !finite_setpoint(*message)) {
        setpoint_received_ns_ = -1;
        return;
      }
      setpoint_ = *message;
      setpoint_received_ns_ = steady_now_ns();
      setpoint_sequence_ = authority_.sequence;
    });
  mode_sub_ = create_subscription<px4_msgs::msg::OffboardControlMode>(
    "offboard/cmd_mode", qos,
    [this](px4_msgs::msg::OffboardControlMode::SharedPtr message) {
      if (!accept_timestamp(offboard_cpp::TimestampStream::MODE, message->timestamp)) {
        mode_received_ns_ = -1;
        return;
      }
      mode_ = *message;
      mode_received_ns_ = steady_now_ns();
      mode_sequence_ = authority_.sequence;
    });
  ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "fmu/out/vehicle_command_ack", qos,
    std::bind(&OffboardControlNode::on_ack, this, std::placeholders::_1));
  authority_sub_ = create_subscription<std_msgs::msg::String>(
    "offboard/authority", qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      AuthorityRecord parsed;
      if (!parse_authority(message->data, &parsed) ||
          (authority_.sequence != 0 && parsed.sequence <= authority_.sequence)) {
        authority_.received_ns = -1;
        return;
      }
      parsed.received_ns = steady_now_ns();
      authority_ = parsed;
    });
  kill_sub_ = create_subscription<std_msgs::msg::Bool>(
    "offboard/kill", qos,
    [this](std_msgs::msg::Bool::SharedPtr message) {
      physical_kill_ = message->data;
      kill_received_ns_ = steady_now_ns();
    });
  manual_arm_sub_ = create_subscription<std_msgs::msg::Bool>(
    "offboard/manual_arm_enable", qos,
    [this](std_msgs::msg::Bool::SharedPtr message) {
      manual_arm_enable_ = message->data;
      manual_arm_received_ns_ = steady_now_ns();
    });
  activation_sub_ = create_subscription<std_msgs::msg::Bool>(
    "offboard/manual_enable", qos,
    [this](std_msgs::msg::Bool::SharedPtr message) { activation_requested_ = message->data; });
  recovery_sub_ = create_subscription<std_msgs::msg::Bool>(
    "offboard/manual_recovery", qos,
    [this](std_msgs::msg::Bool::SharedPtr message) { recovery_requested_ = message->data; });
  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&OffboardControlNode::on_timer, this));
}

std::int64_t OffboardControlNode::steady_now_ns() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool OffboardControlNode::fresh(std::int64_t received_ns) const
{
  const auto now = steady_now_ns();
  return received_ns >= 0 && now >= received_ns && now - received_ns < freshness_ns_;
}

bool OffboardControlNode::finite_setpoint(const px4_msgs::msg::TrajectorySetpoint & message) const
{
  return std::all_of(std::begin(message.position), std::end(message.position),
    [](float value) { return std::isfinite(value); }) && std::isfinite(message.yaw);
}

bool OffboardControlNode::accept_timesync_timestamp(std::uint64_t timestamp_us)
{
  const auto result = timestamp_gate_.observe_timesync(timestamp_us);
  if (result != offboard_cpp::TimestampResult::ACCEPTED && gate_.state() != offboard_cpp::GateState::WAIT) {
    timestamp_fault_latched_ = true;
  }
  return result == offboard_cpp::TimestampResult::ACCEPTED;
}

bool OffboardControlNode::accept_timestamp(
  offboard_cpp::TimestampStream stream, std::uint64_t timestamp_us)
{
  const auto result = timestamp_gate_.observe(stream, timestamp_us);
  if (result != offboard_cpp::TimestampResult::ACCEPTED && gate_.state() != offboard_cpp::GateState::WAIT) {
    timestamp_fault_latched_ = true;
  }
  return result == offboard_cpp::TimestampResult::ACCEPTED;
}

void OffboardControlNode::reset_timestamp_epoch_inputs()
{
  timestamp_gate_.restart_epoch();
  timestamp_fault_latched_ = false;
  status_received_ns_ = odom_received_ns_ = timesync_received_ns_ = rc_received_ns_ = -1;
  setpoint_received_ns_ = mode_received_ns_ = -1;
  rc_valid_ = false;
  odom_valid_ = false;
  vehicle_in_offboard_ = false;
  vehicle_armed_ = false;
  vehicle_status_generation_ = 0;
}

bool OffboardControlNode::graph_has_only_gate_writer() const
{
  const auto only_this_node = [this](const char * topic) {
    const auto endpoints = get_publishers_info_by_topic(topic);
    return endpoints.size() == 1 && endpoints.front().node_name() == get_name();
  };
  return only_this_node("/fmu/in/trajectory_setpoint") &&
         only_this_node("/fmu/in/offboard_control_mode") &&
         only_this_node("/fmu/in/vehicle_command");
}

bool OffboardControlNode::parse_authority(const std::string & value, AuthorityRecord * record) const
{
  std::unordered_map<std::string, std::string> fields;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ';')) {
    const auto separator = token.find('=');
    if (separator == std::string::npos || separator == 0 || fields.count(token.substr(0, separator)) != 0) {
      return false;
    }
    fields.emplace(token.substr(0, separator), token.substr(separator + 1));
  }
  if (fields.size() != 6 || !fields.count("owner") || !fields.count("lease") || !fields.count("epoch") ||
      !fields.count("sequence") || !fields.count("writers") || !fields.count("owners")) {
    return false;
  }
  try {
    record->owner = fields.at("owner");
    record->lease = fields.at("lease");
    record->epoch = fields.at("epoch");
    record->sequence = std::stoull(fields.at("sequence"));
    record->single_writer = fields.at("writers") == "1";
    record->single_owner = fields.at("owners") == "1";
  } catch (const std::exception &) {
    return false;
  }
  return !record->owner.empty() && !record->lease.empty() && !record->epoch.empty() && record->sequence != 0;
}

offboard_cpp::GateInputs OffboardControlNode::inputs() const
{
  offboard_cpp::GateInputs result;
  const auto ros_now = now().nanoseconds();
  result.clock_monotonic = timestamp_config_valid_ && !timestamp_fault_latched_ &&
    (last_ros_time_ns_ < 0 || ros_now >= last_ros_time_ns_);
  result.vehicle_status_fresh = fresh(status_received_ns_) &&
    timestamp_gate_.current(offboard_cpp::TimestampStream::VEHICLE_STATUS);
  result.odometry_fresh = fresh(odom_received_ns_) && odom_valid_ &&
    timestamp_gate_.current(offboard_cpp::TimestampStream::ODOMETRY);
  result.timesync_fresh = fresh(timesync_received_ns_) && timestamp_gate_.timesync_ready();
  result.rc_fresh = fresh(rc_received_ns_) && rc_valid_ &&
    timestamp_gate_.current(offboard_cpp::TimestampStream::RC);
  result.kill_fresh = fresh(kill_received_ns_);
  result.setpoint_fresh = fresh(setpoint_received_ns_) &&
    timestamp_gate_.current(offboard_cpp::TimestampStream::SETPOINT);
  result.mode_fresh = fresh(mode_received_ns_) &&
    timestamp_gate_.current(offboard_cpp::TimestampStream::MODE);
  result.setpoint_mode_paired = result.setpoint_fresh && result.mode_fresh &&
    setpoint_sequence_ != 0 && setpoint_sequence_ == mode_sequence_ &&
    std::llabs(setpoint_received_ns_ - mode_received_ns_) <= kPairingWindowNs;
  result.kill_latched = physical_kill_;
  result.vehicle_in_offboard = vehicle_in_offboard_;
  result.vehicle_armed = vehicle_armed_;
  result.vehicle_status_generation = vehicle_status_generation_;
  result.manual_arm_enable = manual_arm_enable_ && fresh(manual_arm_received_ns_);
  result.authority.fresh = fresh(authority_.received_ns);
  // The signed/approved authority heartbeat is necessary but not sufficient:
  // continuously reject a second ROS writer on any PX4 control input as well.
  result.authority.single_writer = authority_.single_writer && graph_has_only_gate_writer();
  result.authority.single_owner = authority_.single_owner;
  result.authority.owner_id = authority_.owner;
  result.authority.lease_id = authority_.lease;
  result.authority.epoch = authority_.epoch;
  result.authority.sequence = authority_.sequence;
  return result;
}

void OffboardControlNode::on_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr message)
{
  // Invalid ACK timestamps do not reach the observer; accept_timestamp latches
  // timestamp_fault_latched_ outside WAIT, so the following timer tick emits
  // no PX4 input and transitions the production gate to FAULT_LATCHED.
  if (!accept_timestamp(offboard_cpp::TimestampStream::COMMAND_ACK, message->timestamp)) {
    return;
  }
  offboard_cpp::CommandAck ack;
  ack.command = message->command;
  ack.target_system = message->target_system;
  ack.target_component = message->target_component;
  ack.from_external = message->from_external;
  ack.status_generation = vehicle_status_generation_;
  ack.result = message->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED
    ? offboard_cpp::AckResult::ACCEPTED : offboard_cpp::AckResult::REJECTED;
  const auto result = gate_.observe_ack(steady_now_ns(), ack, inputs().authority);
  if (result.fault_latched) {
    RCLCPP_ERROR(get_logger(), "Offboard gate latched on ACK: %s", result.reason);
  }
}

void OffboardControlNode::on_timer()
{
  const auto current_inputs = inputs();
  offboard_cpp::GateDecision result;
  if (recovery_requested_) {
    recovery_requested_ = false;
    if (timestamp_fault_latched_) {
      reset_timestamp_epoch_inputs();
      result = gate_.tick(steady_now_ns(), inputs());
    } else {
      result = gate_.request_manual_recovery(steady_now_ns(), current_inputs);
    }
  } else if (activation_requested_) {
    activation_requested_ = false;
    result = gate_.request_manual_activation(steady_now_ns(), current_inputs);
  } else {
    result = gate_.tick(steady_now_ns(), current_inputs);
  }
  last_ros_time_ns_ = now().nanoseconds();
  if (result.fault_latched) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Offboard gate latched: %s", result.reason);
  }
  adapter_->apply(result, setpoint_, mode_, now().nanoseconds() / 1000);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControlNode>());
  rclcpp::shutdown();
  return 0;
}
