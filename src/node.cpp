#include <node.hpp>

#include <graph_guard.hpp>

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
constexpr std::int64_t kPairingWindowNs = 100000000LL;
constexpr std::int64_t kMaxTimestampAgeUs = 3000000LL;
constexpr std::int64_t kMaxTimestampFutureUs = 100000LL;
}

OffboardControlNode::OffboardControlNode()
: Node("offboard_control_node"),
  odometry_freshness_ns_(declare_parameter<std::int64_t>("freshness.odometry_ns", 200000000LL)),
  rc_freshness_ns_(declare_parameter<std::int64_t>("freshness.rc_ns", 300000000LL)),
  timesync_freshness_ns_(declare_parameter<std::int64_t>("freshness.timesync_ns", 2500000000LL)),
  vehicle_status_freshness_ns_(declare_parameter<std::int64_t>("freshness.vehicle_status_ns", 1200000000LL)),
  land_detected_freshness_ns_(declare_parameter<std::int64_t>("freshness.land_detected_ns", 1800000000LL)),
  setpoint_freshness_ns_(declare_parameter<std::int64_t>("freshness.setpoint_ns", 150000000LL)),
  mode_freshness_ns_(declare_parameter<std::int64_t>("freshness.mode_ns", 150000000LL)),
  operator_freshness_ns_(declare_parameter<std::int64_t>("freshness.operator_ns", 300000000LL)),
  authority_freshness_ns_(declare_parameter<std::int64_t>("freshness.authority_ns", 500000000LL)),
  timestamp_max_age_us_(declare_parameter<std::int64_t>("safety.timestamp_max_age_us", 2000000)),
  timestamp_max_future_us_(declare_parameter<std::int64_t>("safety.timestamp_max_future_us", 100000)),
  home_surface_z_(declare_parameter<double>("mission.home_surface_z", 0.0)),
  platform_surface_z_(declare_parameter<double>("mission.platform_surface_z", 0.0)),
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
    declare_parameter<bool>("takeoff_land.enable_arm", false)),
  landing_monitor_(
    declare_parameter<std::int64_t>("landing.stable_ns", 1000000000LL),
    declare_parameter<double>("landing.max_vertical_speed", 0.15),
    declare_parameter<double>("landing.height_tolerance", 0.25),
    static_cast<std::uint32_t>(declare_parameter<int>("landing.minimum_samples", 2)))
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
      if (odom_valid_) {
        altitude_z_ = message->position[2];
        vertical_speed_ = message->velocity[2];
      }
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
  land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "fmu/out/vehicle_land_detected", qos,
    [this](px4_msgs::msg::VehicleLandDetected::SharedPtr message) {
      if (!accept_timestamp(offboard_cpp::TimestampStream::LAND_DETECTED, message->timestamp)) {
        return;
      }
      vehicle_landed_ = message->landed;
      land_detected_timestamp_us_ = message->timestamp;
      land_received_ns_ = steady_now_ns();
    });
  authority_sub_ = create_subscription<std_msgs::msg::String>(
    "offboard/authority", qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      AuthorityRecord parsed;
      if (!parse_authority(message->data, &parsed) ||
          (authority_.sequence != 0 && parsed.sequence < authority_.sequence)) {
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
    [this](std_msgs::msg::Bool::SharedPtr message) {
      activation_requested_ = message->data && !last_activation_signal_;
      last_activation_signal_ = message->data;
    });
  command_request_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "offboard/command_request", qos,
    [this](std_msgs::msg::UInt8::SharedPtr message) {
      switch (static_cast<offboard_cpp::MissionRequest>(message->data)) {
        case offboard_cpp::MissionRequest::LAND_HOME:
          contact_surface_z_ = home_surface_z_;
          break;
        case offboard_cpp::MissionRequest::LAND_PLATFORM:
          contact_surface_z_ = platform_surface_z_;
          break;
        case offboard_cpp::MissionRequest::DISARM:
        case offboard_cpp::MissionRequest::REARM:
          break;
        case offboard_cpp::MissionRequest::NONE:
        default:
          RCLCPP_ERROR(get_logger(), "Rejected non-whitelisted mission request: %u", message->data);
          return;
      }
      mission_request_ = static_cast<offboard_cpp::MissionRequest>(message->data);
      ++mission_request_generation_;
    });
  landing_confirmed_pub_ = create_publisher<std_msgs::msg::Bool>(
    "offboard/landing_confirmed", qos);
  flight_state_pub_ = create_publisher<std_msgs::msg::String>("offboard/flight_state", qos);
  fault_reason_pub_ = create_publisher<std_msgs::msg::String>("offboard/fault_reason", qos);
  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&OffboardControlNode::on_timer, this));
}

std::int64_t OffboardControlNode::steady_now_ns() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool OffboardControlNode::fresh(
  std::int64_t received_ns, std::int64_t max_age_ns) const
{
  const auto now = steady_now_ns();
  return max_age_ns > 0 && received_ns >= 0 && now >= received_ns &&
         now - received_ns < max_age_ns;
}

bool OffboardControlNode::finite_setpoint(const px4_msgs::msg::TrajectorySetpoint & message) const
{
  return std::all_of(std::begin(message.position), std::end(message.position),
    [](float value) { return std::isfinite(value); }) && std::isfinite(message.yaw);
}

bool OffboardControlNode::accept_timesync_timestamp(std::uint64_t timestamp_us)
{
  const auto result = timestamp_gate_.observe_timesync(timestamp_us, steady_now_ns());
  if (result != offboard_cpp::TimestampResult::ACCEPTED && gate_.state() != offboard_cpp::GateState::WAIT) {
    timestamp_fault_latched_ = true;
  }
  return result == offboard_cpp::TimestampResult::ACCEPTED;
}

bool OffboardControlNode::accept_timestamp(
  offboard_cpp::TimestampStream stream, std::uint64_t timestamp_us)
{
  const auto result = timestamp_gate_.observe(stream, timestamp_us, steady_now_ns());
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
  setpoint_received_ns_ = mode_received_ns_ = land_received_ns_ = -1;
  rc_valid_ = false;
  odom_valid_ = false;
  vehicle_in_offboard_ = false;
  vehicle_armed_ = false;
  vehicle_status_generation_ = 0;
  vehicle_landed_ = false;
  land_detected_timestamp_us_ = 0;
  landing_confirmed_ = false;
  landing_monitor_.reset();
}

bool OffboardControlNode::graph_has_only_gate_writer() const
{
  return offboard_cpp::graph_has_only_gate_writer(*this);
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
  const auto steady_now = steady_now_ns();
  result.vehicle_status_fresh = fresh(status_received_ns_, vehicle_status_freshness_ns_) &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::VEHICLE_STATUS, steady_now,
      static_cast<std::uint64_t>(vehicle_status_freshness_ns_ / 1000));
  result.odometry_fresh = fresh(odom_received_ns_, odometry_freshness_ns_) && odom_valid_ &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::ODOMETRY, steady_now,
      static_cast<std::uint64_t>(odometry_freshness_ns_ / 1000));
  result.timesync_fresh = fresh(timesync_received_ns_, timesync_freshness_ns_) &&
    timestamp_gate_.timesync_ready();
  result.rc_fresh = fresh(rc_received_ns_, rc_freshness_ns_) && rc_valid_ &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::RC, steady_now,
      static_cast<std::uint64_t>(rc_freshness_ns_ / 1000));
  result.kill_fresh = fresh(kill_received_ns_, operator_freshness_ns_);
  result.setpoint_fresh = fresh(setpoint_received_ns_, setpoint_freshness_ns_) &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::SETPOINT, steady_now,
      static_cast<std::uint64_t>(setpoint_freshness_ns_ / 1000));
  result.mode_fresh = fresh(mode_received_ns_, mode_freshness_ns_) &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::MODE, steady_now,
      static_cast<std::uint64_t>(mode_freshness_ns_ / 1000));
  result.setpoint_mode_paired = result.setpoint_fresh && result.mode_fresh &&
    setpoint_sequence_ != 0 && setpoint_sequence_ == mode_sequence_ &&
    std::llabs(setpoint_received_ns_ - mode_received_ns_) <= kPairingWindowNs;
  result.kill_latched = physical_kill_;
  result.vehicle_in_offboard = vehicle_in_offboard_;
  result.vehicle_armed = vehicle_armed_;
  result.landing_confirmed = landing_confirmed_;
  result.vehicle_status_generation = vehicle_status_generation_;
  result.manual_arm_enable = manual_arm_enable_ && fresh(manual_arm_received_ns_, operator_freshness_ns_);
  result.mission_request = mission_request_;
  result.mission_request_generation = mission_request_generation_;
  result.authority.fresh = fresh(authority_.received_ns, authority_freshness_ns_);
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
  if (message->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
    ack.result = offboard_cpp::AckResult::ACCEPTED;
  } else if (
    message->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS)
  {
    ack.result = offboard_cpp::AckResult::IN_PROGRESS;
  } else {
    ack.result = offboard_cpp::AckResult::REJECTED;
  }
  const auto result = gate_.observe_ack(steady_now_ns(), ack, inputs().authority);
  if (result.fault_latched) {
    RCLCPP_ERROR(get_logger(), "Offboard gate latched on ACK: %s", result.reason);
  }
}

void OffboardControlNode::on_timer()
{
  const auto steady_now = steady_now_ns();
  offboard_cpp::LandingObservation landing;
  landing.land_detected_fresh = fresh(land_received_ns_, land_detected_freshness_ns_) &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::LAND_DETECTED, steady_now,
      static_cast<std::uint64_t>(land_detected_freshness_ns_ / 1000));
  landing.vehicle_status_fresh = fresh(status_received_ns_, vehicle_status_freshness_ns_) &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::VEHICLE_STATUS, steady_now,
      static_cast<std::uint64_t>(vehicle_status_freshness_ns_ / 1000));
  landing.odometry_fresh = fresh(odom_received_ns_, odometry_freshness_ns_) && odom_valid_ &&
    timestamp_gate_.current(
      offboard_cpp::TimestampStream::ODOMETRY, steady_now,
      static_cast<std::uint64_t>(odometry_freshness_ns_ / 1000));
  landing.landed = vehicle_landed_;
  landing.land_detected_timestamp_us = land_detected_timestamp_us_;
  landing.altitude_z = altitude_z_;
  landing.vertical_speed = vertical_speed_;
  landing.contact_surface_z = contact_surface_z_;
  landing_confirmed_ = landing_monitor_.update(steady_now, landing);
  std_msgs::msg::Bool landing_message;
  landing_message.data = landing_confirmed_;
  landing_confirmed_pub_->publish(landing_message);

  const auto current_inputs = inputs();
  offboard_cpp::GateDecision result;
  if (activation_requested_) {
    activation_requested_ = false;
    result = gate_.request_manual_activation(steady_now, current_inputs);
  } else {
    result = gate_.tick(steady_now, current_inputs);
  }
  last_ros_time_ns_ = now().nanoseconds();
  if (result.fault_latched) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Offboard gate latched: %s", result.reason);
  }
  std_msgs::msg::String state_message;
  state_message.data = offboard_cpp::gate_state_name(result.state);
  flight_state_pub_->publish(state_message);
  std_msgs::msg::String fault_message;
  fault_message.data = result.fault_latched ? result.reason : "";
  fault_reason_pub_->publish(fault_message);
  adapter_->apply(
    result, setpoint_, mode_,
    static_cast<std::int64_t>(timestamp_gate_.estimated_px4_now(steady_now)));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControlNode>());
  rclcpp::shutdown();
  return 0;
}
