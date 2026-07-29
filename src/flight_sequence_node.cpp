#include <flight_sequence.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace
{
std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

class FlightSequenceNode : public rclcpp::Node
{
public:
  FlightSequenceNode()
  : Node("flight_sequence_node"),
    freshness_ns_(declare_parameter<std::int64_t>("safety.freshness_ns", 500000000LL)),
    sequence_(make_config())
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    command_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/offboard/cmd", qos);
    mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/offboard/cmd_mode", qos);
    request_pub_ = create_publisher<std_msgs::msg::UInt8>("/offboard/command_request", qos);

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr message) {
        const bool valid = std::all_of(
          message->position.begin(), message->position.end(),
          [](float value) {return std::isfinite(value);});
        if (valid) {
          for (std::size_t i = 0; i < position_.size(); ++i) {
            position_[i] = message->position[i];
          }
          odom_received_ns_ = steady_now_ns();
        }
      });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr message) {
        armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        status_received_ns_ = steady_now_ns();
      });
    timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
      "/fmu/out/timesync_status", qos,
      [this](px4_msgs::msg::TimesyncStatus::SharedPtr message) {
        px4_time_us_ = message->timestamp;
        timesync_received_ns_ = steady_now_ns();
      });
    car_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/mission/car_target", qos,
      [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr message) {
        if (std::all_of(
            message->position.begin(), message->position.end(),
            [](float value) {return std::isfinite(value);}))
        {
          for (std::size_t i = 0; i < car_target_.size(); ++i) {
            car_target_[i] = message->position[i];
          }
          car_received_ns_ = steady_now_ns();
        }
      });
    start_sub_ = bool_subscription("/mission/start", &start_);
    follow_sub_ = bool_subscription("/mission/follow_complete", &follow_complete_);
    drop_sub_ = bool_subscription("/mission/drop_complete", &drop_complete_);
    landing_sub_ = bool_subscription("/offboard/landing_confirmed", &landing_confirmed_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&FlightSequenceNode::on_timer, this));
  }

private:
  offboard_cpp::FlightConfig make_config()
  {
    offboard_cpp::FlightConfig config;
    const auto task_id = declare_parameter<int>("mission.task_id", 1);
    config.task = task_id == 2 ? offboard_cpp::MissionTask::TASK2 : offboard_cpp::MissionTask::TASK1;
    config.takeoff_speed = declare_parameter<double>("mission.takeoff_speed", 0.5);
    config.cruise_speed = declare_parameter<double>("mission.cruise_speed", 1.0);
    config.home_land_speed = declare_parameter<double>("mission.home_land_speed", 0.3);
    config.platform_land_speed = declare_parameter<double>("mission.platform_land_speed", 0.2);
    config.takeoff_height = declare_parameter<double>("mission.takeoff_height", 1.0);
    config.hover_seconds = declare_parameter<double>("mission.hover_seconds", 3.0);
    config.platform_hold_seconds = declare_parameter<double>("mission.platform_hold_seconds", 5.0);
    config.position_tolerance = declare_parameter<double>("mission.position_tolerance", 0.2);
    config.home_surface_z = declare_parameter<double>("mission.home_surface_z", 0.0);
    config.platform_surface_z = declare_parameter<double>("mission.platform_surface_z", 0.0);
    config.follow_offset_x = declare_parameter<double>("mission.follow_offset_x", 0.0);
    config.follow_offset_y = declare_parameter<double>("mission.follow_offset_y", 0.0);
    config.yaw = declare_parameter<double>("mission.yaw", 0.0);
    if (config.takeoff_speed <= 0.0 || config.cruise_speed <= 0.0 ||
      config.home_land_speed <= 0.0 || config.platform_land_speed <= 0.0 ||
      config.takeoff_height <= 0.0 || config.position_tolerance <= 0.0)
    {
      throw std::invalid_argument("mission speeds, height and tolerance must be positive");
    }
    return config;
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscription(
    const char * topic, bool * value)
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    return create_subscription<std_msgs::msg::Bool>(
      topic, qos, [value](std_msgs::msg::Bool::SharedPtr message) {*value = message->data;});
  }

  bool fresh(std::int64_t received_ns, std::int64_t now_ns) const
  {
    return received_ns >= 0 && now_ns >= received_ns && now_ns - received_ns < freshness_ns_;
  }

  std::uint64_t px4_now_us(std::int64_t now_ns) const
  {
    if (!fresh(timesync_received_ns_, now_ns)) {
      return 0;
    }
    return px4_time_us_ +
      static_cast<std::uint64_t>(std::max<std::int64_t>(0, now_ns - timesync_received_ns_) / 1000);
  }

  void on_timer()
  {
    const auto now_ns = steady_now_ns();
    offboard_cpp::FlightInputs inputs;
    inputs.now_ns = now_ns;
    inputs.start = start_;
    inputs.odometry_fresh = fresh(odom_received_ns_, now_ns);
    inputs.vehicle_status_fresh = fresh(status_received_ns_, now_ns);
    inputs.armed = armed_;
    inputs.landing_confirmed = landing_confirmed_;
    inputs.position = position_;
    inputs.car_target_fresh = fresh(car_received_ns_, now_ns);
    inputs.car_target = car_target_;
    inputs.follow_complete = follow_complete_;
    inputs.drop_complete = drop_complete_;
    const auto output = sequence_.tick(inputs);
    const auto timestamp = px4_now_us(now_ns);
    if (!output.setpoint_valid || timestamp == 0) {
      return;
    }

    px4_msgs::msg::TrajectorySetpoint setpoint;
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    setpoint.position = {
      static_cast<float>(output.position[0]),
      static_cast<float>(output.position[1]),
      static_cast<float>(output.position[2])};
    setpoint.velocity = {nan, nan, nan};
    setpoint.acceleration = {nan, nan, nan};
    setpoint.jerk = {nan, nan, nan};
    setpoint.yaw = static_cast<float>(output.yaw);
    setpoint.yawspeed = nan;
    setpoint.timestamp = timestamp;

    px4_msgs::msg::OffboardControlMode mode;
    mode.timestamp = timestamp;
    mode.position = true;
    command_pub_->publish(setpoint);
    mode_pub_->publish(mode);

    if (output.request != offboard_cpp::MissionRequest::NONE) {
      std_msgs::msg::UInt8 request;
      request.data = static_cast<std::uint8_t>(output.request);
      request_pub_->publish(request);
      RCLCPP_INFO(
        get_logger(), "Mission %s requested command %u",
        offboard_cpp::mission_state_name(output.state), request.data);
    }
  }

  const std::int64_t freshness_ns_;
  offboard_cpp::FlightSequence sequence_;
  std::array<double, 3> position_{};
  std::array<double, 3> car_target_{};
  std::int64_t odom_received_ns_{-1};
  std::int64_t status_received_ns_{-1};
  std::int64_t timesync_received_ns_{-1};
  std::int64_t car_received_ns_{-1};
  std::uint64_t px4_time_us_{0};
  bool armed_{false};
  bool start_{false};
  bool follow_complete_{false};
  bool drop_complete_{false};
  bool landing_confirmed_{false};

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr command_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr request_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr car_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landing_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightSequenceNode>());
  rclcpp::shutdown();
  return 0;
}
