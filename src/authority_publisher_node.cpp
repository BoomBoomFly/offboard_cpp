#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

class OffboardAuthorityNode : public rclcpp::Node
{
public:
  OffboardAuthorityNode()
  : Node("offboard_authority_node"),
    owner_(declare_parameter<std::string>("authority.owner", "")),
    lease_(declare_parameter<std::string>("authority.lease", "")),
    epoch_(declare_parameter<std::string>("authority.epoch", "")),
    input_freshness_ns_(
      declare_parameter<std::int64_t>("authority.input_freshness_ns", 500000000LL))
  {
    if (owner_.empty() || lease_.empty() || epoch_.empty() || input_freshness_ns_ <= 0) {
      throw std::invalid_argument("authority owner/lease/epoch and freshness are required");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    authority_pub_ = create_publisher<std_msgs::msg::String>("/offboard/authority", qos);
    kill_pub_ = create_publisher<std_msgs::msg::Bool>("/offboard/kill", qos);
    activation_pub_ = create_publisher<std_msgs::msg::Bool>("/offboard/manual_enable", qos);
    arm_pub_ = create_publisher<std_msgs::msg::Bool>("/offboard/manual_arm_enable", qos);

    kill_sub_ = input("/operator/kill", &kill_, &kill_received_ns_);
    activation_sub_ = input("/operator/activation", &activation_, &activation_received_ns_);
    arm_sub_ = input("/operator/arm_enable", &arm_enable_, &arm_received_ns_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&OffboardAuthorityNode::publish, this));
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr input(
    const char * topic, bool * value, std::int64_t * received)
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    return create_subscription<std_msgs::msg::Bool>(
      topic, qos,
      [value, received](std_msgs::msg::Bool::SharedPtr message) {
        *value = message->data;
        *received = steady_now_ns();
      });
  }

  bool fresh(std::int64_t received, std::int64_t now) const
  {
    return received >= 0 && now >= received && now - received < input_freshness_ns_;
  }

  bool endpoint_is(const char * topic, const char * node_name) const
  {
    const auto endpoints = get_publishers_info_by_topic(topic);
    return endpoints.size() == 1 &&
      endpoints.front().node_name() == node_name &&
      endpoints.front().node_namespace() == "/";
  }

  void publish()
  {
    const auto now = steady_now_ns();
    const bool writers =
      endpoint_is("/fmu/in/trajectory_setpoint", "offboard_control_node") &&
      endpoint_is("/fmu/in/offboard_control_mode", "offboard_control_node") &&
      endpoint_is("/fmu/in/vehicle_command", "offboard_control_node");
    const bool owners =
      endpoint_is("/offboard/cmd", "flight_sequence_node") &&
      endpoint_is("/offboard/cmd_mode", "flight_sequence_node") &&
      endpoint_is("/offboard/command_request", "flight_sequence_node");

    std_msgs::msg::String authority;
    std::ostringstream encoded;
    encoded << "owner=" << owner_ << ";lease=" << lease_ << ";epoch=" << epoch_
            << ";sequence=1;writers=" << (writers ? 1 : 0)
            << ";owners=" << (owners ? 1 : 0);
    authority.data = encoded.str();
    authority_pub_->publish(authority);

    std_msgs::msg::Bool value;
    value.data = !fresh(kill_received_ns_, now) || kill_;
    kill_pub_->publish(value);
    value.data = fresh(activation_received_ns_, now) && activation_;
    activation_pub_->publish(value);
    value.data = fresh(arm_received_ns_, now) && arm_enable_;
    arm_pub_->publish(value);
  }

  const std::string owner_;
  const std::string lease_;
  const std::string epoch_;
  const std::int64_t input_freshness_ns_;
  bool kill_{true};
  bool activation_{false};
  bool arm_enable_{false};
  std::int64_t kill_received_ns_{-1};
  std::int64_t activation_received_ns_{-1};
  std::int64_t arm_received_ns_{-1};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr authority_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr activation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardAuthorityNode>());
  rclcpp::shutdown();
  return 0;
}
