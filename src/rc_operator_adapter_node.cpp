#include <rc_operator_adapter.hpp>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <px4_msgs/msg/rc_channels.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
namespace
{
std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
class RcOperatorAdapterNode : public rclcpp::Node
{
public:
  RcOperatorAdapterNode() : Node("rc_operator_adapter_node"), adapter_(make_config())
  {
    edge_hold_ns_ = declare_parameter<std::int64_t>("rc_operator.edge_hold_ns", 200000000LL);
    if (edge_hold_ns_ <= 0 || edge_hold_ns_ >= 300000000LL) {
      throw std::invalid_argument("RC operator edge hold must be in (0, 300ms)");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    kill_pub_ = create_publisher<std_msgs::msg::Bool>("/operator/kill", qos);
    activation_pub_ = create_publisher<std_msgs::msg::Bool>("/operator/activation", qos);
    arm_pub_ = create_publisher<std_msgs::msg::Bool>("/operator/arm_enable", qos);
    recovery_pub_ = create_publisher<std_msgs::msg::Bool>("/operator/recovery", qos);
    rc_sub_ = create_subscription<px4_msgs::msg::RcChannels>("/fmu/out/rc_channels", qos,
      [this](px4_msgs::msg::RcChannels::SharedPtr message) {
        offboard_cpp::RcSample sample;
        sample.timestamp_us = message->timestamp;
        sample.received_ns = steady_now_ns();
        sample.signal_lost = message->signal_lost;
        const auto count = std::min<std::size_t>(message->channel_count, message->channels.size());
        sample.channels.assign(message->channels.begin(), message->channels.begin() + count);
        publish(adapter_.update(sample, sample.received_ns), sample.received_ns);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
      [this]() {
        const auto now_ns = steady_now_ns();
        publish(adapter_.timeout(now_ns), now_ns);
      });
  }
private:
  offboard_cpp::RcOperatorConfig make_config()
  {
    offboard_cpp::RcOperatorConfig config;
    config.kill_channel = declare_parameter<int>("rc_operator.kill_channel", -1);
    config.activation_channel = declare_parameter<int>("rc_operator.activation_channel", -1);
    config.arm_enable_channel = declare_parameter<int>("rc_operator.arm_enable_channel", -1);
    config.recovery_channel = declare_parameter<int>("rc_operator.recovery_channel", -1);
    config.kill_threshold = declare_parameter<double>("rc_operator.kill_threshold", 2.0);
    config.activation_threshold = declare_parameter<double>("rc_operator.activation_threshold", 2.0);
    config.arm_enable_threshold = declare_parameter<double>("rc_operator.arm_enable_threshold", 2.0);
    config.recovery_threshold = declare_parameter<double>("rc_operator.recovery_threshold", 2.0);
    config.freshness_ns = declare_parameter<std::int64_t>("freshness.rc_ns", 300000000LL);
    return config;
  }
  void publish(const offboard_cpp::OperatorSignals & signals, std::int64_t now_ns)
  {
    if (signals.activation) {activation_until_ns_ = now_ns + edge_hold_ns_;}
    if (signals.recovery) {recovery_until_ns_ = now_ns + edge_hold_ns_;}
    std_msgs::msg::Bool message;
    message.data = !signals.valid || signals.kill;
    kill_pub_->publish(message);
    message.data = signals.valid && now_ns < activation_until_ns_;
    activation_pub_->publish(message);
    message.data = signals.valid && signals.arm_enable;
    arm_pub_->publish(message);
    message.data = signals.valid && now_ns < recovery_until_ns_;
    recovery_pub_->publish(message);
  }
  offboard_cpp::RcOperatorAdapter adapter_;
  std::int64_t edge_hold_ns_{0};
  std::int64_t activation_until_ns_{-1};
  std::int64_t recovery_until_ns_{-1};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr activation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_pub_;
  rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RcOperatorAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
