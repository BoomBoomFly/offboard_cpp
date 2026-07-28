#ifndef NODE_HPP
#define NODE_HPP

#include "rclcpp/qos.hpp"
#include <safety_gate_adapter.hpp>

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode();
    void init(const std::shared_ptr<OffboardControlNode>& self);

private:

    std::unique_ptr<SafetyGateAdapter> safety_gate_;

    // 订阅器
    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr state_sub;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr offboard_sub;
    rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_sub;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer;
    bool graph_has_only_gate_writer() const;
};

#endif // NODE_HPP
