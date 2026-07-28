#include <node.hpp>
#include <topics.hpp>

OffboardControlNode::OffboardControlNode() : Node("offboard_control_node"){
}

void OffboardControlNode::init(const std::shared_ptr<OffboardControlNode>& self) {
    (void)self;
    // This is intentionally false by default.  The adapter additionally has
    // no production authority envelope, so no automatic arm path exists.
    this->declare_parameter<bool>("takeoff_land.enable_arm", false);
    safety_gate_ = std::make_unique<SafetyGateAdapter>(*this);
    // SafetyGateAdapter owns TimestampStream::VEHICLE_STATUS,
    // TimestampStream::ODOMETRY, TimestampStream::RC,
    // TimestampStream::SETPOINT, TimestampStream::MODE and
    // TimestampStream::COMMAND_ACK validation for these callbacks.

    // 定义 QoS 策略
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();
    
    odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry", qos_px4, 
        [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            safety_gate_->observe_odometry(*msg, this->now().nanoseconds() / 1000);
        });
    
    state_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        offboard_topics::kVehicleStatus, qos_px4,
        [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            safety_gate_->observe_vehicle_status(*msg, this->now().nanoseconds() / 1000);
        });

    timesync_sub = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        "fmu/out/timesync_status", qos_px4,
        [this](px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
            safety_gate_->observe_timesync(*msg, this->now().nanoseconds() / 1000);
        });

    rc_sub = this->create_subscription<px4_msgs::msg::RcChannels>(
        "fmu/out/rc_channels", qos_px4, 
        [this](px4_msgs::msg::RcChannels::SharedPtr msg) {
            safety_gate_->observe_rc(*msg, this->now().nanoseconds() / 1000);
        });

    offboard_sub = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "offboard/cmd", qos_px4, 
        [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
            safety_gate_->observe_setpoint(*msg, this->now().nanoseconds() / 1000);
        });

    offboard_mode_sub = this->create_subscription<px4_msgs::msg::OffboardControlMode>(
        "offboard/cmd_mode", qos_px4, 
        [this](px4_msgs::msg::OffboardControlMode::SharedPtr msg) {
            safety_gate_->observe_mode(*msg, this->now().nanoseconds() / 1000);
        });

    battery_sub = this->create_subscription<px4_msgs::msg::BatteryStatus>(
        "fmu/out/battery_status", qos_px4, 
        [this](px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            safety_gate_->observe_battery(*msg, this->now().nanoseconds() / 1000);
        });

    ack_sub = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "fmu/out/vehicle_command_ack", qos_px4,
        [this](px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
            safety_gate_->observe_ack(*msg, this->now().nanoseconds() / 1000);
        });

    // 创建定时器，定期调用状态机
    timer = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz
        [this]() {
            if (safety_gate_->timestamp_fault_latched_() ||
                !safety_gate_->timestamp_config_valid_()) {
                return;
            }
            safety_gate_->tick(this->now().nanoseconds() / 1000,
                               graph_has_only_gate_writer());
        });

    RCLCPP_INFO(this->get_logger(), "Offboard Control Node initialized");
}

bool OffboardControlNode::graph_has_only_gate_writer() const {
    return safety_gate_->graph_has_only_gate_writer();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建控制节点
    auto node = std::make_shared<OffboardControlNode>();
    
    // 初始化节点（必须在 shared_ptr 创建后调用）
    node->init(node);
    
    RCLCPP_INFO(node->get_logger(), "Offboard Control Node 启动成功！");
    
    // 运行节点
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
