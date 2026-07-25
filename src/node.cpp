#include <node.hpp>
#include <topics.hpp>

OffboardControlNode::OffboardControlNode() : Node("offboard_control_node"){
}

void OffboardControlNode::init(const std::shared_ptr<OffboardControlNode>& self) {
    // 获取参数配置
    param.getStaticParam(self);
    param.initDynamicParams(self);
    param_cb_ = this->add_on_set_parameters_callback(
        [this, self](const std::vector<rclcpp::Parameter>& parameters) {
            return param.updateDynamicParams(self, parameters);
        });
    #ifdef TEXT_RC
    this->declare_parameter("mock_rc_mode", 0.0);
    this->declare_parameter("mock_rc_gear", 0.0);
    #endif
    // 创建状态机
    fsm = std::make_unique<CtrlFSM>(param, self);

    // 定义 QoS 策略
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();
    
    // 初始化发布者
    fsm->offboard_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", qos_px4);
    fsm->trigger_pub = this->create_publisher<std_msgs::msg::Bool>(
        "offboard/trigger", qos_px4);
    fsm->offboard_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "fmu/in/offboard_control_mode", qos_px4);
    fsm->vehicle_com_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "fmu/in/vehicle_command", qos_px4);

    // 初始化订阅者
    odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry", qos_px4, 
        [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            fsm->odom_data.feed(msg, param);
        });
    
    state_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        offboard_topics::kVehicleStatus, qos_px4,
        [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            fsm->state_data.feed(msg);
        });

    rc_sub = this->create_subscription<px4_msgs::msg::RcChannels>(
        "fmu/out/rc_channels", qos_px4, 
        [this](px4_msgs::msg::RcChannels::SharedPtr msg) {
            fsm->rc_data.feed(msg, param);
        });

    offboard_sub = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "offboard/cmd", qos_px4, 
        [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
            fsm->offboard_data.feed(msg);
        });

    offboard_mode_sub = this->create_subscription<px4_msgs::msg::OffboardControlMode>(
        "offboard/cmd_mode", qos_px4, 
        [this](px4_msgs::msg::OffboardControlMode::SharedPtr msg) {
            fsm->offboard_mode_data.feed(msg);
        });

    battery_sub = this->create_subscription<px4_msgs::msg::BatteryStatus>(
        "fmu/out/battery_status", qos_px4, 
        [this](px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            fsm->battery_data.feed(msg);
        });

    takeoff_land_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "offboard/takeoff_land", qos_px4, 
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
            fsm->takeoff_land_data.feed_takeoff_land(msg);
        });

    land_detected_sub = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "fmu/out/vehicle_land_detected", qos_px4, 
        [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
            fsm->takeoff_land_data.feed_landed(msg);
        });

    // 创建定时器，定期调用状态机
    timer = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz
        [this]() {
            fsm->FSM();
        });

    RCLCPP_INFO(this->get_logger(), "Offboard Control Node initialized");
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