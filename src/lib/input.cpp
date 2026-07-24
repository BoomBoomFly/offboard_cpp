#include "lib/input.hpp"
#include <algorithm>
#include <array>


// RC_Data_t
// 初始化
RC_Data_t::RC_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    mode = -1.0;
    gear = -1.0;
    last_mode = -1.0;
    last_gear = -1.0;
    p = 0.0;
    i = 0.0;
    d = 0.0;
    for (int i = 0; i < 4; ++i){
        ch[i] = 0.0;
    }
}

// 检测 RC 是否合法
bool RC_Data_t::check_validity() const{
    return has_received && valid && !msg.signal_lost;
}

bool RC_Data_t::is_fresh(const rclcpp::Time& now_time, double timeout_s) const{
    if (!check_validity() || timeout_s < 0.0 ||
        now_time.get_clock_type() != rcv_stamp.get_clock_type()){
        return false;
    }

    const double age_s = (now_time - rcv_stamp).seconds();
    return age_s >= 0.0 && age_s < timeout_s;
}

void RC_Data_t::invalidate(const char* reason){
    valid = false;
    mode = -1.0;
    gear = -1.0;
    p = 0.0;
    i = 0.0;
    d = 0.0;
    std::fill_n(ch, 4, 0.0);
    is_hover_mode = false;
    enter_hover_mode = false;
    is_offboard = false;
    enter_offboard = false;
    have_init_last_mode = false;
    have_init_last_gear = false;
    last_mode = -1.0;
    last_gear = -1.0;
    RCLCPP_ERROR(node_->get_logger(), "Rejecting RC frame: %s", reason);
}

// 检测摇杆是否回正
bool RC_Data_t::check_centered(){
    return valid && std::abs(ch[0]) < 1e-5 && std::abs(ch[1]) < 1e-5 &&
           std::abs(ch[2]) < 1e-5 && std::abs(ch[3]) < 1e-5;
}

// 映射遥控器
void RC_Data_t::feed(px4_msgs::msg::RcChannels::SharedPtr pMsg, const Param_t& param){
    if (!pMsg){
        invalidate("null message");
        return;
    }

    msg = *pMsg;
    rcv_stamp = node_->now();
    has_received = true;
    valid = false;

    if (msg.signal_lost){
        invalidate("signal_lost is set");
        return;
    }

    constexpr std::size_t STICK_CHANNEL_COUNT = 4;
    const std::array<int, 5> configured_channels{
        param.rc_debug.ch_p,
        param.rc_debug.ch_i,
        param.rc_debug.ch_d,
        param.rc_debug.ch_mode,
        param.rc_debug.ch_gear,
    };

    std::size_t required_channel_count = STICK_CHANNEL_COUNT;
    for (const int channel : configured_channels){
        if (channel < 0 ||
            static_cast<std::size_t>(channel) >= msg.channels.size()){
            invalidate("configured channel index is out of bounds");
            return;
        }
        required_channel_count =
            std::max(required_channel_count, static_cast<std::size_t>(channel + 1));
    }

    if (msg.channel_count > msg.channels.size() ||
        msg.channel_count < required_channel_count){
        invalidate("channel_count does not cover all required channels");
        return;
    }

    const auto normalized_channel_is_valid = [this](std::size_t channel){
        constexpr float RANGE_EPSILON = 1e-4F;
        const float value = msg.channels[channel];
        return std::isfinite(value) &&
               value >= -1.0F - RANGE_EPSILON &&
               value <= 1.0F + RANGE_EPSILON;
    };

    for (std::size_t channel = 0; channel < STICK_CHANNEL_COUNT; ++channel){
        if (!normalized_channel_is_valid(channel)){
            invalidate("stick channel is non-finite or outside normalized range");
            return;
        }
    }
    for (const int channel : configured_channels){
        if (!normalized_channel_is_valid(static_cast<std::size_t>(channel))){
            invalidate("mapped channel is non-finite or outside normalized range");
            return;
        }
    }


    // 提取遥控器通道数据（根据实际通道映射调整）
    for(std::size_t channel = 0; channel < STICK_CHANNEL_COUNT; ++channel){
        ch[channel] = static_cast<double>(msg.channels[channel]);
        // 对处于死区的数据进行处理
        // 防止对摇杆过度敏感
        if (ch[channel] > DEAD_ZONE){
            ch[channel] = (ch[channel] - DEAD_ZONE) / (1 - DEAD_ZONE);
        }
        else if (ch[channel] < - DEAD_ZONE){
            ch[channel] = (ch[channel] + DEAD_ZONE) / (1 - DEAD_ZONE);
        }
        else{
            ch[channel] = 0.0;
        }
    }

    mode = static_cast<double>(msg.channels[param.rc_debug.ch_mode]);
    gear = static_cast<double>(msg.channels[param.rc_debug.ch_gear]);
    #ifdef TEXT_RC
        double mock_mode = 0.0;
        double mock_gear = 0.0;
        node_->get_parameter_or("mock_rc_mode", mock_mode, 0.0);
        node_->get_parameter_or("mock_rc_gear", mock_gear, 0.0);
        if (!std::isfinite(mock_mode) || !std::isfinite(mock_gear) ||
            mock_mode < -1.0 || mock_mode > 1.0 ||
            mock_gear < -1.0 || mock_gear > 1.0){
            invalidate("mock RC switch is non-finite or outside normalized range");
            return;
        }
        mode = mock_mode;
        gear = mock_gear;
    #endif
    // 这里归一到了 [0, 1] ，如有别的需求，自行进行修改
    p = (static_cast<double>(msg.channels[param.rc_debug.ch_p]) + 1.0) / 2.0;
    i = (static_cast<double>(msg.channels[param.rc_debug.ch_i]) + 1.0) / 2.0;
    d = (static_cast<double>(msg.channels[param.rc_debug.ch_d]) + 1.0) / 2.0;
    valid = true;
    enter_hover_mode = false;
    enter_offboard = false;

    // 检测模式切换
    if (!have_init_last_mode) {
        last_mode = mode;
        have_init_last_mode = true;
    }

    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }

    // 检测是否进入悬停模式
    if (mode > API_MODE_THRESHOLD_VALUE){
        if (last_mode < API_MODE_THRESHOLD_VALUE){
            enter_hover_mode = true;
        }
        else{
            enter_hover_mode = false;
        }
        is_hover_mode = true;
    }
    else{
        is_hover_mode = false;
        is_offboard = false;
        enter_offboard = false;
    }

    // 只有在悬浮模式
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE){
            enter_offboard = true;
        }
        else if (gear < GEAR_SHIFT_VALUE){
            enter_offboard = false;
        }
        if (gear > GEAR_SHIFT_VALUE){
            is_offboard = true;
        }
        else{
            is_offboard = false;
        }
    }

    last_mode = mode;
    last_gear = gear;
}

// Odom_Data_t
// 初始化
Odom_Data_t::Odom_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
    q.setIdentity();
    recv_new_msg = false;
}

// 回调
void Odom_Data_t::feed(px4_msgs::msg::VehicleOdometry::SharedPtr pMsg, const Param_t& param){
    rcv_stamp =  node_ -> now();
    msg = *pMsg;
    recv_new_msg = true;
    bool is_first_msg = (rcv_stamp.nanoseconds() == 0);     // 判断是不是第一帧

    Eigen::Vector3d new_p(msg.position[0], msg.position[1], msg.position[2]);

    //  突变检查逻辑
    if (is_first_msg) {
        pos_jump = false; // 第一帧跳过检查
    } 
    else {
        if ((new_p - p).norm() > param.odom_pos_jump) {
            RCLCPP_ERROR(node_->get_logger(), "Odom 数据发生跳变!");
            pos_jump = true;
        } 
        else {
            pos_jump = false;
        }
    }

    // 更新 p 缓存值
    p = new_p;

    // 提取 v, q, w
    v << msg.velocity[0], msg.velocity[1], msg.velocity[2];
    q = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    w << msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2];

    // 处理机体坐标系速度，如果 Odom 里的速度是相对于机体坐标系的，需要旋转到世界坐标系
    #ifdef VEL_IN_BODY 
        v = q * v; 

        static int count = 0;
        if (count++ % 500 == 0) {
            RCLCPP_WARN(node_->get_logger(), "VEL_IN_BODY is enabled! Velocity has been rotated.");
        }
    #endif

    // 检查频率
    static int one_min_count = 9999;
    static rclcpp::Time last_clear_count_time = node_->now();;

    if ((rcv_stamp - last_clear_count_time).seconds() > 1.0) {
        if (one_min_count < 100 && last_clear_count_time.nanoseconds() != 0) {
            RCLCPP_WARN(node_->get_logger(), "Odom frequency too low: %d Hz", one_min_count);
        }
        one_min_count = 0;
        last_clear_count_time = rcv_stamp;
    }
    one_min_count++;
}

// State_Data_t
// 初始化
State_Data_t::State_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
}

// 记录当前模式
void State_Data_t::feed(px4_msgs::msg::VehicleStatus::SharedPtr pMsg){
    current_state = *pMsg;
}

// OffboardMode_Data_t
// 初始化
Offboard_Data_t::Offboard_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 缓存接收到的目标点
void Offboard_Data_t::feed(px4_msgs::msg::TrajectorySetpoint::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();

    // 提取 p, v, a, j, yaw, yaw_rate
    p << msg.position[0], msg.position[1], msg.position[2];
    v << msg.velocity[0], msg.velocity[1], msg.velocity[2];
    a << msg.acceleration[0], msg.acceleration[1], msg.acceleration[2];
    j << msg.jerk[0], msg.jerk[1], msg.jerk[2];
    yaw = msg.yaw;
    yaw_rate = msg.yawspeed;
}

// OffboardMode_Data_t
// 初始化
Offboard_Mode_Data_t::Offboard_Mode_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 回调
void Offboard_Mode_Data_t::feed(px4_msgs::msg::OffboardControlMode::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();
}

// Battery_Data_t
//初始化
Battery_Data_t::Battery_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 获取电池电量信息
void Battery_Data_t::feed(px4_msgs::msg::BatteryStatus::SharedPtr pMsg){
    msg = *pMsg;
    rcv_stamp =  node_ -> now();
    volt = msg.voltage_v;
    percentage = msg.remaining;
    flyTime = msg.time_remaining_s;
    warning = msg.warning;
}

// Takeoff_Land_Data_t
// 初始化
Takeoff_Land_Data_t::Takeoff_Land_Data_t(const rclcpp::Node::SharedPtr& node) : node_(node){
    rcv_stamp =  node_ -> now();
}

// 获取起飞信息
void Takeoff_Land_Data_t::feed_takeoff_land(std_msgs::msg::UInt8::SharedPtr pMsg){
    rcv_stamp =  node_ -> now();
    triggered = true;
    takeoff_land_cmd = pMsg->data;
}

void Takeoff_Land_Data_t::feed_landed(px4_msgs::msg::VehicleLandDetected::SharedPtr pMsg){
    land_msg = *pMsg;
    landed = pMsg->landed;
}
