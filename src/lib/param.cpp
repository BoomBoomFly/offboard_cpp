#include <lib/param.hpp>

Param_t::Param_t(){
    msg_timeout.rc = 0.5;
    msg_timeout.odom = 0.5;
    msg_timeout.offboard = 0.5;
    msg_timeout.offboardMode = 0.5;
    msg_timeout.bat = 0.5;

    takeoff_land.enable = true;
    takeoff_land.enable_arm = false;
    takeoff_land.speed = 0.3;
    takeoff_land.height = 1.0;

    low_voltage = 13.2;
    odom_pos_jump = 0.3;

    rc_debug.p = 1.5;
    rc_debug.i = 0.01;
    rc_debug.d = 0.15;
    rc_debug.ch_p = 5;
    rc_debug.ch_i = 6;
    rc_debug.ch_d = 7;
    rc_debug.ch_mode = 8;
    rc_debug.ch_gear = 9;
}

void Param_t::getStaticParam(const std::shared_ptr<rclcpp::Node>& node){
    readStaticParam(node, "msg_timeout.rc", msg_timeout.rc);
    readStaticParam(node, "msg_timeout.odom", msg_timeout.odom);
    readStaticParam(node, "msg_timeout.offboard", msg_timeout.offboard);
    readStaticParam(node, "msg_timeout.offboardMode", msg_timeout.offboardMode);
    readStaticParam(node, "msg_timeout.bat", msg_timeout.bat);
    readStaticParam(node, "takeoff_land.enable", takeoff_land.enable);
    readStaticParam(node, "takeoff_land.enable_arm", takeoff_land.enable_arm);
    readStaticParam(node, "takeoff_land.speed", takeoff_land.speed);
    readStaticParam(node, "takeoff_land.height", takeoff_land.height);
    readStaticParam(node, "low_voltage", low_voltage);
    readStaticParam(node, "odom_pos_jump", odom_pos_jump);
    // 如需调整 ch 名称，请修改下面的三个参数
    readStaticParam(node, "rc_debug.ch_p", rc_debug.ch_p);
    readStaticParam(node, "rc_debug.ch_i", rc_debug.ch_i);
    readStaticParam(node, "rc_debug.ch_d", rc_debug.ch_d);
    readStaticParam(node, "rc_debug.ch_mode", rc_debug.ch_mode);
    readStaticParam(node, "rc_debug.ch_gear", rc_debug.ch_gear);
}

void Param_t::readDynamicParam(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, double& val){
    val = node->declare_parameter<double>(name, val);

    rclcpp::Parameter param(name, val);
    RCLCPP_INFO(node->get_logger(),
                "Read dynamic param %s: %s",
                name.c_str(),
                param.value_to_string().c_str());
}

void Param_t::initDynamicParams(const std::shared_ptr<rclcpp::Node>& node){
    readDynamicParam(node, "rc_debug.p", rc_debug.p);
    readDynamicParam(node, "rc_debug.i", rc_debug.i);
    readDynamicParam(node, "rc_debug.d", rc_debug.d);
}

rcl_interfaces::msg::SetParametersResult Param_t::updateDynamicParams(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<rclcpp::Parameter>& parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto& param : parameters) {
        if (param.get_name() == "rc_debug.p"){
            rc_debug.p = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated dynamic param rc_debug.p: %f", rc_debug.p);
        }
            
        else if (param.get_name() == "rc_debug.i"){
            rc_debug.i = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated dynamic param rc_debug.i: %f", rc_debug.i);
        }
            
        else if (param.get_name() == "rc_debug.d"){
            rc_debug.d = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated dynamic param rc_debug.d: %f", rc_debug.d);
        }
    }
    return result;
}
