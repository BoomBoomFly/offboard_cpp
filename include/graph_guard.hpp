#ifndef OFFBOARD_CPP_GRAPH_GUARD_HPP
#define OFFBOARD_CPP_GRAPH_GUARD_HPP

#include <rclcpp/rclcpp.hpp>

namespace offboard_cpp
{

// ROS 2 graph endpoint names are split into a node name and namespace.  Check
// both fields, rather than accepting a same-named node in another namespace.
bool graph_has_only_gate_writer(const rclcpp::Node & node);

}  // namespace offboard_cpp

#endif  // OFFBOARD_CPP_GRAPH_GUARD_HPP
