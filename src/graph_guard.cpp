#include <graph_guard.hpp>

#include <array>
#include <string>

namespace offboard_cpp
{

bool graph_has_only_gate_writer(const rclcpp::Node & node)
{
  constexpr std::array<const char *, 3> kControlTopics{
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/vehicle_command",
  };
  for (const auto * topic : kControlTopics) {
    const auto endpoints = node.get_publishers_info_by_topic(topic);
    if (endpoints.size() != 1 || endpoints.front().node_name() != node.get_name() ||
      endpoints.front().node_namespace() != node.get_namespace())
    {
      return false;
    }
  }
  return true;
}

}  // namespace offboard_cpp
