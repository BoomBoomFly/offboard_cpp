#include <rclcpp/rclcpp.hpp>

#include "offboard_cpp/offboard_mission_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_cpp::OffboardMissionNode>());
  rclcpp::shutdown();
  return 0;
}
