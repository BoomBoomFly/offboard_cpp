#include <rclcpp/rclcpp.hpp>

#include "offboard_cpp/gateway/offboard_gateway_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_cpp::OffboardGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
