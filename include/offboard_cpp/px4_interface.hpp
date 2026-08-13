#pragma once

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "offboard_cpp/mission_types.hpp"

namespace offboard_cpp
{

namespace detail
{
constexpr std::uint8_t kPx4TargetSystem = 1;
constexpr std::uint8_t kPx4TargetComponent = 1;
constexpr std::uint8_t kMissionSourceSystem = 1;
constexpr std::uint16_t kMissionSourceComponent = 1;

bool is_mission_offboard_ack(const px4_msgs::msg::VehicleCommandAck & ack);
bool is_local_position_healthy(const px4_msgs::msg::VehicleLocalPosition & position);
}  // namespace detail

class Px4Interface
{
public:
  explicit Px4Interface(rclcpp::Node & node);
  ~Px4Interface();
  MissionInputs snapshot(std::int64_t steady_now_us) const;
  void publish(const MissionActions & actions, std::int64_t steady_now_us);

private:
  struct Data;
  std::unique_ptr<Data> data_;
};

}  // namespace offboard_cpp
