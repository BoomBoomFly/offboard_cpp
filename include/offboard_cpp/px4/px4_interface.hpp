#pragma once

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "offboard_cpp/mission/mission_types.hpp"

namespace offboard_cpp
{

namespace detail
{
// 命令目标是 PX4 实体；source 是本节点身份。PX4 会将 source 回填到 ACK 的 target，
// 因此 ACK 关联必须同时匹配二者，不能只按 command 号匹配其他发送者的回复。
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
  // 本类是唯一允许写入 /fmu/in/* 的生产边界；时间同步失效时宁可不发布过期时间戳。
  void publish(const MissionActions & actions, std::int64_t steady_now_us);

private:
  struct Data;
  std::unique_ptr<Data> data_;
};

}  // namespace offboard_cpp
