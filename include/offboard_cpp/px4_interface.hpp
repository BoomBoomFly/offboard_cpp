#pragma once

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "offboard_cpp/mission_types.hpp"

namespace offboard_cpp
{

class Px4Interface
{
public:
  explicit Px4Interface(rclcpp::Node & node);
  ~Px4Interface();
  MissionInputs snapshot(std::int64_t steady_now_us) const;
  void publish(const MissionActions & actions, std::int64_t steady_now_us);

private:
  struct Data;
  rclcpp::Node & node_;
  std::unique_ptr<Data> data_;
};

}  // namespace offboard_cpp
