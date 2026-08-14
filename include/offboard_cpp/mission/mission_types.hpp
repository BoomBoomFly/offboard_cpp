#pragma once

#include <array>
#include <cstdint>

namespace offboard_cpp
{

enum class AckResult : std::uint8_t { NONE, ACCEPTED, REJECTED };

struct PositionReset {
  // PX4 EKF2 局部坐标重置的计数与增量。执行器用它平移冻结的 home/target，避免
  // 坐标系跳变被误判为飞机偏离目标。
  std::uint8_t xy_counter{};
  std::uint8_t z_counter{};
  std::uint8_t heading_counter{};
  float delta_x{};
  float delta_y{};
  float delta_z{};
  float delta_heading{};
};

struct MissionConfig {
  // 所有距离为 m、速度为 m/s、时长为 s；位置均使用 PX4 本地 NED 坐标系。
  double takeoff_height_m{1.5};
  double prestream_duration_s{1.0};
  double offboard_ack_timeout_s{2.0};
  double offboard_state_timeout_s{2.0};
  double position_tolerance_m{0.20};
  double velocity_tolerance_mps{0.15};
  double stable_duration_s{1.0};
};

struct MissionInputs {
  // now_us 是单调时钟，用于超时与稳定窗口，不能使用会被系统校时跳变的墙上时间。
  std::int64_t now_us{};
  bool status_fresh{};
  bool local_position_fresh{};
  bool local_position_healthy{};
  bool armed{};
  bool offboard{};
  bool failsafe{};
  bool landed{};
  // 落地检测每收到一帧递增；LAND 后只接受更晚的一帧 landed=true，不能消费旧缓存。
  std::uint64_t landed_sequence{};
  std::uint8_t latest_arming_reason{};
  std::array<double, 3> position_ned{};
  std::array<double, 3> velocity_ned{};
  double heading_rad{};
  PositionReset reset{};
  // 仅记录发给本 mission source 的 Offboard 命令 ACK；序号防止重复读取旧 ACK。
  std::uint64_t ack_sequence{};
  AckResult offboard_ack{AckResult::NONE};
};

struct MissionActions {
  // 执行器的唯一输出。Px4Interface 是这些动作到 /fmu/in/* 的唯一生产写入者。
  bool publish_setpoint{};
  std::array<double, 3> position_ned{};
  double yaw_rad{};
  bool request_offboard{};
  bool request_land{};
};

}  // namespace offboard_cpp
