#include "offboard_cpp/px4/px4_interface.hpp"

#include <cmath>
#include <chrono>
#include <limits>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace offboard_cpp
{
namespace detail
{
bool is_mission_offboard_ack(const px4_msgs::msg::VehicleCommandAck & ack)
{
  // PX4 将 VehicleCommand.source_* 回填到 VehicleCommandAck.target_*；只按 command
  // 匹配会把 QGC 或其他节点的 DO_SET_MODE 回复错误地当作本任务 ACK。
  return ack.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
         ack.target_system == kMissionSourceSystem &&
         ack.target_component == kMissionSourceComponent;
}

bool is_mission_land_ack(const px4_msgs::msg::VehicleCommandAck & ack)
{
  return ack.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND &&
         ack.target_system == kMissionSourceSystem &&
         ack.target_component == kMissionSourceComponent;
}

bool is_local_position_healthy(const px4_msgs::msg::VehicleLocalPosition & position)
{
  return position.xy_valid && position.z_valid && position.v_xy_valid && position.v_z_valid &&
         position.heading_good_for_control &&
         std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(position.vx) &&
         std::isfinite(position.vy) && std::isfinite(position.vz) &&
         std::isfinite(position.heading);
}
}  // namespace detail

namespace
{
constexpr std::int64_t kStatusMaxAgeUs = 1200000;
constexpr std::int64_t kPositionMaxAgeUs = 250000;
constexpr std::int64_t kLandMaxAgeUs = 1200000;
constexpr std::int64_t kTimesyncMaxAgeUs = 2500000;

bool fresh(std::int64_t received_us, std::int64_t now_us, std::int64_t max_age_us)
{
  // received_us 与 now_us 都来自 steady_clock；负 age 代表时钟域/调用约束已被破坏。
  return received_us >= 0 && now_us >= received_us && now_us - received_us < max_age_us;
}

std::int64_t steady_now_us()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

struct Px4Interface::Data
{
  // 回调写入最近一帧，timer 通过 snapshot 读取。同一节点以默认单线程 spin 运行，
  // 因而不额外加锁；若改为 MultiThreadedExecutor，必须同步这些共享字段。
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub;
  px4_msgs::msg::VehicleStatus status{};
  px4_msgs::msg::VehicleLocalPosition position{};
  bool landed{};
  std::uint64_t landed_sequence{};
  AckResult ack{AckResult::NONE};
  AckResult land_ack{AckResult::NONE};
  std::uint64_t land_ack_sequence{};
  std::uint64_t ack_sequence{};
  std::int64_t status_received_us{-1};
  std::int64_t position_received_us{-1};
  std::int64_t land_received_us{-1};
  std::int64_t timesync_received_us{-1};
  std::uint64_t px4_timestamp_us{};
};

Px4Interface::~Px4Interface() = default;

Px4Interface::Px4Interface(rclcpp::Node & node) : data_(std::make_unique<Data>())
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  // 这些是整个生产系统唯一的 PX4 控制写入方；其他包只能经 ExecuteFlight 请求飞行。
  data_->mode_pub = node.create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos);
  data_->setpoint_pub = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos);
  data_->command_pub = node.create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos);
  data_->status_sub = node.create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", qos, [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
      data_->status = *msg;
      data_->status_received_us = steady_now_us();
    });
  data_->position_sub = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos,
    [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
      data_->position = *msg;
      data_->position_received_us = steady_now_us();
    });
  data_->land_sub = node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "/fmu/out/vehicle_land_detected", qos,
    [this](px4_msgs::msg::VehicleLandDetected::ConstSharedPtr msg) {
      data_->landed = msg->landed;
      ++data_->landed_sequence;
      data_->land_received_us = steady_now_us();
    });
  data_->ack_sub = node.create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", qos,
    [this](px4_msgs::msg::VehicleCommandAck::ConstSharedPtr msg) {
      // IN_PROGRESS 是中间状态，不应误判为拒绝。
      if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {
        return;
      }
      if (detail::is_mission_land_ack(*msg)) {
        data_->land_ack = msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED ?
          AckResult::ACCEPTED : AckResult::REJECTED;
        ++data_->land_ack_sequence;
      }
      if (detail::is_mission_offboard_ack(*msg)) {
        // 用匹配 ACK 到达次数驱动状态机；不复用旧 ACK，也不对拒绝结果自动重试。
        data_->ack = msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED ?
          AckResult::ACCEPTED : AckResult::REJECTED;
        ++data_->ack_sequence;
      }
    });
  data_->timesync_sub = node.create_subscription<px4_msgs::msg::TimesyncStatus>(
    "/fmu/out/timesync_status", qos, [this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr msg) {
      if (msg->timestamp == 0) { return; }
      data_->px4_timestamp_us = msg->timestamp;
      data_->timesync_received_us = steady_now_us();
    });
}

MissionInputs Px4Interface::snapshot(std::int64_t steady_now_us) const
{
  MissionInputs inputs;
  inputs.now_us = steady_now_us;
  inputs.timesync_fresh = fresh(data_->timesync_received_us, steady_now_us, kTimesyncMaxAgeUs);
  inputs.auto_land = data_->status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
  inputs.land_ack = data_->land_ack;
  inputs.land_ack_sequence = data_->land_ack_sequence;
  inputs.status_fresh = fresh(data_->status_received_us, steady_now_us, kStatusMaxAgeUs);
  inputs.local_position_fresh = fresh(data_->position_received_us, steady_now_us, kPositionMaxAgeUs);
  inputs.local_position_healthy = detail::is_local_position_healthy(data_->position);
  inputs.armed = data_->status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  inputs.offboard = data_->status.nav_state ==
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  inputs.failsafe = data_->status.failsafe || data_->status.failsafe_and_user_took_over;
  inputs.landed = fresh(data_->land_received_us, steady_now_us, kLandMaxAgeUs) && data_->landed;
  inputs.landed_sequence = data_->landed_sequence;
  inputs.latest_arming_reason = data_->status.latest_arming_reason;
  inputs.position_ned = {data_->position.x, data_->position.y, data_->position.z};
  inputs.velocity_ned = {data_->position.vx, data_->position.vy, data_->position.vz};
  inputs.heading_rad = data_->position.heading;
  inputs.reset = {data_->position.xy_reset_counter, data_->position.z_reset_counter,
    data_->position.heading_reset_counter, data_->position.delta_xy[0],
    data_->position.delta_xy[1], data_->position.delta_z, data_->position.delta_heading};
  inputs.ack_sequence = data_->ack_sequence;
  inputs.offboard_ack = data_->ack;
  return inputs;
}

bool Px4Interface::publish(const MissionActions & actions, std::int64_t steady_now_us)
{
  // 使用 timesync 的 DDS 边界时间域；PX4 反序列化时减同步 offset。
  if (!fresh(data_->timesync_received_us, steady_now_us, kTimesyncMaxAgeUs))
  {
     return false;
  }
  const auto timestamp = data_->px4_timestamp_us +
    static_cast<std::uint64_t>(steady_now_us - data_->timesync_received_us);
  if (actions.publish_setpoint)
  {
    // 只声明 position 控制；速度、加速度和 jerk 为 NaN，避免 PX4 将它们当作约束。
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = timestamp;
    mode.position = true;
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = timestamp;
    setpoint.position = {static_cast<float>(actions.position_ned[0]),
      static_cast<float>(actions.position_ned[1]), static_cast<float>(actions.position_ned[2])};
    setpoint.velocity = {nan, nan, nan};
    setpoint.acceleration = {nan, nan, nan};
    setpoint.jerk = {nan, nan, nan};
    setpoint.yaw = static_cast<float>(actions.yaw_rad);
    setpoint.yawspeed = nan;
    data_->mode_pub->publish(mode);
    data_->setpoint_pub->publish(setpoint);
  }
  if (actions.request_offboard || actions.request_land)
  {
    px4_msgs::msg::VehicleCommand command{};
    command.timestamp = timestamp;
    command.target_system = detail::kPx4TargetSystem;
    command.target_component = detail::kPx4TargetComponent;
    command.source_system = detail::kMissionSourceSystem;
    command.source_component = detail::kMissionSourceComponent;
    command.from_external = true;
    if (actions.request_offboard)
    {
      // PX4 DO_SET_MODE: param1=custom mode enabled, param2=PX4 Offboard main mode。
      command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
      command.param1 = 1.0F;
      command.param2 = 6.0F;
    }
    else
    {
      // LAND 不携带位置目标；一旦状态机发出该请求，就转入不可取消的落地确认流程。
      command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    }
    data_->command_pub->publish(command);
  }
  return true;
}
}  // namespace offboard_cpp
