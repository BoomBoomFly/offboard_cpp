#include "offboard_cpp/offboard_mission_node.hpp"

#include <chrono>

#include "offboard_cpp/mission_controller.hpp"
#include "offboard_cpp/px4_interface.hpp"

#include <boomboom_common_ros2/conversions.hpp>

namespace offboard_cpp
{
namespace
{
std::uint16_t common_state(MissionState state)
{
  switch (state) {
    case MissionState::BOOT:
    case MissionState::WAIT_DISARMED: return BOOMBOOM_MISSION_STATE_DISARMED;
    case MissionState::WAIT_LOCALIZATION: return BOOMBOOM_MISSION_STATE_WAIT_EKF;
    case MissionState::READY: return BOOMBOOM_MISSION_STATE_WAIT_ARM_EDGE;
    case MissionState::OFFBOARD_PRESTREAM: return BOOMBOOM_MISSION_STATE_PRESTREAM;
    case MissionState::REQUEST_OFFBOARD: return BOOMBOOM_MISSION_STATE_REQUEST_OFFBOARD;
    case MissionState::TAKEOFF: return BOOMBOOM_MISSION_STATE_TAKEOFF;
    case MissionState::HOVER: return BOOMBOOM_MISSION_STATE_HOVER;
    case MissionState::RETURN_LOCAL: return BOOMBOOM_MISSION_STATE_RETURN_HOME;
    case MissionState::LAND_REQUEST: return BOOMBOOM_MISSION_STATE_LAND;
    case MissionState::WAIT_LANDED: return BOOMBOOM_MISSION_STATE_LANDED;
    case MissionState::COMPLETE: return BOOMBOOM_MISSION_STATE_PILOT_TAKEOVER;
  }
  return BOOMBOOM_MISSION_STATE_UNKNOWN;
}
}

OffboardMissionNode::OffboardMissionNode() : Node("offboard_mission_node")
{
  MissionConfig config;
  config.takeoff_height_m = declare_parameter<double>("mission.takeoff_height_m", 1.5);
  config.hover_duration_s = declare_parameter<double>("mission.hover_duration_s", 60.0);
  config.prestream_duration_s = declare_parameter<double>("mission.prestream_duration_s", 1.0);
  config.offboard_ack_timeout_s = declare_parameter<double>("mission.offboard_ack_timeout_s", 2.0);
  config.offboard_state_timeout_s = declare_parameter<double>("mission.offboard_state_timeout_s", 2.0);
  config.position_tolerance_m = declare_parameter<double>("mission.position_tolerance_m", 0.20);
  config.velocity_tolerance_mps = declare_parameter<double>("mission.velocity_tolerance_mps", 0.15);
  config.stable_duration_s = declare_parameter<double>("mission.stable_duration_s", 1.0);
  controller_ = std::make_unique<MissionController>(config);
  px4_ = std::make_unique<Px4Interface>(*this);
  status_pub_ = create_publisher<boomboom_common::msg::Status>("/boomboom/mission/status", 10);
  state_pub_ = create_publisher<boomboom_common::msg::State>("/boomboom/mission/state", 10);
  faults_pub_ = create_publisher<boomboom_common::msg::Faults>("/boomboom/mission/faults", 10);
  event_pub_ = create_publisher<boomboom_common::msg::Event>("/boomboom/mission/event", 10);
  cancel_service_ = create_service<std_srvs::srv::Trigger>(
    "/offboard/cancel_mission",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (controller_->state() != MissionState::HOVER) {
        response->success = false;
        response->message = "mission cancellation is only available while hovering";
        return;
      }
      cancel_requested_ = true;
      response->success = true;
      response->message = "mission cancellation accepted: returning to local home";
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { on_timer(); });
}

OffboardMissionNode::~OffboardMissionNode() = default;

void OffboardMissionNode::on_timer()
{
  const auto steady_now_us = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  auto inputs = px4_->snapshot(steady_now_us);
  inputs.cancel_requested = cancel_requested_;
  cancel_requested_ = false;
  const auto actions = controller_->tick(inputs);
  const auto state = static_cast<int>(controller_->state());
  if (state != last_state_) {
    RCLCPP_INFO(get_logger(), "mission state: %s", mission_state_name(controller_->state()));
    const auto status = boomboom_status_make(
      BOOMBOOM_STATUS_DOMAIN_MISSION, BOOMBOOM_STATUS_CONDITION_ACTIVE,
      static_cast<std::uint32_t>(state));
    status_pub_->publish(boomboom_common_ros2::to_msg(status));
    const auto snapshot = boomboom_state_snapshot_make(
      common_state(controller_->state()), controller_->state_reason(), controller_->entered_at_us());
    state_pub_->publish(boomboom_common_ros2::to_msg(snapshot));
    boomboom_event_t event{};
    event.sequence = ++event_sequence_;
    event.time_us = static_cast<std::uint64_t>(steady_now_us);
    event.status = status;
    event.arg0 = state;
    event_pub_->publish(boomboom_common_ros2::to_msg(event));
    last_state_ = state;
  }
  faults_pub_->publish(boomboom_common_ros2::to_msg(controller_->faults()));
  px4_->publish(actions, steady_now_us);
}
}  // namespace offboard_cpp
