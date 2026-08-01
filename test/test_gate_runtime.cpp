#include <chrono>
#include <array>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <graph_guard.hpp>
#include <safety_gate.hpp>
#include <safety_gate_adapter.hpp>

namespace
{

using offboard_cpp::GateDecision;
using offboard_cpp::GateInputs;
using offboard_cpp::SafetyGate;

std::shared_ptr<rclcpp::Context> test_context;

GateInputs ready_inputs()
{
  GateInputs inputs;
  inputs.vehicle_status_fresh = true;
  inputs.odometry_fresh = true;
  inputs.timesync_fresh = true;
  inputs.rc_fresh = true;
  inputs.kill_fresh = true;
  inputs.setpoint_fresh = true;
  inputs.mode_fresh = true;
  inputs.setpoint_mode_paired = true;
  inputs.authority.fresh = true;
  inputs.authority.single_writer = true;
  inputs.authority.single_owner = true;
  inputs.authority.owner_id = "operator-a";
  inputs.authority.lease_id = "lease-1";
  inputs.authority.epoch = "epoch-1";
  inputs.authority.sequence = 7;
  inputs.vehicle_status_generation = 1;
  return inputs;
}

bool wait_for(const std::function<bool()> & predicate, rclcpp::executors::SingleThreadedExecutor & executor)
{
  for (int count = 0; count < 200; ++count) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

class GateRuntimeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = test_context;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    const auto node_options = rclcpp::NodeOptions().context(test_context);
    gate_node_ = std::make_shared<rclcpp::Node>("offboard_control_node", node_options);
    observer_node_ = std::make_shared<rclcpp::Node>("offboard_gate_observer", node_options);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    adapter_ = std::make_unique<offboard_cpp::SafetyGateAdapter>(*gate_node_, qos);
    setpoint_sub_ = observer_node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos,
      [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr) { ++setpoint_count_; });
    mode_sub_ = observer_node_->create_subscription<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos,
      [this](px4_msgs::msg::OffboardControlMode::SharedPtr) { ++mode_count_; });
    command_sub_ = observer_node_->create_subscription<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos,
      [this](px4_msgs::msg::VehicleCommand::SharedPtr) { ++command_count_; });
    executor_->add_node(gate_node_);
    executor_->add_node(observer_node_);
    ASSERT_TRUE(wait_for(
      [this]() { return offboard_cpp::graph_has_only_gate_writer(*gate_node_); }, *executor_));
  }

  void TearDown() override
  {
    executor_->remove_node(observer_node_);
    executor_->remove_node(gate_node_);
  }

  void expect_no_px4_input(const GateDecision & decision)
  {
    px4_msgs::msg::TrajectorySetpoint setpoint;
    px4_msgs::msg::OffboardControlMode mode;
    adapter_->apply(decision, setpoint, mode, 1000);
    for (int count = 0; count < 10; ++count) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(setpoint_count_, 0U);
    EXPECT_EQ(mode_count_, 0U);
    EXPECT_EQ(command_count_, 0U);
  }

  GateDecision rejection_after_prestream(const std::function<void(GateInputs &)> & alter)
  {
    SafetyGate gate("operator-a", "lease-1", "epoch-1");
    auto inputs = ready_inputs();
    EXPECT_FALSE(gate.request_manual_activation(0, inputs).fault_latched);
    EXPECT_FALSE(gate.tick(0, inputs).fault_latched);
    alter(inputs);
    return gate.tick(1, inputs);
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr gate_node_;
  rclcpp::Node::SharedPtr observer_node_;
  std::unique_ptr<offboard_cpp::SafetyGateAdapter> adapter_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr command_sub_;
  std::size_t setpoint_count_{0};
  std::size_t mode_count_{0};
  std::size_t command_count_{0};
};

TEST_F(GateRuntimeTest, FoxyGraphRejectsDuplicateWriterAndSameNameOtherNamespace)
{
  const auto node_options = rclcpp::NodeOptions().context(test_context);
  auto impostor = std::make_shared<rclcpp::Node>(
    "offboard_control_node", "/impostor", node_options);
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto duplicate = impostor->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos);
  (void)duplicate;
  executor_->add_node(impostor);
  ASSERT_TRUE(wait_for(
    [this]() { return !offboard_cpp::graph_has_only_gate_writer(*gate_node_); }, *executor_));
  executor_->remove_node(impostor);
}

TEST_F(GateRuntimeTest, EveryRejectedPathProducesZeroPx4Inputs)
{
  const std::array<std::function<void(GateInputs &)>, 10> rejectors{
    [](GateInputs & value) { value.odometry_fresh = false; },
    [](GateInputs & value) { value.timesync_fresh = false; },
    [](GateInputs & value) { value.kill_latched = true; },
    [](GateInputs & value) { value.setpoint_mode_paired = false; },
    [](GateInputs & value) { value.authority.owner_id = "other"; },
    [](GateInputs & value) { value.authority.lease_id = "other"; },
    [](GateInputs & value) { value.authority.epoch = "other"; },
    [](GateInputs & value) { value.authority.single_writer = false; },
    [](GateInputs & value) { value.clock_monotonic = false; },
    [](GateInputs & value) { value.setpoint_fresh = false; },
  };
  for (const auto & reject : rejectors) {
    const auto decision = rejection_after_prestream(reject);
    ASSERT_TRUE(decision.fault_latched);
    expect_no_px4_input(decision);
  }

  auto manual_inputs = ready_inputs();
  SafetyGate auto_owner_gate("operator-a", "lease-1", "epoch-1", true, 1000000000LL, 20);
  ASSERT_EQ(auto_owner_gate.tick(0, manual_inputs).state, offboard_cpp::GateState::PRESTREAM);
  manual_inputs.authority.single_owner = false;
  expect_no_px4_input(auto_owner_gate.tick(1, manual_inputs));

  auto inputs = ready_inputs();
  SafetyGate gate("operator-a", "lease-1", "epoch-1", true, 1000000000LL, 20);
  ASSERT_FALSE(gate.request_manual_activation(0, inputs).fault_latched);
  ASSERT_EQ(gate.tick(0, inputs).state, offboard_cpp::GateState::PRESTREAM);
  for (int count = 1; count < 20; ++count) {
    gate.tick(count * 50000000LL, inputs);
  }
  ASSERT_EQ(gate.tick(1000000000LL, inputs).command, offboard_cpp::CommandKind::SET_MODE_OFFBOARD);
  auto rejected_ack = offboard_cpp::CommandAck{};
  rejected_ack.command = SafetyGate::kVehicleCmdDoSetMode;
  rejected_ack.target_system = SafetyGate::kTargetSystem;
  rejected_ack.target_component = SafetyGate::kTargetComponent;
  rejected_ack.from_external = true;
  rejected_ack.result = offboard_cpp::AckResult::REJECTED;
  expect_no_px4_input(gate.observe_ack(1050000000LL, rejected_ack, inputs.authority));
}

}  // namespace

int main(int argc, char ** argv)
{
  test_context = std::make_shared<rclcpp::Context>();
  test_context->init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  test_context->shutdown("test completed");
  test_context.reset();
  return result;
}
