#include <gtest/gtest.h>

#include <safety_gate.hpp>
#include <timestamp_gate.hpp>

namespace {

SafetyGateInputs healthy_inputs() {
  SafetyGateInputs in;
  in.vehicle_status_fresh = true;
  in.odometry_fresh = true;
  in.battery_fresh = true;
  in.timesync_fresh = true;
  in.rc_fresh = true;
  in.kill_fresh = true;
  in.setpoint_fresh = true;
  in.mode_fresh = true;
  in.setpoint_mode_paired = true;
  in.clock_monotonic = true;
  in.kill_latched = false;
  in.single_writer = true;
  in.single_owner = true;
  in.owner_id = 11;
  in.lease_id = 22;
  in.epoch = 1;
  in.sequence = 7;
  return in;
}

void prestream(SafetyGate& gate, SafetyGateInputs& in) {
  gate.tick(1, in);
  for (std::uint64_t now = 50'001; now <= 1'000'001; now += 50'000) {
    gate.tick(now, in);
  }
}

TEST(SafetyGateTest, PositivePrestreamNeverArmsWithoutExplicitHumanAuthorization) {
  SafetyGate gate;
  auto in = healthy_inputs();
  prestream(gate, in);
  EXPECT_EQ(gate.state(), GateState::PRESTREAM);
  EXPECT_TRUE(gate.allows_control_payload());
  EXPECT_EQ(gate.command_to_send(), PendingCommand::NONE);
}

TEST(SafetyGateTest, StaleFeedbackRejectsBeforeAnyFmuInputPublication) {
  SafetyGate gate;
  auto in = healthy_inputs();
  in.odometry_fresh = false;
  gate.tick(1, in);
  EXPECT_TRUE(gate.fault_latched());
  EXPECT_FALSE(gate.allows_control_payload());  // trajectory/mode count == 0
  EXPECT_EQ(gate.command_to_send(), PendingCommand::NONE);  // command count == 0
}

TEST(SafetyGateTest, RcLossBatteryLossAndDuplicateWriterAreLatched) {
  for (auto mutate : {0, 1, 2}) {
    SafetyGate gate;
    auto in = healthy_inputs();
    if (mutate == 0) in.rc_fresh = false;
    if (mutate == 1) in.battery_fresh = false;
    if (mutate == 2) in.single_writer = false;
    gate.tick(1, in);
    EXPECT_EQ(gate.state(), GateState::FAULT_LATCHED);
    EXPECT_FALSE(gate.allows_control_payload());
    EXPECT_EQ(gate.command_to_send(), PendingCommand::NONE);
  }
}

TEST(SafetyGateTest, OwnerLeaseEpochAndClockChangesFailClosed) {
  for (auto mutate : {0, 1, 2, 3}) {
    SafetyGate gate;
    auto in = healthy_inputs();
    gate.tick(1, in);
    if (mutate == 0) in.owner_id++;
    if (mutate == 1) in.lease_id++;
    if (mutate == 2) in.epoch++;
    if (mutate == 3) in.clock_monotonic = false;
    gate.tick(50'001, in);
    EXPECT_EQ(gate.state(), GateState::FAULT_LATCHED);
    EXPECT_FALSE(gate.allows_control_payload());
  }
}

TEST(SafetyGateTest, AckRejectionTimeoutAndCorrelationMismatchProduceNoPayload) {
  SafetyGate gate;
  auto in = healthy_inputs();
  in.manual_activation = true;
  prestream(gate, in);
  ASSERT_EQ(gate.command_to_send(), PendingCommand::MODE);
  gate.observe_ack(1'000'002, CommandAck{400, 1, 1, true, false});
  EXPECT_TRUE(gate.fault_latched());
  EXPECT_FALSE(gate.allows_control_payload());

  SafetyGate rejected_gate;
  in = healthy_inputs();
  in.manual_activation = true;
  prestream(rejected_gate, in);
  rejected_gate.observe_ack(1'000'002, CommandAck{176, 1, 1, false, false});
  EXPECT_TRUE(rejected_gate.fault_latched());
  EXPECT_FALSE(rejected_gate.allows_control_payload());

  SafetyGate timeout_gate;
  in = healthy_inputs();
  in.manual_activation = true;
  prestream(timeout_gate, in);
  timeout_gate.tick(2'100'002, in);
  EXPECT_TRUE(timeout_gate.fault_latched());
  EXPECT_EQ(timeout_gate.command_to_send(), PendingCommand::NONE);
}

TEST(SafetyGateTest, BoundaryPrestreamAndAcceptedAcksRequireTwoHumanGatedSteps) {
  SafetyGate gate;
  auto in = healthy_inputs();
  in.manual_activation = true;
  gate.tick(1, in);
  for (std::uint64_t now = 50'001; now < 1'000'001; now += 50'000) {
    gate.tick(now, in);
  }
  EXPECT_EQ(gate.command_to_send(), PendingCommand::NONE);
  gate.tick(1'000'001, in);
  ASSERT_EQ(gate.command_to_send(), PendingCommand::MODE);
  EXPECT_EQ(gate.take_command_to_send(), PendingCommand::MODE);
  EXPECT_EQ(gate.take_command_to_send(), PendingCommand::NONE);
  gate.observe_ack(1'000'002, CommandAck{176, 1, 1, true, false});
  EXPECT_EQ(gate.state(), GateState::STANDBY_DISARMED);

  gate.tick(1'050'002, in);
  EXPECT_EQ(gate.command_to_send(), PendingCommand::NONE);
  in.manual_arm_enable = true;
  gate.tick(1'100'002, in);
  ASSERT_EQ(gate.command_to_send(), PendingCommand::ARM);
  gate.observe_ack(1'100'003, CommandAck{400, 1, 1, true, false});
  EXPECT_EQ(gate.state(), GateState::ACTIVE);
  EXPECT_TRUE(gate.allows_control_payload());
}

TEST(SafetyGateTest, SourceRestartAndOwnerLossLatchAndKeepAllOutputCountsZero) {
  SafetyGate gate;
  auto in = healthy_inputs();
  gate.tick(1, in);
  gate.source_epoch_changed();
  EXPECT_TRUE(gate.fault_latched());
  EXPECT_FALSE(gate.allows_control_payload());
  EXPECT_EQ(gate.command_to_send(), PendingCommand::NONE);

  SafetyGate owner_loss_gate;
  in = healthy_inputs();
  owner_loss_gate.tick(1, in);
  in.single_owner = false;
  owner_loss_gate.tick(2, in);
  EXPECT_TRUE(owner_loss_gate.fault_latched());
  EXPECT_FALSE(owner_loss_gate.allows_control_payload());
}

TEST(SafetyGateTest, FaultRequiresExplicitManualResetAndDoesNotRestoreActive) {
  SafetyGate gate;
  auto in = healthy_inputs();
  in.kill_latched = true;
  gate.tick(1, in);
  in.kill_latched = false;
  gate.tick(2, in);
  EXPECT_EQ(gate.state(), GateState::FAULT_LATCHED);
  in.manual_reset = true;
  gate.tick(3, in);
  EXPECT_EQ(gate.state(), GateState::WAIT);
  EXPECT_NE(gate.state(), GateState::ACTIVE);
  EXPECT_FALSE(gate.allows_control_payload());
}

TEST(TimestampGateTest, ZeroFrozenBackwardAndRestartAreBounded) {
  TimestampGate timestamps;
  EXPECT_EQ(timestamps.observe(TimestampStream::ODOMETRY, 0, 1), TimestampFault::ZERO);
  EXPECT_TRUE(timestamps.fault_latched());
  timestamps.restart_epoch();
  EXPECT_EQ(timestamps.observe(TimestampStream::ODOMETRY, 100, 1), TimestampFault::NONE);
  EXPECT_EQ(timestamps.observe(TimestampStream::ODOMETRY, 100, 2), TimestampFault::FROZEN);
  timestamps.restart_epoch();
  EXPECT_EQ(timestamps.observe(TimestampStream::ODOMETRY, 100, 1), TimestampFault::NONE);
  EXPECT_EQ(timestamps.observe(TimestampStream::ODOMETRY, 99, 2), TimestampFault::BACKWARD);
}

TEST(TimestampGateTest, FreshnessBoundaryAndTimesyncLossFailClosed) {
  TimestampGate timestamps(500);
  timestamps.observe_timesync(10, 100);
  EXPECT_TRUE(timestamps.timesync_fresh(600));
  EXPECT_FALSE(timestamps.timesync_fresh(601));
  EXPECT_EQ(timestamps.observe(TimestampStream::VEHICLE_STATUS, 20, 100), TimestampFault::NONE);
  EXPECT_TRUE(timestamps.fresh(TimestampStream::VEHICLE_STATUS, 600));
  EXPECT_FALSE(timestamps.fresh(TimestampStream::VEHICLE_STATUS, 601));
}

}  // namespace
