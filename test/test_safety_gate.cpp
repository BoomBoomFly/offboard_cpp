#include <cassert>
#include <iostream>

#include <safety_gate.hpp>
#include <timestamp_gate.hpp>

using offboard_cpp::AckResult;
using offboard_cpp::CommandAck;
using offboard_cpp::CommandKind;
using offboard_cpp::GateInputs;
using offboard_cpp::GateState;
using offboard_cpp::SafetyGate;
using offboard_cpp::TimestampGate;
using offboard_cpp::TimestampResult;
using offboard_cpp::TimestampStream;

namespace
{
GateInputs ready_inputs(std::uint64_t sequence = 7)
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
  inputs.authority.sequence = sequence;
  inputs.vehicle_status_generation = 1;
  return inputs;
}

SafetyGate gate(bool enable_arm = false)
{
  return SafetyGate("operator-a", "lease-1", "epoch-1", enable_arm);
}

void prestream_to_mode(SafetyGate & value, GateInputs & inputs)
{
  assert(value.request_manual_activation(0, inputs).state == GateState::WAIT);
  assert(value.tick(0, inputs).state == GateState::PRESTREAM);
  for (int i = 1; i < 20; ++i) {
    assert(value.tick(i * 50000000LL, inputs).publish_setpoint);
  }
  const auto request = value.tick(1000000000LL, inputs);
  assert(request.state == GateState::REQUEST_MODE);
  assert(request.command == CommandKind::SET_MODE_OFFBOARD);
}

CommandAck accepted(std::uint32_t command)
{
  CommandAck ack;
  ack.command = command;
  ack.target_system = SafetyGate::kTargetSystem;
  ack.target_component = SafetyGate::kTargetComponent;
  ack.status_generation = 1;
  ack.from_external = true;
  ack.result = AckResult::ACCEPTED;
  return ack;
}

void test_happy_path_disarmed()
{
  auto value = gate(true);
  auto inputs = ready_inputs();
  inputs.manual_arm_enable = true;
  prestream_to_mode(value, inputs);
  assert(!value.observe_ack(
    1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority).fault_latched);
  inputs.vehicle_in_offboard = true;
  ++inputs.vehicle_status_generation;
  const auto arm_request = value.tick(1100000000LL, inputs);
  assert(arm_request.state == GateState::REQUEST_ARM && arm_request.command == CommandKind::ARM);
  auto arm_ack = accepted(SafetyGate::kVehicleCmdArmDisarm);
  arm_ack.status_generation = inputs.vehicle_status_generation;
  assert(!value.observe_ack(1110000000LL, arm_ack, inputs.authority).fault_latched);
  inputs.vehicle_armed = true;
  ++inputs.vehicle_status_generation;
  const auto active = value.tick(1120000000LL, inputs);
  assert(active.state == GateState::ACTIVE && active.publish_setpoint && active.publish_mode);
}

void test_ack_reject_timeout_command_and_sequence_fail_closed()
{
  auto inputs = ready_inputs();
  auto no_pending = gate(true);
  assert(no_pending.observe_ack(
    0, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority).fault_latched);

  auto rejected = gate(true);
  prestream_to_mode(rejected, inputs);
  auto bad = accepted(SafetyGate::kVehicleCmdDoSetMode);
  bad.result = AckResult::REJECTED;
  assert(rejected.observe_ack(1050000000LL, bad, inputs.authority).fault_latched);

  auto wrong_command = gate(true);
  prestream_to_mode(wrong_command, inputs);
  assert(wrong_command.observe_ack(
    1050000000LL, accepted(SafetyGate::kVehicleCmdArmDisarm), inputs.authority).fault_latched);

  auto wrong_sequence = gate(true);
  prestream_to_mode(wrong_sequence, inputs);
  auto changed_authority = inputs.authority;
  ++changed_authority.sequence;
  assert(wrong_sequence.observe_ack(
    1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), changed_authority).fault_latched);

  auto wrong_target = gate(true);
  prestream_to_mode(wrong_target, inputs);
  auto target = accepted(SafetyGate::kVehicleCmdDoSetMode);
  target.target_component = 2;
  assert(wrong_target.observe_ack(1050000000LL, target, inputs.authority).fault_latched);

  auto wrong_origin = gate(true);
  prestream_to_mode(wrong_origin, inputs);
  auto origin = accepted(SafetyGate::kVehicleCmdDoSetMode);
  origin.from_external = false;
  assert(wrong_origin.observe_ack(1050000000LL, origin, inputs.authority).fault_latched);

  auto timeout = gate(true);
  prestream_to_mode(timeout, inputs);
  assert(timeout.tick(4000000001LL, inputs).fault_latched);

  auto late = gate(true);
  prestream_to_mode(late, inputs);
  assert(late.observe_ack(
    4000000001LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority).fault_latched);
}

void test_every_readiness_failure_and_restart_is_zero_output()
{
  auto inputs = ready_inputs();
  const auto check = [&inputs](auto alter) {
    auto value = gate();
    value.request_manual_activation(0, inputs);
    value.tick(0, inputs);
    alter(inputs);
    const auto result = value.tick(1, inputs);
    assert(result.fault_latched && !result.publish_setpoint && !result.publish_mode &&
      result.command == CommandKind::NONE);
    inputs = ready_inputs();
  };
  check([](GateInputs & value) { value.rc_fresh = false; });
  check([](GateInputs & value) { value.kill_latched = true; });
  check([](GateInputs & value) { value.kill_fresh = false; });
  check([](GateInputs & value) { value.vehicle_status_fresh = false; });
  check([](GateInputs & value) { value.odometry_fresh = false; });
  check([](GateInputs & value) { value.timesync_fresh = false; });
  check([](GateInputs & value) { value.mode_fresh = false; });
  check([](GateInputs & value) { value.setpoint_mode_paired = false; });
  check([](GateInputs & value) { value.authority.single_writer = false; });
  check([](GateInputs & value) { value.authority.owner_id = "wrong"; });
  check([](GateInputs & value) { value.authority.lease_id = "wrong"; });
  check([](GateInputs & value) { value.setpoint_fresh = false; });
  check([](GateInputs & value) { value.clock_monotonic = false; });

  auto value = gate();
  assert(value.restart().state == GateState::WAIT);
  assert(!value.tick(0, GateInputs{}).publish_setpoint);
}

void test_manual_recovery_never_auto_active()
{
  auto value = gate();
  auto inputs = ready_inputs();
  value.request_manual_activation(0, inputs);
  value.tick(0, inputs);
  inputs.kill_latched = true;
  assert(value.tick(1, inputs).fault_latched);
  inputs = ready_inputs();
  assert(value.tick(2, inputs).state == GateState::FAULT_LATCHED);
  assert(value.request_manual_recovery(3, inputs).state == GateState::WAIT);
  assert(value.tick(4, inputs).state == GateState::WAIT);
  assert(value.request_manual_activation(5, inputs).state == GateState::WAIT);
  assert(value.tick(6, inputs).state == GateState::PRESTREAM);
}

void test_arm_requires_explicit_enable_and_manual_gate()
{
  auto inputs = ready_inputs();
  inputs.vehicle_in_offboard = true;
  auto disabled = gate(false);
  assert(disabled.request_manual_activation(0, inputs).state == GateState::WAIT);
  assert(disabled.tick(0, inputs).state == GateState::PRESTREAM);
  for (int i = 1; i < 20; ++i) {
    assert(disabled.tick(i * 50000000LL, inputs).publish_setpoint);
  }
  const auto standby = disabled.tick(1000000000LL, inputs);
  assert(standby.state == GateState::STANDBY_DISARMED && standby.command == CommandKind::NONE &&
    !standby.publish_setpoint && !standby.publish_mode);

  auto enabled = gate(true);
  prestream_to_mode(enabled, inputs);
  enabled.observe_ack(1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  ++inputs.vehicle_status_generation;
  const auto waiting_for_manual_arm = enabled.tick(1100000000LL, inputs);
  assert(waiting_for_manual_arm.state == GateState::REQUEST_MODE &&
    waiting_for_manual_arm.command == CommandKind::NONE);
  inputs.manual_arm_enable = true;
  auto armed = gate(true);
  prestream_to_mode(armed, inputs);
  armed.observe_ack(1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  ++inputs.vehicle_status_generation;
  const auto arm = armed.tick(1100000000LL, inputs);
  assert(arm.state == GateState::REQUEST_ARM && arm.command == CommandKind::ARM);
  // Armed status that predates the arm ACK cannot complete the transaction.
  inputs.vehicle_armed = true;
  assert(armed.tick(1110000000LL, inputs).state == GateState::REQUEST_ARM);
  auto arm_ack = accepted(SafetyGate::kVehicleCmdArmDisarm);
  arm_ack.status_generation = inputs.vehicle_status_generation;
  assert(!armed.observe_ack(1120000000LL, arm_ack, inputs.authority).fault_latched);
  assert(armed.tick(1130000000LL, inputs).state == GateState::REQUEST_ARM);
  ++inputs.vehicle_status_generation;
  assert(armed.tick(1140000000LL, inputs).state == GateState::ACTIVE);
}

void test_px4_timestamp_gate_rejects_bad_clock_data_and_old_epochs()
{
  // TimesyncStatus.timestamp is the PX4 v1.16 boot-usec baseline for every
  // stream exercised below; a restart creates a new, non-inheriting epoch.
  TimestampGate timestamps(500, 100);
  assert(timestamps.observe_timesync(0) == TimestampResult::ZERO);
  assert(timestamps.observe_timesync(1000) == TimestampResult::ACCEPTED);
  assert(timestamps.observe_timesync(1000) == TimestampResult::FROZEN);
  assert(timestamps.observe_timesync(999) == TimestampResult::BACKWARD);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 0) == TimestampResult::ZERO);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000) == TimestampResult::ACCEPTED);
  assert(timestamps.current(TimestampStream::VEHICLE_STATUS));
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000) == TimestampResult::FROZEN);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 999) == TimestampResult::BACKWARD);
  assert(timestamps.observe(TimestampStream::MODE, 1101) == TimestampResult::FUTURE);
  assert(timestamps.observe_timesync(2000) == TimestampResult::ACCEPTED);
  assert(!timestamps.current(TimestampStream::VEHICLE_STATUS));
  assert(timestamps.observe(TimestampStream::ODOMETRY, 1499) == TimestampResult::STALE);

  timestamps.restart_epoch();
  assert(!timestamps.timesync_ready());
  assert(!timestamps.current(TimestampStream::VEHICLE_STATUS));
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000) == TimestampResult::NO_TIMESYNC);
  assert(timestamps.observe_timesync(100) == TimestampResult::ACCEPTED);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000) == TimestampResult::FUTURE);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 100) == TimestampResult::ACCEPTED);
  assert(timestamps.observe_timesync(101) == TimestampResult::ACCEPTED);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 100) == TimestampResult::FROZEN);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 101) == TimestampResult::ACCEPTED);
}
}  // namespace

int main()
{
  test_happy_path_disarmed();
  test_ack_reject_timeout_command_and_sequence_fail_closed();
  test_every_readiness_failure_and_restart_is_zero_output();
  test_manual_recovery_never_auto_active();
  test_arm_requires_explicit_enable_and_manual_gate();
  test_px4_timestamp_gate_rejects_bad_clock_data_and_old_epochs();
  std::cout << "safety gate tests passed\n";
  return 0;
}
