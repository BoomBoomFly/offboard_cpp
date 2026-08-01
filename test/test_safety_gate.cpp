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
  return SafetyGate("operator-a", "lease-1", "epoch-1", enable_arm, 1000000000LL, 20);
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

void activate_gate(SafetyGate & value, GateInputs & inputs)
{
  inputs.manual_arm_enable = true;
  prestream_to_mode(value, inputs);
  value.observe_ack(1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  inputs.vehicle_in_offboard = true;
  ++inputs.vehicle_status_generation;
  assert(value.tick(1100000000LL, inputs).command == CommandKind::ARM);
  auto arm_ack = accepted(SafetyGate::kVehicleCmdArmDisarm);
  arm_ack.status_generation = inputs.vehicle_status_generation;
  value.observe_ack(1110000000LL, arm_ack, inputs.authority);
  inputs.vehicle_armed = true;
  ++inputs.vehicle_status_generation;
  assert(value.tick(1120000000LL, inputs).state == GateState::ACTIVE);
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

void test_prestream_accepts_an_already_active_offboard_mode()
{
  auto value = gate(false);
  auto inputs = ready_inputs();
  inputs.vehicle_in_offboard = true;
  value.request_manual_activation(0, inputs);
  assert(value.tick(0, inputs).state == GateState::PRESTREAM);
  for (int i = 1; i < 20; ++i) {
    assert(value.tick(i * 50000000LL, inputs).publish_setpoint);
  }
  const auto established = value.tick(1000000000LL, inputs);
  assert(established.state == GateState::REQUEST_MODE);
  assert(established.command == CommandKind::NONE);
  assert(established.publish_setpoint && established.publish_mode);
  const auto waiting = value.tick(1050000000LL, inputs);
  assert(waiting.state == GateState::REQUEST_MODE);
  assert(waiting.command == CommandKind::NONE);
  assert(!waiting.fault_latched);
}

void test_manual_arm_starts_prestream_and_disarm_cancels_it()
{
  auto inputs = ready_inputs();
  SafetyGate value("operator-a", "lease-1", "epoch-1", false, 1000000000LL, 20, true);
  const auto waiting = value.tick(0, inputs);
  assert(waiting.state == GateState::WAIT);
  assert(!waiting.publish_setpoint && !waiting.publish_mode);

  inputs.vehicle_armed = true;
  const auto prestream = value.tick(1, inputs);
  assert(prestream.state == GateState::PRESTREAM);
  assert(prestream.publish_setpoint && prestream.publish_mode);

  inputs.vehicle_armed = false;
  const auto cancelled = value.tick(2, inputs);
  assert(cancelled.state == GateState::WAIT);
  assert(!cancelled.publish_setpoint && !cancelled.publish_mode);
  assert(!cancelled.fault_latched);
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

  auto status_timeout = gate(true);
  prestream_to_mode(status_timeout, inputs);
  status_timeout.observe_ack(
    1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  assert(status_timeout.tick(4050000000LL, inputs).fault_latched);
}

void test_each_ack_rejection_and_timeout()
{
  auto inputs = ready_inputs();
  inputs.manual_arm_enable = true;
  auto arm_rejected = gate(true);
  prestream_to_mode(arm_rejected, inputs);
  arm_rejected.observe_ack(1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  inputs.vehicle_in_offboard = true;
  ++inputs.vehicle_status_generation;
  assert(arm_rejected.tick(1100000000LL, inputs).command == CommandKind::ARM);
  auto rejected_ack = accepted(SafetyGate::kVehicleCmdArmDisarm);
  rejected_ack.result = AckResult::REJECTED;
  assert(arm_rejected.observe_ack(1110000000LL, rejected_ack, inputs.authority).fault_latched);

  inputs = ready_inputs();
  auto arm_timeout = gate(true);
  inputs.manual_arm_enable = true;
  prestream_to_mode(arm_timeout, inputs);
  arm_timeout.observe_ack(1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  inputs.vehicle_in_offboard = true;
  ++inputs.vehicle_status_generation;
  arm_timeout.tick(1100000000LL, inputs);
  assert(arm_timeout.tick(4100000001LL, inputs).fault_latched);

  inputs = ready_inputs();
  auto land_rejected = gate(true);
  activate_gate(land_rejected, inputs);
  inputs.mission_request = offboard_cpp::MissionRequest::LAND_HOME;
  inputs.mission_request_generation = 1;
  assert(land_rejected.tick(1130000000LL, inputs).command == CommandKind::LAND);
  rejected_ack = accepted(SafetyGate::kVehicleCmdNavLand);
  rejected_ack.result = AckResult::REJECTED;
  assert(land_rejected.observe_ack(1140000000LL, rejected_ack, inputs.authority).fault_latched);

  inputs = ready_inputs();
  auto land_timeout = gate(true);
  activate_gate(land_timeout, inputs);
  inputs.mission_request = offboard_cpp::MissionRequest::LAND_HOME;
  inputs.mission_request_generation = 1;
  land_timeout.tick(1130000000LL, inputs);
  assert(land_timeout.tick(4130000001LL, inputs).fault_latched);

  inputs = ready_inputs();
  auto disarm_rejected = gate(true);
  activate_gate(disarm_rejected, inputs);
  inputs.mission_request = offboard_cpp::MissionRequest::LAND_HOME;
  inputs.mission_request_generation = 1;
  disarm_rejected.tick(1130000000LL, inputs);
  disarm_rejected.observe_ack(1140000000LL, accepted(SafetyGate::kVehicleCmdNavLand), inputs.authority);
  inputs.landing_confirmed = true;
  inputs.mission_request = offboard_cpp::MissionRequest::DISARM;
  inputs.mission_request_generation = 2;
  assert(disarm_rejected.tick(1150000000LL, inputs).command == CommandKind::DISARM);
  rejected_ack = accepted(SafetyGate::kVehicleCmdArmDisarm);
  rejected_ack.result = AckResult::REJECTED;
  assert(disarm_rejected.observe_ack(1160000000LL, rejected_ack, inputs.authority).fault_latched);

  inputs = ready_inputs();
  auto disarm_timeout = gate(true);
  activate_gate(disarm_timeout, inputs);
  inputs.mission_request = offboard_cpp::MissionRequest::LAND_HOME;
  inputs.mission_request_generation = 1;
  disarm_timeout.tick(1130000000LL, inputs);
  disarm_timeout.observe_ack(1140000000LL, accepted(SafetyGate::kVehicleCmdNavLand), inputs.authority);
  inputs.landing_confirmed = true;
  inputs.mission_request = offboard_cpp::MissionRequest::DISARM;
  inputs.mission_request_generation = 2;
  disarm_timeout.tick(1150000000LL, inputs);
  assert(disarm_timeout.tick(4150000001LL, inputs).fault_latched);
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
  check([](GateInputs & value) { value.kill_latched = true; });
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

void test_activation_and_ack_timestamps_cannot_rollback()
{
  auto inputs = ready_inputs();
  auto value = gate();
  assert(value.request_manual_activation(10, inputs).state == GateState::WAIT);
  const auto rollback = value.tick(9, inputs);
  assert(rollback.fault_latched && !rollback.publish_setpoint && !rollback.publish_mode &&
    rollback.command == CommandKind::NONE);

  auto pending = gate(true);
  prestream_to_mode(pending, inputs);
  assert(!pending.observe_ack(
    1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority).fault_latched);
  const auto ack_rollback = pending.tick(1049999999LL, inputs);
  assert(ack_rollback.fault_latched && !ack_rollback.publish_setpoint && !ack_rollback.publish_mode &&
    ack_rollback.command == CommandKind::NONE);
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
  assert(standby.state == GateState::REQUEST_MODE && standby.command == CommandKind::NONE &&
    standby.publish_setpoint && standby.publish_mode);

  auto enabled = gate(true);
  inputs.vehicle_in_offboard = false;
  prestream_to_mode(enabled, inputs);
  enabled.observe_ack(1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  inputs.vehicle_in_offboard = true;
  ++inputs.vehicle_status_generation;
  const auto arm = enabled.tick(1100000000LL, inputs);
  auto & armed = enabled;
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
  TimestampGate timestamps(500, 100);
  assert(timestamps.observe_timesync(0, 0) == TimestampResult::ZERO);
  assert(timestamps.observe_timesync(1000, 0) == TimestampResult::ACCEPTED);
  assert(timestamps.observe_timesync(1000, 1) == TimestampResult::FROZEN);
  assert(timestamps.observe_timesync(999, 2) == TimestampResult::BACKWARD);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000, 0) ==
    TimestampResult::ACCEPTED);
  assert(timestamps.current(TimestampStream::VEHICLE_STATUS, 0));
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000, 1) ==
    TimestampResult::FROZEN);
  assert(timestamps.observe(TimestampStream::MODE, 1201, 100000) == TimestampResult::FUTURE);
  assert(timestamps.observe(TimestampStream::ODOMETRY, 1001, 1000000) == TimestampResult::STALE);
  assert(!timestamps.current(TimestampStream::VEHICLE_STATUS, 1000000));
  assert(timestamps.observe_timesync(2000, -1) == TimestampResult::ZERO);

  timestamps.restart_epoch();
  assert(!timestamps.timesync_ready());
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000, 0) ==
    TimestampResult::NO_TIMESYNC);
  assert(timestamps.observe_timesync(100, 10) == TimestampResult::ACCEPTED);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 1000, 10) ==
    TimestampResult::FUTURE);
  assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, 100, 10) ==
    TimestampResult::ACCEPTED);
  assert(timestamps.observe_timesync(101, 9) == TimestampResult::BACKWARD);
}

void test_sixty_second_real_frequency_replay()
{
  TimestampGate timestamps(3000000, 100000);
  auto inputs = ready_inputs();
  SafetyGate safety("operator-a", "lease-1", "epoch-1", false);
  inputs.vehicle_in_offboard = true;
  assert(!safety.request_manual_activation(0, inputs).fault_latched);
  const std::uint64_t base_us = 1000000;
  int next_timesync = 0;
  int next_status = 0;
  int next_land = 0;
  bool jitter = false;
  for (int tick = 0; tick <= 3000; ++tick) {
    const std::int64_t now_ns = static_cast<std::int64_t>(tick) * 20000000LL;
    const std::uint64_t now_us = base_us + static_cast<std::uint64_t>(now_ns / 1000);
    if (tick >= next_timesync) {
      assert(timestamps.observe_timesync(now_us, now_ns) == TimestampResult::ACCEPTED);
      next_timesync += jitter ? 49 : 51;
      jitter = !jitter;
    }
    if (tick >= next_status) {
      assert(timestamps.observe(TimestampStream::VEHICLE_STATUS, now_us, now_ns) ==
        TimestampResult::ACCEPTED);
      next_status += (tick / 25) % 2 == 0 ? 24 : 26;
    }
    if (tick >= next_land) {
      assert(timestamps.observe(TimestampStream::LAND_DETECTED, now_us, now_ns) ==
        TimestampResult::ACCEPTED);
      next_land += (tick / 50) % 2 == 0 ? 48 : 52;
    }
    if (tick % 5 == 0) {
      assert(timestamps.observe(TimestampStream::RC, now_us, now_ns) ==
        TimestampResult::ACCEPTED);
    }
    assert(timestamps.observe(TimestampStream::ODOMETRY, now_us, now_ns) ==
      TimestampResult::ACCEPTED);
    assert(timestamps.observe(TimestampStream::SETPOINT, now_us, now_ns) ==
      TimestampResult::ACCEPTED);
    assert(timestamps.observe(TimestampStream::MODE, now_us, now_ns) ==
      TimestampResult::ACCEPTED);
    assert(timestamps.current(TimestampStream::VEHICLE_STATUS, now_ns, 1200000));
    assert(timestamps.current(TimestampStream::LAND_DETECTED, now_ns, 1800000));
    assert(timestamps.current(TimestampStream::RC, now_ns, 300000));
    assert(timestamps.current(TimestampStream::ODOMETRY, now_ns, 200000));
    assert(timestamps.current(TimestampStream::SETPOINT, now_ns, 150000));
    assert(timestamps.current(TimestampStream::MODE, now_ns, 150000));
    assert(!safety.tick(now_ns, inputs).fault_latched);
  }
  assert(!timestamps.current(TimestampStream::ODOMETRY, 60201000000LL, 200000));
}

void test_land_disarm_and_rearm_ack_lifecycle()
{
  auto value = gate(true);
  auto inputs = ready_inputs();
  inputs.manual_arm_enable = true;
  prestream_to_mode(value, inputs);
  value.observe_ack(
    1050000000LL, accepted(SafetyGate::kVehicleCmdDoSetMode), inputs.authority);
  inputs.vehicle_in_offboard = true;
  ++inputs.vehicle_status_generation;
  assert(value.tick(1100000000LL, inputs).command == CommandKind::ARM);
  auto arm_ack = accepted(SafetyGate::kVehicleCmdArmDisarm);
  arm_ack.status_generation = inputs.vehicle_status_generation;
  value.observe_ack(1110000000LL, arm_ack, inputs.authority);
  inputs.vehicle_armed = true;
  ++inputs.vehicle_status_generation;
  assert(value.tick(1120000000LL, inputs).state == GateState::ACTIVE);

  inputs.mission_request = offboard_cpp::MissionRequest::LAND_PLATFORM;
  inputs.mission_request_generation = 1;
  const auto land = value.tick(1130000000LL, inputs);
  assert(land.state == GateState::REQUEST_LAND && land.command == CommandKind::LAND);
  auto progress = accepted(SafetyGate::kVehicleCmdNavLand);
  progress.result = AckResult::IN_PROGRESS;
  assert(value.observe_ack(1140000000LL, progress, inputs.authority).publish_mode);
  assert(value.observe_ack(
    1150000000LL, accepted(SafetyGate::kVehicleCmdNavLand),
    inputs.authority).state == GateState::LANDING);

  inputs.landing_confirmed = true;
  inputs.mission_request = offboard_cpp::MissionRequest::DISARM;
  inputs.mission_request_generation = 2;
  const auto disarm = value.tick(1160000000LL, inputs);
  assert(disarm.state == GateState::REQUEST_DISARM && disarm.command == CommandKind::DISARM);
  auto disarm_ack = accepted(SafetyGate::kVehicleCmdArmDisarm);
  disarm_ack.status_generation = inputs.vehicle_status_generation;
  value.observe_ack(1170000000LL, disarm_ack, inputs.authority);
  inputs.vehicle_armed = false;
  inputs.vehicle_in_offboard = false;
  ++inputs.vehicle_status_generation;
  assert(value.tick(1180000000LL, inputs).state == GateState::STANDBY_DISARMED);

  inputs.mission_request = offboard_cpp::MissionRequest::REARM;
  inputs.mission_request_generation = 3;
  assert(value.tick(1190000000LL, inputs).state == GateState::PRESTREAM);
  for (int i = 1; i < 20; ++i) {
    value.tick(1190000000LL + i * 50000000LL, inputs);
  }
  const auto mode_request = value.tick(2190000000LL, inputs);
  assert(mode_request.command == CommandKind::SET_MODE_OFFBOARD);
  auto mode_ack = accepted(SafetyGate::kVehicleCmdDoSetMode);
  mode_ack.status_generation = inputs.vehicle_status_generation;
  value.observe_ack(2200000000LL, mode_ack, inputs.authority);
  inputs.vehicle_in_offboard = true;
  ++inputs.vehicle_status_generation;
  assert(value.tick(2210000000LL, inputs).command == CommandKind::ARM);
  arm_ack.status_generation = inputs.vehicle_status_generation;
  value.observe_ack(2220000000LL, arm_ack, inputs.authority);
  inputs.vehicle_armed = true;
  ++inputs.vehicle_status_generation;
  assert(value.tick(2230000000LL, inputs).state == GateState::ACTIVE);

  inputs.vehicle_armed = false;
  assert(value.tick(2240000000LL, inputs).state == GateState::FAULT_LATCHED);
}
}  // namespace

int main()
{
  test_happy_path_disarmed();
  test_prestream_accepts_an_already_active_offboard_mode();
  test_manual_arm_starts_prestream_and_disarm_cancels_it();
  test_ack_reject_timeout_command_and_sequence_fail_closed();
  test_each_ack_rejection_and_timeout();
  test_every_readiness_failure_and_restart_is_zero_output();
  test_activation_and_ack_timestamps_cannot_rollback();
  test_arm_requires_explicit_enable_and_manual_gate();
  test_px4_timestamp_gate_rejects_bad_clock_data_and_old_epochs();
  test_sixty_second_real_frequency_replay();
  test_land_disarm_and_rearm_ack_lifecycle();
  std::cout << "safety gate tests passed\n";
  return 0;
}
