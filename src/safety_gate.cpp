#include <safety_gate.hpp>

namespace offboard_cpp
{
namespace
{
constexpr std::int64_t kPrestreamNs = 1000000000LL;
constexpr std::uint32_t kPrestreamSamples = 20;
constexpr std::int64_t kCommandTimeoutNs = 3000000000LL;
}  // namespace

SafetyGate::SafetyGate(
  std::string expected_owner, std::string expected_lease, std::string expected_epoch,
  bool enable_arm)
: expected_owner_(std::move(expected_owner)),
  expected_lease_(std::move(expected_lease)),
  expected_epoch_(std::move(expected_epoch)),
  enable_arm_(enable_arm)
{
}

bool SafetyGate::authority_matches(const Authority & authority) const
{
  return authority.fresh && authority.single_writer && authority.single_owner &&
         !expected_owner_.empty() && !expected_lease_.empty() && !expected_epoch_.empty() &&
         authority.owner_id == expected_owner_ && authority.lease_id == expected_lease_ &&
         authority.epoch == expected_epoch_ && authority.sequence != 0;
}

bool SafetyGate::ready(const GateInputs & inputs) const
{
  return inputs.clock_monotonic && !inputs.kill_latched &&
         inputs.vehicle_status_fresh && inputs.odometry_fresh && inputs.timesync_fresh &&
         inputs.rc_fresh && inputs.kill_fresh && inputs.setpoint_fresh && inputs.mode_fresh &&
         inputs.setpoint_mode_paired && authority_matches(inputs.authority);
}

GateDecision SafetyGate::decision(
  bool setpoint, bool mode, CommandKind command, const char * reason) const
{
  return GateDecision{setpoint, mode, command, state_, state_ == GateState::FAULT_LATCHED, reason};
}

void SafetyGate::clear_pending()
{
  pending_command_ = CommandKind::NONE;
  pending_sequence_ = 0;
  pending_deadline_ns_ = -1;
}

bool SafetyGate::consume_request(const GateInputs & inputs, MissionRequest request)
{
  if (inputs.mission_request_generation == 0 ||
    inputs.mission_request_generation == last_request_generation_ ||
    inputs.mission_request != request)
  {
    return false;
  }
  last_request_generation_ = inputs.mission_request_generation;
  return true;
}

void SafetyGate::begin_rearm(std::int64_t now_ns)
{
  mode_acknowledged_ = false;
  arm_acknowledged_ = false;
  disarm_acknowledged_ = false;
  mode_ack_status_generation_ = 0;
  arm_ack_status_generation_ = 0;
  disarm_ack_status_generation_ = 0;
  confirmation_deadline_ns_ = -1;
  prestream_started_ns_ = now_ns;
  prestream_samples_ = 0;
  state_ = GateState::PRESTREAM;
}

void SafetyGate::begin_pending(CommandKind command, std::int64_t now_ns, std::uint64_t sequence)
{
  pending_command_ = command;
  pending_sequence_ = sequence;
  pending_deadline_ns_ = now_ns + kCommandTimeoutNs;
}

GateDecision SafetyGate::latch(const char * reason)
{
  clear_pending();
  mode_acknowledged_ = false;
  arm_acknowledged_ = false;
  disarm_acknowledged_ = false;
  mode_ack_status_generation_ = 0;
  arm_ack_status_generation_ = 0;
  disarm_ack_status_generation_ = 0;
  confirmation_deadline_ns_ = -1;
  manual_activation_granted_ = false;
  state_ = GateState::FAULT_LATCHED;
  return decision(false, false, CommandKind::NONE, reason);
}

GateDecision SafetyGate::tick(std::int64_t now_ns, const GateInputs & inputs)
{
  if (now_ns < 0 || (last_tick_ns_ >= 0 && now_ns < last_tick_ns_) || !inputs.clock_monotonic) {
    return latch("clock jump");
  }
  last_tick_ns_ = now_ns;
  if (state_ == GateState::FAULT_LATCHED) {
    return decision(false, false, CommandKind::NONE, "manual recovery required");
  }
  if (!ready(inputs)) {
    // Any loss after PRESTREAM is a latched safety event.  WAIT intentionally
    // remains quiet so boot-order races cannot become a self-latching restart loop.
    if (state_ != GateState::WAIT) {
      return latch(inputs.kill_latched ? "kill latch" : "readiness lost");
    }
    return decision(false, false, CommandKind::NONE, "waiting for fresh complete authority");
  }

  if (pending_command_ != CommandKind::NONE && now_ns >= pending_deadline_ns_) {
    return latch("command ACK timeout");
  }
  if (confirmation_deadline_ns_ >= 0 && now_ns >= confirmation_deadline_ns_) {
    return latch("command status confirmation timeout");
  }
  if (pending_command_ != CommandKind::NONE &&
      inputs.authority.sequence != pending_sequence_) {
    return latch("authority sequence changed while command pending");
  }

  switch (state_) {
    case GateState::WAIT:
      if (!manual_activation_granted_) {
        return decision(false, false, CommandKind::NONE, "manual activation required");
      }
      state_ = GateState::PRESTREAM;
      prestream_started_ns_ = now_ns;
      prestream_samples_ = 0;
      return decision(false, false, CommandKind::NONE, "prestream entered");
    case GateState::PRESTREAM:
      ++prestream_samples_;
      if (now_ns - prestream_started_ns_ >= kPrestreamNs && prestream_samples_ >= kPrestreamSamples) {
        if (!enable_arm_) {
          state_ = GateState::STANDBY_DISARMED;
          return decision(false, false, CommandKind::NONE, "standby: arming disabled");
        }
        state_ = GateState::REQUEST_MODE;
        begin_pending(CommandKind::SET_MODE_OFFBOARD, now_ns, inputs.authority.sequence);
        return decision(true, true, CommandKind::SET_MODE_OFFBOARD, "request offboard mode");
      }
      return decision(true, true, CommandKind::NONE, "prestream");
    case GateState::REQUEST_MODE:
      if (!mode_acknowledged_) {
        return decision(true, true, CommandKind::NONE, "awaiting mode ACK");
      }
      if (!inputs.vehicle_in_offboard ||
          inputs.vehicle_status_generation <= mode_ack_status_generation_) {
        return decision(true, true, CommandKind::NONE, "awaiting fresh offboard status");
      }
      confirmation_deadline_ns_ = -1;
      if (inputs.manual_arm_enable) {
        state_ = GateState::REQUEST_ARM;
        begin_pending(CommandKind::ARM, now_ns, inputs.authority.sequence);
        return decision(true, true, CommandKind::ARM, "request arm");
      }
      return decision(true, true, CommandKind::NONE, "awaiting manual arm enable");
    case GateState::REQUEST_ARM:
      if (!arm_acknowledged_) {
        return decision(true, true, CommandKind::NONE, "awaiting arm ACK");
      }
      if (inputs.vehicle_armed && inputs.vehicle_status_generation > arm_ack_status_generation_) {
        confirmation_deadline_ns_ = -1;
        state_ = GateState::ACTIVE;
        return decision(true, true, CommandKind::NONE, "active armed");
      }
      return decision(true, true, CommandKind::NONE, "awaiting arm ACK and status");
    case GateState::ACTIVE:
      if (!inputs.vehicle_armed || !inputs.vehicle_in_offboard) {
        return latch("armed or offboard state lost while active");
      }
      if (consume_request(inputs, MissionRequest::LAND_HOME) ||
        consume_request(inputs, MissionRequest::LAND_PLATFORM))
      {
        state_ = GateState::REQUEST_LAND;
        begin_pending(CommandKind::LAND, now_ns, inputs.authority.sequence);
        return decision(true, true, CommandKind::LAND, "request land");
      }
      return decision(true, true, CommandKind::NONE, "active");
    case GateState::REQUEST_LAND:
      return decision(true, true, CommandKind::NONE, "awaiting land ACK");
    case GateState::LANDING:
      if (!inputs.vehicle_armed) {
        state_ = GateState::STANDBY_DISARMED;
        return decision(false, false, CommandKind::NONE, "landed and auto-disarmed");
      }
      if (consume_request(inputs, MissionRequest::DISARM)) {
        if (!inputs.landing_confirmed) {
          return latch("disarm requested before landing confirmation");
        }
        state_ = GateState::REQUEST_DISARM;
        begin_pending(CommandKind::DISARM, now_ns, inputs.authority.sequence);
        return decision(false, false, CommandKind::DISARM, "request disarm");
      }
      return decision(false, false, CommandKind::NONE, "landing");
    case GateState::REQUEST_DISARM:
      if (!disarm_acknowledged_) {
        return decision(false, false, CommandKind::NONE, "awaiting disarm ACK");
      }
      if (!inputs.vehicle_armed &&
        inputs.vehicle_status_generation > disarm_ack_status_generation_)
      {
        confirmation_deadline_ns_ = -1;
        state_ = GateState::STANDBY_DISARMED;
        return decision(false, false, CommandKind::NONE, "disarmed");
      }
      return decision(false, false, CommandKind::NONE, "awaiting fresh disarmed status");
    case GateState::STANDBY_DISARMED:
      if (consume_request(inputs, MissionRequest::REARM)) {
        if (!enable_arm_ || !inputs.manual_arm_enable || inputs.vehicle_armed) {
          return latch("rearm denied");
        }
        begin_rearm(now_ns);
        return decision(false, false, CommandKind::NONE, "rearm prestream entered");
      }
      return decision(false, false, CommandKind::NONE, "standby: arming disabled");
    case GateState::FAULT_LATCHED:
      break;
  }
  return latch("invalid state");
}

GateDecision SafetyGate::observe_ack(
  std::int64_t monotonic_ns, const CommandAck & ack, const Authority & authority)
{
  if (state_ == GateState::FAULT_LATCHED) {
    return decision(false, false, CommandKind::NONE, "ACK rejected while latched");
  }
  if (monotonic_ns < 0 || (last_tick_ns_ >= 0 && monotonic_ns < last_tick_ns_) ||
      pending_command_ == CommandKind::NONE || monotonic_ns >= pending_deadline_ns_) {
    return latch("unexpected or late ACK");
  }
  last_tick_ns_ = monotonic_ns;
  std::uint32_t expected_command = kVehicleCmdArmDisarm;
  if (pending_command_ == CommandKind::SET_MODE_OFFBOARD) {
    expected_command = kVehicleCmdDoSetMode;
  } else if (pending_command_ == CommandKind::LAND) {
    expected_command = kVehicleCmdNavLand;
  }
  if (!authority_matches(authority) || authority.sequence != pending_sequence_ ||
      (ack.result != AckResult::ACCEPTED && ack.result != AckResult::IN_PROGRESS) ||
      ack.command != expected_command ||
      ack.target_system != kTargetSystem || ack.target_component != kTargetComponent ||
      !ack.from_external) {
    return latch("ACK rejected or correlation mismatch");
  }
  if (ack.result == AckResult::IN_PROGRESS) {
    return decision(
      pending_command_ != CommandKind::DISARM,
      pending_command_ != CommandKind::DISARM,
      CommandKind::NONE, "command in progress");
  }
  if (pending_command_ == CommandKind::SET_MODE_OFFBOARD) {
    mode_acknowledged_ = true;
    mode_ack_status_generation_ = ack.status_generation;
    confirmation_deadline_ns_ = monotonic_ns + kCommandTimeoutNs;
  } else if (pending_command_ == CommandKind::ARM) {
    arm_acknowledged_ = true;
    arm_ack_status_generation_ = ack.status_generation;
    confirmation_deadline_ns_ = monotonic_ns + kCommandTimeoutNs;
  } else if (pending_command_ == CommandKind::LAND) {
    state_ = GateState::LANDING;
  } else if (pending_command_ == CommandKind::DISARM) {
    disarm_acknowledged_ = true;
    disarm_ack_status_generation_ = ack.status_generation;
    confirmation_deadline_ns_ = monotonic_ns + kCommandTimeoutNs;
  }
  clear_pending();
  return decision(false, false, CommandKind::NONE, "ACK accepted; awaiting fresh status");
}

GateDecision SafetyGate::request_manual_recovery(std::int64_t monotonic_ns, const GateInputs & inputs)
{
  if (state_ != GateState::FAULT_LATCHED || !ready(inputs) || monotonic_ns < last_tick_ns_) {
    return decision(false, false, CommandKind::NONE, "manual recovery denied");
  }
  state_ = GateState::WAIT;
  last_tick_ns_ = monotonic_ns;
  prestream_started_ns_ = -1;
  prestream_samples_ = 0;
  mode_acknowledged_ = false;
  arm_acknowledged_ = false;
  disarm_acknowledged_ = false;
  mode_ack_status_generation_ = 0;
  arm_ack_status_generation_ = 0;
  disarm_ack_status_generation_ = 0;
  confirmation_deadline_ns_ = -1;
  manual_activation_granted_ = false;
  clear_pending();
  return decision(false, false, CommandKind::NONE, "manual recovery acknowledged; waiting");
}

GateDecision SafetyGate::request_manual_activation(
  std::int64_t monotonic_ns, const GateInputs & inputs)
{
  if (state_ != GateState::WAIT || !ready(inputs) || monotonic_ns < last_tick_ns_) {
    return decision(false, false, CommandKind::NONE, "manual activation denied");
  }
  last_tick_ns_ = monotonic_ns;
  manual_activation_granted_ = true;
  return decision(false, false, CommandKind::NONE, "manual activation acknowledged");
}

GateDecision SafetyGate::restart()
{
  state_ = GateState::WAIT;
  last_tick_ns_ = -1;
  prestream_started_ns_ = -1;
  prestream_samples_ = 0;
  mode_acknowledged_ = false;
  arm_acknowledged_ = false;
  disarm_acknowledged_ = false;
  mode_ack_status_generation_ = 0;
  arm_ack_status_generation_ = 0;
  disarm_ack_status_generation_ = 0;
  confirmation_deadline_ns_ = -1;
  last_request_generation_ = 0;
  manual_activation_granted_ = false;
  clear_pending();
  return decision(false, false, CommandKind::NONE, "restart safe");
}

}  // namespace offboard_cpp
