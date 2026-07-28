#include <safety_gate.hpp>

namespace {
constexpr std::uint16_t kModeCommand = 176;  // MAV_CMD_DO_SET_MODE
constexpr std::uint16_t kArmCommand = 400;   // MAV_CMD_COMPONENT_ARM_DISARM
}  // namespace

bool SafetyGate::healthy(const SafetyGateInputs& in) const {
  return in.vehicle_status_fresh && in.odometry_fresh && in.battery_fresh &&
         in.timesync_fresh && in.rc_fresh && in.kill_fresh &&
         in.setpoint_fresh && in.mode_fresh && in.setpoint_mode_paired &&
         in.clock_monotonic && !in.kill_latched && authority_valid(in);
}

bool SafetyGate::authority_valid(const SafetyGateInputs& in) const {
  return in.single_writer && in.single_owner && in.owner_id != 0 &&
         in.lease_id != 0 && in.epoch != 0 && in.sequence != 0;
}

void SafetyGate::latch(const char* reason) {
  state_ = GateState::FAULT_LATCHED;
  pending_ = PendingCommand::NONE;
  fault_reason_ = reason;
}

void SafetyGate::source_epoch_changed() {
  latch("source epoch changed; manual recovery required");
}

void SafetyGate::begin_command(PendingCommand command, std::uint64_t now_us,
                                const SafetyGateInputs& in) {
  pending_ = command;
  command_started_us_ = now_us;
  pending_owner_id_ = in.owner_id;
  pending_lease_id_ = in.lease_id;
  pending_epoch_ = in.epoch;
  pending_sequence_ = in.sequence;
  command_emitted_ = false;
  state_ = command == PendingCommand::MODE ? GateState::REQUEST_MODE
                                            : GateState::REQUEST_ARM;
}

void SafetyGate::tick(std::uint64_t now_us, const SafetyGateInputs& in) {
  if (state_ == GateState::FAULT_LATCHED) {
    if (in.manual_reset && healthy(in)) {
      state_ = GateState::WAIT;
      fault_reason_.clear();
      authority_owner_id_ = 0;
      authority_lease_id_ = 0;
      authority_epoch_ = 0;
      authority_sequence_ = 0;
    }
    return;
  }

  if (!healthy(in)) {
    latch("input stale, invalid, duplicate, kill, or clock fault; manual recovery required");
    return;
  }

  if (authority_owner_id_ != 0 &&
      (in.owner_id != authority_owner_id_ || in.lease_id != authority_lease_id_ ||
       in.epoch != authority_epoch_ || in.sequence != authority_sequence_)) {
    latch("authority source epoch or lease changed; manual recovery required");
    return;
  }
  authority_owner_id_ = in.owner_id;
  authority_lease_id_ = in.lease_id;
  authority_epoch_ = in.epoch;
  authority_sequence_ = in.sequence;

  if (pending_ != PendingCommand::NONE) {
    if (in.owner_id != pending_owner_id_ || in.lease_id != pending_lease_id_ ||
        in.epoch != pending_epoch_ || in.sequence != pending_sequence_) {
      latch("authority sequence changed while command pending");
      return;
    }
    if (now_us - command_started_us_ > kCommandAckTimeoutUs) {
      latch("command ACK timeout");
    }
    return;
  }

  switch (state_) {
    case GateState::WAIT:
      state_ = GateState::PRESTREAM;
      prestream_start_us_ = now_us;
      prestream_samples_ = 1;
      break;
    case GateState::STANDBY_DISARMED:
      // A successful mode request never arms automatically.  A separate,
      // still-present human arm authorization is required.
      if (in.manual_activation && in.manual_arm_enable) {
        begin_command(PendingCommand::ARM, now_us, in);
      }
      break;
    case GateState::PRESTREAM:
      ++prestream_samples_;
      // manual activation required: prestream alone can never request a mode.
      if (in.manual_activation &&
          now_us - prestream_start_us_ >= kPrestreamDurationUs &&
          prestream_samples_ >= kPrestreamSamples) {
        begin_command(PendingCommand::MODE, now_us, in);
      }
      break;
    case GateState::ACTIVE:
      break;
    default:
      break;
  }
}

void SafetyGate::observe_ack(std::uint64_t /*now_us*/, const CommandAck& ack) {
  if (pending_ == PendingCommand::NONE) {
    return;
  }
  if (ack.command != expected_command() || ack.target_system != 1 ||
      ack.target_component != 1 || (!ack.accepted && !ack.in_progress)) {
    latch("ACK rejected or correlation mismatch");
    return;
  }
  if (ack.in_progress && !ack.accepted) {
    return;
  }

  if (pending_ == PendingCommand::MODE) {
    pending_ = PendingCommand::NONE;
    state_ = GateState::STANDBY_DISARMED;
  } else {
    pending_ = PendingCommand::NONE;
    state_ = GateState::ACTIVE;
  }
}

bool SafetyGate::allows_control_payload() const {
  return state_ == GateState::PRESTREAM || state_ == GateState::ACTIVE;
}

PendingCommand SafetyGate::command_to_send() const { return pending_; }

PendingCommand SafetyGate::take_command_to_send() {
  if (command_emitted_ || pending_ == PendingCommand::NONE) {
    return PendingCommand::NONE;
  }
  command_emitted_ = true;
  return pending_;
}

std::uint16_t SafetyGate::expected_command() const {
  return pending_ == PendingCommand::MODE ? kModeCommand : kArmCommand;
}
