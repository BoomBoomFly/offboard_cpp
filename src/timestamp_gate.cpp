#include <timestamp_gate.hpp>

void TimestampGate::observe_timesync(std::uint64_t timestamp_us,
                                     std::uint64_t received_us) {
  if (timestamp_us == 0 || received_us == 0) {
    timestamp_fault_latched_ = true;
    return;
  }
  if (last_timesync_us_ != 0 && timestamp_us <= last_timesync_us_) {
    timestamp_fault_latched_ = true;
    return;
  }
  last_timesync_us_ = timestamp_us;
  last_timesync_received_us_ = received_us;
}

TimestampFault TimestampGate::observe(TimestampStream stream,
                                      std::uint64_t timestamp_us,
                                      std::uint64_t received_us) {
  if (!timestamp_config_valid_) {
    timestamp_fault_latched_ = true;
    return TimestampFault::NO_TIMESYNC;
  }
  if (timestamp_us == 0 || received_us == 0) {
    timestamp_fault_latched_ = true;
    return TimestampFault::ZERO;
  }
  const auto index = static_cast<std::size_t>(stream);
  const auto previous = last_timestamp_us_[index];
  if (previous != 0 && timestamp_us == previous) {
    timestamp_fault_latched_ = true;
    return TimestampFault::FROZEN;
  }
  if (previous != 0 && timestamp_us < previous) {
    timestamp_fault_latched_ = true;
    return TimestampFault::BACKWARD;
  }
  last_timestamp_us_[index] = timestamp_us;
  received_us_[index] = received_us;
  return TimestampFault::NONE;
}

bool TimestampGate::fresh(TimestampStream stream, std::uint64_t now_us) const {
  const auto receipt = received_us_[static_cast<std::size_t>(stream)];
  return !timestamp_fault_latched_ && receipt != 0 && now_us >= receipt &&
         now_us - receipt <= max_age_us_;
}

bool TimestampGate::timesync_fresh(std::uint64_t now_us) const {
  return !timestamp_fault_latched_ && last_timesync_us_ != 0 &&
         now_us >= last_timesync_received_us_ &&
         now_us - last_timesync_received_us_ <= max_age_us_;
}

void TimestampGate::restart_epoch() {
  last_timestamp_us_.fill(0);
  received_us_.fill(0);
  last_timesync_us_ = 0;
  last_timesync_received_us_ = 0;
  timestamp_fault_latched_ = false;
}
