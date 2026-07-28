#include <timestamp_gate.hpp>

namespace offboard_cpp
{
TimestampGate::TimestampGate(std::uint64_t max_age_us, std::uint64_t max_future_us)
: max_age_us_(max_age_us), max_future_us_(max_future_us)
{
}

TimestampResult TimestampGate::observe_timesync(std::uint64_t timestamp_us)
{
  if (timestamp_us == 0) {
    return TimestampResult::ZERO;
  }
  if (timesync_timestamp_us_ != 0) {
    if (timestamp_us == timesync_timestamp_us_) {
      return TimestampResult::FROZEN;
    }
    if (timestamp_us < timesync_timestamp_us_) {
      return TimestampResult::BACKWARD;
    }
  }
  timesync_timestamp_us_ = timestamp_us;
  return TimestampResult::ACCEPTED;
}

TimestampResult TimestampGate::observe(TimestampStream stream, std::uint64_t timestamp_us)
{
  if (timestamp_us == 0) {
    return TimestampResult::ZERO;
  }
  if (timesync_timestamp_us_ == 0) {
    return TimestampResult::NO_TIMESYNC;
  }
  const auto previous = last_timestamp_us_.at(index(stream));
  if (previous != 0) {
    if (timestamp_us == previous) {
      return TimestampResult::FROZEN;
    }
    if (timestamp_us < previous) {
      return TimestampResult::BACKWARD;
    }
  }
  if (timestamp_us > timesync_timestamp_us_ &&
      timestamp_us - timesync_timestamp_us_ > max_future_us_) {
    return TimestampResult::FUTURE;
  }
  if (timesync_timestamp_us_ > timestamp_us && timesync_timestamp_us_ - timestamp_us > max_age_us_) {
    return TimestampResult::STALE;
  }
  last_timestamp_us_.at(index(stream)) = timestamp_us;
  return TimestampResult::ACCEPTED;
}

bool TimestampGate::current(TimestampStream stream) const
{
  const auto timestamp_us = last_timestamp_us_.at(index(stream));
  return timesync_timestamp_us_ != 0 && timestamp_us != 0 &&
         timestamp_us <= timesync_timestamp_us_ &&
         timesync_timestamp_us_ - timestamp_us <= max_age_us_;
}

void TimestampGate::restart_epoch()
{
  timesync_timestamp_us_ = 0;
  last_timestamp_us_.fill(0);
}

}  // namespace offboard_cpp
