#include <timestamp_gate.hpp>

namespace offboard_cpp
{
TimestampGate::TimestampGate(std::uint64_t max_age_us, std::uint64_t max_future_us)
: max_age_us_(max_age_us), max_future_us_(max_future_us)
{
}

TimestampResult TimestampGate::observe_timesync(
  std::uint64_t timestamp_us, std::int64_t monotonic_ns)
{
  if (timestamp_us == 0 || monotonic_ns < 0) {
    return TimestampResult::ZERO;
  }
  if (timesync_received_ns_ >= 0 && monotonic_ns < timesync_received_ns_) {
    return TimestampResult::BACKWARD;
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
  timesync_received_ns_ = monotonic_ns;
  return TimestampResult::ACCEPTED;
}

std::uint64_t TimestampGate::estimated_px4_now(std::int64_t monotonic_ns) const
{
  if (timesync_timestamp_us_ == 0 || timesync_received_ns_ < 0 ||
    monotonic_ns < timesync_received_ns_)
  {
    return 0;
  }
  return timesync_timestamp_us_ +
         static_cast<std::uint64_t>(monotonic_ns - timesync_received_ns_) / 1000U;
}

TimestampResult TimestampGate::observe(
  TimestampStream stream, std::uint64_t timestamp_us, std::int64_t monotonic_ns)
{
  if (timestamp_us == 0) {
    return TimestampResult::ZERO;
  }
  const auto estimated_now_us = estimated_px4_now(monotonic_ns);
  if (estimated_now_us == 0) {
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
  if (timestamp_us > estimated_now_us &&
      timestamp_us - estimated_now_us > max_future_us_) {
    return TimestampResult::FUTURE;
  }
  if (estimated_now_us > timestamp_us && estimated_now_us - timestamp_us > max_age_us_) {
    return TimestampResult::STALE;
  }
  last_timestamp_us_.at(index(stream)) = timestamp_us;
  return TimestampResult::ACCEPTED;
}

bool TimestampGate::current(
  TimestampStream stream, std::int64_t monotonic_ns, std::uint64_t max_age_us) const
{
  const auto timestamp_us = last_timestamp_us_.at(index(stream));
  const auto estimated_now_us = estimated_px4_now(monotonic_ns);
  const auto age_limit = max_age_us == 0 ? max_age_us_ : max_age_us;
  return estimated_now_us != 0 && timestamp_us != 0 &&
         timestamp_us <= estimated_now_us + max_future_us_ &&
         (timestamp_us >= estimated_now_us || estimated_now_us - timestamp_us <= age_limit);
}

void TimestampGate::restart_epoch()
{
  timesync_timestamp_us_ = 0;
  timesync_received_ns_ = -1;
  last_timestamp_us_.fill(0);
}

}  // namespace offboard_cpp
