#ifndef OFFBOARD_CPP_TIMESTAMP_GATE_HPP
#define OFFBOARD_CPP_TIMESTAMP_GATE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace offboard_cpp
{

// Contract: PX4 v1.16 TimesyncStatus.timestamp is the PX4 boot-time
// microsecond baseline. VehicleStatus, VehicleOdometry, RcChannels,
// OffboardControlMode, TrajectorySetpoint and VehicleCommandAck timestamps
// must be in that same epoch. Receive-time freshness alone cannot detect
// replay, frozen, stale, or future data.
enum class TimestampStream : std::size_t {
  VEHICLE_STATUS,
  ODOMETRY,
  RC,
  SETPOINT,
  MODE,
  COMMAND_ACK,
  LAND_DETECTED,
  COUNT,
};

enum class TimestampResult {
  ACCEPTED,
  ZERO,
  NO_TIMESYNC,
  FROZEN,
  BACKWARD,
  FUTURE,
  STALE,
};

class TimestampGate
{
public:
  explicit TimestampGate(std::uint64_t max_age_us = 500000, std::uint64_t max_future_us = 100000);

  TimestampResult observe_timesync(std::uint64_t timestamp_us);
  TimestampResult observe(TimestampStream stream, std::uint64_t timestamp_us);
  bool current(TimestampStream stream) const;
  bool timesync_ready() const { return timesync_timestamp_us_ != 0; }
  void restart_epoch();

private:
  static constexpr std::size_t kStreamCount = static_cast<std::size_t>(TimestampStream::COUNT);
  static std::size_t index(TimestampStream stream) { return static_cast<std::size_t>(stream); }

  std::uint64_t max_age_us_;
  std::uint64_t max_future_us_;
  std::uint64_t timesync_timestamp_us_{0};
  std::array<std::uint64_t, kStreamCount> last_timestamp_us_{};
};

}  // namespace offboard_cpp

#endif  // OFFBOARD_CPP_TIMESTAMP_GATE_HPP
