#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

enum class TimestampStream : std::size_t {
  VEHICLE_STATUS,
  ODOMETRY,
  RC,
  BATTERY,
  SETPOINT,
  MODE,
  COMMAND_ACK,
  COUNT,
};

enum class TimestampFault { NONE, ZERO, NO_TIMESYNC, FROZEN, BACKWARD, FUTURE, STALE };

class TimestampGate {
 public:
  explicit TimestampGate(std::uint64_t max_age_us = 500'000)
      : max_age_us_(max_age_us) {}

  void observe_timesync(std::uint64_t timestamp_us, std::uint64_t received_us);
  TimestampFault observe(TimestampStream stream, std::uint64_t timestamp_us,
                         std::uint64_t received_us);
  [[nodiscard]] bool fresh(TimestampStream stream, std::uint64_t now_us) const;
  [[nodiscard]] bool fault_latched() const { return timestamp_fault_latched_; }
  [[nodiscard]] bool configuration_valid() const { return timestamp_config_valid_; }
  [[nodiscard]] bool timesync_fresh(std::uint64_t now_us) const;
  void restart_epoch();

 private:
  static constexpr std::size_t kCount = static_cast<std::size_t>(TimestampStream::COUNT);
  std::array<std::uint64_t, kCount> last_timestamp_us_{};
  std::array<std::uint64_t, kCount> received_us_{};
  std::uint64_t last_timesync_us_{0};
  std::uint64_t last_timesync_received_us_{0};
  std::uint64_t max_age_us_;
  bool timestamp_fault_latched_{false};
  bool timestamp_config_valid_{true};
};
