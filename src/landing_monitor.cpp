#include <landing_monitor.hpp>

#include <cmath>

namespace offboard_cpp
{

LandingMonitor::LandingMonitor(
  std::int64_t stable_ns, double max_vertical_speed, double height_tolerance,
  std::uint32_t minimum_landed_samples)
: stable_ns_(stable_ns),
  max_vertical_speed_(max_vertical_speed),
  height_tolerance_(height_tolerance),
  minimum_landed_samples_(minimum_landed_samples)
{
}

bool LandingMonitor::update(
  std::int64_t now_ns, const LandingObservation & observation)
{
  const bool valid = now_ns >= 0 && stable_ns_ > 0 && minimum_landed_samples_ >= 2 &&
    max_vertical_speed_ >= 0.0 && height_tolerance_ >= 0.0 &&
    observation.land_detected_timestamp_us != 0 &&
    observation.land_detected_fresh && observation.vehicle_status_fresh &&
    observation.odometry_fresh && observation.landed &&
    std::isfinite(observation.altitude_z) &&
    std::isfinite(observation.vertical_speed) &&
    std::isfinite(observation.contact_surface_z) &&
    std::abs(observation.vertical_speed) <= max_vertical_speed_ &&
    std::abs(observation.altitude_z - observation.contact_surface_z) <= height_tolerance_;

  if (!valid) {
    reset();
    return false;
  }
  if (last_landed_timestamp_us_ != 0 &&
    observation.land_detected_timestamp_us < last_landed_timestamp_us_)
  {
    reset();
    return false;
  }
  if (observation.land_detected_timestamp_us == last_landed_timestamp_us_) {
    return landed_samples_ >= minimum_landed_samples_ &&
      (last_landed_timestamp_us_ - first_landed_timestamp_us_) * 1000U >=
      static_cast<std::uint64_t>(stable_ns_);
  }
  if (first_landed_timestamp_us_ == 0) {
    first_landed_timestamp_us_ = observation.land_detected_timestamp_us;
    last_landed_timestamp_us_ = observation.land_detected_timestamp_us;
    landed_samples_ = 1;
    return false;
  }
  last_landed_timestamp_us_ = observation.land_detected_timestamp_us;
  ++landed_samples_;
  return landed_samples_ >= minimum_landed_samples_ &&
    (last_landed_timestamp_us_ - first_landed_timestamp_us_) * 1000U >=
    static_cast<std::uint64_t>(stable_ns_);
}

void LandingMonitor::reset()
{
  first_landed_timestamp_us_ = 0;
  last_landed_timestamp_us_ = 0;
  landed_samples_ = 0;
}

}  // namespace offboard_cpp
