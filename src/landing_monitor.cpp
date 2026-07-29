#include <landing_monitor.hpp>

#include <cmath>

namespace offboard_cpp
{

LandingMonitor::LandingMonitor(
  std::int64_t stable_ns, double max_vertical_speed, double height_tolerance)
: stable_ns_(stable_ns),
  max_vertical_speed_(max_vertical_speed),
  height_tolerance_(height_tolerance)
{
}

bool LandingMonitor::update(
  std::int64_t now_ns, const LandingObservation & observation)
{
  const bool valid = now_ns >= 0 && stable_ns_ > 0 &&
    max_vertical_speed_ >= 0.0 && height_tolerance_ >= 0.0 &&
    observation.land_detected_fresh && observation.vehicle_status_fresh &&
    observation.odometry_fresh && observation.landed &&
    std::isfinite(observation.altitude_z) &&
    std::isfinite(observation.vertical_speed) &&
    std::isfinite(observation.contact_surface_z) &&
    std::abs(observation.vertical_speed) <= max_vertical_speed_ &&
    std::abs(observation.altitude_z - observation.contact_surface_z) <= height_tolerance_;

  if (!valid) {
    stable_since_ns_ = -1;
    return false;
  }
  if (stable_since_ns_ < 0 || now_ns < stable_since_ns_) {
    stable_since_ns_ = now_ns;
    return false;
  }
  return now_ns - stable_since_ns_ >= stable_ns_;
}

void LandingMonitor::reset()
{
  stable_since_ns_ = -1;
}

}  // namespace offboard_cpp
