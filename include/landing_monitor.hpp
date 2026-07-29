#ifndef OFFBOARD_CPP_LANDING_MONITOR_HPP
#define OFFBOARD_CPP_LANDING_MONITOR_HPP

#include <cstdint>

namespace offboard_cpp
{

struct LandingObservation
{
  bool land_detected_fresh{false};
  bool vehicle_status_fresh{false};
  bool odometry_fresh{false};
  bool landed{false};
  double altitude_z{0.0};
  double vertical_speed{0.0};
  double contact_surface_z{0.0};
};

class LandingMonitor
{
public:
  LandingMonitor(std::int64_t stable_ns, double max_vertical_speed, double height_tolerance);

  bool update(std::int64_t now_ns, const LandingObservation & observation);
  void reset();

private:
  const std::int64_t stable_ns_;
  const double max_vertical_speed_;
  const double height_tolerance_;
  std::int64_t stable_since_ns_{-1};
};

}  // namespace offboard_cpp

#endif  // OFFBOARD_CPP_LANDING_MONITOR_HPP
