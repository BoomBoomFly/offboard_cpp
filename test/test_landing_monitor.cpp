#include <cassert>

#include <landing_monitor.hpp>

using offboard_cpp::LandingMonitor;
using offboard_cpp::LandingObservation;

int main()
{
  LandingMonitor monitor(1000000000LL, 0.15, 0.25);
  LandingObservation observation;
  observation.land_detected_fresh = true;
  observation.vehicle_status_fresh = true;
  observation.odometry_fresh = true;
  observation.landed = true;
  observation.altitude_z = 2.0;
  observation.contact_surface_z = 2.0;
  observation.vertical_speed = 0.1;

  assert(!monitor.update(0, observation));
  assert(!monitor.update(999999999LL, observation));
  assert(monitor.update(1000000000LL, observation));

  observation.vehicle_status_fresh = false;
  assert(!monitor.update(1100000000LL, observation));
  observation.vehicle_status_fresh = true;
  observation.vertical_speed = 0.2;
  assert(!monitor.update(1200000000LL, observation));
  observation.vertical_speed = 0.0;
  observation.altitude_z = 2.3;
  assert(!monitor.update(1300000000LL, observation));
  observation.altitude_z = 2.0;
  assert(!monitor.update(1400000000LL, observation));
  assert(monitor.update(2400000000LL, observation));
  return 0;
}
