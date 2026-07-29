#include <cassert>

#include <flight_sequence.hpp>

using offboard_cpp::FlightConfig;
using offboard_cpp::FlightInputs;
using offboard_cpp::FlightSequence;
using offboard_cpp::MissionRequest;
using offboard_cpp::MissionState;
using offboard_cpp::MissionTask;

namespace
{
FlightInputs base()
{
  FlightInputs inputs;
  inputs.odometry_fresh = true;
  inputs.vehicle_status_fresh = true;
  inputs.armed = true;
  return inputs;
}

void follow_command(
  FlightSequence & sequence, FlightInputs & inputs, MissionState desired, int limit = 1000)
{
  for (int i = 0; i < limit && sequence.state() != desired; ++i) {
    inputs.now_ns += 20000000LL;
    const auto output = sequence.tick(inputs);
    inputs.position = output.position;
  }
  assert(sequence.state() == desired);
}

void test_task1_normal_landing()
{
  FlightConfig config;
  config.task = MissionTask::TASK1;
  config.hover_seconds = 0.04;
  config.takeoff_speed = 5.0;
  config.cruise_speed = 5.0;
  config.home_land_speed = 5.0;
  FlightSequence sequence(config);
  auto inputs = base();
  inputs.start = true;
  assert(sequence.tick(inputs).state == MissionState::TAKEOFF);
  follow_command(sequence, inputs, MissionState::HOVER_3S);
  follow_command(sequence, inputs, MissionState::ACQUIRE_CAR);
  inputs.car_target_fresh = true;
  inputs.car_target = {2.0, 1.0, -1.0};
  assert(sequence.tick(inputs).state == MissionState::FOLLOW);
  inputs.follow_complete = true;
  assert(sequence.tick(inputs).state == MissionState::DROP);
  inputs.drop_complete = true;
  assert(sequence.tick(inputs).state == MissionState::RETURN_HOME);
  follow_command(sequence, inputs, MissionState::HOME_DESCEND);
  bool land_requested = false;
  for (int i = 0; i < 100; ++i) {
    inputs.now_ns += 20000000LL;
    const auto output = sequence.tick(inputs);
    inputs.position = output.position;
    land_requested = land_requested || output.request == MissionRequest::LAND_HOME;
  }
  assert(land_requested);
  inputs.landing_confirmed = true;
  assert(sequence.tick(inputs).state == MissionState::LAND_CONFIRMED);
  assert(sequence.tick(inputs).request == MissionRequest::DISARM);
  inputs.armed = false;
  assert(sequence.tick(inputs).state == MissionState::DISARMED);
  assert(sequence.tick(inputs).state == MissionState::COMPLETE);
}

void test_task2_platform_land_rearm_and_home_land()
{
  FlightConfig config;
  config.task = MissionTask::TASK2;
  config.takeoff_speed = 5.0;
  config.cruise_speed = 5.0;
  config.home_land_speed = 5.0;
  config.platform_land_speed = 5.0;
  config.platform_hold_seconds = 0.04;
  config.platform_surface_z = 0.5;
  FlightSequence sequence(config);
  auto inputs = base();
  inputs.start = true;
  sequence.tick(inputs);
  follow_command(sequence, inputs, MissionState::ACQUIRE_CAR);
  inputs.car_target_fresh = true;
  inputs.car_target = {2.0, 1.0, 0.5};
  sequence.tick(inputs);
  inputs.follow_complete = true;
  assert(sequence.tick(inputs).state == MissionState::PLATFORM_ALIGN);
  follow_command(sequence, inputs, MissionState::PLATFORM_DESCEND);
  bool platform_land = false;
  for (int i = 0; i < 100; ++i) {
    inputs.now_ns += 20000000LL;
    const auto output = sequence.tick(inputs);
    inputs.position = output.position;
    platform_land = platform_land || output.request == MissionRequest::LAND_PLATFORM;
  }
  assert(platform_land);
  inputs.landing_confirmed = true;
  assert(sequence.tick(inputs).state == MissionState::PLATFORM_LANDED);
  assert(sequence.tick(inputs).request == MissionRequest::DISARM);
  inputs.armed = false;
  assert(sequence.tick(inputs).state == MissionState::HOLD_5S);
  follow_command(sequence, inputs, MissionState::REARM);
  assert(sequence.tick(inputs).request == MissionRequest::REARM);
  inputs.armed = true;
  inputs.landing_confirmed = false;
  assert(sequence.tick(inputs).state == MissionState::PLATFORM_TAKEOFF);
  follow_command(sequence, inputs, MissionState::RETURN_HOME);
  follow_command(sequence, inputs, MissionState::HOME_LAND);
  bool home_land = false;
  for (int i = 0; i < 100; ++i) {
    inputs.now_ns += 20000000LL;
    const auto output = sequence.tick(inputs);
    inputs.position = output.position;
    home_land = home_land || output.request == MissionRequest::LAND_HOME;
  }
  assert(home_land);
  inputs.landing_confirmed = true;
  assert(sequence.tick(inputs).request == MissionRequest::DISARM);
  inputs.armed = false;
  assert(sequence.tick(inputs).state == MissionState::COMPLETE);
}
}  // namespace

int main()
{
  test_task1_normal_landing();
  test_task2_platform_land_rearm_and_home_land();
  return 0;
}
