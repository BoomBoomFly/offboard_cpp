#include <cassert>
#include <cmath>

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
  inputs.vehicle_in_offboard = true;
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

void test_start_is_latched_until_fresh_armed_status()
{
  FlightConfig config;
  config.task = MissionTask::VERTICAL_TEST;
  FlightSequence sequence(config);
  auto inputs = base();
  inputs.armed = false;
  inputs.start = true;
  assert(sequence.tick(inputs).state == MissionState::WAIT_START);
  inputs.start = false;
  assert(sequence.tick(inputs).state == MissionState::WAIT_START);
  inputs.armed = true;
  assert(sequence.tick(inputs).state == MissionState::TAKEOFF);
}

void test_auto_takeoff_waits_for_confirmed_offboard()
{
  FlightConfig config;
  config.task = MissionTask::VERTICAL_TEST;
  config.auto_takeoff = true;
  FlightSequence sequence(config);
  auto inputs = base();
  inputs.vehicle_in_offboard = false;
  assert(sequence.tick(inputs).state == MissionState::WAIT_START);
  inputs.vehicle_in_offboard = true;
  assert(sequence.tick(inputs).state == MissionState::TAKEOFF);
}

void test_vertical_hover_holds_relative_to_start_position()
{
  FlightConfig config;
  config.task = MissionTask::VERTICAL_TEST;
  config.takeoff_height = 0.5;
  config.takeoff_speed = 5.0;
  config.relative_takeoff_height = true;
  config.hold_after_takeoff = true;
  FlightSequence sequence(config);
  auto inputs = base();
  inputs.position = {1.0, -2.0, 0.35};
  inputs.start = true;
  assert(sequence.tick(inputs).state == MissionState::TAKEOFF);
  inputs.start = false;
  follow_command(sequence, inputs, MissionState::HOVER, 200);
  inputs.now_ns += 10000000000LL;
  const auto output = sequence.tick(inputs);
  assert(output.state == MissionState::HOVER);
  assert(output.request == MissionRequest::NONE);
  assert(output.position[0] == 1.0);
  assert(output.position[1] == -2.0);
  assert(std::abs(output.position[2] - (-0.15)) < 1.0e-9);
}

void test_vertical_test_can_auto_land_when_hold_is_disabled()
{
  FlightConfig config;
  config.task = MissionTask::VERTICAL_TEST;
  config.takeoff_height = 0.5;
  config.takeoff_speed = 0.3;
  config.home_land_speed = 0.2;
  config.hover_seconds = 0.04;
  FlightSequence sequence(config);
  auto inputs = base();
  inputs.start = true;
  assert(sequence.tick(inputs).state == MissionState::TAKEOFF);
  inputs.now_ns += 20000000LL;
  assert(sequence.tick(inputs).state == MissionState::TAKEOFF);
  inputs.start = false;
  follow_command(sequence, inputs, MissionState::HOVER_3S, 200);
  follow_command(sequence, inputs, MissionState::HOME_DESCEND, 20);
  bool land_requested = false;
  for (int i = 0; i < 200; ++i) {
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
  test_start_is_latched_until_fresh_armed_status();
  test_auto_takeoff_waits_for_confirmed_offboard();
  test_vertical_hover_holds_relative_to_start_position();
  test_vertical_test_can_auto_land_when_hold_is_disabled();
  test_task2_platform_land_rearm_and_home_land();
  return 0;
}
