#ifndef OFFBOARD_CPP_FLIGHT_SEQUENCE_HPP
#define OFFBOARD_CPP_FLIGHT_SEQUENCE_HPP

#include <array>
#include <cstdint>

#include <safety_gate.hpp>

namespace offboard_cpp
{

enum class MissionTask : std::uint8_t { TASK1 = 1, TASK2 = 2, VERTICAL_TEST = 3 };

enum class MissionState
{
  WAIT_START,
  TAKEOFF,
  HOVER_3S,
  ACQUIRE_CAR,
  FOLLOW,
  DROP,
  RETURN_HOME,
  HOME_DESCEND,
  LAND_CONFIRMED,
  DISARMED,
  PLATFORM_ALIGN,
  PLATFORM_DESCEND,
  PLATFORM_LANDED,
  HOLD_5S,
  REARM,
  PLATFORM_TAKEOFF,
  HOME_LAND,
  COMPLETE,
};

struct FlightConfig
{
  MissionTask task{MissionTask::TASK1};
  double takeoff_speed{0.5};
  double cruise_speed{1.0};
  double home_land_speed{0.3};
  double platform_land_speed{0.2};
  double takeoff_height{1.0};
  double hover_seconds{3.0};
  double platform_hold_seconds{5.0};
  double position_tolerance{0.20};
  double home_surface_z{0.0};
  double platform_surface_z{0.0};
  double follow_offset_x{0.0};
  double follow_offset_y{0.0};
  double yaw{0.0};
};

struct FlightInputs
{
  std::int64_t now_ns{0};
  bool start{false};
  bool odometry_fresh{false};
  bool vehicle_status_fresh{false};
  bool armed{false};
  bool landing_confirmed{false};
  std::array<double, 3> position{};
  bool car_target_fresh{false};
  std::array<double, 3> car_target{};
  bool follow_complete{false};
  bool drop_complete{false};
};

struct FlightOutput
{
  MissionState state{MissionState::WAIT_START};
  bool setpoint_valid{false};
  std::array<double, 3> position{};
  double yaw{0.0};
  MissionRequest request{MissionRequest::NONE};
};

class FlightSequence
{
public:
  explicit FlightSequence(FlightConfig config);
  FlightOutput tick(const FlightInputs & inputs);
  MissionState state() const {return state_;}

private:
  void enter(MissionState next, std::int64_t now_ns);
  bool close_to(const std::array<double, 3> & a, const std::array<double, 3> & b) const;
  void move_toward(std::array<double, 3> target, double speed, double dt);
  MissionRequest once(MissionRequest request);

  FlightConfig config_;
  MissionState state_{MissionState::WAIT_START};
  std::int64_t entered_ns_{-1};
  std::int64_t previous_ns_{-1};
  std::array<double, 3> home_{};
  std::array<double, 3> command_{};
  std::array<double, 3> last_car_target_{};
  bool have_home_{false};
  bool start_authorized_{false};
  bool have_car_target_{false};
  MissionRequest last_request_{MissionRequest::NONE};
};

const char * mission_state_name(MissionState state);

}  // namespace offboard_cpp

#endif  // OFFBOARD_CPP_FLIGHT_SEQUENCE_HPP
