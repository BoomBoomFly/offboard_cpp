#pragma once

#include <array>
#include <cstdint>

#include "offboard_cpp/mission_types.hpp"

namespace offboard_cpp
{

enum class ModeResult : std::uint8_t { RUNNING, SUCCEEDED };

struct PositionTarget {
  std::array<double, 3> position_ned{};
  double yaw_rad{};
};

struct ModeUpdate {
  MissionActions actions{};
  ModeResult result{ModeResult::RUNNING};
};

MissionActions make_position_setpoint(const PositionTarget & target);

class ModeBase
{
public:
  virtual ~ModeBase() = default;
  virtual void on_activate(std::int64_t now_us) = 0;
  virtual ModeUpdate update_setpoint(
    const MissionInputs & inputs, const PositionTarget & target) = 0;
};

class ReachPositionMode final : public ModeBase
{
public:
  ReachPositionMode(
    double position_tolerance_m, double velocity_tolerance_mps, double stable_duration_s);
  void on_activate(std::int64_t now_us) override;
  ModeUpdate update_setpoint(
    const MissionInputs & inputs, const PositionTarget & target) override;

private:
  double position_tolerance_m_;
  double velocity_tolerance_mps_;
  std::int64_t stable_duration_us_;
  std::int64_t stable_since_us_{-1};
};

class HoldPositionMode final : public ModeBase
{
public:
  explicit HoldPositionMode(double duration_s);
  void on_activate(std::int64_t now_us) override;
  ModeUpdate update_setpoint(
    const MissionInputs & inputs, const PositionTarget & target) override;

private:
  std::int64_t duration_us_;
  std::int64_t activated_at_us_{};
};

}  // namespace offboard_cpp
