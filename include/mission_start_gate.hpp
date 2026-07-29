#ifndef OFFBOARD_CPP_MISSION_START_GATE_HPP
#define OFFBOARD_CPP_MISSION_START_GATE_HPP
#include <cstdint>
namespace offboard_cpp
{
struct MissionStartContext
{
  std::uint32_t mission_id{0};
  std::uint32_t session_id{0};
  std::uint32_t sequence{0};
  std::uint32_t epoch{0};
};
class MissionStartGate
{
public:
  MissionStartGate(std::uint32_t expected_task, std::uint32_t expected_epoch, std::int64_t freshness_ns);
  bool observe_context(const MissionStartContext & context, std::int64_t received_ns);
  bool accept(std::uint32_t mission_id, std::int64_t now_ns, bool mission_running);
  void restart(std::uint32_t expected_epoch);
  bool consumed() const {return consumed_;}
private:
  std::uint32_t expected_task_;
  std::uint32_t expected_epoch_;
  std::int64_t freshness_ns_;
  MissionStartContext pending_{};
  std::int64_t context_received_ns_{-1};
  bool consumed_{false};
};
}  // namespace offboard_cpp
#endif  // OFFBOARD_CPP_MISSION_START_GATE_HPP
