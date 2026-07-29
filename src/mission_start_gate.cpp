#include <mission_start_gate.hpp>
#include <stdexcept>
namespace offboard_cpp
{
namespace
{
bool newer_u8(std::uint32_t candidate, std::uint32_t current)
{
  const auto delta = static_cast<std::uint8_t>(candidate - current);
  return delta != 0 && delta < 128;
}
}
MissionStartGate::MissionStartGate(
  std::uint32_t expected_task, std::uint32_t expected_epoch, std::int64_t freshness_ns)
: expected_task_(expected_task), expected_epoch_(expected_epoch), freshness_ns_(freshness_ns)
{
  if (expected_task_ == 0 || expected_epoch_ == 0 || freshness_ns_ <= 0) {
    throw std::invalid_argument("START task, expected source epoch and freshness are required");
  }
}
bool MissionStartGate::observe_context(
  const MissionStartContext & context, std::int64_t received_ns)
{
  if (received_ns < 0 || context.mission_id == 0 || context.session_id == 0 ||
    context.sequence == 0 || context.epoch == 0 || context.epoch != expected_epoch_ || consumed_)
  {
    return false;
  }
  if (context_received_ns_ >= 0) {
    if (received_ns < context_received_ns_) {return false;}
    if (context.session_id == pending_.session_id) {
      if (!newer_u8(context.sequence, pending_.sequence)) {return false;}
    } else if (!newer_u8(context.session_id, pending_.session_id)) {
      return false;
    }
  }
  pending_ = context;
  context_received_ns_ = received_ns;
  return true;
}
bool MissionStartGate::accept(
  std::uint32_t mission_id, std::int64_t now_ns, bool mission_running)
{
  if (mission_running) {
    pending_ = MissionStartContext{};
    context_received_ns_ = -1;
    return false;
  }
  if (consumed_ || mission_id == 0 || mission_id != expected_task_ ||
    pending_.mission_id != mission_id || pending_.session_id == 0 || pending_.sequence == 0 ||
    pending_.epoch != expected_epoch_ || context_received_ns_ < 0 || now_ns < context_received_ns_ ||
    now_ns - context_received_ns_ >= freshness_ns_)
  {
    return false;
  }
  consumed_ = true;
  return true;
}
void MissionStartGate::restart(std::uint32_t expected_epoch)
{
  if (expected_epoch == 0) {throw std::invalid_argument("new START epoch is required");}
  expected_epoch_ = expected_epoch;
  pending_ = MissionStartContext{};
  context_received_ns_ = -1;
  consumed_ = false;
}
}  // namespace offboard_cpp
