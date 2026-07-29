#include <rc_operator_adapter.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace offboard_cpp
{
RcOperatorAdapter::RcOperatorAdapter(RcOperatorConfig config)
: config_(std::move(config))
{
  if (!config_valid()) {
    throw std::invalid_argument("RC operator channels and thresholds must be explicitly calibrated");
  }
}
bool RcOperatorAdapter::config_valid() const
{
  const int channels[] = {config_.kill_channel, config_.activation_channel,
    config_.arm_enable_channel, config_.recovery_channel};
  const double thresholds[] = {config_.kill_threshold, config_.activation_threshold,
    config_.arm_enable_threshold, config_.recovery_threshold};
  for (const auto channel : channels) {
    if (channel < 0 || channel >= 18) {return false;}
  }
  for (const auto threshold : thresholds) {
    if (!std::isfinite(threshold) || threshold < -1.0 || threshold > 1.0) {return false;}
  }
  return config_.freshness_ns > 0;
}
OperatorSignals RcOperatorAdapter::evaluate(
  const RcSample & sample, std::int64_t now_ns, bool update_edges)
{
  OperatorSignals result;
  const int highest = std::max(std::max(config_.kill_channel, config_.activation_channel),
    std::max(config_.arm_enable_channel, config_.recovery_channel));
  bool valid = sample.timestamp_us != 0 && sample.received_ns >= 0 && now_ns >= sample.received_ns &&
    now_ns - sample.received_ns < config_.freshness_ns && !sample.signal_lost &&
    highest < static_cast<int>(sample.channels.size());
  for (const auto value : sample.channels) {
    valid = valid && std::isfinite(value) && value >= -1.0F && value <= 1.0F;
  }
  if (!valid) {
    last_activation_level_ = false;
    last_recovery_level_ = false;
    return result;
  }
  const bool kill_level = sample.channels[config_.kill_channel] >= config_.kill_threshold;
  const bool activation_level = sample.channels[config_.activation_channel] >= config_.activation_threshold;
  const bool arm_level = sample.channels[config_.arm_enable_channel] >= config_.arm_enable_threshold;
  const bool recovery_level = sample.channels[config_.recovery_channel] >= config_.recovery_threshold;
  result.valid = true;
  result.kill = kill_level;
  result.arm_enable = arm_level;
  result.activation = update_edges && activation_level && !last_activation_level_;
  result.recovery = update_edges && recovery_level && !last_recovery_level_;
  if (update_edges) {
    last_activation_level_ = activation_level;
    last_recovery_level_ = recovery_level;
  }
  return result;
}
OperatorSignals RcOperatorAdapter::update(const RcSample & sample, std::int64_t now_ns)
{
  last_sample_ = sample;
  have_sample_ = true;
  return evaluate(last_sample_, now_ns, true);
}
OperatorSignals RcOperatorAdapter::timeout(std::int64_t now_ns)
{
  if (!have_sample_) {
    return OperatorSignals{};
  }
  return evaluate(last_sample_, now_ns, false);
}
}  // namespace offboard_cpp
