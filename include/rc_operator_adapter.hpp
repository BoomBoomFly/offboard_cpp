#ifndef OFFBOARD_CPP_RC_OPERATOR_ADAPTER_HPP
#define OFFBOARD_CPP_RC_OPERATOR_ADAPTER_HPP

#include <cstdint>
#include <vector>

namespace offboard_cpp
{
struct RcOperatorConfig
{
  int kill_channel{-1};
  int activation_channel{-1};
  int arm_enable_channel{-1};
  double kill_threshold{2.0};
  double activation_threshold{2.0};
  double arm_enable_threshold{2.0};
  std::int64_t freshness_ns{0};
};
struct RcSample
{
  std::uint64_t timestamp_us{0};
  std::int64_t received_ns{-1};
  bool signal_lost{true};
  std::vector<float> channels;
};
struct OperatorSignals
{
  bool kill{true};
  bool activation{false};
  bool arm_enable{false};
  bool valid{false};
};
class RcOperatorAdapter
{
public:
  explicit RcOperatorAdapter(RcOperatorConfig config);
  OperatorSignals update(const RcSample & sample, std::int64_t now_ns);
  OperatorSignals timeout(std::int64_t now_ns);
private:
  bool config_valid() const;
  OperatorSignals evaluate(const RcSample & sample, std::int64_t now_ns, bool update_edges);
  RcOperatorConfig config_;
  RcSample last_sample_{};
  bool have_sample_{false};
  bool last_activation_level_{false};
};
}  // namespace offboard_cpp
#endif  // OFFBOARD_CPP_RC_OPERATOR_ADAPTER_HPP
