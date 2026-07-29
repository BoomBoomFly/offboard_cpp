#include <cassert>
#include <stdexcept>
#include <mission_start_gate.hpp>
#include <rc_operator_adapter.hpp>
using offboard_cpp::MissionStartContext;
using offboard_cpp::MissionStartGate;
using offboard_cpp::RcOperatorAdapter;
using offboard_cpp::RcOperatorConfig;
using offboard_cpp::RcSample;
namespace
{
RcOperatorConfig rc_config()
{
  RcOperatorConfig config;
  config.kill_channel = 0;
  config.activation_channel = 1;
  config.arm_enable_channel = 2;
  config.recovery_channel = 3;
  config.kill_threshold = 0.5;
  config.activation_threshold = 0.5;
  config.arm_enable_threshold = 0.5;
  config.recovery_threshold = 0.5;
  config.freshness_ns = 300000000LL;
  return config;
}
void test_rc_fail_closed_and_edges()
{
  bool rejected_defaults = false;
  try {RcOperatorAdapter invalid(RcOperatorConfig{});} catch (const std::invalid_argument &) {
    rejected_defaults = true;
  }
  assert(rejected_defaults);
  RcOperatorAdapter adapter(rc_config());
  RcSample sample;
  sample.timestamp_us = 1000;
  sample.received_ns = 0;
  sample.signal_lost = false;
  sample.channels = {-1.0F, -1.0F, 1.0F, -1.0F};
  auto signals = adapter.update(sample, 0);
  assert(signals.valid && !signals.kill && !signals.activation && signals.arm_enable);
  sample.timestamp_us = 1001;
  sample.received_ns = 20000000LL;
  sample.channels[1] = 1.0F;
  signals = adapter.update(sample, sample.received_ns);
  assert(signals.activation);
  sample.timestamp_us = 1002;
  sample.received_ns += 20000000LL;
  assert(!adapter.update(sample, sample.received_ns).activation);
  assert(adapter.timeout(100000000LL).arm_enable);
  assert(adapter.timeout(400000000LL).kill);
  sample.timestamp_us = 1003;
  sample.received_ns = 500000000LL;
  sample.signal_lost = true;
  assert(adapter.update(sample, sample.received_ns).kill);
  sample.signal_lost = false;
  sample.channels[0] = 2.0F;
  assert(adapter.update(sample, sample.received_ns).kill);
}
void test_start_session_freshness_duplicate_and_old_epoch()
{
  MissionStartGate gate(3, 0x12345678U, 500000000LL);
  MissionStartContext good{3, 7, 9, 0x12345678U};
  assert(gate.observe_context(good, 100));
  assert(!gate.accept(2, 101, false));
  assert(gate.accept(3, 102, false));
  assert(!gate.accept(3, 103, false));
  assert(!gate.observe_context(MissionStartContext{3, 8, 10, 0x12345678U}, 104));

  MissionStartGate ordering(3, 0x12345678U, 500000000LL);
  assert(ordering.observe_context(MissionStartContext{3, 10, 10, 0x12345678U}, 0));
  assert(!ordering.observe_context(MissionStartContext{3, 10, 9, 0x12345678U}, 1));
  assert(ordering.observe_context(MissionStartContext{3, 10, 11, 0x12345678U}, 2));
  assert(!ordering.observe_context(MissionStartContext{3, 9, 12, 0x12345678U}, 3));
  assert(ordering.observe_context(MissionStartContext{3, 11, 1, 0x12345678U}, 4));

  MissionStartGate stale(3, 0x12345678U, 500000000LL);
  assert(stale.observe_context(good, 0));
  assert(!stale.accept(3, 500000000LL, false));
  MissionStartGate running(3, 0x12345678U, 500000000LL);
  assert(running.observe_context(good, 0));
  assert(!running.accept(3, 1, true));
  assert(!running.accept(3, 2, false));

  gate.restart(0x87654321U);
  assert(!gate.observe_context(good, 200));
  MissionStartContext new_epoch{3, 8, 1, 0x87654321U};
  assert(gate.observe_context(new_epoch, 201));
  assert(gate.accept(3, 202, false));
}
}
int main()
{
  test_rc_fail_closed_and_edges();
  test_start_session_freshness_duplicate_and_old_epoch();
  return 0;
}
