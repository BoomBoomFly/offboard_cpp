#ifndef OFFBOARD_CPP_SAFETY_GATE_HPP
#define OFFBOARD_CPP_SAFETY_GATE_HPP

#include <cstdint>
#include <string>

namespace offboard_cpp
{

// This class deliberately has no ROS dependency.  The ROS adapter is only a
// transport for these inputs and is the sole owner of PX4 input publishers.
enum class GateState {
  WAIT,
  PRESTREAM,
  REQUEST_MODE,
  REQUEST_ARM,
  ACTIVE,
  REQUEST_LAND,
  LANDING,
  REQUEST_DISARM,
  STANDBY_DISARMED,
  FAULT_LATCHED,
};

enum class CommandKind {
  NONE,
  SET_MODE_OFFBOARD,
  ARM,
  LAND,
  DISARM,
};

enum class MissionRequest : std::uint8_t {
  NONE = 0,
  LAND_HOME = 1,
  DISARM = 2,
  REARM = 3,
  LAND_PLATFORM = 4,
};

enum class AckResult {
  ACCEPTED,
  IN_PROGRESS,
  REJECTED,
  TIMEOUT,
  MISMATCH,
};

struct Authority {
  bool fresh{false};
  bool single_writer{false};
  bool single_owner{false};
  std::string owner_id;
  std::string lease_id;
  std::string epoch;
  std::uint64_t sequence{0};
};

struct GateInputs {
  bool vehicle_status_fresh{false};
  bool odometry_fresh{false};
  bool timesync_fresh{false};
  bool rc_fresh{false};
  bool kill_fresh{false};
  bool setpoint_fresh{false};
  bool mode_fresh{false};
  bool setpoint_mode_paired{false};
  bool clock_monotonic{true};
  bool kill_latched{false};
  bool vehicle_in_offboard{false};
  bool vehicle_armed{false};
  bool landing_confirmed{false};
  std::uint64_t vehicle_status_generation{0};
  bool manual_arm_enable{false};
  MissionRequest mission_request{MissionRequest::NONE};
  std::uint64_t mission_request_generation{0};
  Authority authority{};
};

struct CommandAck {
  std::uint32_t command{0};
  std::uint8_t target_system{0};
  std::uint16_t target_component{0};
  bool from_external{false};
  std::uint64_t status_generation{0};
  AckResult result{AckResult::MISMATCH};
};

struct GateDecision {
  bool publish_setpoint{false};
  bool publish_mode{false};
  CommandKind command{CommandKind::NONE};
  GateState state{GateState::WAIT};
  bool fault_latched{false};
  const char * reason{"waiting"};
};

const char * gate_state_name(GateState state);

class SafetyGate
{
public:
  static constexpr std::uint16_t kVehicleCmdDoSetMode = 176;
  static constexpr std::uint16_t kVehicleCmdArmDisarm = 400;
  static constexpr std::uint16_t kVehicleCmdNavLand = 21;
  static constexpr std::uint8_t kTargetSystem = 1;
  static constexpr std::uint8_t kTargetComponent = 1;

  explicit SafetyGate(
    std::string expected_owner, std::string expected_lease, std::string expected_epoch,
    bool auto_arm = false, std::int64_t prestream_ns = 2000000000LL,
    std::uint32_t prestream_samples = 40,
    bool require_armed_before_offboard = false);

  GateDecision tick(std::int64_t monotonic_ns, const GateInputs & inputs);
  // ACK has no request sequence in PX4.  Correlation is therefore the one
  // pending command plus deadline, exact command/target/external fields, and
  // an unchanged current authority sequence.
  GateDecision observe_ack(
    std::int64_t monotonic_ns, const CommandAck & ack, const Authority & authority);
  GateDecision request_manual_activation(std::int64_t monotonic_ns, const GateInputs & inputs);
  GateDecision restart();

  GateState state() const { return state_; }
  bool fault_latched() const { return state_ == GateState::FAULT_LATCHED; }
  std::uint64_t pending_sequence() const { return pending_sequence_; }

private:
  bool stream_ready(const GateInputs & inputs) const;
  bool ready(const GateInputs & inputs) const;
  bool authority_matches(const Authority & authority) const;
  bool consume_request(const GateInputs & inputs, MissionRequest request);
  void begin_rearm(std::int64_t now_ns);
  GateDecision decision(bool setpoint, bool mode, CommandKind command, const char * reason) const;
  GateDecision latch(const char * reason);
  void begin_pending(CommandKind command, std::int64_t now_ns, std::uint64_t sequence);
  void clear_pending();

  std::string expected_owner_;
  std::string expected_lease_;
  std::string expected_epoch_;
  bool auto_arm_{false};
  // When enabled, PX4 must first report a manual RC arm before this gate
  // emits any Offboard setpoint or mode command.
  bool require_armed_before_offboard_{false};
  std::int64_t prestream_ns_{2000000000LL};
  std::uint32_t prestream_samples_required_{40};
  GateState state_{GateState::WAIT};
  std::int64_t last_tick_ns_{-1};
  std::int64_t prestream_started_ns_{-1};
  std::uint32_t prestream_samples_{0};
  CommandKind pending_command_{CommandKind::NONE};
  std::uint64_t pending_sequence_{0};
  std::int64_t pending_deadline_ns_{-1};
  std::int64_t confirmation_deadline_ns_{-1};
  bool mode_acknowledged_{false};
  bool arm_acknowledged_{false};
  bool disarm_acknowledged_{false};
  std::uint64_t mode_ack_status_generation_{0};
  std::uint64_t arm_ack_status_generation_{0};
  std::uint64_t disarm_ack_status_generation_{0};
  std::uint64_t last_request_generation_{0};
  bool manual_activation_granted_{false};
};

}  // namespace offboard_cpp

#endif  // OFFBOARD_CPP_SAFETY_GATE_HPP
