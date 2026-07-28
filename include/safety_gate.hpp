#pragma once

#include <cstdint>
#include <optional>
#include <string>

// This class is deliberately ROS-free so every refusal transition can be unit
// tested without creating a DDS publisher.  The adapter is the only code that
// converts an allowed decision into a PX4 input publication.
enum class GateState {
  WAIT,
  PRESTREAM,
  REQUEST_MODE,
  REQUEST_ARM,
  ACTIVE,
  STANDBY_DISARMED,
  FAULT_LATCHED,
};

enum class PendingCommand { NONE, MODE, ARM };

struct SafetyGateInputs {
  bool vehicle_status_fresh{false};
  bool odometry_fresh{false};
  bool battery_fresh{false};
  bool timesync_fresh{false};
  bool rc_fresh{false};
  bool kill_fresh{false};
  bool setpoint_fresh{false};
  bool mode_fresh{false};
  bool setpoint_mode_paired{false};
  bool clock_monotonic{false};
  bool kill_latched{true};
  bool manual_arm_enable{false};
  bool manual_activation{false};
  bool manual_reset{false};
  bool single_writer{false};
  bool single_owner{false};
  std::uint64_t owner_id{0};
  std::uint64_t lease_id{0};
  std::uint64_t epoch{0};
  std::uint64_t sequence{0};
};

struct CommandAck {
  std::uint16_t command{0};
  std::uint8_t target_system{0};
  std::uint16_t target_component{0};
  bool accepted{false};
  bool in_progress{false};
};

class SafetyGate {
 public:
  static constexpr std::uint64_t kPrestreamDurationUs = 1'000'000;
  static constexpr std::uint32_t kPrestreamSamples = 20;
  static constexpr std::uint64_t kCommandAckTimeoutUs = 1'000'000;

  void tick(std::uint64_t now_us, const SafetyGateInputs& inputs);
  void observe_ack(std::uint64_t now_us, const CommandAck& ack);
  void source_epoch_changed();

  [[nodiscard]] GateState state() const { return state_; }
  [[nodiscard]] bool fault_latched() const { return state_ == GateState::FAULT_LATCHED; }
  [[nodiscard]] bool allows_control_payload() const;
  [[nodiscard]] PendingCommand command_to_send() const;
  PendingCommand take_command_to_send();
  [[nodiscard]] std::uint16_t expected_command() const;
  [[nodiscard]] const std::string& fault_reason() const { return fault_reason_; }

 private:
  [[nodiscard]] bool healthy(const SafetyGateInputs& inputs) const;
  [[nodiscard]] bool authority_valid(const SafetyGateInputs& inputs) const;
  void latch(const char* reason);
  void begin_command(PendingCommand command, std::uint64_t now_us,
                     const SafetyGateInputs& inputs);

  GateState state_{GateState::WAIT};
  PendingCommand pending_{PendingCommand::NONE};
  std::uint64_t prestream_start_us_{0};
  std::uint32_t prestream_samples_{0};
  std::uint64_t command_started_us_{0};
  std::uint64_t pending_owner_id_{0};
  std::uint64_t pending_lease_id_{0};
  std::uint64_t pending_epoch_{0};
  std::uint64_t pending_sequence_{0};
  bool command_emitted_{false};
  std::uint64_t authority_owner_id_{0};
  std::uint64_t authority_lease_id_{0};
  std::uint64_t authority_epoch_{0};
  std::uint64_t authority_sequence_{0};
  std::string fault_reason_;
};
