#include "offboard_cpp/gateway_state_mapping.hpp"

#include <boomboom_common/msg/flight_state.hpp>

namespace offboard_cpp
{

std::uint8_t gateway_flight_state_value(GatewayState state, GatewayCommand command)
{
  using FlightState = boomboom_common::msg::FlightState;
  switch (state) {
    case GatewayState::WAIT_DISARMED: return FlightState::IDLE;
    case GatewayState::READY:
    case GatewayState::WAIT_RC_ARM: return FlightState::READY;
    case GatewayState::OFFBOARD_PRESTREAM:
    case GatewayState::REQUEST_OFFBOARD:
    case GatewayState::EXECUTING:
      return command == GatewayCommand::HOLD ? FlightState::HOLDING : FlightState::EXECUTING;
    case GatewayState::IDLE: return FlightState::HOLDING;
    case GatewayState::LAND_REQUEST:
    case GatewayState::WAIT_LANDED: return FlightState::LANDING;
    case GatewayState::TAKEOVER:
    case GatewayState::FAILED: return FlightState::FAILED;
  }
  return FlightState::UNKNOWN;
}

}  // namespace offboard_cpp
