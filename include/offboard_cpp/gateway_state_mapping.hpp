#pragma once

#include <cstdint>

#include "offboard_cpp/gateway_executor.hpp"

namespace offboard_cpp
{

std::uint8_t gateway_flight_state_value(GatewayState state, GatewayCommand command);

}  // namespace offboard_cpp
