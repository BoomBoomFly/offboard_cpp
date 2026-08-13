#include <gtest/gtest.h>

#include <boomboom_common/msg/flight_state.hpp>

#include "offboard_cpp/gateway_state_mapping.hpp"

TEST(GatewayStateMapping, TakeoverAndFailureNeverReportComplete)
{
  using offboard_cpp::GatewayCommand;
  using offboard_cpp::GatewayState;
  using boomboom_common::msg::FlightState;

  EXPECT_EQ(
    offboard_cpp::gateway_flight_state_value(GatewayState::TAKEOVER, GatewayCommand::GOTO),
    FlightState::FAILED);
  EXPECT_EQ(
    offboard_cpp::gateway_flight_state_value(GatewayState::FAILED, GatewayCommand::GOTO),
    FlightState::FAILED);
  EXPECT_NE(
    offboard_cpp::gateway_flight_state_value(GatewayState::TAKEOVER, GatewayCommand::GOTO),
    FlightState::COMPLETE);
  EXPECT_NE(
    offboard_cpp::gateway_flight_state_value(GatewayState::FAILED, GatewayCommand::GOTO),
    FlightState::COMPLETE);
}
