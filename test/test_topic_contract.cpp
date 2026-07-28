#include <gtest/gtest.h>

#include <topics.hpp>

namespace
{

TEST(TopicContractTest, VehicleStatusMatchesPx4V1162BridgeContract)
{
    EXPECT_STREQ(
        offboard_topics::kVehicleStatus,
        "fmu/out/vehicle_status_v1");
}

}  // namespace
