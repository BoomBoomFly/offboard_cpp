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

TEST(TopicContractTest, IntegrationTopicsHaveDedicatedMeanings)
{
    EXPECT_STREQ(offboard_topics::kMissionStart, "/mission/start");
    EXPECT_STREQ(offboard_topics::kMissionStartContext, "/mission/start/context");
    EXPECT_STREQ(offboard_topics::kUavMissionState, "/uav/mission_state");
}

}  // namespace
