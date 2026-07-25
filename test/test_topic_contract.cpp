#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <string>

#include <topics.hpp>

#ifndef OFFBOARD_NODE_SOURCE
#error "OFFBOARD_NODE_SOURCE must identify the production node source file"
#endif

namespace
{

TEST(TopicContractTest, VehicleStatusMatchesPx4V1162BridgeContract)
{
    EXPECT_STREQ(
        offboard_topics::kVehicleStatus,
        "fmu/out/vehicle_status_v1");
}

TEST(TopicContractTest, ProductionSubscriptionUsesContractWithoutLegacyLiteral)
{
    std::ifstream input(OFFBOARD_NODE_SOURCE);
    ASSERT_TRUE(input.is_open()) << "Unable to inspect " << OFFBOARD_NODE_SOURCE;

    const std::string source{
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};

    EXPECT_NE(
        source.find(
            "create_subscription<px4_msgs::msg::VehicleStatus>("
            "\n        offboard_topics::kVehicleStatus"),
        std::string::npos);
    EXPECT_EQ(
        source.find("\"fmu/out/vehicle_status\""),
        std::string::npos);
}

}  // namespace
