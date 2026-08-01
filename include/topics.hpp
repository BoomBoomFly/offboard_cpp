#ifndef OFFBOARD_CPP_TOPICS_HPP
#define OFFBOARD_CPP_TOPICS_HPP

namespace offboard_topics
{

// PX4 v1.16 exposes the versioned VehicleStatus interface on this deployed
// uXRCE-DDS graph.  Keep all consumers on the one verified runtime endpoint.
inline constexpr char kVehicleStatus[] = "/fmu/out/vehicle_status_v1";
inline constexpr char kMissionStart[] = "/mission/start";
inline constexpr char kMissionStartContext[] = "/mission/start/context";
inline constexpr char kUavMissionState[] = "/uav/mission_state";

}  // namespace offboard_topics

#endif  // OFFBOARD_CPP_TOPICS_HPP
