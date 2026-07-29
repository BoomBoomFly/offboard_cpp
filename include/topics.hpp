#ifndef OFFBOARD_CPP_TOPICS_HPP
#define OFFBOARD_CPP_TOPICS_HPP

namespace offboard_topics
{

inline constexpr char kVehicleStatus[] = "fmu/out/vehicle_status_v1";
inline constexpr char kMissionStart[] = "/mission/start";
inline constexpr char kMissionStartContext[] = "/mission/start/context";
inline constexpr char kUavMissionState[] = "/uav/mission_state";

}  // namespace offboard_topics

#endif  // OFFBOARD_CPP_TOPICS_HPP
