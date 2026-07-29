file(READ "${SOURCE}" CONTENT)
foreach(REQUIRED
    "create_subscription<std_msgs::msg::UInt32>"
    "create_subscription<std_msgs::msg::UInt64>"
    "offboard_topics::kMissionStartContext"
    "durability_volatile")
  string(FIND "${CONTENT}" "${REQUIRED}" POSITION)
  if(POSITION EQUAL -1)
    message(FATAL_ERROR "missing START contract: ${REQUIRED}")
  endif()
endforeach()
foreach(FORBIDDEN
    "Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_"
    "create_subscription<std_msgs::msg::Bool>(\n      offboard_topics::kMissionStart")
  string(FIND "${CONTENT}" "${FORBIDDEN}" POSITION)
  if(NOT POSITION EQUAL -1)
    message(FATAL_ERROR "forbidden START contract: ${FORBIDDEN}")
  endif()
endforeach()
