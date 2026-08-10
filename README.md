# PX4 Offboard mission

`offboard_mission_node` is the sole writer of `/fmu/in/offboard_control_mode`,
`/fmu/in/trajectory_setpoint`, and `/fmu/in/vehicle_command`.

It observes a disarmed VehicleStatus before becoming ready.  Only an arm rising
edge whose PX4 `latest_arming_reason` is `STICK_GESTURE` or `RC_SWITCH` starts
one run.  The node never sends arm, disarm, or kill commands.

The mission streams position setpoints for at least one second, requests
Offboard, waits for its accepted ACK and actual Offboard nav state, takes off
1.5 m from the frozen EKF2 local home, stabilizes, hovers for 60 s, returns in
XY, and sends one PX4 Land command.  Local-position loss requests Land in
place; mode exit or failsafe ends the run without retrying Offboard.

Launch does not arm the vehicle:

```bash
ros2 launch offboard_cpp offboard_mission.launch.py
```

PX4 timestamps are derived from `/fmu/out/timesync_status`.  State, faults and
state-change events use `boomboom_common` on `/boomboom/mission/*`.
