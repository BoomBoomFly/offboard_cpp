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

While in `HOVER`, an operator may call `/offboard/cancel_mission` using
`std_srvs/srv/Trigger`. A successful call follows the normal local-home return
and Land path. Cancellation is unavailable before `HOVER` or after Land has
been requested; RC takeover, PX4 failsafe, and localization loss remain higher
priority.

Launch does not arm the vehicle:

```bash
ros2 launch offboard_cpp offboard_mission.launch.py
```

PX4 timestamps are derived from `/fmu/out/timesync_status`.  State, faults and
state-change events use `boomboom_common` on `/boomboom/mission/*`.

## Humble SITL evidence

In `gz_x500`, a real RC arm edge has completed the normal 1.5 m takeoff,
approximately 60 s hover, local-home return, Land, and disarm sequence.  A
separate HOVER run accepted `/offboard/cancel_mission`; recorded state/event
data shows the `CANCELLED` return-home transition, followed by an accepted PX4
Land command and a new `landed = true` sample.

These are Humble SITL results only. They do not validate Foxy, Jetson, real
VIO, or aircraft operation.

## Build and controller test

```bash
cd px4/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select offboard_cpp --symlink-install
source install/setup.bash
ctest --test-dir build/offboard_cpp --output-on-failure
```
