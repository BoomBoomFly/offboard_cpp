# PX4 Offboard 网关

`offboard_gateway_node` 是 `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint` 和
`/fmu/in/vehicle_command` 默认且唯一的写入方。它使用 `boomboom_common/action/ExecuteFlight`
提供 `/offboard/execute_flight`，并在 `/boomboom/flight_state` 发布当前基于 EKF2 的快照。

任一时刻只有一个网关目标控制设定点。第一个 `TAKEOFF` 会等待健康的本地位置、已观测到的
`DISARMED` 状态，以及符合条件的 `STICK_GESTURE` 或 `RC_SWITCH` 解锁上升沿。随后它以 20 Hz
发送位置设定点至少一秒，并同时要求匹配的 PX4 Offboard ACK 和实际 Offboard 导航状态。该进程从不发送
ARM、DISARM 或 Kill 命令。

起飞后，`GOTO`、`HOLD`、`RETURN_HOME` 和 `LAND` 是顺序 Action 命令。`RETURN_HOME` 会在当前高度
返回冻结的本地起点 XY，但不会降落。没有活动目标时，Offboard 网关保持最后一个安全目标。`LAND` 前取消
Action 会变为原地 `HOLD`；它不会自动返航，也不会重新取得 PX4 控制权。本地位置丢失会请求原地 PX4 `LAND`，
而 RC 接管或 PX4 故障保护会结束网关，且不重试。已经发送的 `LAND` 请求不可逆。

启动网关：

```bash
ros2 launch px4_bringup offboard_gateway.launch.py
```

PX4 时间戳派生自 `/fmu/out/timesync_status`。网关状态通过
`/boomboom/flight_state` 传递。新 Action gateway 尚未完成 SITL 或实机验证。

## 构建和测试

```bash
cd px4/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select offboard_cpp --symlink-install
source install/setup.bash
ctest --test-dir build/offboard_cpp --output-on-failure
```
