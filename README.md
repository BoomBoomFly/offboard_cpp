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

只能启动一个写入方：

```bash
# 默认网关写入方
ros2 launch px4_bringup offboard_gateway.launch.py

# 仅限旧版验证写入方；不得与网关启动项同时使用
ros2 launch px4_bringup offboard_mission.launch.py
```

`offboard_mission_node` 和 `offboard_mission.launch.py` 仍是已记录第一阶段验证的弃用兼容路径。
网关启动项不会启动它们。

## 旧版任务实现

`offboard_mission_node` 是 `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint` 和
`/fmu/in/vehicle_command` 的唯一写入方。

在内部，`MissionExecutor` 负责任务编排、获取 Offboard 和安全状态转换。它会启用小型本地
`ModeBase` 实现：`ReachPositionMode` 生成设定点直至位置持续稳定，而 `HoldPositionMode`
在设定时长内保持目标。这些是包内抽象，不依赖、不注册到、也不冒充 PX4 ROS 2 Interface Library 的外部模式。

它在就绪前会观察到未解锁的 VehicleStatus。只有 PX4 `latest_arming_reason` 为 `STICK_GESTURE` 或
`RC_SWITCH` 的解锁上升沿才能启动一次运行。该节点从不发送解锁、上锁或终止命令。

该任务持续发送位置设定点至少一秒，请求 Offboard，等待已接受的 ACK 和实际 Offboard 导航状态，从冻结的
EKF2 本地起点起飞 1.5 m，稳定后悬停 60 s，在 XY 平面返航，并发送一条 PX4 `LAND` 命令。本地位置丢失会请求
原地 `LAND`；模式退出或故障保护会结束运行，不重试 Offboard。只有 `DO_SET_MODE` ACK 的目标系统/组件
与对应命令所使用的任务源系统/组件相符时，才接受该 ACK。

处于 `HOVER` 时，操作员可通过 `std_srvs/srv/Trigger` 调用 `/offboard/cancel_mission`。调用成功后将遵循
正常的本地起点返航和 `LAND` 路径。`HOVER` 前或已请求 `LAND` 后均不能取消；RC 接管、PX4 故障保护和定位丢失
仍具有更高优先级。

启动项不会解锁飞行器：

```bash
ros2 launch px4_bringup offboard_mission.launch.py
```

PX4 时间戳派生自 `/fmu/out/timesync_status`。状态、故障和状态变更事件通过
`/boomboom/mission/*` 上的 `boomboom_common` 传递。

## Humble SITL 证据

在 `gz_x500` 中，一次真实 RC 解锁上升沿已经完成正常的 1.5 m 起飞、约 60 s 悬停、本地起点返航、`LAND` 和
上锁序列。另一次 `HOVER` 运行接受了 `/offboard/cancel_mission`；记录的状态/事件数据显示了 `CANCELLED`
返航状态转换，随后是已接受的 PX4 `LAND` 命令和一条新的 `landed = true` 样本。

这些仅是 Humble SITL 结果，不验证 Foxy、Jetson、真实 VIO 或飞行器运行。

## 构建和 Mode/Executor 测试

```bash
cd px4/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select offboard_cpp --symlink-install
source install/setup.bash
ctest --test-dir build/offboard_cpp --output-on-failure
```
