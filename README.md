# PX4 Offboard

PX4 无人机 Offboard 模式底层控制(C++)

基于 ubuntu20.04 系统下的 ros2(foxy)

该功能包仅作为 Offboard **底层主控**

提供小型无人机蜂群的

消息依赖：[px4_msgs](https://github.com/PX4/px4_msgs/tree/release/1.14)

通信依赖: [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.2)

具体 Offboard 轨迹控制需要外部规划器(或自行写个轨迹节点，并将轨迹发布至对应话题)

## Demo

首次起飞示例 和 动物检测系统，具体请前往 [src/examples/README.md](./src/examples/README.md)

动物检测系统中的 YOLO 识别部分请前往 [YOLO and 飞桨推理(python)](https://github.com/AyasOwen/cv_yolo_paddle_pkg)

## Hardware

其中**蜂群**所使用的无人机硬件清单在 [hardware/hardware.pdf](./hardware/hardware.pdf)

## 注意事项

在 PX4 1.14.3 固件中, 正常编译固件会导致 PX4 不会向 **Micro-XRCE-DDS-Agent** 发布 **电池、RC、着陆检测** 的话题, 需要下载固件源码，修改后重新编译、烧录，才能有这些话题。

- 固件下载

```bash
git clone -b v1.14.3 git@github.com:PX4/PX4-Autopilot.git
```

- 需要修改该配置文件: `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`

```yaml
publications:
  # ... 保留原有内容 ...
  # 添加以下内容
  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus

  - topic: /fmu/out/rc_channels
    type: px4_msgs::msg::RcChannels

  - topic: /fmu/out/vehicle_land_detected
    type: px4_msgs::msg::VehicleLandDetected
```

- 编译

```bash
# 进入 conda 环境
# 默认已经下好所有前置
conda activate ros2
sudo apt install proxychains4
sudo apt-get install git

cd ./PX4-Autopilot/
# 补全子模块
git submodule update --init –recursive

# 安装依赖
sudo apt install ros-dev-tools
cd ./Tools/setup

# 修改 requirements.txt 文件内容
vim ./requirements.txt
# 将 matplotlit>=3.0.*改为 3.0.1或3.0.0
# 保存退出

chmod +x ubuntu.sh
./ubuntu.sh

# 编译
cd ~/px4/PX4-Autopilot/
# chmod +x Tools/check_submodules.sh
# chmod +x Tools/simulation/gazebo-classic/*.sh

# 编译你所需的版本
# 仿真用
# make px4_sitl gazebo-classic

# 实机用
make px4_fmu-v3_default
```

## FSM 有限状态机框架图

框架参考了浙大高飞老师的 [PX4CTRL](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master/src/realflight_modules/px4ctrl) 的部分框架

	         系统启动
	            |
	            |
	            v
	-------> 位置控制 <-------------------
	|         ^   |   \                 |
	|         |   |    \                |
	|         |   |     > 自动起飞       |
	|         |   |      /              |
	|         |   |     /               |
	|         |   |    /                |
	|         |   v   /                 |
	|       自动悬停 <                   |
	|         ^   |  \                  |
	|         |   |   \                 |
	|         |	  |    > 自动降落 -------|
	|         |   |          ^          |
	|         |   v          |          |
	-------- OFFBOARD--->定高模式--------

### 状态机说明

状态机包含以下状态:
- **POSITION**: 位置控制模式 (初始状态)
- **AUTO_HOVER**: 自动悬浮，等待进入Offboard
- **OFFBOARD**: Offboard 模式，执行外部指令
- **AUTO_TAKEOFF**: 自动起飞
- **AUTO_LAND**: 自动降落
- **WARNING**: 低电量警告，自动降落

### 自定义轨迹

参考 `src/offboard_demo.cpp` 修改航点列表:

```cpp
waypoints_ = {
    {x1, y1, z1, yaw1},  // 航点1 (NED坐标系)
    {x2, y2, z2, yaw2},  // 航点2
    // ... 添加更多航点
};
```

### 参数配置

编辑 `config/ctrl_param.yaml` 修改参数:

```yaml
msg_timeout:
  rc: 0.5              # 遥控器超时时间
  odom: 0.5            # 里程计超时时间
  
takeoff_land:
  enable: true         # 启用自动起飞降落
  enable_arm: true     # 自动解锁
  speed: 0.3           # 起飞降落速度 (m/s)
  height: 1.0          # 起飞高度 (m)
  
low_voltage: 13.2      # 低电压报警阈值
odom_pos_jump: 0.3     # 位置跳变容忍度

rc_debug:
  ch_mode: 8           # 遥控器悬浮模式通道
  ch_gear: 9           # 遥控器 Offboard 模式通道
```

### 节点说明

- **订阅话题**:

  - `/fmu/out/vehicle_odometry` - 里程计数据

  - `/fmu/out/vehicle_status_v1` - 飞控状态（PX4 v1.16.2）

  - `/fmu/out/rc_channels` - 遥控器通道

  - `/fmu/out/battery_status` - 电池状态

  - `/fmu/out/vehicle_land_detected` - 着陆检测

  - `/offboard/cmd` - Offboard 控制指令

  - `/offboard/cmd_mode` - Offboard 控制模式

  - `/offboard/takeoff_land` - 起飞/降落命令

- **发布话题**:

  - `/fmu/in/trajectory_setpoint` - 轨迹设定点

  - `/fmu/in/offboard_control_mode` - Offboard 控制模式

  - `/fmu/in/vehicle_command` - 飞行器命令

  - `/offboard/trigger` - 用于触发外部命令并统一时间戳

### Debug 说明

在 **CMakeLists.txt** 文件内预留了 Debug 用的宏定义：

- `add_compile_definitions(TEXT_RC)` - 没有遥控器时测试 RC 用

- `add_compile_definitions(VEL_IN_BODY)` - 处理机体坐标系速度，如果 Odom 里的速度是相对于机体坐标系的，需要旋转到世界坐标系

- `add_compile_definitions(TEXT_OFFBOARD)` - 打印外部命令时间戳用

在 **text** 文件夹内有预设好的 Debug 用脚本：

- 依赖

```bash
pip3 install pynput -i https://mirrors.huaweicloud.com/repository/pypi/simple
```

- `mock_rc_control.py` - 用于在仿真环境下模拟 RC 通道按键

### 无人机蜂群使用说明

- 蜂群 demo 启动

一次启动三个无人机节点，需要修改的请前往 `launch/offboard_swarm_control.launch.py` 进行修改

```BASH
ros2 launch offboard_cpp offboard_swarm_control.launch.py
```

- 飞控设置说明

需要在 PX4 飞控端进行设置，可以选择在编译固件时通过修改固件来实现每台无人机所发布的话题命名空间不同

这边提供另一种已经编译烧录固件后的实现方法：

在无人机的 SD 卡中，进入 /etc/ 目录（如果没有就新建），创建一个名为 extras.txt 的文件。

在文件中写入以下指令（假设你使用的是 TELEM2，串口名为 /dev/ttyS2，波特率 921600）：

```BASH
# 先停止默认启动的客户端
uxrce_dds_client stop
# 重新启动并带上命名空间 -n drone1
uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600 -n drone1
```

## 故障排除

### 问题1: 无法接收飞控数据

- 检查 PX4 是否正常运行

- 检查 DDS 连接: `ros2 topic list | grep fmu`

### 问题2: 无法进入Offboard模式

- 确认遥控器通道映射正确

- 检查参数文件中的 `ch_mode` 和 `ch_gear` 配置

- `ch_mode` - 悬浮模式扳机

- `ch_gear` - 命令模式扳机

- 确保里程计数据正常

### 问题3: 编译错误

```bash
# 清理构建
rm -rf build/offboard_cpp install/offboard_cpp log/offboard_cpp
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select offboard_cpp --cmake-clean-first
```

## 安全注意事项

1. **首次使用**: 建议在仿真环境中测试
2. **电量监控**: 系统会在低电量时自动降落
3. **失控保护**: 遥控器失联或里程计失效时自动返回位置(高度)控制模式(如果里程计失效且没有定高信息，则会返回手动模式)
4. **急停**: 随时可以通过遥控器从命令模式切换回位置模式（或悬浮模式）

## 其他部分

[兼容 ros2(foxy) 的 vision_to_mavros 功能包实现(BoomBoomFly, Not open source)](https://github.com/BoomBoomFly/ros2_foxy_vision_to_mavros)

[OpenCv 等基础功能实现(C++)](https://github.com/AyasOwen/opencv_cpp)

[D435 和 T265 的联合使用(blog, ros1 暂未做 ros2 移植)](https://ayasowen.github.io/2024/11/17/T265%E5%92%8CD435%E8%81%94%E5%90%88%E4%BD%BF%E7%94%A8/)

By[@CGC12123](https://github.com/CGC12123) [适用于机器人上/下位机串口通信的驱动（上位机部分）](https://github.com/BoomBoomFly/serial_driver_ros2)