import rclpy
from rclpy.node import Node
from px4_msgs.msg import RcChannels
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import sys, select, termios, tty
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MockRCControlNative(Node):
    def __init__(self):
        super().__init__('mock_rc_native')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.rc_pub = self.create_publisher(RcChannels, '/fmu/out/rc_channels', qos)
        self.param_client = self.create_client(SetParameters, '/offboard_control_node/set_parameters')
        
        self.mode_state = 0.0
        self.gear_state = 0.0
        
        # 保存终端原始设置
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except termios.error:
            self.settings = None

        print("\n" + "="*30)
        print("=== ROS 2 模拟遥控终端已启动 ===")
        print("直接按键执行操作 (无需回车):")
        print(" [1] : 切换 Mode (0.0 <-> 1.0)")
        print(" [2] : 切换 Gear (0.5 <-> 1.0)")
        print(" [q] : 安全退出")
        print("="*30 + "\n")

    def get_key(self):
        if self.settings is None: return None
        tty.setraw(sys.stdin.fileno())
        # 等待 0.1s 看是否有输入
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_data(self):
        # 1. 修改参数
        if self.param_client.service_is_ready():
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name='mock_rc_mode', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.mode_state)),
                Parameter(name='mock_rc_gear', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.gear_state))
            ]
            self.param_client.call_async(req)
        else:
            self.get_logger().warn("参数服务未就绪，仅发送 RC 触发包")

        # 2. 发送符合 PX4 v1.16 RcChannels 归一化语义的有效帧
        msg = RcChannels()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_last_valid = msg.timestamp
        msg.channels = [0.0] * 18
        msg.channel_count = 18
        msg.signal_lost = False
        
        self.rc_pub.publish(msg)
        
        status = f"Mode: {self.mode_state} | Gear: {self.gear_state}"
        sys.stdout.write(f"\r\033[K[发送成功] {status}")
        sys.stdout.flush()

def main():
    rclpy.init()
    node = MockRCControlNative()
    
    try:
        while rclpy.ok():
            key = node.get_key()
            if key == '1':
                node.mode_state = 1.0 if node.mode_state == 0.0 else 0.0
                node.send_data()
            elif key == '2':
                # 假设 gear 阈值是 0.75，切换 0.5 和 1.0
                node.gear_state = 1.0 if node.gear_state < 0.75 else 0.5
                node.send_data()
            elif key == 'q' or key == '\x03': # q 或 Ctrl+C
                print("\n\n正在安全退出并恢复终端...")
                break
            
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:
        print(f"\n运行时错误: {e}")
    finally:
        if node.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()