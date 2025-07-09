import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)  # wait for serial to boot
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Connected to ESP32")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if abs(linear) < 0.05 and abs(angular) < 0.05:
            self.send_command("Neutral")
        elif linear > 0:
            self.send_command("Forward")
        elif linear < 0:
            self.send_command("Backward")

    def send_command(self, cmd):
        self.serial_port.write((cmd + '\n').encode())
        self.get_logger().info(f"Sent: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()