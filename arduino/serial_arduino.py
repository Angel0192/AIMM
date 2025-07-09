import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.serial_conn = serial.Serial('/dev/tty.usbmodemXXXX', 115200)  # CHANGE THIS

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        cmd = f"{linear:.2f},{angular:.2f}\n"
        self.get_logger().info(f'Sending: {cmd.strip()}')
        self.serial_conn.write(cmd.encode())

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
