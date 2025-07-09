import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('cmd_vel publisher node started!')

    def timer_callback(self):
        msg = Twist()

        # Customize these values for linear and angular movement
        msg.linear.x = 0.5   # forward speed
        msg.angular.z = 0.0  # no rotation

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
