import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import math

class SinMotorPublisher(Node):

    def __init__(self):
        super().__init__('sin_motor_publisher')
        self.publisher_ = self.create_publisher(String, 'motor_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.sin(self.x):.2f}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.x += math.pi/10

    def sin(self, x):
        return 45 * math.sin(x)


def main(args=None):
    rclpy.init(args=args)

    sin_motor_publisher = SinMotorPublisher()

    rclpy.spin(sin_motor_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sin_motor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()