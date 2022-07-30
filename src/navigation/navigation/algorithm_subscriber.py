import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class AlgorithmSubscriber(Node):

    def __init__(self):
        super().__init__('algorithm_subscriber')
        self.subscription = self.create_subscription(
            String,
            'camera_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Camera data: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    algorithm_subscriber = AlgorithmSubscriber()

    rclpy.spin(algorithm_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    algorithm_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()