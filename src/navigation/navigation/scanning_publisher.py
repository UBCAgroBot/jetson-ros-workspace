import rclpy
from .algorithm_publisher import AlgorithmPublisher


class ScanningPublisher(AlgorithmPublisher):

    def __init__(self):
        super().__init__('scanning')


def main(args=None):
    rclpy.init(args=args)
    scanning_publisher = ScanningPublisher()
    rclpy.spin(scanning_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanning_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
