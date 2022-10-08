import rclpy
from .algorithm_publisher import AlgorithmPublisher


class ScanningPublisher(AlgorithmPublisher):

    def __init__(self, debug=False):
        super().__init__('scanning', debug)


def main(args=None):
    rclpy.init(args=args)

    # set debug to true to use the mock camera publisher
    scanning_publisher = ScanningPublisher(debug=False)

    rclpy.spin(scanning_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanning_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
