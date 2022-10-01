import rclpy
from .algorithm_publisher import AlgorithmPublisher


class CenterRowPublisher(AlgorithmPublisher):

    def __init__(self, debug=False):
        super().__init__('center_row', debug)


def main(args=None):
    rclpy.init(args=args)

    # set debug to true to use the mock camera publisher
    center_row_publisher = CenterRowPublisher(debug=False)

    rclpy.spin(center_row_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    center_row_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
