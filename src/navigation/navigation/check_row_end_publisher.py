import rclpy
from .algorithm_publisher import AlgorithmPublisher


class CheckRowEndPublisher(AlgorithmPublisher):

    def __init__(self, debug=False):
        super().__init__('check_row_end', debug)


def main(args=None):
    rclpy.init(args=args)

    # set debug to true to use the mock camera publisher
    check_row_end_publisher = CheckRowEndPublisher(debug=False)

    rclpy.spin(check_row_end_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    check_row_end_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
