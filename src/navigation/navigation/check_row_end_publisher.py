import rclpy

from .algorithm_publisher import AlgorithmPublisher


class CheckRowEndPublisher(AlgorithmPublisher):

    def __init__(self):
        super().__init__('check_row_end')


def main(args=None):
    rclpy.init(args=args)
    check_row_end_publisher = CheckRowEndPublisher()
    rclpy.spin(check_row_end_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    check_row_end_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
