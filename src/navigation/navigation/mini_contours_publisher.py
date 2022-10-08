import rclpy
from .algorithm_publisher import AlgorithmPublisher


class MiniContoursPublisher(AlgorithmPublisher):

    def __init__(self, debug=False):
        super().__init__('mini_contours', debug)


def main(args=None):
    rclpy.init(args=args)

    # set debug to true to use the mock camera publisher
    mini_contours_publisher = MiniContoursPublisher(debug=False)

    rclpy.spin(mini_contours_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mini_contours_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
