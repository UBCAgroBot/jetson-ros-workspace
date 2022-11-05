import rclpy

from .algorithm_publisher import AlgorithmPublisher


class MiniContoursPublisher(AlgorithmPublisher):

    def __init__(self):
        super().__init__('mini_contour')


def main(args=None):
    rclpy.init(args=args)
    mini_contours_publisher = MiniContoursPublisher()
    rclpy.spin(mini_contours_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mini_contours_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
