import rclpy

from .algorithm_publisher import AlgorithmPublisher


class MiniContoursDownwardsPublisher(AlgorithmPublisher):

    def __init__(self):
        super().__init__('mini_contours_downward')


def main(args=None):
    rclpy.init(args=args)
    mini_contours_downwards_publisher = MiniContoursDownwardsPublisher()
    rclpy.spin(mini_contours_downwards_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mini_contours_downwards_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
