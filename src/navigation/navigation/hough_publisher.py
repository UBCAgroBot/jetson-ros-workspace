import rclpy

from .algorithm_publisher import AlgorithmPublisher


class HoughPublisher(AlgorithmPublisher):

    def __init__(self):
        super().__init__('hough')


def main(args=None):
    rclpy.init(args=args)
    hough_publisher = HoughPublisher()
    rclpy.spin(hough_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hough_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
