import rclpy
from .algorithm_publisher import AlgorithmPublisher


class HoughPublisher(AlgorithmPublisher):

    def __init__(self, debug=False):
        super().__init__('hough', debug)


def main(args=None):
    rclpy.init(args=args)

    # set debug to true to use the mock camera publisher
    hough_publisher = HoughPublisher(debug=False)

    rclpy.spin(hough_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hough_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
