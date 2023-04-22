import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray


class ImgPost(Node):

    def __init__(self):
        super().__init__('img_post')
        self.subscription = self.create_subscription(
            BoundingBox2DArray,
            'image_rec/frcnn_prediction',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(str(msg.boxes))


def main(args=None):
    rclpy.init(args=args)

    img_post = ImgPost()

    rclpy.spin(img_post)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    img_post.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()