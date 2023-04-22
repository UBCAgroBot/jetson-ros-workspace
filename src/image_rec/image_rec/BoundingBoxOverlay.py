import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
from std_msgs.msg import Header
import numpy as np

NUM_CLASSES = 4
RESIZE_TO = 512


class BoundingBoxOverlay(Node):

    def __init__(self):
        super().__init__('bounding_box_overlay')
        self.get_logger().info("Bounding box overlay node has been started")
        self.pub = self.create_publisher(Image, 'image_rec/bounding_box_overlay', 10)
        self.sub = self.create_subscription(
            Image, 'image_rec/frcnn_prediction', self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Get bounding box information from the message
        boxes = []
        for box in data.bounding_boxes.boxes:
            xmin = int(box.center.x - box.size_x / 2)
            ymin = int(box.center.y - box.size_y / 2)
            xmax = int(box.center.x + box.size_x / 2)
            ymax = int(box.center.y + box.size_y / 2)
            boxes.append([xmin, ymin, xmax, ymax])

        # Draw bounding boxes on the image
        for box in boxes:
            cv2.rectangle(cv_image, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)

        # Publish the overlaid image
        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    node = BoundingBoxOverlayNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
