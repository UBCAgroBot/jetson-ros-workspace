import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageOverlay(Node):

    def __init__(self):
        super().__init__('image_overlay')
        self.subscription_prediction = self.create_subscription(
            BoundingBox2DArray,
            'image_rec/frcnn_prediction',
            self.prediction_callback,
            10)
        self.subscription_image = self.create_subscription(
            Image,
            'image_rec/pre_processed_image',
            self.image_callback,
            10)
        self.cv_bridge = CvBridge()

    def prediction_callback(self, msg):
        self.bounding_boxes = msg.boxes

    def image_callback(self, msg):
        self.input_image = self.cv_bridge.imgmsg_to_cv2(msg)

        if hasattr(self, 'bounding_boxes'):
            overlay_image = self.input_image.copy()

            for bbox in self.bounding_boxes:
                x_min = int(bbox.x_offset)
                y_min = int(bbox.y_offset)
                x_max = int(bbox.x_offset + bbox.width)
                y_max = int(bbox.y_offset + bbox.height)
                cv2.rectangle(overlay_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(overlay_image, "weed", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            cv2.imshow('Overlay', overlay_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_overlay = ImageOverlay()

    rclpy.spin(image_overlay)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_overlay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
