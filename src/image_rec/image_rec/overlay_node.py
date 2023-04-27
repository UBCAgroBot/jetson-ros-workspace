import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from agrobot_msgs.msg import BoundingBox2DArray, BoundingBoxMinMax
import numpy as np


class OverlayNode(Node):

    def __init__(self):
        super().__init__('overlay_node')
        self.get_logger().info("OverlayNode node has been started")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            BoundingBox2DArray, 'image_rec/frcnn_prediction', self.callback, 10)
        self.sub_img = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.overlay_color = (255, 0, 0) # red

        # Create a publisher for the bounding box coordinates
        self.pub = self.create_publisher(BoundingBoxMinMax, 'image_rec/bounding_box', 10)

    def callback(self, bbox_array):
        bbox_list = []
        for bbox in bbox_array.boxes:
            xmin = (bbox.center.x - bbox.size_x / 2)
            ymin = (bbox.center.y - bbox.size_y / 2)
            xmax = (bbox.center.x + bbox.size_x / 2)
            ymax = (bbox.center.y + bbox.size_y / 2)
            bbox_list.append([xmin,ymin,xmax,ymax])
            bbox_msg = BoundingBoxMinMax()
            bbox_msg.x_min = xmin
            bbox_msg.y_min = ymin
            bbox_msg.x_max = xmax
            bbox_msg.y_max = ymax
            self.pub.publish(bbox_msg)

        self.bbox_list = bbox_list

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Resize the frame to 512x512
        resized_frame = cv2.resize(frame, (512, 512))

        if hasattr(self, 'bbox_list'):
            # Overlay bounding boxes on the frame
            for bbox in self.bbox_list:
                xmin, ymin, xmax, ymax = bbox
                xmin = int(xmin)
                ymin = int(ymin)
                xmax = int(xmax)
                ymax = int(ymax)
                cv2.rectangle(resized_frame, (xmin, ymin), (xmax, ymax), self.overlay_color, 2)
                cv2.putText(resized_frame, "weed", (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.overlay_color, 2)

        # Show the overlay image
        cv2.imshow("Overlay Image", resized_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    overlay_node = OverlayNode()

    rclpy.spin(overlay_node)

    overlay_node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
