import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from agrobot_msgs.msg import FrcnnBoundingBox, BoundingBoxMinMax
import numpy as np


class GuiOverlayNodeSave(Node):

    def __init__(self):
        super().__init__('gui_overlay_node_save')
        self.get_logger().info("GuiOverlayNodeSave node has been started")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            FrcnnBoundingBox, 'image_rec/frcnn_plant_prediction', self.callback, 10)
        self.sub_img = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.overlay_color1 = (255, 0, 0) # blue
        self.overlay_color2 = (0, 0, 255) # green

        # Create a publisher for the bounding box coordinates
        self.pub = self.create_publisher(BoundingBoxSave, 'image_rec/bounding_box', 10)

    def callback(self, bbox_array):
        bbox_list = []
        for bbox in bbox_array.boxes:
            xmin = (bbox.center_x - bbox.size_x / 2)
            ymin = (bbox.center_y - bbox.size_y / 2)
            xmax = (bbox.center_x + bbox.size_x / 2)
            ymax = (bbox.center_y + bbox.size_y / 2)
            confidence_value = bbox.confidence_val
            plant_id = bbox.plant_id
            bbox_list.append([xmin,ymin,xmax,ymax,confidence_value,plant_id])
            bbox_msg = BoundingBoxSave()
            bbox_msg.x_min = xmin
            bbox_msg.y_min = ymin
            bbox_msg.x_max = xmax
            bbox_msg.y_max = ymax
            bbox_msg.confidence_val = confidence_value
            bbox_msg.plant_id = plant_id
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
                xmin, ymin, xmax, ymax, confidence, plantid = bbox
                xmin = int(xmin)
                ymin = int(ymin)
                xmax = int(xmax)
                ymax = int(ymax)
                confidence = float(confidence)
                plantid = int(plantid)
                if (confidence >= 0.5):
                    cv2.rectangle(resized_frame, (xmin, ymin), (xmax, ymax), self.overlay_color, 2)
                if (plantid == 1):
                    cv2.putText(resized_frame, "maize", (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.overlay_color1, 2)
                if (plantid == 0):
                    cv2.putText(resized_frame, "bean", (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.overlay_color2, 2)

        # Show the overlay image
        cv2.imshow("Overlay Image", resized_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    overlay_node = GuiOverlayNodeSave()

    rclpy.spin(overlay_node)

    overlay_node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()