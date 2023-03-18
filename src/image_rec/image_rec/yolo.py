import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
from std_msgs.msg import Header


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo')
        self.get_logger().info("YoloNode node has been started")
        self.pub = self.create_publisher(Image, 'image_rec/yolo_prediction', 10)
        self.sub = self.create_subscription(Image, 'image_rec/pre_processed_image', self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #TODO: Run YOLO on image, get weed bounding boxes

        #convert weed bounding boxes to ROS message
        weed_preds = [[1,2,3,4], [5,6,7,8]]
        weed_preds_msg = bbox_list_to_msg(weed_preds)

        #update header information (propogate image header through)
        weed_preds_msg.header = data.header

        #publish
        self.pub.publish(weed_preds_msg)


def bbox_list_to_msg(bbox_list):
    """Converts list of bounding boxes to ROS message.
        Must be in [xcenter, ycenter, width, height] format"""
    bbox_arr_msg = BoundingBox2DArray()
    for bbox in bbox_list:
        bbox_msg = BoundingBox2D()
        bbox_msg.center.x = bbox[0]
        bbox_msg.center.y = bbox[1]
        bbox_msg.size_x = bbox[2]
        bbox_msg.size_y = bbox[3]

        bbox_arr_msg.bounding_boxes.append(bbox_msg)

    return bbox_arr_msg



def main(args=None):
    rclpy.init(args=args)

    yolo_node = YoloNode()

    rclpy.spin(yolo_node)

    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()