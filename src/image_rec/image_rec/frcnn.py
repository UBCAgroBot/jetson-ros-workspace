import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
from std_msgs.msg import Header


class FrcnnNode(Node):

    def __init__(self):
        super().__init__('frcnn')
        self.get_logger().info("FrcnnNode node has been started")
        self.pub = self.create_publisher(BoundingBox2DArray, 'image_rec/frcnn_prediction', 10)
        self.sub = self.create_subscription(Image, 'image_rec/pre_processed_image', self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #TODO: Run FRCNN on image, get weed bounding boxes

        #convert weed bounding boxes to ROS message
        weed_preds = [[1,2,3,4], [5,6,7,8]]
        weed_preds = [convert_edgebox_to_cbox(bbox) for bbox in weed_preds]
        weed_preds_msg = bbox_list_to_msg(weed_preds)

        #update header information (propogate image header through)
        weed_preds_msg.header = data.header

        #publish
        self.pub.publish(weed_preds_msg)


def convert_edgebox_to_cbox(edgebox):
    """Converts bounding box from [xmin, ymin, xmax, ymax] format 
        to [xcenter, ycenter, width, height]"""
    cx = edgebox[0] + edgebox[2]/2
    cy = edgebox[1] + edgebox[3]/2
    width = edgebox[2]-edgebox[0]
    height = edgebox[3]-edgebox[1]
    return [cx, cy, width, height]

def bbox_list_to_msg(bbox_list):
    """Converts list of bounding boxes to ROS message.
        Must be in [xcenter, ycenter, width, height] format"""
    bbox_arr_msg = BoundingBox2DArray()
    for bbox in bbox_list:
        bbox_msg = BoundingBox2D()
        bbox_msg.center.x = bbox[0]
        bbox_msg.center.y = bbox[1]
        bbox_msg.size_x = float(bbox[2])
        bbox_msg.size_y = float(bbox[3])

        bbox_arr_msg.boxes.append(bbox_msg)

    return bbox_arr_msg



def main(args=None):
    rclpy.init(args=args)

    frcnn_node = FrcnnNode()

    rclpy.spin(frcnn_node)

    frcnn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()