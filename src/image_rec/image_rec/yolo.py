import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from image_rec.msg import BoundingBox2D, BoundingBox2DArray
from std_msgs.msg import Header
import torch
import onnxruntime as ort
import numpy as np

RESIZE_TO = 512


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo')
        self.get_logger().info("YoloNode node has been started")
        self.pub = self.create_publisher(BoundingBox2DArray, 'image_rec/yolo_prediction', 10)
        self.sub = self.create_subscription(Image, 'image_rec/pre_processed_image', self.callback, 10)
        self.bridge = CvBridge()

        self.ort_sess = ort.InferenceSession('model.onnx')
        self.input_name = self.ort_sess.get_inputs()[0].name
        self.output_name = self.ort_sess.get_outputs()[0].name

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        image = process_image(image)

        #TODO: Run YOLO on image, get weed bounding boxes
        output = self.ort_sess.run([self.output_name], {self.input_name: image})[0]

        #convert weed bounding boxes to ROS message
        weed_preds = [[1,2,3,4], [5,6,7,8]]
        weed_preds_msg = bbox_list_to_msg(weed_preds)

        #update header information (propogate image header through)
        weed_preds_msg.header = data.header

        #publish
        self.pub.publish(weed_preds_msg)


def process_image(image):
    #DOUBLE CHECK THIS FUNCTION
    """Pre-processes image for YOLO"""
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (RESIZE_TO, RESIZE_TO))
    image = image.transpose(2, 0, 1).astype(np.float32)
    image = image / 255.0
    return image

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

    yolo_node = YoloNode()

    rclpy.spin(yolo_node)

    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()