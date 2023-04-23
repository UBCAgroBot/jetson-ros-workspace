import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2DArray
from cv_bridge import CvBridge
import cv2

class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')
        self.get_logger().info('visualization_node has been started')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'image_rec/visualization_node', 10)
        self.sub = self.create_subscription(
            Image, 'image_rec/pre_processed_image', self.image_callback, 10)
        self.pred_sub = self.create_subscription(
            BoundingBox2DArray, 'image_rec/frcnn_prediction', self.prediction_callback, 10)

        self.image = None
        self.prediction = None

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            print(e)

    def prediction_callback(self, msg):
        self.prediction = msg

    def run(self):
        while rclpy.ok():
            if self.image is not None and self.prediction is not None:
                image = self.image.copy()
                for bbox in self.prediction.boxes:
                    x1 = int(bbox.center.x - bbox.size_x / 2)
                    y1 = int(bbox.center.y - bbox.size_y / 2)
                    x2 = int(bbox.center.x + bbox.size_x / 2)
                    y2 = int(bbox.center.y + bbox.size_y / 2)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.imshow('FRCNN Predictions', image)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
