import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import Image

RESIZE_WIDTH = 512
RESIZE_HEIGHT = 512

SKIP_FRAMES = 10 #Only publish every SKIP_FRAMES frames


class PreProcessor(Node):

    def __init__(self):
        super().__init__('pre_processor')
        self.get_logger().info("PreProcessor node has been started")
        self.pub = self.create_publisher(Image, 'image_rec/pre_processed_image', 10)
        #Change to camera raw image topic
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        self.counter = 0

    def callback(self, data):
        if (self.counter % SKIP_FRAMES == 0):
            try:
                image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            cv2.imshow("feed", image)
            cv2.waitKey(1)

            resized_img = cv2.resize(image, (RESIZE_WIDTH, RESIZE_HEIGHT))
            resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img, "bgr8")

            #pass header information through
            resized_img_msg.header = data.header

            self.pub.publish(resized_img_msg)
        else:
            pass
        self.counter += 1

            
    




def main(args=None):
    rclpy.init(args=args)

    pre_processor = PreProcessor()

    rclpy.spin(pre_processor)

    pre_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()