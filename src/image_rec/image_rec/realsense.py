import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Header

class RealsenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.bridge = CvBridge()

        # Create a publisher for the camera image
        self.image_pub = self.create_publisher(Image, '/camera/color/image', 10)

        # Initialize the OpenCV video capture object for the camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def publish_image(self):
        # Capture a frame from the camera
        ret, frame = self.cap.read()
        if not ret:
            return

        # Create a header for the camera image
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera'

        # Create an Image message from the frame
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header = header

        # Publish the Image message
        self.image_pub.publish(image_msg)

    def main_loop(self):
        while rclpy.ok():
            self.publish_image()

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseNode()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()