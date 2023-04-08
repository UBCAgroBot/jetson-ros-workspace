import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import pyrealsense2 as rs2
import numpy as np
from cv_bridge import CvBridge

class RealsenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.bridge = CvBridge()

        # Create a publisher for the camera image
        self.image_pub = self.create_publisher(Image, '/camera/color/image', 10)

        # Configure the RealSense camera
        self.pipeline = rs2.pipeline()
        self.config = rs2.config()
        self.config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)
        self.pipeline.start(self.config)

    def publish_image(self):
        # Get the color image from the RealSense camera
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        # Create a header for the camera image
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera'

        # Create an Image message from the color image
        image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
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