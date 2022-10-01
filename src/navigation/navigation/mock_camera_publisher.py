import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import sys
import cv2 as cv

class MockCameraPublisher(Node):

    def __init__(self):
        super().__init__('mock_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'mock_image_stream', 10)
        timer_period = 1 / 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.br = CvBridge()

        # we expect images/ to contain a bunch of images that we can use to mock a camera stream
        filenames = glob.glob(f"src/navigation/images/*.png")
        filenames.sort()
        
        print("Reading images")
        self.images = [cv.imread(img) for img in filenames]
        print("Read images")

        # exit if no images are found
        if len(self.images) == 0:
            print("No images found in ../images/")
            sys.exit(1)

        # create a pointer to keep track of which image we are on
        self.pointer = 0

    def timer_callback(self):
        msg = self.br.cv2_to_imgmsg(self.images[self.pointer], "bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing image {self.pointer}')
        self.pointer += 1
        
        if self.pointer >= len(self.images):
            self.pointer = 0
            print("Looping back to first image")


def main(args=None):
    rclpy.init(args=args)

    mock_camera_publisher = MockCameraPublisher()

    rclpy.spin(mock_camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mock_camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
