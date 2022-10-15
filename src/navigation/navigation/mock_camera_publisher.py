import glob
import sys

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image

class MockCameraPublisher(Node):

    def __init__(self):
        super().__init__('mock_camera_publisher')
        self.declare_parameter('video', '', ParameterDescriptor(description='Sets the video feed to be used'))
        self.video = self.get_parameter('video').value

        self.publisher_ = self.create_publisher(Image, 'navigation/mock_camera', 10)
        # create a pointer to keep track of which image we are on
        self.pointer = 0
        self.timer_period = 1 / 30  # seconds
        self.br = CvBridge()

        if self.video != '':
            self.video_stream = cv.VideoCapture(f'src/navigation/videos/{self.video}.mp4')
            self.timer = self.create_timer(self.timer_period, self.video_callback)
        else:
            self.timer = self.create_timer(self.timer_period, self.image_callback)

            # we expect images/ to contain a bunch of images that we can use to mock a camera stream
            self.filenames = glob.glob(f'src/navigation/images/*.png')
            self.filenames.sort()

            # exit if no images are found
            if len(self.filenames) == 0:
                print('No images found in ../images/')
                sys.exit(1)

    def video_callback(self):
        if self.video_stream.isOpened():
            ret, frame = self.video_stream.read()
            self.pointer += 1
            # If the last frame is reached, reset the capture and the frame_counter
            if self.pointer == self.video_stream.get(cv.CAP_PROP_FRAME_COUNT):
                self.pointer = 0
                self.video_stream.set(cv.CAP_PROP_POS_FRAMES, 0)
                print('Looping back to first image')
            self.publish_image(frame)

    def image_callback(self):
        img = cv.imread(self.filenames[self.pointer])
        self.pointer += 1
        if self.pointer >= len(self.filenames):
            self.pointer = 0
            print('Looping back to first image')
        self.publish_image(img)

    def publish_image(self, img):
        msg = self.br.cv2_to_imgmsg(img, "bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing image {self.pointer}')


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
