from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
import cv2 as cv
from cv_bridge import CvBridge
import sys
sys.path.append(".")

from src.helper_scripts.get_algorithm import get_algorithm

class AlgorithmPublisher(Node):

    def __init__(self, algorithm_name, image_topic='/camera/color/image_raw'):
        super().__init__('algorithm_publisher')
        self.declare_parameter('mock', False, ParameterDescriptor(description='Sets image_topic to the mock camera topic'))
        self.is_mock_feed = self.get_parameter('mock').value
        self.declare_parameter('show', True, ParameterDescriptor(description='Sets whether or not process_frame will show frames'))
        self.show = self.get_parameter('show').value

        # if 'mock' command line argument is true, we will use the mock camera publisher topic
        self.topic = 'navigation/mock_camera' if self.is_mock_feed else image_topic
        self.encoding = 'bgr8'

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(String, f'navigation/{algorithm_name}', 10)
        self.br = CvBridge()
        try:
            self.algorithm = get_algorithm(algorithm_name)
        except ValueError as err:
            sys.exit(err.args)
        print(self.algorithm, self.topic)

    def display_frame(self, win_name, frame):
        if self.show:
            cv.imshow(win_name, frame)
            cv.waitKey(1)

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, self.encoding)
        self.display_frame('input', current_frame)
        processed_frame, angle = self.algorithm.process_frame(current_frame, show=self.show)
        msg = String()
        msg.data = str(angle)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing angle: {angle}')
        self.display_frame('output', processed_frame)
