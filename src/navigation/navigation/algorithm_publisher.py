from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import cv2 as cv
from cv_bridge import CvBridge
import sys
sys.path.append(".")

from src.helper_scripts.get_algorithm import get_algorithm

class AlgorithmPublisher(Node):

    def __init__(self, algorithm_name, debug=False, image_topic='/camera/color/image_raw'):
        super().__init__('algorithm_publisher')
        # if debug is true, we will use the mock camera publisher topic
        self.topic = 'navigation/mock_camera' if debug else image_topic
        self.encoding = 'bgr8'

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(String, f'navigation/{algorithm_name}', 10)
        self.br = CvBridge()
        self.debug = debug
        try:
            self.algorithm = get_algorithm(algorithm_name)
        except ValueError as err:
            sys.exit(err.args)
        print(self.algorithm, self.topic)

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, self.encoding)
        cv.imshow('input', current_frame)
        cv.waitKey(1)
        processed_image, intersection_point = self.algorithm.processFrame(current_frame, show=self.debug)
        msg = String()
        msg.data = str(intersection_point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing intersection point: {intersection_point}')
        cv.imshow('output', processed_image)
        cv.waitKey(1)
