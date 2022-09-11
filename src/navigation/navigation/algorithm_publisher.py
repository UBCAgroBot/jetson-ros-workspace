from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import sys
sys.path.append(".")

from src.helper_scripts.get_algorithm import get_algorithm

class AlgorithmPublisher(Node):

    def __init__(self, algorithm, debug=False):
        super().__init__('algorithm_publisher')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.debug = debug
        try:
            self.algorithm = get_algorithm(algorithm)
        except ValueError as err:
            sys.exit(err.args)

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        self.algorithm(current_frame, show=True)
        if self.debug:
            cv2.imshow('camera', current_frame)
            cv2.waitKey(1)
