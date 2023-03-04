import sys

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.append(".")
from src.helper_scripts.get_algorithm import get_algorithm
from src.helper_scripts.node_setup_helper import node_setup_helper

sys.path.append("./Navigation/")
from Navigation.gui import startGUI

global appGUI

class AlgorithmPublisher(Node):

    def __init__(self, image_topic='/camera/color/image_raw'):
        super().__init__('algorithm_publisher')
        node_setup_helper(self)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mock', False, ParameterDescriptor(description='Sets image_topic to the mock camera topic')),
                ('show', True, ParameterDescriptor(description='Sets whether or not process_frame will show frames')),
                ('alg', '', ParameterDescriptor(description='Name of algorithm to use'))
            ]
        )
        self.is_mock_feed = self.get_parameter('mock').value
        self.show = self.get_parameter('show').value
        self.algorithm_name = self.get_parameter('alg').value

        if self.algorithm_name == '':
            raise ValueError("Algorithm is not defined. Please set the 'alg parameter")

        # if 'mock' command line argument is true, we will use the mock camera publisher topic
        self.topic = 'navigation/mock_camera' if self.is_mock_feed else image_topic
        self.encoding = 'bgr8'

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(String, f'navigation/{self.algorithm_name}', 10)
        self.br = CvBridge()
        try:
            self.algorithm = get_algorithm(self.algorithm_name)
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
        
        if appGUI.isActive():
            appGUI.update_dict({'standard': current_frame})
            self.algorithm.update_lower_hsv(appGUI.getLowerHSV())
            self.algorithm.update_upper_hsv(appGUI.getUpperHSV())
            processed, angle = self.algorithm.get_extra_content(
            current_frame, show=True)
            current_frame = appGUI.apply_filter(current_frame)
            appGUI.update_dict({'processed': processed})
            appGUI.update_fps(-1)
            appGUI.render_image()
        # processed_frame, angle = self.algorithm.process_frame(current_frame, show=self.show)

        msg = String()
        msg.data = str(angle)
        self.publisher.publish(msg)
        self.get_logger().info(f'angle: {angle}')
        self.display_frame('output', processed)


def main(args=None):
    rclpy.init(args=args)
    check_row_end_publisher = AlgorithmPublisher()
    rclpy.spin(check_row_end_publisher)

    global appGUI
    appGUI = startGUI('name', name1="standard", name2="processed")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    check_row_end_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
