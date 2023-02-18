import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import time
import sys
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

sys.path.append(".")
from src.helper_scripts.arduino_control import arduino_control
from src.helper_scripts.node_setup_helper import node_setup_helper


class PostProcessorPublisher(Node):

    def __init__(self):
        super().__init__('post_processing_publisher')
        node_setup_helper(self)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '', ParameterDescriptor(description='Port name for Arduino')),
            ]
        )
        self.ARDUINO_PORT = self.get_parameter('port').value
        if self.ARDUINO_PORT == '':
            raise ValueError("Port is not defined. Please set the 'port' parameter")

        self.SEND_TIME_CONSTANT = 0.5
        self.LEFT_THRESHOLD = -10
        self.RIGHT_THRESHOLD = 10

        self.arduino_controller = arduino_control(port=self.ARDUINO_PORT)
        self.last_send_time = time.time()

        self.topic_scanning = 'navigation/scanning'
        self.topic_mini_contour = 'navigation/mini_contour'
        self.topic_mini_contour_downward = 'navigation/mini_contour_downward'
        self.topic_center_row = 'navigation/center_row'
        self.topic_check_row_end = 'navigation/check_row_end'
        self.topic_hough = 'navigation/hough'
        self.topic_seesaw = 'navigation/seesaw'
        self.topic_center_down = 'navigation/center_down'
        self.counter = 0  # track which message we are sending
        self.array_size = 10
        self.angles = np.zeros(self.array_size)

        self.subscription_scanning = self.create_subscription(
            String,
            self.topic_scanning,
            self.listener_callback,
            qos_profile_sensor_data)

        self.subscription_mini_contour = self.create_subscription(
            String,
            self.topic_mini_contour,
            self.listener_callback,
            qos_profile_sensor_data)

        self.subscription_hough = self.create_subscription(
            String,
            self.topic_hough,
            self.listener_callback,
            qos_profile_sensor_data)

        self.subscription_center_row = self.create_subscription(
            String,
            self.topic_center_row,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        self.subscription_Seesaw = self.create_subscription(
            String,
            self.topic_seesaw,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)
        
        self.subscription_center_down = self.create_subscription(
            String, 
            self.topic_center_down,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        # self.subscription_mini_contour_downward = self.create_subscription(
        #     String,
        #     self.topic_mini_contour_downward,
        #     self.listener_callback,
        #     qos_profile_sensor_data)

        # self.subscription_check_row_end = self.create_subscription(
        #     String,
        #     self.topic_check_row_end,
        #     self.listener_callback,
        #     qos_profile_sensor_data)

    def listener_callback(self, in_msg: String):
        # Used in debugging angles stored for avg calculation
        # print(self.angles)
        out_msg = String()
        if in_msg.data != 'None':
            self.angles[self.counter % self.array_size] = float(in_msg.data)
            self.counter += 1
        out_angle = np.mean(self.angles)

        current_time = time.time()
        if (current_time > self.last_send_time + self.SEND_TIME_CONSTANT):
            self.last_send_time = time.time()
            turn = ''
            if out_angle < self.LEFT_THRESHOLD:
                turn = 'R'
            elif out_angle > self.RIGHT_THRESHOLD:
                turn = 'L'
            else:
                turn = 'S'
            self.get_logger().info(f'direction: (F, {turn}), angle: {out_angle}')
            self.arduino_controller.send(move="F", turn=turn)


def main(args=None):
    rclpy.init(args=args)
    post_processor_publisher = PostProcessorPublisher()
    rclpy.spin(post_processor_publisher)
    post_processor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
