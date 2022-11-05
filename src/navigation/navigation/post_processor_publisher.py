import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class PostProcessorPublisher(Node):

    def __init__(self):
        super().__init__('post_processing_publisher')
        self.publisher_ = self.create_publisher(String, 'navigation/post_processing', 10)
        self.topic_scanning = 'navigation/scanning'
        self.topic_mini_contour = 'navigation/mini_contour'
        self.topic_mini_contour_downward = 'navigation/mini_contour_downward'
        self.topic_center_row = 'navigation/center_row'
        self.topic_check_row_end = 'navigation/check_row_end'
        self.topic_hough = 'navigation/hough'
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

        # self.subscription_mini_contour_downward = self.create_subscription(
        #     String,
        #     self.topic_mini_contour_downward,
        #     self.listener_callback,
        #     qos_profile_sensor_data)

        self.subscription_hough = self.create_subscription(
            String,
            self.topic_hough,
            self.listener_callback,
            qos_profile_sensor_data)

        # self.subscription_check_row_end = self.create_subscription(
        #     String,
        #     self.topic_check_row_end,
        #     self.listener_callback,
        #     qos_profile_sensor_data)

        self.subscription_center_row = self.create_subscription(
            String,
            self.topic_center_row,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        # prevent unused variable warnings
        self.subscription_center_row
        # self.subscription_check_row_end             
        self.subscription_hough
        self.subscription_mini_contour
        # self.subscription_mini_contour_downward
        self.subscription_scanning

    def listener_callback(self, in_msg: String):
        # print(self.angles)
        out_msg = String()
        if in_msg.data != 'None':
            self.angles[self.counter % self.array_size] = float(in_msg.data)
            self.counter += 1
        out_angle = f'{np.mean(self.angles):.2f}'
        # TODO: Chihan will change use out_angle and change out_msg.data with his output
        out_msg.data = out_angle
        self.publisher_.publish(out_msg)
        self.get_logger().info('Subscribed angle: {:10s} Published angle: {:10s}'.format(str(in_msg.data), out_angle))


def main(args=None):
    rclpy.init(args=args)

    post_processor_publisher = PostProcessorPublisher()

    rclpy.spin(post_processor_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    post_processor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
