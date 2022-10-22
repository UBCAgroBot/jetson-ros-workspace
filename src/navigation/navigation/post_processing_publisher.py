import glob
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import sys

class PostProcessingPublisher(Node):

    def __init__(self):
        super().__init__('post_processing_publisher')
        self.publisher_ = self.create_publisher(String, 'navigation/post_processing', 10)
        self.topic_scanning = 'scanning'
        self.topic_mini_contours = 'mini_contours'
        self.topic_mini_contours_downwards = 'mini_contours_downwards'
        self.topic_center_row = 'center_row'
        self.topic_check_row_end = 'check_row_end'
        self.topic_hough = 'hough'
        self.counter = 0  # track which message we are sending
        self.angles = np.zeros(10)

        self.subscription_scanning = self.create_subscription(
            String,
            self.topic_scanning,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        self.subscription_mini_contours = self.create_subscription(
            String,
            self.topic_mini_contours,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)
        
        self.subscription_mini_contours_downwards = self.create_subscription(
            String,
            self.topic_mini_contours_downwards,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        self.subscription_hough = self.create_subscription(
            String,
            self.topic_hough,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        self.subscription_check_row_end = self.create_subscription(
            String,
            self.topic_check_row_end,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        self.subscription_center_row = self.create_subscription(
            String,
            self.topic_center_row,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)

        self.subscription_center_row                # prevent unused variable warnings
        self.subscription_check_row_end             
        self.subscription_hough                     
        self.subscription_mini_contours             
        self.subscription_mini_contours_downwards   
        self.subscription_scanning                  
        
        
    def listener_callback(self, data:String):
        msg = String()
        if data is not None:  # prevent empty strings
            self.angles[self.counter//10] = data
            msg.data = String(np.mean(self.angles))
        else:
            msg.data = None

        print(data, type(data)) # val and type checking

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing string {self.counter}')
        self.counter += 1
        

def main(args=None):
    rclpy.init(args=args)

    post_processing_publisher = PostProcessingPublisher()

    rclpy.spin(post_processing_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    post_processing_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
