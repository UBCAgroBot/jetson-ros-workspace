import glob
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import sys

class PostProcessingPublisher(Node):

    def __init__(self, topic:String):
        super().__init__('post_processing_publisher')
        self.publisher_ = self.create_publisher(String, 'navigation/post_processing', 10)
        self.topic = topic
        self.counter = 0  # track which message we are sending
        self.xvalues = np.zeros(10)
        self.yvalues = np.zeros(10)

        self.subscription = self.create_subscription(
            String,
            self.topic,
            self.listener_callback,  # instead of callback, look for wait to get information
            qos_profile_sensor_data)
        
        self.subscrifption  # prevent unused variable warning
        


    def listener_callback(self, data:String):
        msg = Float32MultiArray()
        if data is not None:  # prevent empty strings
            self.xvalues[self.counter//10] = data[0]
            self.yvalues[self.counter//10] = data[1]
            msg.data = [np.mean(self.xvalues), np.mean(self.yvalues)]
        else:
            msg.data = [-1, -1]

        print(data, type(data))

        self.publisher_.publish(String(msg))
        self.get_logger().info(f'Publishing string {self.pointer}')
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
