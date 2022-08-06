import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import math
import serial

class SinMotorPublisher(Node):

    def __init__(self):
        super().__init__('sin_motor_publisher')
        self.publisher_ = self.create_publisher(String, 'motor_topic', 10)
        self.PERIOD = 0.5  # seconds
        self.MAX_ANGLE = 45
        self.STEP_SIZE = math.pi/10
        self.timer = self.create_timer(self.PERIOD, self.timer_callback)
        self.arduino = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=0.1)
        self.x = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.sin(self.x):.2f}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'{self.sin(self.x):.2f}')
        self.x += self.STEP_SIZE

        self.arduino.write(bytes(f'{self.sin(self.x):.2f}', 'utf-8'))
        time.sleep(0.05)
        data = self.arduino.readline()
        print(data)

    def sin(self, x):
        return self.MAX_ANGLE * math.sin(x)


def main(args=None):
    rclpy.init(args=args)

    sin_motor_publisher = SinMotorPublisher()

    rclpy.spin(sin_motor_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sin_motor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()