import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agrobot_msgs.msg import BoundingBox2DArray
from vision_msgs.msg import BoundingBox2D
import math
import random

class DummyGenerator(Node):

    def __init__(self):
        super().__init__('dummy_generator')
        self.get_logger().info("Dummy Generator")
        self.publisher_ = self.create_publisher(BoundingBox2DArray, 'image_rec/dummy_generator', 10)
        self.timer = self.create_timer(2.0, self.send_bounding_box)


    def send_bounding_box(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        
        bbox_arr_msg = BoundingBox2DArray()
        xpos = float(random.randint(0,512))
        self.get_logger().info("detected bbox xpos = "+str(xpos))
        bbox_msg = BoundingBox2D()
        bbox_msg.center.x = xpos
        bbox_msg.center.y = 200.0
        bbox_msg.size_x = 10.0
        bbox_msg.size_y = 10.0

        bbox_arr_msg.boxes.append(bbox_msg)

        # weed_bounding_box = WeedBoundingBox()
        # weed_bounding_box.header = header
        # weed_bounding_box.center_x = 200.0
        # weed_bounding_box.center_y = 200.0
        # weed_bounding_box.size_x = 10.0
        # weed_bounding_box.size_y =10.0
        # weed_bounding_box.obj_id = 9999
        self.publisher_.publish(bbox_arr_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

