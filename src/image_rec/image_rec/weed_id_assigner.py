import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agrobot_msgs.msg import BoundingBox2DArray, WeedBoundingBox
import math


class WeedIDAssigner(Node):

    def __init__(self):
        super().__init__('weed_id_assigner')
        self.declare_parameter('distance_threshold', 10.0)
        self.declare_parameter('area_ratio_threshold_lower', 0.95)
        self.declare_parameter('area_ratio_threshold_upper', 1.05)
        self.bounding_boxes = {}
        self.publisher_ = self.create_publisher(WeedBoundingBox, 'weed_bounding_boxes', 10)
        self.subscription = self.create_subscription(
            BoundingBox2DArray,
            'image_rec/frcnn_prediction',
            self.bounding_box_callback,
            10)
        # Create a timer that calls update_bounding_boxes every second
        self.timer = self.create_timer(1.0, self.update_bounding_boxes)

    def bounding_box_callback(self, msg):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        weed_bounding_boxes = []
        for box in msg.boxes:
            center_x = (box.center_x)
            center_y = (box.center_y)
            area = (box.size_x) * (box.size_y)
            obj_id = self.get_id_for_box((center_x, center_y), area)
            weed_bounding_box = WeedBoundingBox()
            weed_bounding_box.header = header
            weed_bounding_box.center_x = center_x
            weed_bounding_box.center_y = center_y
            weed_bounding_box.size_x = box.max_x - box.min_x
            weed_bounding_box.size_y = box.max_y - box.min_y
            weed_bounding_box.obj_id = obj_id
            weed_bounding_boxes.append(weed_bounding_box)
        self.publisher_.publish(weed_bounding_boxes)

    def get_id_for_box(self, center, area):
        for obj_id, (prev_center, prev_area, timestamp) in self.bounding_boxes.items():
            distance = math.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
            area_ratio = area / prev_area
            if distance < self.get_parameter('distance_threshold').value and area_ratio > self.get_parameter('area_ratio_threshold_lower').value and area_ratio < self.get_parameter('area_ratio_higher'):
                self.bounding_boxes[obj_id] = (center, area, self.get_clock().now())
                return obj_id
        new_id = max(self.bounding_boxes.keys(), default=-1) + 1
        self.bounding_boxes[new_id] = (center, area, self.get_clock().now())
        return new_id

    def update_bounding_boxes(self):
        now = self.get_clock().now()
        to_delete = []
        for obj_id, (_, _, timestamp) in self.bounding_boxes.items():
            time_diff = now - timestamp
            if time_diff.nanoseconds > 2000000000:  # two seconds
                to_delete.append(obj_id)
        for obj_id in to_delete:
            del self.bounding_boxes[obj_id]


def main(args=None):
    rclpy.init(args=args)
    node = WeedIDAssigner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
