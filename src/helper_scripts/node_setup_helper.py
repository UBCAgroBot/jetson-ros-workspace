from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


'''
Defines common functionality for Nodes
'''


def node_setup_helper(node: Node):
    node.declare_parameters(
        namespace='',
        parameters=[
            ('verbosity', 20, ParameterDescriptor(
                description='Verbosity log messages: 0 (UNSET), 10 (DEBUG), 30 (INFO), 40 (WARN), 50 (ERROR) and 60 (FATAL) in ascending order.' +
                'a logger will only process log messages with severity at or higher than a specified level chosen for the logger.')
             )
        ]
    )
    logger_level: int = node.get_parameter('verbosity').value
    node.get_logger().info(f'Setting logger verbosity to {str(logger_level)}')
    node.get_logger().set_level(logger_level)
