import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    config = os.path.join(get_package_share_directory('navigation'), 'config', 'params.yml')

    return LaunchDescription([
        Node(
            package='navigation',
            executable='mock_camera_publisher',
            namespace='navigation',
            name='mock_camera_publisher',
            parameters=[config]
        ),
        Node(
            package='navigation',
            executable='algorithm_publisher',
            namespace='navigation',
            name='algorithm_publisher_seesaw',
            parameters=[config]
        ),
        Node(
            package='navigation',
            executable='algorithm_publisher',
            namespace='navigation',
            name='algorithm_publisher_center_downward',
            parameters=[config]
        ),
        Node(
            package='navigation',
            executable='post_processor_publisher',
            namespace='navigation',
            name='post_processor_publisher',
            parameters=[config]
        )
    ])
