import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ground_vio'), 'config', 'ground_vio.yaml')
    return LaunchDescription([
        Node(
            package='ground_vio',
            executable='vio_node',
            name='ground_vio',
            output='screen',
            parameters=[config],
        ),
    ])
