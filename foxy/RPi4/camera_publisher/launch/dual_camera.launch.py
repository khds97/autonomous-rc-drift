from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_publisher',
            executable='dual_camera_publisher',
            name='dual_camera_publisher',
            output='screen',
            parameters=[{
                'fixed_device': '/dev/video0',
                'rotating_device': '/dev/video2',
                'width': 640,
                'height': 480,
                'fps': 30,
            }],
        ),
    ])
