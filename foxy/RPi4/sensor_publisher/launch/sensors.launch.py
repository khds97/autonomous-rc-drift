from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_publisher',
            executable='dual_camera_publisher',
            name='dual_camera_publisher',
            output='screen',
            parameters=[{
                'fixed_device': '/dev/video0',
                'rotating_device': '/dev/video2',
                'width': 640,
                'height': 480,
                'fps': 30,
                # Constant stamp shift in seconds, for a measured camera-to-IMU offset
                'fixed_time_offset_s': 0.0,
                'rotating_time_offset_s': 0.0,
            }],
        ),
        Node(
            package='sensor_publisher',
            executable='imu_node',
            name='ism330dhcx_publisher',
            output='screen',
            parameters=[{
                'i2c_device': '/dev/i2c-0',
                'frame_id': 'imu_link',
                'gpio_chip': 'gpiochip0',
                'gpio_line': 23,
            }],
        ),
    ])
