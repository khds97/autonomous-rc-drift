from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Only the vehicle sensor node is started here.
    # The camera / IMU driver of the redesigned hardware is started separately.
    return LaunchDescription([
        Node(
            package='driftmodel',
            executable='driftmodel_node',
            name='driftmodel_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': 34.0,
                'speed_window_s': 0.1,
                'motor_counts_per_rev': 6.0,
                'magnets_per_wheel': 8.0,
                'rpm_timeout_s': 0.5,
            }],
        ),
    ])
