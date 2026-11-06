import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('driftmodel')

    # The odometry source (e.g. a ROS 2 port of VINS-Fusion, or the new camera pipeline) and the IMU / camera
    # drivers are started separately. They must publish nav_msgs/Odometry and sensor_msgs/Imu on the topics
    # given by the `odom_topic` and `imu_topic` parameters of DriftEnv (see driftmodel_env.py).
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ACMSTM32'),

        # operator input from the STM32 board -> /ctrl_val
        Node(
            package='driftmodel',
            executable='record.py',
            name='controller',
            output='screen',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
        ),

        # starts dagger.py when /pose_flag receives flag == true
        Node(
            package='driftmodel',
            executable='pose_flag.py',
            name='pose_flag',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(share, 'demo.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
