#!/usr/bin/env python3
"""Logs the synchronized odometry / sensors / IMU / operator input to a CSV file (written when the node stops).

Parameters:
  output          CSV file name (default result.csv)
  odom_topic, imu_topic, sensors_topic, ctrl_val_topic   same as DriftEnv
"""

import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from driftmodel_msg.msg import Sensors, CtrlVal

from preprocess import FEATURE_NAMES

COLUMNS = ['stamp'] + FEATURE_NAMES + ['steer', 'throttle', 'mode_flag', 'punishment_flag']


class SyncLogger(Node):

    def __init__(self):
        super().__init__('sync_sub')

        self.declare_parameter('output', 'result.csv')
        self.declare_parameter('odom_topic', '/loop_fusion_node/odometry_rect')
        self.declare_parameter('imu_topic', '/stereo_inertial_publisher/imu')
        self.declare_parameter('sensors_topic', 'sensors')
        self.declare_parameter('ctrl_val_topic', 'ctrl_val')

        self.rows = []

        subs = [
            message_filters.Subscriber(self, Odometry, self.get_parameter('odom_topic').value, qos_profile_sensor_data),
            message_filters.Subscriber(self, Sensors, self.get_parameter('sensors_topic').value, qos_profile_sensor_data),
            message_filters.Subscriber(self, Imu, self.get_parameter('imu_topic').value, qos_profile_sensor_data),
            message_filters.Subscriber(self, CtrlVal, self.get_parameter('ctrl_val_topic').value, qos_profile_sensor_data),
        ]
        self.tss = message_filters.ApproximateTimeSynchronizer(subs, 100, 0.1)
        self.tss.registerCallback(self.callback)

    def callback(self, odometry, sensors, imu, ctrl_val):
        p = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        v = odometry.twist.twist.linear
        values = {
            'px': p.x, 'py': p.y, 'pz': p.z,
            'qw': q.w, 'qx': q.x, 'qy': q.y, 'qz': q.z,
            'vx': v.x, 'vy': v.y, 'vz': v.z,
            'ax': imu.linear_acceleration.x, 'ay': imu.linear_acceleration.y, 'az': imu.linear_acceleration.z,
            'gx': imu.angular_velocity.x, 'gy': imu.angular_velocity.y, 'gz': imu.angular_velocity.z,
            'rpm': sensors.rpm, 'tire_pwm': sensors.tire_pwm,
            'rl_speed': sensors.rl_speed, 'rr_speed': sensors.rr_speed, 'current': sensors.current,
        }
        stamp = odometry.header.stamp.sec + odometry.header.stamp.nanosec * 1e-9
        # rows are collected in a list (appending to a DataFrame at every message gets slower and slower)
        self.rows.append([stamp] + [values[n] for n in FEATURE_NAMES] +
                         [ctrl_val.steer, ctrl_val.throttle, int(ctrl_val.mode_flag), int(ctrl_val.punishment_flag)])

    def write(self):
        path = self.get_parameter('output').value
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            writer.writerows(self.rows)
        self.get_logger().info('wrote {} rows to {}'.format(len(self.rows), path))


def main(args=None):
    rclpy.init(args=args)
    node = SyncLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
