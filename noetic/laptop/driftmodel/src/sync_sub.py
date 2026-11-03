#!/usr/bin/env python

import rospy
from driftmodel_msg.msg import sensors, ctrl_val
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import message_filters
import pandas as pd
import numpy as np
import sys

# Custom Gym Environment with ROS Integration
class DriftEnv:
    def __init__(self):

        self.flag_ = 0
        self.flag = 0

        self.flag_t = 0

        self.prev_px = 0
        self.prev_py = 0
        self.prev_pz = 0

        self.px = 0
        self.py = 0
        self.pz = 0

        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.qw = 0

        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.gx = 0
        self.gy = 0
        self.gz = 0

        self.rpm = 0
        self.tire_pwm = 0
        self.rl_speed = 0
        self.rr_speed = 0
        self.current = 0

        self.steer = 0
        self.throttle = 0

        #self.usb_receiver = USBReceiver()

        rospy.init_node('gym-ros-drift')

        self.path_sub = message_filters.Subscriber("/loop_fusion_node/odometry_rect", Odometry)
        #self.vel_sub = message_filters.Subscriber("/vins_estimator/odometry", Odometry)
        self.sensors_sub = message_filters.Subscriber("/sensors", sensors)
        self.imu_sub = message_filters.Subscriber("/stereo_inertial_publisher/imu", Imu)
        self.ctrl_val_sub = message_filters.Subscriber("/ctrl_val", ctrl_val)

        self.tss = message_filters.ApproximateTimeSynchronizer([self.path_sub, self.sensors_sub, self.imu_sub, self.ctrl_val_sub], queue_size=100, slop=0.1)
        self.tss.registerCallback(self.callback)

        self.columns = ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'rpm', 'tire_pwm', 'rl_speed', 'rr_speed', 'current', 'steer', 'throttle', 'flag']
        self.df = pd.DataFrame([], columns=self.columns)

    def callback(self, odometry, sensors, imu, ctrl_val):

        self.px = odometry.pose.pose.position.x
        self.py = odometry.pose.pose.position.y
        self.pz = odometry.pose.pose.position.z

        self.qx = odometry.pose.pose.orientation.x
        self.qy = odometry.pose.pose.orientation.y
        self.qz = odometry.pose.pose.orientation.z
        self.qw = odometry.pose.pose.orientation.w

        self.vx = odometry.twist.twist.linear.x
        self.vy = odometry.twist.twist.linear.y
        self.vz = odometry.twist.twist.linear.z
 
        self.ax = imu.linear_acceleration.x
        self.ay = imu.linear_acceleration.y
        self.az = imu.linear_acceleration.z

        self.gx = imu.angular_velocity.x
        self.gy = imu.angular_velocity.y
        self.gz = imu.angular_velocity.z

        self.tire_pwm = sensors.tire_pwm
        
        self.current = sensors.current
        self.rpm = sensors.rpm * 0.001
        self.tire_pwm = sensors.tire_pwm * 0.001
        self.rl_speed = sensors.rl_speed * 0.001
        self.rr_speed = sensors.rr_speed * 0.001
        
        self.steer = ctrl_val.steer
        self.throttle = ctrl_val.throttle
        self.flag = ctrl_val.flag

        temp = pd.DataFrame([[self.px, self.py, self.pz, self.qx, self.qy, self.qz, self.qw, self.vx, self.vy, self.vz, self.ax, self.ay, self.az, self.gx, self.gy, self.gz, self.rpm, self.tire_pwm, self.rl_speed, self.rr_speed, self.current, self.steer, self.throttle, self.flag]], columns=self.columns)
	
        self.df = pd.concat([self.df, temp])
	
def main(args):

    dcb = DriftEnv()
    
    while not rospy.is_shutdown():
                
    	rospy.sleep(0.033)
    	
    dcb.df.to_csv(f"./result.csv", index=None)
    
        
if __name__ == "__main__":

    main(sys.argv)              
        
