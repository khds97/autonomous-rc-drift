#!/usr/bin/env python

import rospy
from driftmodel_msg.msg import pose_flag
import subprocess

import signal

def flag_cb(data):
    
    rospy.loginfo("Receiving flag")
    
    if data.flag == 1:
        subprocess.run(["python", "/home/ubuntu/ros_noetic_ws/src/driftmodel/src/dagger.py"])
    else:
        rospy.loginfo("Proper condition not met, not starting dagger.py.")

def SignalHandler_SIGINT(SignalNumber,Frame):
     print('SignalHandler of signal.SIGINT')

#register the signal with Signal handler
signal.signal(signal.SIGINT,SignalHandler_SIGINT)
    
def main():
    
    rospy.init_node('pose_flag')
    
    sub = rospy.Subscriber('/pose_flag', pose_flag, flag_cb, queue_size=1)

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

