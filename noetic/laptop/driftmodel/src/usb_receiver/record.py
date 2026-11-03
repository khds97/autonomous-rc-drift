#! /usr/bin/env python

import rospy
from driftmodel_msg.msg import ctrl_val, sensors
from driftmodel.usb_receiver import USBReceiver

usb_receiver = USBReceiver()

global rpm

def rpm_callback(msg):

    rpm = msg.rpm

def main():
    
    pub = rospy.Publisher('ctrl_val', ctrl_val, queue_size=100)
    
    rospy.init_node('driftmodel_pc')
    
    rate = rospy.Rate(100)
    
    msg = ctrl_val()
    
    forward_flag = 1
    
    double_click_cnt = 0
    
    #rospy.Subscriber("chatter", String, rpm_callback)
    
    while not rospy.is_shutdown():
    
        usb_receiver.receiveData()
              
        if(usb_receiver.mode_flag()==0):
            
            msg.steer = usb_receiver.steer()
            
            if(usb_receiver.throttle() < 1.79):
                msg.forward_throttle =  usb_receiver.throttle()
                msg.brake_throttle =  0
                #print("Forward: ", msg.forward_throttle)
                
            else:
                msg.forward_throttle =  0
                msg.brake_throttle =  usb_receiver.throttle()
                #print("brake: ", msg.brake_throttle)            
            
                    
            '''elif((usb_receiver.throttle() > 1.815) and (forward_flag==1) and (double_click_cnt==0)):
                msg.forward_throttle =  0
                msg.brake_throttle =  usb_receiver.throttle()
                msg.backward_throttle = 0
                #forward_flag = 0
                double_click_cnt = 1
                print("brake: ", msg.brake_throttle)'''
                    
        if(usb_receiver.mode_flag()==1):
            
            msg.steer = 1.525
            msg.forward_throttle = 0
            msg.brake_throttle = 1.815     
            #usb_receiver.transmitData(2.0, 2.0)
        
        msg.mode_flag = usb_receiver.mode_flag()
        msg.punishment_flag = usb_receiver.punishment_flag()
            
        #rospy.loginfo(msg)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        pub.publish(msg)
              
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
