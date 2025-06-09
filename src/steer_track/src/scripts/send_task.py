#!/usr/bin/env python
# coding:utf-8
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from bottom_control import BottomControl
from packInfo import packInfo
import numpy as np
import time
import os

if __name__ == "__main__":

    rospy.init_node("send_task")

    fan_input_pub = rospy.Publisher('fan_input', Float32MultiArray,queue_size=1)
    velocity_input_pub = rospy.Publisher('velocity_input', Twist,queue_size=1)
    
    fan_input = Float32MultiArray()  
    fan_input.data=[0.0,0.0,0.0]
    velocity_input = Twist()
    velocity_input.angular.x= 0
    velocity_input.angular.y= 0
    velocity_input.angular.z= 0
    velocity_input.linear.x= 0
    velocity_input.linear.y= 0 
    velocity_input.linear.z= 0      

    init=1
        

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        fan_input_pub.publish(fan_input)
        velocity_input_pub.publish(velocity_input)
         
        rate.sleep()
