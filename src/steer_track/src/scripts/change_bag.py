#!/usr/bin/env python3
# coding:utf-8
import numpy as np  
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import os
import rospy
import tf2_ros
import tf2_geometry_msgs
import transforms3d as tfs
import cv2
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from geometry_msgs.msg import Pose, PoseStamped,PointStamped
# from geometry_msgs.msg import PointStamped
from scipy.optimize import minimize  
from nav_msgs.msg import Path

final_pose=PoseStamped()
VIO_pose= PoseStamped()
i=0
def pose_callback(data):
    # 这个函数会在接收到消息时被调用
    # 你可以在这里处理接收到的Pose数据
    # final_pose.header.stamp=rospy.Time.now()
    final_pose.header.frame_id="final_body"
    final_pose.pose.position.x=data.position.x
    final_pose.pose.position.y=data.position.x
    final_pose.pose.position.z=data.position.z
    final_pose.pose.orientation.w=data.orientation.w
    final_pose.pose.orientation.x=data.orientation.x
    final_pose.pose.orientation.y=data.orientation.y
    final_pose.pose.orientation.z=data.orientation.z
    # pub.publish(final_pose)
    
def pose2_callback(data):  
    final_pose.header.stamp=data.header.stamp
    pub.publish(final_pose)  

def path_callback(data):
    global i
    VIO_pose.header.frame_id="VIO_pose"
    VIO_pose.header.stamp =data.poses[-1].header.stamp
    VIO_pose.pose=data.poses[-1].pose
    pub.publish(VIO_pose) 
    i=i+1
    print(i)
    # if len(data.poses)>2000:
    #     while i<len(data.poses):
    #         VIO_pose.header.stamp =data.poses[i].header.stamp
    #         VIO_pose.pose=data.poses[i].pose
    #         pub.publish(VIO_pose) 
    #         i+=1
    #         print(i)

        
if __name__ == "__main__":
    # global A,B,R1,R2
    rospy.init_node("change_bag_node", anonymous=False)
    # pub = rospy.Publisher('/final_body', PoseStamped, queue_size=10)
    # rospy.Subscriber('/final_pose', Pose, pose_callback)
    # rospy.Subscriber('/vrpn_client_node/steer_tk/pose', PoseStamped, pose2_callback)
    
    pub = rospy.Publisher('VIO_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('vins_estimator/path', Path, path_callback)
    rospy.spin()
         