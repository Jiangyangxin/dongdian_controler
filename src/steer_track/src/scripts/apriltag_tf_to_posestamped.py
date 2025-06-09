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
# import transforms3d as tfs
import cv2
# 将apriltag输出的tf消息转化为posestamped消息


#已修改apriltag_ros源码，此文件已作废
#已修改apriltag_ros源码，此文件已作废
#已修改apriltag_ros源码，此文件已作废
#已修改apriltag_ros源码，此文件已作废
#已修改apriltag_ros源码，此文件已作废
#已修改apriltag_ros源码，此文件已作废




# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped
from tf2_geometry_msgs import PoseStamped
# from geometry_msgs.msg import PointStamped
from scipy.optimize import minimize  


if __name__ == "__main__":
    # global A,B,R1,R2
    rospy.init_node("tf_to_pose_node", anonymous=False)
    
    buffer1 = tf2_ros.Buffer()
    listener1 = tf2_ros.TransformListener(buffer1)
    pose_target1=PoseStamped()
    pose_pub = rospy.Publisher('apriltag_tf_pose', PoseStamped, queue_size=1)


    while not rospy.is_shutdown():    
        try:
            # buffer1.waitForTransform("hikrobot_camera","apriltag",rospy.Time(0),rospy.Duration(3.0))
            tfs1 = buffer1.lookup_transform("hikrobot_camera","apriltag",rospy.Time(0),rospy.Duration(0.5))
            pose_target1.header.frame_id= "apriltag_tf_pose"
            pose_target1.pose.position.x=tfs1.transform.translation.x
            pose_target1.pose.position.y=tfs1.transform.translation.y
            pose_target1.pose.position.z=tfs1.transform.translation.z
            pose_target1.pose.orientation.w=tfs1.transform.rotation.w
            pose_target1.pose.orientation.x=tfs1.transform.rotation.x
            pose_target1.pose.orientation.y=tfs1.transform.rotation.y
            pose_target1.pose.orientation.z=tfs1.transform.rotation.z
            pose_target1.header.stamp=rospy.Time.now(); 
            pose_pub.publish(pose_target1)

    
        except Exception as e:
            rospy.logerr("异常:%s",e)
            exit(0)

    if(rospy.is_shutdown()):
        time.sleep(1.0)
        os._exit(0)  