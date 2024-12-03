/*
 * @Author: lxs
 * @Date: 2022-11-23 11:26:37
 * @LastEditTime: 2022-12-07 07:55:38
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /new_steer/src/steer_track/src/pose_vel_pub_node.cpp
 * Copyright (c) 2021 LXScience&Technology. All rights reserved.
 */
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "steer_track/get_pose_vel.h"

GetPoseVel *pose_vel;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_vel_pub_node");     //初始化ROS节点
    // if (argc != 2)
    // {
    //     ROS_INFO("Please set climbot_name");
    //     return 1;
    // }
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);
    float* pose;
    float* vel;
    
    //pose_vel = new GetPoseVel(nh, "test", 40);//nh_(nh), climbot_name_(climbot_name), frequency_(frequency)
    pose_vel = new GetPoseVel(nh, argv[1], 40);//其他内容包含在get_pose_vel.cpp 中,argv[1]为optitrack的刚体名字
    ros::spin();
    return 0;
}
