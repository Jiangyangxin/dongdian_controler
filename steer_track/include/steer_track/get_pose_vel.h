/*
 * @Author: lxs
 * @Date: 2022-11-23 11:28:14
 * @LastEditTime: 2022-12-06 07:31:27
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /new_steer/src/steer_track/include/steer_track/get_pose_vel.h
 * Copyright (c) 2021 LXScience&Technology. All rights reserved.
 */
#ifndef CLIMBOT_VEL_CONTROL_GET_POSE_VEL
#define CLIMBOT_VEL_CONTROL_GET_POSE_VEL

#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <vector>
#include "math.h"


class GetPoseVel
{
private:
    ros::NodeHandle nh_;
    std::string climbot_name_;      //机器人名称（Optitrack定义名称）
    std::vector<float> pose_;       //机器人位姿
    std::vector<float> vel_;    //机器人速度
    std::vector<float> filter_vel_; //滤波后速度
    float pose_2d_[3];  //平面投影位姿
    float vel_2d_[3];   //平面投影速度
    std::vector<std::vector<float>> pose_queue_;    //位姿序列
    std::vector<std::vector<float>> vel_queue_;     //速度序列
    ros::Subscriber pose_sub_ ;     //机器人位姿订阅器
    ros::Publisher pose_vel_pub_;
    float frequency_;             //机器人位姿采样频率
    const float fir_a_[26] = {
  -0.0006956761794133,-0.002120592795595,-0.004378002076251,-0.006705587509768,
    -0.0073969593179,-0.003939494651136, 0.006321671791697,   0.0251365935248,
    0.05228012542714,  0.08494130810849,   0.1179562851727,   0.1449676030656,
     0.1601932777121,   0.1601932777121,   0.1449676030656,   0.1179562851727,
    0.08494130810849,  0.05228012542714,   0.0251365935248, 0.006321671791697,
  -0.003939494651136,  -0.0073969593179,-0.006705587509768,-0.004378002076251,
  -0.002120592795595,-0.0006956761794133
};                //滤波器系数

public:

    /*!
    * \brief 构造函数
    * 
    * \param nh ROS节点句柄
    * \param climbot_name Motive定义的机器人名称
    */
    GetPoseVel(ros::NodeHandle &nh, std::string climbot_name, const float &frequency);


    /*!
    * \brief 位姿订阅回调函数
    * 
    * \param msg 话题"Climbot/pose"消息
    */
    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);


    /*!
    * \brief 机器人位姿微分计算速度
    */
   void CalculateVel();


    /*!
    * \brief 获取机器人平面投影位姿信息
    * 
    * \return 机器人的位姿
    */
    float* GetPose2D();


    /*!
    * \brief 获取机器人平面投影速度信息
    * 
    * \return 机器人的速度
    */
    float* GetVel2D();

};

#endif
