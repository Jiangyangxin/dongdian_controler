/*** 
 * @Author: lxs
 * @Date: 2023-02-21 14:09:04
 * @LastEditTime: 2023-02-21 17:06:07
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /openloop_ws/src/steer_track/include/steer_track/imu_node.h
 * @jntm jntm jntm jntm jntm
 */
/*
 * @Author: lxs
 * @Date: 2023-02-20 21:37:07
 * @LastEditTime: 2023-02-20 22:24:34
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: \steerPrototype-debug_motor_info\steer_track\include\steer_track\imu_node.h
 * Copyright (c) 2021 LXScience&Technology. All rights reserved.
 */
#ifndef CLIMBOT_IMU_NODE
#define CLIMBOT_IMU_NODE

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include <serial/serial.h>
#include <log4cplus/log4cplus.h>
#include "boost/format.hpp"
#include "pthread.h"
#include <string>
#include <vector>
#include "steer_track/serial_instruction.h"
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>


class Serial_IMU: public Serial_Prototype
{
public:
    void SerialInit(std::string port);
    std::string SerialRead();
};

class kalman_filter
{
public:
    Eigen::Vector2d x;
    Eigen::Matrix2d A;
    Eigen::Matrix2d P_last;
    Eigen::Matrix2d P_pre;
    Eigen::Matrix2d Q;
    Eigen::Matrix2d K;
    Eigen::Matrix2d H;
    Eigen::Matrix2d R;
    kalman_filter();
    float step(double w, double z, double dt);
};


#endif