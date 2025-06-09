/*
 * @Author: lxs
 * @Date: 2022-11-29 03:14:39
 * @LastEditTime: 2023-05-10 03:15:54
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /new_steer/src/steer_track/include/steer_track/serial_debug_node.h
 * Copyright (c) 2021 LXScience&Technology. All rights reserved.
 */
#ifndef CLIMBOT_VEL_CONTROL_SERIAL_DEBUG
#define CLIMBOT_VEL_CONTROL_SERIAL_DEBUG

#include <ros/ros.h>
#include <serial/serial.h>
#include "steer_track/serial_instruction.h"
#include "steer_track/all_debug_data.h"
#include "steer_track/robot_pose_vel.h"
#include "steer_track/task_node_debug.h"
#include <log4cplus/log4cplus.h>
#include "boost/format.hpp"
#include "pthread.h"
#include <string>
#include <vector>
#include <queue>
#include <ctime>
#include <algorithm>

class Serial_Debug: public Serial_Prototype
{
public:
    void SerialInit(std::string port);
    std::string SerialRead();
};

typedef struct
{
    float tar_th[3];
    float now_th[3];
    float now_vel[3];
    float tar_force[3];
    float now_force[3];
} DR2;

typedef struct
{
    float now_pos[3];
    float now_vel[3];
    float tar_force[3];
    float now_force[3];
} DR1;


class motor_info
{
public:
    DR1 dr1;
    DR2 dr2;
    float fan_pre[3];
    double sec_f;
    boost::format fmt;
    motor_info()
    {
        fmt = boost::format(
            "steer%d:\n"\
            "tar_th:%.3f, now_th:%.3f,now_vel:%.3f, tar_force:%.3f, now_force:%.3f\n"\
            "now_vel:%.3f, tar_force:%.3f, now_force:%.3f\n"\
            "time:%f\n");
        dr1 = {0};
        dr2 = {0};
        fan_pre[0] = 0;
        fan_pre[1] = 0;
        fan_pre[3] = 0;
        sec_f = 0;
    }
    void fill_msg(steer_track::motor_info & msg);
};

void parase_motor_info(const std::string s, motor_info& m);
void init_logger();
void SerialDataCallback(const ros::TimerEvent&);
void TaskNodeDebugDataCallback(const steer_track::task_node_debugConstPtr& msg);
void RobotPoseVelCallback(const steer_track::robot_pose_velConstPtr& msg);


#endif