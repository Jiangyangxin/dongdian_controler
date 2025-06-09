/*** 
 * @Author: lxs
 * @Date: 2021-07-09 13:50:04
 * @LastEditTime: 2021-08-20 08:52:40
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /steer_track_ws/src/steer_track/src/serial_node.cpp
 * @jntm jntm jntm jntm jntm
 */
#include "ros/ros.h"
#include "pthread.h"
#include "serial/serial.h"
#include "steer_track/serial_instruction.h"
#include "steer_track/motion_instruction.h"
#include "steer_track/steer_th_vel.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"


ros::Subscriber motor_instruction_sub;
ros::Subscriber command_sub;
ros::Publisher fan_data_pub;

ros::Timer command_uart_data_timer;

//串口指令对象
Serial_Instruction serial_instruction;//自定义类Serial_Instruction

int fan_1 = 0.0, fan_2 = 0.0, fan_3 = 0.0;
float vx = 0.0, vy = 0.0, vw = 0.0;

//底层串口指令发布定时器
ros::Timer bottom_serial_timer;
//风机数据获取定时器
ros::Timer fan_data_timer;


/*!
* \brief 电机指令回调函数
* 
* \param msg 由task_node发布的"motor_instruction"话题消息
*/
void SetMotorCallback(const steer_track::steer_th_velConstPtr& msg);

/*** 
 * @Description: 指令消息回调函数
 * @param {const} tcp_server_node发布的指令消息
 * @return {*}
 */
void CommandCallback(const steer_track::motion_instructionConstPtr& motion_msg);

/**
 * @description: 定时器回调函数，每0.1秒读取一次串口serial_command_instruction的数据
 * @param {const} ros
 * @return {*}
 */
void CommandDataCallback(const ros::TimerEvent&);

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "serial_node");
    if (argc != 2)
    {
        ROS_INFO("Please set 1 serial port!");
        return 1;
    }
    
    //创建节点句柄
    ros::NodeHandle nh;

    //初始化串口
    serial_instruction.SerialInit(argv[1]);
    // serial_command_instruction.SerialInit(argv[2]);

    //电机指令订阅器
    motor_instruction_sub = nh.subscribe("motor_instruction", 1, SetMotorCallback);
    //指令订阅器
    command_sub = nh.subscribe("motion_instruction", 1, CommandCallback);
    
    //风机数据发布器
    fan_data_pub = nh.advertise<std_msgs::String>("fan_data", 1);

    //command串口读取定时器(周期为100ms)
    command_uart_data_timer = nh.createTimer(ros::Duration(0.1), CommandDataCallback);

    ros::spin();
    
    return 0;
}


void SetMotorCallback(const steer_track::steer_th_velConstPtr& msg)
{
    serial_instruction.setSteerTHV(msg);//具体函数执行在Serial_Instruction.cpp中，发送stm32舵机，风机和时间戳
    return;
}

void CommandCallback(const steer_track::motion_instructionConstPtr& motion_msg)
{
    if (motion_msg->mode == "motion")    //发送motion指令
    {
        serial_instruction.SerialWrite("motion\r\n");
    }
    else if (motion_msg->mode == "stcorr")    //发送stcorr指令,舵轮方向校准
    {
        serial_instruction.SerialWrite("stcorr\r\n");
    }
    else if (motion_msg->mode == "fans")    //发送fans指令
    {
        serial_instruction.SerialWrite("fans\r\n");
    }
    else if (motion_msg->mode == "stopF")
    {
        serial_instruction.SerialWrite("stopF\r\n");
    }
    // else if (motion_msg->mode == "set_fan")
    // {
    //     serial_instruction.SetFan(motion_msg->fan_1, motion_msg->fan_2, motion_msg->fan_3);
    // }
}

/**
 * @description: 定时器回调函数，每0.1秒读取一次串口serial_command_instruction的数据
 * @param {const} ros
 * @return {*}
 */
void CommandDataCallback(const ros::TimerEvent&)
{
    std::string data_str;
    data_str = serial_instruction.SerialRead();//读取串口数据

    if(data_str.size()!=0)
    {
        ROS_INFO_STREAM(data_str.c_str());
        std::string::size_type st, en;//化为不定长的字符串，保证上下位机字符串数据完全匹配
        st = data_str.find("fan\t");
        if (st != std::string::npos)
        {
            en = data_str.find("\r\n",st);
            if (en != std::string::npos)
            {
                std_msgs::String fan_data_msg;
                fan_data_msg.data = data_str.substr(st, en-st+2);
                fan_data_pub.publish(fan_data_msg);//发布风机负压数据
            }
        }
    }
    return;
}