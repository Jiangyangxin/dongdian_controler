#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

float image_x=0;
float image_y=0;
float real_robot_x=0;
float real_robot_y=0;
float real_robot_z=0;
// 计算物体实际坐标的函数
geometry_msgs::Point calculateObjectPosition(
    const geometry_msgs::Point& robot_actual_pos,
    const geometry_msgs::Point& robot_pixel_pos,
    const geometry_msgs::Point& object_pixel_pos,
    double image_width, double image_height)
{
    // 1. 将像素坐标转换为以图像中心为原点的坐标系
    double robot_pixel_centered_x = robot_pixel_pos.x - image_width/2;
    double robot_pixel_centered_y = -(robot_pixel_pos.y - image_height/2); // Y轴反转
    
    double object_pixel_centered_x = object_pixel_pos.x - image_width/2;
    double object_pixel_centered_y = -(object_pixel_pos.y - image_height/2); // Y轴反转
    
    // 2. 计算比例因子 (像素坐标与实际坐标的比例)
    double scale_x = (robot_actual_pos.x != 0) ? robot_pixel_centered_x / robot_actual_pos.x : 1.0;
    double scale_y = (robot_actual_pos.y != 0) ? robot_pixel_centered_y / robot_actual_pos.y : 1.0;
    
    // 防止除零错误
    if (scale_x == 0) scale_x = 1.0;
    if (scale_y == 0) scale_y = 1.0;
    
    // 3. 计算物体的实际坐标
    geometry_msgs::Point object_actual_pos;
    object_actual_pos.x = object_pixel_centered_x / scale_x;
    object_actual_pos.y = object_pixel_centered_y / scale_y;
    object_actual_pos.z = robot_actual_pos.z; // 保持Z坐标相同
    
    return object_actual_pos;
}

void RobotimagexyDataCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    image_x=msg->data[0];
    image_y=msg->data[1];
}
void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    real_robot_x=msg->pose.position.x;
    real_robot_y=msg->pose.position.y;
    real_robot_z=msg->pose.position.z;
}

int main(int argc, char** argv)//此文件尚未测试，需要和赵子同联调
{
    ros::init(argc, argv, "pixel_to_actual_converter");
    ros::NodeHandle nh;
    ros::Subscriber robot_imagexy_sub = nh.subscribe("/robot_imagexy", 1, RobotimagexyDataCallback);
    std::string topic_name = "final_pose"; //mix_position
    ros::Subscriber robot_realpos_sub = nh.subscribe(topic_name, 1, PoseCallback); //机器人位姿订阅器


    // 示例数据
    geometry_msgs::Point robot_actual_pos;
    robot_actual_pos.x = real_robot_x;  // 机器人的实际X坐标（米）
    robot_actual_pos.y = real_robot_y;  // 机器人的实际Y坐标（米）
    robot_actual_pos.z = real_robot_z;  // 机器人的实际Z坐标（米）
    
    geometry_msgs::Point robot_pixel_pos;
    robot_pixel_pos.x = image_x;   // 机器人的像素U坐标（U1）
    robot_pixel_pos.y = image_y;   // 机器人的像素V坐标（V1）
    robot_pixel_pos.z = 0.0;
    
    geometry_msgs::Point object_pixel_pos;
    object_pixel_pos.x = 700;  // 物体的像素U坐标（U2）
    object_pixel_pos.y = 400;  // 物体的像素V坐标（V2）
    object_pixel_pos.z = 0.0;

    // 图像尺寸
    double image_width = 1440;
    double image_height = 1080;
        
    // 计算物体实际坐标
    geometry_msgs::Point object_actual_pos = calculateObjectPosition(
        robot_actual_pos, robot_pixel_pos, object_pixel_pos,
        image_width, image_height);
    
    ROS_INFO("Object actual position: x=%.2f, y=%.2f, z=%.2f",
             object_actual_pos.x, object_actual_pos.y, object_actual_pos.z);
    
    return 0;
}