#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h" 
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "std_msgs/Float32MultiArray.h"
#include <string>
using namespace std;

float P_frock_C1[3]={10,10,10}; //工装相对于相机的x,y,z 
float line_base=10; //线的初始长度
float line_length=0;
float pose_[3]={0,0,0};
float motor_round=0;
int motor_round_to_line = 10;
int motor_line=0;
float motor_speed=0;

void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{

    line_length=0;
    
    //x,y,z数据
    pose_[0] = msg->pose.position.x;
    pose_[1] = msg->pose.position.y;
    pose_[2] = msg->pose.position.z;

    Eigen::Quaterniond quaternion4;
    quaternion4.x() = msg->pose.orientation.x;
    quaternion4.y() = msg->pose.orientation.y;
    quaternion4.z() = msg->pose.orientation.z;
    quaternion4.w() = msg->pose.orientation.w;

    for(int i=0;i<3;i++)
    {
        // line_length+= pow(P_frock_C1[i]+pose_[i]); // x2+y2+z2
    }
    line_length=abs(sqrt(line_length)) + line_base; // ✔x2+y2+z2 + line_base

}

void motorCallback(const  std_msgs::Float32MultiArray::ConstPtr& msg )
{
    motor_round=msg->data.at(0);
    motor_line=motor_round* motor_round_to_line;

}

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "line_node");
    //创建节点句柄
    ros::NodeHandle nh;
    ros::Subscriber pose_sub ;
    ros::Subscriber motor_sub;

    pose_sub  = nh.subscribe("final_pose",10, poseCallback);
    motor_sub = nh.subscribe("frock_motor",10, motorCallback);
    while(ros::ok())
    {  
        try
        {
            motor_speed=10*(line_length-motor_line);

            
        }
            catch(const std::exception& e)
        {
            ROS_INFO("异常信息:%s",e.what());
        }
        
        ros::spin();
    } 
    
    return 0;
}