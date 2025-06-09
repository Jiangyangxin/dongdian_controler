#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h" 
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "std_msgs/Float32MultiArray.h"
#include <string>
#include <math.h>
#include "steer_track/motion_instruction.h"
#include "steer_track/robot_pose_vel.h"
using namespace std;

int close_lock=0;
int init=0;
double P_frock_C1[3]={0.5,3.9,-1.2}; //工装相对于相机的x,y,z //z相对于相机靠后，x向右，y向上
double line_base=0; //线的初始长度
double line_length=0;
double pose_[3]={0,0,0};
double motor_lenth_init=0;
double motor_lenth_now=0;
double motor_speed_now=0;
double motor_speed_send=0;
double msg_time=0;
int sec=0;
int stop_flag=1;
double motor_lenth_true=0;
void poseCallback(const steer_track::robot_pose_velConstPtr& msg) //close-loop
{
    // float pose_time = msg->optitrack_pose.header.stamp.toSec();
    // ros::Time t1= ros::Time::now();
    // double t_cur= t1.toSec();
    msg_time = msg->sec_f;
    // cout<<"msg: "<<std::fixed<<msg_time<<endl;
    // if(t_cur-pose_time > 1)
    // {
    //     close_lock=0; //开环模式的收放线
    // }
    // else
    // {
    //     close_lock=1; //闭环模式的收放线
    // }


    line_length=0;
    
    //x,y,z数据 由于相机的y轴朝下,因此实际的y轴三是相反数 ,z轴也是相反数
    pose_[0] = msg->optitrack_pose.pose.position.x;
    pose_[1] = -msg->optitrack_pose.pose.position.y;
    // cout<<"pose_[1]: "<<pose_[1]<<endl;
    pose_[2] = -msg->optitrack_pose.pose.position.z;
    // cout<<"pose_[2]: "<<pose_[2]<<endl;

    Eigen::Quaterniond quaternion4;
    quaternion4.x() = msg->optitrack_pose.pose.orientation.x;
    quaternion4.y() = msg->optitrack_pose.pose.orientation.y;
    quaternion4.z() = msg->optitrack_pose.pose.orientation.z;
    quaternion4.w() = msg->optitrack_pose.pose.orientation.w;

    for(int i=0;i<3;i++)
    {
         line_length+= pow(-P_frock_C1[i]+pose_[i],2); // x2+y2+z2
    }
    line_length=abs(sqrt(line_length)) + line_base; // ✔x2+y2+z2 + line_base
    //line_length为当前机器人和工装的直线距离

    if(init==0)
    {
        if(motor_lenth_now!=0) 
        {
            init=1;
        }       
//反馈的motor_lenth_now是上电时刻的相对收线长度,motor_lenth_init是程序运行时刻的相对收线长度
//若要使程序运行时刻 反馈的线长 == 当前机器人距离工装的线长， 需要设 motor_lenth_init= motor_lenth_now , line_length_init=line_length
//因此，实际motor_speed_send = k*(line_length-line_length_init - (motor_lenth_now-motor_lenth_init)) 
//
        motor_lenth_init= motor_lenth_now  -  line_length; 
    }


    try
    {
        ros::Time t2= ros::Time::now();
        double t_cur2= t2.toSec();   
        motor_lenth_true=0;
        
        motor_lenth_true=motor_lenth_now-motor_lenth_init;
        // cout<<"motor_lenth_true: "<<motor_lenth_true<<endl;
        // cout<<"line_length: "<<line_length<<endl;

        motor_speed_send=0.7*(line_length-motor_lenth_true); //line_length是当前机器人相对于收线器的相对长度
        // motor_lenth_now+=motor_speed_now*(t_cur2-t_cur);
        
    }
        catch(const std::exception& e)
    {
        ROS_INFO("异常信息:%s",e.what());
    }
        
}

void motorCallback(const  std_msgs::Float32MultiArray::ConstPtr& msg )
{
    motor_speed_now=msg->data.at(0);

}

void MotionInstructionCallback(const  steer_track::motion_instruction::ConstPtr& msg )
{
    if(close_lock ==0) //开环模式的收放线
    {
        // motor_speed_send=0;
        if(msg->mode== "increment")
        {   //依靠上位机指令来确定收放线
           stop_flag=0;
           motor_speed_send= -(msg->dy/abs(msg->dy))*0.07;
           sec= 150*(int)(abs((msg->dy / motor_speed_send)))+0.5;
        }

    }
    else if(close_lock ==1)
    {
        if(msg->mode== "increment")
        {   //依靠上位机指令来确定收放线
           stop_flag=0;
        }
    }

    if(msg->mode== "fzmotor")   //fankui
    {   
        motor_lenth_now= msg->motor_linelenth/100; // danwei mm mm mm
    }

    if(msg->mode== "stop")   //无论开环闭环,stop模式下都要停止收放线
    {   
        stop_flag=1;
        motor_speed_send = 0;
    }

}


int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "motor_line");
    //创建节点句柄
    ros::NodeHandle nh;
    ros::Subscriber pose_sub ;
    ros::Subscriber motor_sub;
    ros::Subscriber motion_sub;
    ros::Publisher motor_line_pub= nh.advertise<std_msgs::Float32MultiArray>("motor_line_data",10);
    std_msgs::Float32MultiArray motor_data;

    pose_sub  = nh.subscribe("pose_vel_data",1, poseCallback);
        // pose_sub  = nh.subscribe("/camera_1/apriltag_tf_pose",1, poseCallback);
    // motor_sub = nh.subscribe("frock_motor",1, motorCallback); //反馈的是线速度，最终输出的也是线速度
    motion_sub= nh.subscribe("motion_instruction",1,MotionInstructionCallback);
    ros::Rate r(60);
    while(ros::ok())
    {   
        ros::Time t1= ros::Time::now();
        double t_cur= t1.toSec();
        if(t_cur-msg_time > 1)
        {
            close_lock=0; //开环模式的收放线
            init=0; //下次进入闭环的时候重新初始化
        }
        else
        {
            close_lock=1; //闭环模式的收放线
        }
        
        
        sec--;
        // std::cout<<sec<<std::endl;
        if(sec<=0 && close_lock==0)
        {
            motor_speed_send=0;
        }
        if(stop_flag==1)
        {
            motor_speed_send=0;
        }
        // cout<<"msg_time: "<<std::fixed<<msg_time<<endl;
        // cout<<"t_cur-msg_time: "<<t_cur- msg_time<<endl;

        if(motor_speed_send>0.1)
        {
            motor_speed_send=0.1;
        }
        else if (motor_speed_send<-0.1)
        {
            motor_speed_send=-0.1;
        }

        // cout<<"close_lock: "<<close_lock<<endl;
        // cout<<"motor_lenth_true: "<<motor_lenth_true<<endl;
        // cout<<"motor_speed_send: "<<motor_speed_send<<endl;
        // // cout<<"motor_lenth_init: "<<motor_lenth_init<<endl;
        // cout<<"line_length: "<<line_length<<endl;
        motor_data.data.push_back(motor_speed_send);
        motor_line_pub.publish(motor_data);
        motor_data.data.clear();
        ros::spinOnce();
        r.sleep();
    } 
    
    return 0;  
}