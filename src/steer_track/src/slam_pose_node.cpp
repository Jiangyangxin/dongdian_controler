#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
ros::Publisher pub_vio_pose,pub_vio_path;
vector<Eigen::Vector3d> rec_pos;
vector<Eigen::Vector3d> rec_velocity;
vector<Eigen::Quaterniond> rec_ori;
vector<double> rec_time;
nav_msgs::Path path;
bool time_init_flag,sim_flag;
double data_time,init_time,time_offset;
int countt;
enum vio_robot{Center=0,Left,Down};
ros::Publisher pub_slam_pose_visual;
Eigen::Vector3d estimator_pos;
Eigen::Quaterniond estimator_ori;

void pose_callback(const nav_msgs::PathConstPtr &path_msg)
{
    rec_pos.clear();
    rec_velocity.clear();
    rec_ori.clear();
    rec_time.clear();
    //ROS_INFO("path get!");

    if(!time_init_flag)
    {
        data_time=path_msg->header.stamp.toSec();
        init_time=ros::Time::now().toSec();
        time_offset=init_time-data_time;
        time_init_flag=1;
        if(time_offset>500)
            sim_flag=1;
        else
            sim_flag=0;
        ROS_INFO("Time Complete! time_offset and simflag: %f, %d",time_offset,sim_flag);
    }
    
    int size = path_msg->poses.size();
    if (size < 5)
    {
        Eigen::Vector3d tmp_pos;
        Eigen::Quaterniond tmp_ori;
        tmp_pos.x() = path_msg->poses[size - 1].pose.position.x;
        tmp_pos.y() = path_msg->poses[size - 1].pose.position.y;
        tmp_pos.z() = path_msg->poses[size - 1].pose.position.z;
        tmp_ori.w() = path_msg->poses[size - 1].pose.orientation.w;
        tmp_ori.x() = path_msg->poses[size - 1].pose.orientation.x;
        tmp_ori.y() = path_msg->poses[size - 1].pose.orientation.y;
        tmp_ori.z() = path_msg->poses[size - 1].pose.orientation.z;
    }
    else
    {
        Eigen::Vector3d tmp_pos;
        Eigen::Quaterniond tmp_ori;
        double tmp_time;
        for(int i=0;i<5;i++)
        {
            tmp_pos.x() = path_msg->poses[size - i - 1].pose.position.x;
            tmp_pos.y() = path_msg->poses[size - i - 1].pose.position.y;
            tmp_pos.z() = path_msg->poses[size - i - 1].pose.position.z;
            tmp_ori.w() = path_msg->poses[size - i - 1].pose.orientation.w;
            tmp_ori.x() = path_msg->poses[size - i - 1].pose.orientation.x;
            tmp_ori.y() = path_msg->poses[size - i - 1].pose.orientation.y;
            tmp_ori.z() = path_msg->poses[size - i - 1].pose.orientation.z;
            tmp_time = path_msg->poses[size - i - 1].header.stamp.toSec();
            rec_pos.push_back(tmp_pos);
            rec_ori.push_back(tmp_ori);
            rec_time.push_back(tmp_time);
        }
        for(int i=0;i<4;i++)
        {
            Eigen::Vector3d tmp_v;
            tmp_pos=rec_pos[i]-rec_pos[i+1];
            tmp_v=tmp_pos/(rec_time[i]-rec_time[i+1]);
            rec_velocity.push_back(tmp_v);
        }
    }
}

/*
void estimator_callback(const nav_msgs::PathConstPtr &pose_msg)
{
//帮谭科做实验
Eigen::Vector3d tmp_pos;
    Eigen::Quaterniond tmp_ori;
tmp_pos.x() = pose_msg->poses.back().pose.position.x;
        tmp_pos.y() = pose_msg->poses.back().pose.position.y;
        tmp_pos.z() = pose_msg->poses.back().pose.position.z;
        tmp_ori.w() = pose_msg->poses.back().pose.orientation.w;
        tmp_ori.x() = pose_msg->poses.back().pose.orientation.x;
        tmp_ori.y() = pose_msg->poses.back().pose.orientation.y;
        tmp_ori.z() = pose_msg->poses.back().pose.orientation.z;
        
        
        ros::Time time_now = ros::Time::now();
    double d_time_now = time_now.toSec();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = time_now;
    pose.header.frame_id = "world";
        
                
	Eigen::Vector3d t_offset;
    Eigen::Matrix3d R_offset;
    Eigen::AngleAxisd r1, r2, r3;
    Eigen::Matrix3d tmp_r1, tmp_r2, tmp_r3;
    Eigen::Matrix3d R_ori;
    R_ori=tmp_ori.toRotationMatrix();
    vio_robot mode=Center;
    switch (mode)
    {
    case Center:
        r1 = Eigen::AngleAxisd(180 * 3.1415926 / 180, Eigen::Vector3d(R_ori(0,0), R_ori(1,0), R_ori(2,0)));
        //ROS_INFO("Angle axisd:%f,%f,%f",R_ori(0,0), R_ori(1,0), R_ori(2,0));
        r2 = Eigen::AngleAxisd(0 * 3.1415926 / 180, Eigen::Vector3d(0, 1, 0));
        r3 = Eigen::AngleAxisd(0 * 3.1415926 / 180, Eigen::Vector3d(0, 0, 1));
        t_offset<<0,0,0;
        break;
    
    default:
        break;
    }

    //坐标变换
    tmp_r1 = r1.matrix();
    tmp_r2 = r2.matrix();
    tmp_r3 = r3.matrix();
    R_offset = tmp_r3 * tmp_r2 * tmp_r1;
    R_ori=R_offset*tmp_ori.toRotationMatrix(); 
    tmp_ori=Eigen::Quaterniond(R_ori);

    pose.pose.position.x = tmp_pos.x()+t_offset.x();
    pose.pose.position.y = tmp_pos.y()+t_offset.y();
    pose.pose.position.z = tmp_pos.z()+t_offset.z();
    pose.pose.orientation.w = tmp_ori.w();
    pose.pose.orientation.x = tmp_ori.x();
    pose.pose.orientation.y = tmp_ori.y();
    pose.pose.orientation.z = tmp_ori.z();
    path.header=pose.header;
    path.poses.push_back(pose);
    pub_vio_pose.publish(pose);
    pub_vio_path.publish(path);

}
*/


void estimator_callback(const nav_msgs::PathConstPtr &pose_msg)
{
    if (rec_pos.empty())
    {
        estimator_pos.x() = pose_msg->poses.back().pose.position.x;
        estimator_pos.y() = pose_msg->poses.back().pose.position.y;
        estimator_pos.z() = pose_msg->poses.back().pose.position.z;
        estimator_ori.w() = pose_msg->poses.back().pose.orientation.w;
        estimator_ori.x() = pose_msg->poses.back().pose.orientation.x;
        estimator_ori.y() = pose_msg->poses.back().pose.orientation.y;
        estimator_ori.z() = pose_msg->poses.back().pose.orientation.z;
        rec_pos.push_back(estimator_pos);
        rec_ori.push_back(estimator_ori);
    }
}



void pub_vio()
{
    ros::Time time_now = ros::Time::now();
    double d_time_now = time_now.toSec();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = time_now;
    pose.header.frame_id = "world"; //此处可以修改pose的frame_id

    Eigen::Vector3d tmp_pos;
    Eigen::Quaterniond tmp_ori;
    if (rec_pos.size() < 1)
    {
        countt++;
        if (countt % 100 == 0)
            ROS_WARN("No VIO message!");
        return;
    }

    double time_offset_sim;
    time_offset_sim=time_offset;

    if(sim_flag)
    {
        d_time_now=d_time_now-time_offset;
        time_offset_sim=0;
    }

    if(rec_pos.size()<2)
    {
        tmp_pos=rec_pos[0];
        tmp_ori=rec_ori[0];
    }
    else if ((d_time_now - rec_time[0] < 1.2*time_offset_sim)&&(d_time_now - rec_time[0])>0)
    {
        tmp_pos = rec_velocity[0] * (d_time_now - rec_time[0]) + rec_pos[0];
        tmp_ori = rec_ori[0];
    }
    else
    {
        rec_velocity[0]=0*rec_velocity[0];
        tmp_pos = rec_velocity[0] * (d_time_now - rec_time[0]) + rec_pos[0];
        tmp_ori = rec_ori[0];
    }

    Eigen::Vector3d t_offset;
    Eigen::Matrix3d R_offset;
    Eigen::AngleAxisd r1, r2, r3;
    Eigen::Matrix3d tmp_r1, tmp_r2, tmp_r3;
    Eigen::Matrix3d R_ori;
    R_ori=tmp_ori.toRotationMatrix();
    vio_robot mode=Center;
    switch (mode)
    {
    case Center:
        r1 = Eigen::AngleAxisd(180 * 3.1415926 / 180, Eigen::Vector3d(R_ori(0,0), R_ori(1,0), R_ori(2,0)));
        //ROS_INFO("Angle axisd:%f,%f,%f",R_ori(0,0), R_ori(1,0), R_ori(2,0));
        r2 = Eigen::AngleAxisd(0 * 3.1415926 / 180, Eigen::Vector3d(0, 1, 0));
        r3 = Eigen::AngleAxisd(0 * 3.1415926 / 180, Eigen::Vector3d(0, 0, 1));
        t_offset<<0,0,0;
        break;
    
    default:
        break;
    }

    //坐标变换
    tmp_r1 = r1.matrix();
    tmp_r2 = r2.matrix();
    tmp_r3 = r3.matrix();
    R_offset = tmp_r3 * tmp_r2 * tmp_r1;
    R_ori=R_offset*tmp_ori.toRotationMatrix(); 
    tmp_ori=Eigen::Quaterniond(R_ori);

    pose.pose.position.x = tmp_pos.x()+t_offset.x();
    pose.pose.position.y = tmp_pos.y()+t_offset.y();
    pose.pose.position.z = tmp_pos.z()+t_offset.z();
    pose.pose.orientation.w = tmp_ori.w();
    pose.pose.orientation.x = tmp_ori.x();
    pose.pose.orientation.y = tmp_ori.y();
    pose.pose.orientation.z = tmp_ori.z();
    path.header=pose.header;
    path.poses.push_back(pose);
    pub_vio_pose.publish(pose);
    // pub_vio_path.publish(path);


}

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_nav");
  ros::NodeHandle ns;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Info);
  ros::Subscriber sub_good_loop = ns.subscribe("/vins_estimator/path", 10, pose_callback); //接收vins的话题消息
  pub_vio_pose=ns.advertise<geometry_msgs::PoseStamped>("/slam_nav/vio_pose",10); //将 接收到的path消息进行插值处理，最后生成了pose

    //ros::Subscriber sub_good_loop = ns.subscribe("/pose_graph/pose_graph_path", 10, pose_callback);
    //   ros::Subscriber sub_estimator_pose = ns.subscribe("/vins_estimator/path", 3, estimator_callback); 
    //   pub_vio_path=ns.advertise<nav_msgs::Path>("/slam_nav/vio_path",10);
    //   pub_slam_pose_visual = ns.advertise<visualization_msgs::MarkerArray>("/slam_nav/slam_pose_visual", 10);

  time_init_flag=0;
  countt=0;

  ros::Rate loop_rate(50); //将VIO的消息插值到60
  while (ros::ok())
  {
    pub_vio();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
