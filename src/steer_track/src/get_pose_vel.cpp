#include "steer_track/get_pose_vel.h"
#include "steer_track/robot_pose_vel.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <cmath>
// #include "Eigen/Eigen"
using namespace std;

//获取Optitrack相对位姿的函数 
/*!
 * \brief 构造函数
 * 
 * \param nh ROS节点句柄
 * \param climbot_name Motive定义的机器人名称
 */
GetPoseVel::GetPoseVel(ros::NodeHandle &nh, std::string climbot_name, const float &frequency)
    : nh_(nh), climbot_name_(climbot_name), frequency_(frequency)
{
    //变量初始化
    for (size_t i = 0; i < 6; i++)
    {
        pose_.push_back(0.0);
        vel_.push_back(0.0);
        filter_vel_.push_back(0.0);
    }

    for (size_t i = 0; i < 3; i++)
    {
        pose_queue_.push_back(pose_);
    }

    for (size_t i = 0; i < 26; i++)
    {
        vel_queue_.push_back(vel_);
    }

    //加入apriltag检测的代码，正式使用的时候注释



    std::string topic_name;
    // topic_name = "/vrpn_client_node/" + climbot_name_ + "/pose";
    // topic_name = "final_pose";
    // topic_name= "/slam_nav/vio_pose";
    // topic_name = "apriltag_tf_pose";// only use apriltag
    topic_name = "final_pose"; //mix_position
    pose_sub_ = nh_.subscribe(topic_name, 1, &GetPoseVel::PoseCallback, this); //Optitrack位姿订阅器

    //位姿和速度信息发布器
    pose_vel_pub_ = nh.advertise<steer_track::robot_pose_vel>("pose_vel_data", 1);
}

/*!
 * \brief 位姿订阅回调函数
 * 
 * \param msg 话题"Climbot/pose"消息
 */
void GetPoseVel::PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    //x,y,z数据
    pose_[0] = msg->pose.position.x;
    pose_[1] = msg->pose.position.y;
    pose_[2] = msg->pose.position.z;

    Eigen::Quaterniond quaternion4;
    quaternion4.x() = msg->pose.orientation.x;
    quaternion4.y() = msg->pose.orientation.y;
    quaternion4.z() = msg->pose.orientation.z;
    quaternion4.w() = msg->pose.orientation.w;
    Eigen::Matrix3d rotation_matrix4;
    rotation_matrix4 = quaternion4.matrix();//初始旋转矩阵

    Eigen::Matrix3d rotation_matrix_X;;//初始旋转矩阵
    Eigen::AngleAxisd rotation_vector(M_PI , Eigen::Vector3d(1,0,0));
    rotation_matrix_X=rotation_vector.matrix();

    rotation_matrix4=rotation_matrix4*rotation_matrix_X; //绕x轴旋转90度

    // cout<<"1"<<endl<<rotation_matrix4<<endl;
    Eigen::Vector3d rigid_x_world = rotation_matrix4.col(0);//取第一列
    // cout<<"2"<<endl<<rigid_x_world<<endl;
    float yaw2 = atan2(rigid_x_world(1), rigid_x_world(0));//y,x
    
    steer_track::robot_pose_vel pose_vel_msg;

    // pose_vel_msg.xy[0] = pose_[0];//x
    // pose_vel_msg.xy[1] = -pose_[1];//y 绕x轴旋转90度后，y=-y
    // pose_vel_msg.yaw = yaw2;


//gzf only!!!!!
    pose_vel_msg.xy[0] = pose_[0];//x
    pose_vel_msg.xy[1] = -pose_[1];//y 绕x轴旋转90度后，y=-y
    pose_vel_msg.yaw = yaw2;
/////////////


    // pose_vel_msg.sec_f = (double)(msg->header.stamp.nsec)/1e9 + msg->header.stamp.sec;
    pose_vel_msg.sec_f = (double) ros::Time::now().toSec();
    pose_vel_msg.optitrack_pose = *msg;
    // ROS_INFO_STREAM("sec:"<<msg->header.stamp.sec<<", nsec:"<<msg->header.stamp.nsec<<", sec_f:"<<pose_vel_msg.sec_f);
    pose_vel_pub_.publish(pose_vel_msg);

    return;
}

/*!
 * \brief 机器人位姿微分计算速度
 */
void GetPoseVel::CalculateVel()
{
    float pose_1[6], pose_2[6], pose_3[6];
    for (size_t i = 0; i < 6; i++)
    {
        pose_1[i] = pose_queue_[0][i];
        pose_2[i] = pose_queue_[1][i];
        pose_3[i] = pose_queue_[2][i];
    }

    for (size_t i = 0; i < 6; i++)
    {
        vel_[i] = (3 * pose_3[i] - 4 * pose_2[i] + pose_1[i]) / (2 / frequency_);
    }
    vel_queue_.erase(vel_queue_.begin());
    vel_queue_.push_back(vel_);

    std::vector<float> filter_vel_temp;
    for (size_t i = 0; i < 6; i++)
    {
        filter_vel_temp.push_back(0.0);
    }

    //滤波
    for (size_t i = 0; i < 26; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            filter_vel_temp[j] += fir_a_[i] * vel_queue_[i][j];
        }
    }
    filter_vel_ = filter_vel_temp; //得到滤波后速度
    return;
}

/*!
 * \brief 获取机器人平面投影位姿信息，地平面（-0，2，4），风电叶片（2，1，4）
 * 
 * \return 机器人的位姿
 */
float *GetPoseVel::GetPose2D()
{
    pose_2d_[0] = pose_[2];
    pose_2d_[1] = pose_[1];
    pose_2d_[2] = pose_[5];
    return pose_2d_;
}

/*!
 * \brief 获取机器人平面投影速度信息
 * 
 * \return 机器人的速度
 */
float *GetPoseVel::GetVel2D()
{
    vel_2d_[0] = 0;
    //ROS_INFO("vx:\t%f", vel_2d_[0]);
    vel_2d_[1] = 0;
    //ROS_INFO("vy:\t%f", vel_2d_[1]);
    vel_2d_[2] = 0;
    return vel_2d_;
}
