#pragma once
#include <queue>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <map>
#include "../feature_manager.h" 
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include "initial_alignment.h"


class Static_imu_init
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   Static_imu_init();
   
   //数据入栈
   void imu_push(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
   //判断机器人是否处于静止状态
   bool static_flag();
   //求窗口内的平均数据
   bool imu_avg();
   //根据重力方向，将SLAM坐标系z轴与重力方向对齐
   void g_z_align();
   bool imu_init(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &G, Eigen::VectorXd &x);
   void clear_state();

   //std::vector<Eigen::Matrix<double,7,1>,Eigen::aligned_allocator<Eigen::Matrix<double,7,1>>> imu_buf;
   std::vector<Eigen::Matrix<double,7,1>> imu_buf;
   Eigen::Vector3d acc_avg,gyro_avg;
   Eigen::Vector3d g;
   //acc gyro bias
   Eigen::Vector3d ba,bg;
   Eigen::Matrix3d Rg;
   int size;
};
