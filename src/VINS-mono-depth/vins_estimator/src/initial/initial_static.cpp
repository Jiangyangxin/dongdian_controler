#include "initial_static.h" 
Static_imu_init::Static_imu_init()
{
  imu_buf.clear();
  size=0;
}

void Static_imu_init::imu_push(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity)
{
  size++;
  Eigen::Matrix<double,7,1> imu;
  imu(0) = dt;
  imu.block<3, 1>(1, 0) = linear_acceleration;
  imu.block<3, 1>(4, 0) = angular_velocity;
  imu_buf.push_back(imu);
  //ROS_INFO("static_imu get");
}

void Static_imu_init::clear_state()
{
  imu_buf.clear();
  size=0;
  ROS_INFO("Robot move! Clear measurements.");
}

//求窗口内的平均数据
bool Static_imu_init::imu_avg()
{
    int count=imu_buf.size();
    if(count<100)//200
    {
      ROS_INFO("Too few IMU measurements %d",count);
      return false;
    }
    Eigen::Vector3d acc_sum, gyro_sum;
    for (int i = 0; i < count; i++) 
    {
      Eigen::Matrix<double,7,1> imu=imu_buf[i];
      double t = imu(0);
      double dx = imu(1);
      double dy = imu(2);
      double dz = imu(3);
      Eigen::Vector3d linear_acceleration{dx, dy, dz};
      //std::cout<<linear_acceleration.norm()<<std::endl;

      double rx = imu(4);
      double ry = imu(5);
      double rz = imu(6);
      Eigen::Vector3d angular_velocity{rx, ry, rz};

      acc_sum += linear_acceleration;
      gyro_sum += angular_velocity;
    }
    acc_avg=acc_sum/count;
    gyro_avg=gyro_sum/count;
    if(acc_avg.norm()<9||acc_avg.norm()>11)
    {
      ROS_INFO("Acc measurements isn't near 9.8!%f",acc_avg.norm());
      clear_state();
      return false;
    }
    if(!(gyro_avg.norm()<0.01))
    {
      ROS_INFO("Too big gyro bias! %f",gyro_avg.norm());
      clear_state();
      return false;
    }
    return true;
}

//判断机器人是否处于静止状态,这里不做限制了，由视差去评价是否处于静止
bool Static_imu_init::static_flag()
{
  double a_var = 0;
  for(int i=0;i<size;i++) 
  {
    Eigen::Matrix<double,7,1> imu;
    imu=imu_buf[i];
    Eigen::Vector3d imu_acc{imu(1),imu(2),imu(3)};
    a_var += (imu_acc-acc_avg).dot(imu_acc-acc_avg);
  }
    a_var = std::sqrt(a_var/(size-1));
    if(a_var<5)
    {
      ROS_INFO("a_var %f!",a_var);
      return true;
    }
    else
    {
      ROS_INFO("Robot big motion!");
      ROS_INFO("a_var %f!",a_var);
      return false;
    }
}

//根据重力方向，将SLAM坐标系z轴与重力方向对齐
void Static_imu_init::g_z_align()
{
  Eigen::Vector3d g0;
  g0=acc_avg/acc_avg.norm();
  g=9.81*g0;
  ba=acc_avg-g;
  cout<<"init ba: "<<ba.transpose()<<endl;
  bg<<0,0,0;
  //bg=gyro_avg;
  ROS_INFO_STREAM("init_g: " <<  g.transpose()<<"  g0:"<< acc_avg.norm());
  ROS_INFO_STREAM("init_acc_bias: " <<  ba.transpose());
  ROS_INFO_STREAM("init_gyro_bias: " <<  gyro_avg.transpose());
}

//整合函数
bool Static_imu_init::imu_init(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &G, Eigen::VectorXd &x)
{
  bool result=imu_avg()&&static_flag();
  if(!result)
    return false;
  else
  {
    g_z_align();
    G=g;
    //更新滑动窗口里面的bg
    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] = bg;
    int all_frame_count = all_image_frame.size();
    //维度，n(速度)*3+3(重力)+1(尺度)
    int n_state = all_frame_count * 3 + 3 + 1;
    VectorXd b{n_state};
    x=b;
    x.setZero();
    x(n_state-1)=1;
    x.block<3,1>(n_state-4,0)=G;
    return true;
  }
}

