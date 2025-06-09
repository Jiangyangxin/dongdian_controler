/*
 * @Author: lxs
 * @Date: 2023-02-20 21:36:54
 * @LastEditTime: 2023-03-10 07:58:25
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /new_steer/src/steer_track/src/imu_node.cpp
 * Copyright (c) 2021 LXScience&Technology. All rights reserved.
 */
//老IMU代码

#include "steer_track/imu_node.h"
#include <sensor_msgs/Imu.h>
#include "tf2/LinearMath/Quaternion.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

void imuDataCallback(const ros::TimerEvent&);

ros::Publisher imu_data_pub;
ros::Publisher imu_yaw_pub;
ros::Publisher imu_raw_pub;
ros::Publisher IMU_pub;
ros::Timer imu_timer;
Serial_IMU serial_imu_instruction;
kalman_filter kf;

void usbreset();

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "imu_node");
    if (argc != 2)
    {
        ROS_INFO("Serial_Debug:Please set 1 serial port!");
        return 1;
    }
    
    //创建节点句柄
    ros::NodeHandle nh;

    //初始化串口
    // usbreset();
    serial_imu_instruction.SerialInit(argv[1]);

    //imu数据发布器
    imu_data_pub = nh.advertise<std_msgs::Float32MultiArray>("/imu_data", 1);
    imu_yaw_pub = nh.advertise<std_msgs::Float32>("/imu_yaw", 1);
    imu_raw_pub = nh.advertise<std_msgs::Float32>("/imu_raw", 1);

    IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 20);

    //imu串口读取定时器,imu的频率为200hz
    imu_timer = nh.createTimer(ros::Duration(1/400), imuDataCallback);

    ros::spin();
    
    return 0;
}

std::string get_imu_payload(std::string &fifo)
{
    std::string a, valid_data;
    std::string::size_type st, en;
    a.push_back(0x5a);
    a.push_back(0x5a);
    st = fifo.find(a);
    if(st == std::string::npos)
    {
        return "";
    }
    en = fifo.find(a, st+2);
    if(en == std::string::npos)
    {
        return "";
    }
    valid_data = fifo.substr(st, en-st);
    fifo = fifo.substr(en, fifo.size()-en);
    return valid_data;
}
float uint8_to_float(std::string data)
{
    uint8_t d[4] = {data[0], data[1], data[2], data[3]};
    float *f = (float *)d;
    return *f;
}
bool checkSum(std::string str)
{
    uint8_t sum = 0, i = 0;
    for(i=2; i<str.size()-1; i++)
    {
        sum += (uint8_t)str[i];
    }
    return sum == (uint8_t)str[58];
}

void imuDataCallback(const ros::TimerEvent&)//频率为200hz
{
    static std::string data_str_fifo;
    std::string data_str, read_data;
    static double tal, vl = 0; 
    tal += 1;
    static double wx_biasum=0;
    static double wy_biasum=0;
    static double wz_biasum=0;

    static double wx_bia=0;
    static double wy_bia=0;
    static double wz_bia=0;

    data_str_fifo += serial_imu_instruction.SerialRead();
    if(data_str_fifo.size() > 1000)
    {
        data_str_fifo.clear();
    }
    data_str = get_imu_payload(data_str_fifo);
    if(data_str.size()==59&&checkSum(data_str))
    {
        std_msgs::Float32MultiArray  msg;
        std_msgs::Float32 yaw_msg;
        std_msgs::Float32 pitch_msg;
        std_msgs::Float32 roll_msg;
        msg.data.push_back(uint8_to_float(data_str.substr(2, 4)));
        msg.data.push_back(uint8_to_float(data_str.substr(6, 4)));
        msg.data.push_back(uint8_to_float(data_str.substr(10, 4)));
        msg.data.push_back(uint8_to_float(data_str.substr(14, 4)));
        msg.data.push_back(uint8_to_float(data_str.substr(18, 4)));
        msg.data.push_back(uint8_to_float(data_str.substr(22, 4)));
        // ROS_INFO_STREAM(msg.data[0]<<","<<msg.data[1]<<","<<msg.data[2]<<","<<msg.data[3]<<","<<msg.data[4]<<","<<msg.data[5]);
        // imu_data_pub.publish(msg);
        vl += 1;
        double ax ,ay ,az ,wx,wy,wz;
        ax = msg.data[3];
        ay = msg.data[4];
        az = msg.data[5];
        wz = msg.data[2]*M_PI/180.0;
        wx = msg.data[0]*M_PI/180.0-wx_bia;
        wy = msg.data[1]*M_PI/180.0-wy_bia;
                
        pitch_msg.data=atan2(ax,sqrt(ay*ay+az*az));//根据加速度计算出pitch和roll
        roll_msg.data=atan2(ay,sqrt(ax*ax+az*az));

        if(abs(az) > 0.7)//机器人平放不算
        {
            yaw_msg.data = M_PI_2;
            imu_yaw_pub.publish(yaw_msg);//yaw实际上为世界坐标下的roll
            return;
        }else
        {
            double z = 0, out = 0;
            // ax = ax/sqrt(ax*ax+ay*ay);
            // ay = ay/sqrt(ax*ax+ay*ay);
            z = atan2(ax, ay) - M_PI_2;
            if(z < -M_PI)
            {
                z += 2*M_PI;
            }
            // yaw_msg.data = z;
            // imu_raw_pub.publish(yaw_msg);
            yaw_msg.data  = kf.step(wz, z, 1/200.0);//输出卡尔曼滤波后的yaw数据
            
            imu_yaw_pub.publish(yaw_msg);
        }

     //以下为江扬新改写代码，增加imu标准格式的发送
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu";
        //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
        tf2::Quaternion qtn;//创建四元数对象
        qtn.setRPY(roll_msg.data,pitch_msg.data,yaw_msg.data);//单位为弧度

        imu_data.orientation.x = qtn.getX();
        imu_data.orientation.y = qtn.getY(); 
        imu_data.orientation.z = qtn.getZ();
        imu_data.orientation.w = qtn.getW(); 
        //线加速度
        imu_data.linear_acceleration.x = ax; 
        imu_data.linear_acceleration.y = ay;
        imu_data.linear_acceleration.z = az;
        //角速度
        imu_data.angular_velocity.x = wx; 
        imu_data.angular_velocity.y = wy; 
        imu_data.angular_velocity.z = wz;

        IMU_pub.publish(imu_data);   
    }
    if(tal >= 100)
    {
        // ROS_INFO_STREAM("imu packet loss rate:"<<(1-vl/tal)*100<<"%");
        tal = vl = 0;
    }
}

void Serial_IMU::SerialInit(std::string port)
{
     //打开串口，波特率115200，串口为port
	try 
	{
		ros_ser_.setPort(port);             //串口端口设置
		ros_ser_.setBaudrate(230400);       //串口波特率设置
		serial::Timeout to = serial::Timeout(5, 5, 0, 5, 0);       //串口延时设置
		ros_ser_.setTimeout(to);
        ros_ser_.setStopbits(serial::stopbits_one);                     //一个停止位
        
		ros_ser_.open();        //打开串口
        
	}
	catch (serial::IOException& e) 
	{
        
		ROS_ERROR_STREAM("Unable to open port ");
		return ;
	}

    serial::Timeout a = ros_ser_.getTimeout();
	
	//输出提示消息
	if(ros_ser_.isOpen()) 
	{
        ROS_INFO_STREAM("Serial Port opened, waiting for flush input.");
        ros::Duration(0.01).sleep();
        ROS_INFO_STREAM("flush input.");
        ros_ser_.flushInput();
    } 
	else 
	{
        return;
    }
}

std::string Serial_IMU::SerialRead()
{
    std::string data_str;
    size_t p = ros_ser_.available();

    //串口数据读取
    if (p != 0)
    {
        data_str = ros_ser_.read(p);
    } 
    return data_str;
}

kalman_filter::kalman_filter()
{
    x = Eigen::Vector2d(0, 0);
    P_last = Eigen::MatrixXd::Identity(2, 2)*0.1;
    Q = Eigen::MatrixXd::Identity(2, 2)*1e-4;
    H = Eigen::MatrixXd::Identity(2, 2);
    R = Eigen::MatrixXd::Identity(2, 2)*1;
}

float kalman_filter::step(double w, double z, double dt)
{
    A << 1 ,dt ,0, 1;
    x = A*x;
    P_pre = A*P_last*A.transpose() + Q;
    K = P_pre*(P_pre + R).inverse();
    Eigen::Vector2d z_vec = Eigen::Vector2d(z, w);
    x = x + K*(z_vec - x);
    P_last = P_pre - K*P_pre;
    return x(0);
}

void usbreset()
{
    // const char *filename = "/dev/bus/usb/001/005";
    const char *filename = "/dev/imu_serial";
    int fd;
    int rc;
    // std::string ts = "/dev/bus/usb/001/005";
 
    // filename = ts.c_str();
 
    fd = open(filename, O_WRONLY);
    if (fd < 0) {
        perror("Error opening output file");
        return;
    }
 
    printf("Resetting USB device %s\n", filename);
    rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        perror("Error in ioctl");
        return;
    }
    printf("Reset successful\n");
 
    close(fd);

    const char* device = "/dev/imu_serial";
    int result = chmod(device, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
    if (result == -1) {
        std::cerr << "Failed to chmod " << device << std::endl;
        return;
    }
    std::cout << "Chmod 777 to "<<device<<" successfully" << std::endl;
    return;
}

// void reset_serial_port(const string& port) {
//     int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
//     if (fd == -1) {
//         cout << "Failed to open serial port " << port << endl;
//         return;
//     }

//     // Set the serial port settings
//     struct termios tty;
//     memset(&tty, 0, sizeof(tty));
//     if (tcgetattr(fd, &tty) != 0) {
//         cout << "Failed to get serial port attributes" << endl;
//         close(fd);
//         return;
//     }

//     cfsetospeed(&tty, B230400);
//     cfsetispeed(&tty, B230400);

//     tty.c_cflag &= ~PARENB;
//     tty.c_cflag &= ~CSTOPB;
//     tty.c_cflag &= ~CSIZE;
//     tty.c_cflag |= CS8;
//     tty.c_cflag &= ~CRTSCTS;
//     tty.c_cflag |= CREAD | CLOCAL;

//     tty.c_iflag &= ~(IXON | IXOFF | IXANY);
//     tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

//     tty.c_oflag &= ~OPOST;

//     tty.c_cc[VMIN] = 0;
//     tty.c_cc[VTIME] = 10;

//     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
//         cout << "Failed to set serial port attributes" << endl;
//         close(fd);
//         return;
//     }

//     // Clear the input buffer
//     if (tcflush(fd, TCIFLUSH) != 0) {
//         cout << "Failed to flush serial port input buffer" << endl;
//         close(fd);
//         return;
//     }

//     // Read some data to move the read pointer
//     char buf[1000];
//     ssize_t n = read(fd, buf, sizeof(buf));
//     cout <<n<< " bytes read 1" << endl;
//     // Wait for some data to be received
//     // std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     // n = read(fd, buf, sizeof(buf));
//     // cout <<n<< " bytes read 2" << endl;

//     // Close and reopen the serial port
//     close(fd);
//     fd = open(port.c_str(), O_RDWR | O_NOCTTY);
//     if (fd == -1) {
//         cout << "Failed to reopen serial port " << port << endl;
//         return;
//     }

//     close(fd);
// }