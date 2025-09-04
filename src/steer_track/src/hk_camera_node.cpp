#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "steer_track/hikrobot_camera.h"
#include "tf2_ros/buffer.h" 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h" 
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h" 
#include<Eigen/Dense>
// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
//相机的IP设置在hikrobot_camera.h中定义
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;

tf2_ros::Buffer buffer; 

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    cv::Mat src2;
    cv::Mat src3;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    ros::NodeHandle nh_private("~");
    string hk_name;
    const int hk_loop_rate =20;
    // 如果参数服务器中参数"hk_name"存在， 把"hk_name"的值传递给变量hk_name；如果"hk_name"不存在，就传递默认值"hk_camere1"
    nh_private.param<std::string>("hk_name",hk_name,"hk_camere1");
    
    if (hk_name=="hk_camere1")
    {
        cout<<"hk_name:"<<hk_name<<endl;
        // long target_ip1=3232235586; //表示IP地址为 192.168.0.66
        long target_ip1=3232235842; //表示IP地址为 192.168.1.66
        //IP地址的十进制表示，分别由四段八位的二进制数字拼凑而成，例如 十进制192的二进制是 11000000 
        //192.168.0.66转化为二进制并拼凑，则可得到 11000000 10101000 00000000 01000010 ，最终的十进制数字，就是3232235586
        // camera::Camera MVS_cap(hikrobot_camera,target_ip1,2);//具体看Camera类的构造函数 0为RGB8Packed，1为Mono8，2为BayerRG8
        camera::Camera MVS_cap(hikrobot_camera,target_ip1,1);//具体看Camera类的构造函数 0为RGB8Packed，1为Mono8，2为BayerRG8
        //********** rosnode init **********/
        image_transport::ImageTransport main_cam_image(hikrobot_camera);
        image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera1/image", 1000);
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo camera_info_msg;
        cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
        // cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 MONO8 
        cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;  //相机2

   //********** 60 Hz        **********/
    ros::Rate loop_rate(hk_loop_rate); //2hz test
    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();
        MVS_cap.ReadImg(src);//获取··相机图像
        if (src.empty()) //获取图像失败则等待下一帧
        {
            cout<<"src empty"<<endl;
            continue;
        }

#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else

        cv_ptr->image = src;

#endif
            // std::vector<double> D={-0.05920878623760449, 0.08264318027241738, 0.004728377611474319, 0.004268759332907533};
            // //-0.077658,0.238342,-0.002914,-0.001424,-0.571738
            // boost::array<double, 9UL> 
            // K={1760.2201,0,760.2998,
            // 0,1772.4798,615.5398,
            // 0,0,1};
            // boost::array<double, 12UL> 
            // P={1760.2201,0,760.2998,0,
            // 0,1772.4798,615.5398,0,
            // 0,0,1,0};

            std::vector<double> D={-0.06600557,0.10506498 ,-0.00051733 ,-0.00496308};
            //-0.077658,0.238342,-0.002914,-0.001424,-0.571738
            boost::array<double, 9UL> 
            K={1712.05700519,0,653.91200321,
            0,1709.23560429,545.79754112,
            0,0,1};
            boost::array<double, 12UL> 
            P={1712.05700519,0,653.91200321,0,
            0,1709.23560429,545.79754112,0,
            0,0,1,0};            

            image_msg = *(cv_ptr->toImageMsg());
            image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
            image_msg.header.frame_id = "hikrobot_camera1";
            camera_info_msg.header.frame_id = image_msg.header.frame_id;
            camera_info_msg.header.stamp = image_msg.header.stamp;
            camera_info_msg.height=1080;
            camera_info_msg.width=1440;
            camera_info_msg.D=D;
            camera_info_msg.K=K;
            camera_info_msg.P=P;
            image_pub.publish(image_msg, camera_info_msg);//发布图像消息
            // cout<<"src OK"<<endl;
        }
    }
    


    else if(hk_name=="hk_camere2")
    {
        cout<<"hk_name:"<<hk_name<<endl;
        tf2_ros::TransformBroadcaster static_T_C1_C3_pub; //两个工装相机之间的位姿关系
        geometry_msgs::TransformStamped T_C1_C3;  //两个工装相机之间的位姿关系
        //两个工装相机之间的位姿关系，需要进行双目相机标定，把结果输入进来即可
        T_C1_C3.header.frame_id = "hikrobot_camera1";  
        T_C1_C3.header.stamp = ros::Time::now();  
        T_C1_C3.child_frame_id = "hikrobot_camera2"; 
        // set translation 
        T_C1_C3.transform.translation.x = 2.31994691   ;  //代表child_frame_id相对于frame_id的偏移量 1.60980376
        T_C1_C3.transform.translation.y =  0.03517174;  //-0.0869996597
        T_C1_C3.transform.translation.z = -0.20718683;  //-0.154809312
        // set rotation  
        Eigen::Matrix3d Rotation_matrix2;
        Rotation_matrix2 << 
        0.99862088, -0.01783898, -0.04937728,
        0.01929925,  0.99938561,  0.02925663,
        0.04882503, -0.03016922,  0.99835161;
        Eigen::Quaterniond q2 = Eigen::Quaterniond(Rotation_matrix2); 
        T_C1_C3.transform.rotation.w = q2.w();  
        T_C1_C3.transform.rotation.x = q2.x();   
        T_C1_C3.transform.rotation.y = q2.y();   
        T_C1_C3.transform.rotation.z = q2.z();  

        // long target_ip2=3232235587; //表示IP地址为 192.168.0.67 
        long target_ip2=3232235843; //表示IP地址为 192.168.1.67
        camera::Camera MVS_cap2(hikrobot_camera,target_ip2,1);//相机2 //具体看Camera类的构造函数 0为RGB8Packed，1为Mono8，2为BayerRG8
        image_transport::ImageTransport main_cam_image2(hikrobot_camera);
        image_transport::CameraPublisher image_pub2 = main_cam_image2.advertiseCamera("/hikrobot_camera2/image", 1000);//相机2
        sensor_msgs::Image image_msg2;//相机2
        sensor_msgs::CameraInfo camera_info_msg2;//相机2

        cv_bridge::CvImagePtr cv_ptr2 = boost::make_shared<cv_bridge::CvImage>();
        cv_ptr2->encoding = sensor_msgs::image_encodings::MONO8;  //相机2



        //********** 60 Hz        **********/
        ros::Rate loop_rate(hk_loop_rate); //2hz test

        while (ros::ok())
        {
            loop_rate.sleep();
            ros::spinOnce();
            MVS_cap2.ReadImg(src2);
            if (src2.empty()) //获取图像失败则等待下一帧
            {
                cout<<"src2 empty"<<endl;
                continue;
            }
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
            cv_ptr2->image = src2;
#endif

            std::vector<double> D2={-0.0585698 ,0.10095478 ,-0.00030448  ,0.00006762}; //相机2
            //-0.077658,0.238342,-0.002914,-0.001424,-0.571738
            boost::array<double, 9UL>
            K2={1772.0061507,0,701.02183158,
            0,1772.1639867,550.34122185,
            0,0,1};
            boost::array<double, 12UL> 
            P2={1772.0061507,0,701.02183158,0,
            0,1772.1639867,550.34122185,0,
            0,0,1,0};

            image_msg2 = *(cv_ptr2->toImageMsg());//相机2
            image_msg2.header.stamp = ros::Time::now();
            image_msg2.header.frame_id = "hikrobot_camera2";
            camera_info_msg2.header.frame_id = image_msg2.header.frame_id;
            camera_info_msg2.header.stamp = image_msg2.header.stamp;
            camera_info_msg2.height=1080;
            camera_info_msg2.width=1440;
            camera_info_msg2.D=D2;
            camera_info_msg2.K=K2;
            camera_info_msg2.P=P2;
            image_pub2.publish(image_msg2, camera_info_msg2);
            // cout<<"src2 OK"<<endl;
            // if(!buffer.canTransform("hikrobot_camera2","apriltag",ros::Time(0),NULL)) //等待apriltag数据的发布
            T_C1_C3.header.stamp = ros::Time::now(); 
            static_T_C1_C3_pub.sendTransform(T_C1_C3);
        }


    }

    if (hk_name=="hk_camere3")
    {
        cout<<"hk_name:"<<hk_name<<endl;
        // int target_ip3=3232235588; //表示IP地址为 192.168.0.68
        long target_ip3=3232235844; //表示IP地址为 192.168.1.68
        //IP地址的十进制表示，分别由四段八位的二进制数字拼凑而成，例如 十进制192的二进制是 11000000 
        //192.168.0.66转化为二进制并拼凑，则可得到 11000000 10101000 00000000 01000010 ，最终的十进制数字，就是3232235586
        camera::Camera MVS_cap(hikrobot_camera,target_ip3,1);//具体看Camera类的构造函数 0为RGB8Packed，1为Mono8，2为BayerRG8
        //********** rosnode init **********/
        image_transport::ImageTransport main_cam_image(hikrobot_camera);
        image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera3/image", 1000);
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo camera_info_msg;
        cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
        cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;  // MONO8


   //********** 60 Hz        **********/
    ros::Rate loop_rate(hk_loop_rate); //2hz test
    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();
        MVS_cap.ReadImg(src3);//获取··相机图像
         

        if (src3.empty()) //获取图像失败则等待下一帧
        {
            cout<<"src3 empty"<<endl;
            continue;
        }

#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else

        cv_ptr->image = src3;



        // static const std::string OPENCV_WINDOW = "Image window";
        // cv::namedWindow(OPENCV_WINDOW);
        // cv::imshow(OPENCV_WINDOW, src3);
        // cv::waitKey(0.1);

#endif
            std::vector<double> D={-0.05920878623760449, 0.08264318027241738, 0.004728377611474319, 0.004268759332907533};
            //-0.077658,0.238342,-0.002914,-0.001424,-0.571738
            boost::array<double, 9UL> 
            K={1760.2201,0,760.2998,
            0,1772.4798,615.5398,
            0,0,1};
            boost::array<double, 12UL> 
            P={1760.2201,0,760.2998,0,
            0,1772.4798,615.5398,0,
            0,0,1,0};

            image_msg = *(cv_ptr->toImageMsg());
            image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
            image_msg.header.frame_id = "hikrobot_camera3";
            camera_info_msg.header.frame_id = image_msg.header.frame_id;
            camera_info_msg.header.stamp = image_msg.header.stamp;
            camera_info_msg.height=1080;
            camera_info_msg.width=1440;
            camera_info_msg.D=D;
            camera_info_msg.K=K;
            camera_info_msg.P=P;
            image_pub.publish(image_msg, camera_info_msg);//发布图像消息
            // cout<<"src OK"<<endl;
        }
    }

    return 0;
}