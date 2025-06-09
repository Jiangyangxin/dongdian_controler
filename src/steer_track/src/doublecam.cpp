#include "ros/ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h" 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h" 
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"  
#include "geometry_msgs/PoseStamped.h"  
#include "steer_track/kalman_filter.h"
#include <stdio.h>
#include <iostream>

//双观测相机代码
//已知 hikrobot_camera1->apriltag  hikrobot_camera2->apriltag_cam2 hikrobot_camera1->hikrobot_camera2
//记得去修改apriltag_ros 中的config
int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "doublecam");
    //创建节点句柄
    ros::NodeHandle nh;
    ros::Publisher pose_target1_publisher= nh.advertise<geometry_msgs::PoseStamped>("apriltag_tf_pose",1);
    geometry_msgs::PoseStamped pose_target1;

    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer1(ros::Duration(0.5)); 
    tf2_ros::TransformListener tfListener1(buffer1);
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer2(ros::Duration(0.5)); 
    tf2_ros::TransformListener tfListener2(buffer2);

    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer3(ros::Duration(0.5)); 
    tf2_ros::TransformListener tfListener3(buffer3);

    // 创建 TF 广播对象       
    tf2_ros::TransformBroadcaster apriltag_to_cam1; 
    // tf2::StampedTransform trans;          
    // organize the data  
    geometry_msgs::TransformStamped T_cam1_apriltag1;  
    geometry_msgs::TransformStamped T_cam2_apriltag1;   
    geometry_msgs::TransformStamped T_cam1_apriltag_final;  
      
    // vins.header.frame_id = "hikrobot_camera";  
    // vins.header.stamp = ros::Time::now();  
    // vins.child_frame_id = "world";  

    T_cam1_apriltag_final.header.frame_id = "hikrobot_camera1";  
    T_cam1_apriltag_final.header.stamp = ros::Time::now();  
    T_cam1_apriltag_final.child_frame_id = "apriltag"; 
    // set translation 
    T_cam1_apriltag_final.transform.translation.x = 0;  
    T_cam1_apriltag_final.transform.translation.y = 0;  
    T_cam1_apriltag_final.transform.translation.z = 1;  
    tf2::Quaternion qtn0;  
    qtn0.setRPY(0,0,0);  
    T_cam1_apriltag_final.transform.rotation.w = qtn0.getW();  
    T_cam1_apriltag_final.transform.rotation.x = qtn0.getX();  
    T_cam1_apriltag_final.transform.rotation.y = qtn0.getY();  
    T_cam1_apriltag_final.transform.rotation.z = qtn0.getZ();   
    T_cam1_apriltag_final.header.stamp = ros::Time::now(); 

    T_cam1_apriltag1.header.frame_id = "hikrobot_camera1";  //initial
    T_cam1_apriltag1.header.stamp = ros::Time::now();  
    T_cam1_apriltag1.child_frame_id = "apriltag_cam1"; 
    T_cam1_apriltag1.transform=T_cam1_apriltag_final.transform;



    int cam2_lock=0;
    int cam1_lock=0;
    ros::Rate rate(60);  
    while(ros::ok())
    {  

    try{
        if(buffer1.canTransform("hikrobot_camera1","apriltag_cam1",ros::Time(0),NULL)) //等待apriltag数据的发布
        {
            T_cam1_apriltag1 = buffer1.lookupTransform("hikrobot_camera1","apriltag_cam1",ros::Time(0));
            cam1_lock=1;
        }
        else if(!buffer1.canTransform("hikrobot_camera1","apriltag_cam1",ros::Time(0),NULL))
        {
            cam1_lock=0;
        }
        if(buffer1.canTransform("hikrobot_camera1","apriltag_cam2",ros::Time(0),NULL)) //等待apriltag数据的发布
        {
            // T_cam2_apriltag1 = buffer1.lookupTransform("hikrobot_camera2","apriltag_cam2",ros::Time(0));//这个没啥用
            T_cam1_apriltag1 = buffer1.lookupTransform("hikrobot_camera1","apriltag_cam2",ros::Time(0));
            cam2_lock=1;
        }
        else if(!buffer1.canTransform("hikrobot_camera1","apriltag_cam2",ros::Time(0),NULL))
        {
            cam2_lock=0;
        }
        // cout<<"cam1_lock: "<<cam1_lock<<endl;
        // cout<<"cam2_lock: "<<cam2_lock<<endl;

        if(cam1_lock==1 || cam2_lock==1) //有任意一个相机看到了apriltag
        {
            if(cam1_lock==0 && cam2_lock==1) //cam2 看到了apriltag  hikrobot_camera1->hikrobot_camera2->apriltag_cam2
            {
                T_cam1_apriltag_final.transform=T_cam1_apriltag1.transform;
            }
            else if(cam1_lock==1 && cam2_lock==0)  //hikrobot_camera1->apriltag_cam1
            {
                T_cam1_apriltag_final.transform=T_cam1_apriltag1.transform;
            }
            else
            {
                T_cam1_apriltag_final.transform=T_cam1_apriltag1.transform;
            }
        //加入PoseStamped消息
            pose_target1.header.stamp=ros::Time::now(); 
            pose_target1.header.frame_id= "apriltag_tf_pose";  ///话题名称为 /robot_1/apriltag_tf_pose
            //pose_target1.header.frame_id= "image->header.frame_id"; 
            pose_target1.pose.position.x=T_cam1_apriltag_final.transform.translation.x;
            pose_target1.pose.position.y=T_cam1_apriltag_final.transform.translation.y;
            pose_target1.pose.position.z=T_cam1_apriltag_final.transform.translation.z;
            pose_target1.pose.orientation=T_cam1_apriltag_final.transform.rotation;



            T_cam1_apriltag_final.header.stamp = ros::Time::now(); 

            if((ros::Time::now()-T_cam1_apriltag1.header.stamp).toSec() <0.5)
            {
                pose_target1_publisher.publish(pose_target1);
                apriltag_to_cam1.sendTransform(T_cam1_apriltag_final); //发布hikrobot_camera1->apriltag
            }


        }
        else //没收到任何apriltag消息，不发布消息
        {
            cam1_lock=0;
            cam2_lock=0;
        }

        cam1_lock=0;
        cam2_lock=0;

        
    }
        catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        ROS_INFO("异常信息:%s",e.what());

    }

        rate.sleep();  
        ros::spinOnce();    
    } 
    
    return 0;
}