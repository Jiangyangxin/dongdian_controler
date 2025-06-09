#include "ros/ros.h"
#include "tf2_ros/buffer.h" 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h" 
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"  
#include "steer_track/kalman_filter.h"
#include <stdio.h>
#include <iostream>

//测试用代码，平时不调用
int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "tf_test");
    //创建节点句柄
    ros::NodeHandle nh;
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer0; 
    tf2_ros::TransformListener tfListener0(buffer0);
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer1; 
    tf2_ros::TransformListener tfListener1(buffer1);
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer2; 
    tf2_ros::TransformListener tfListener2(buffer2);

    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer3; 
    tf2_ros::TransformListener tfListener3(buffer3);

     // 创建 TF 订阅对象    
    tf2_ros::Buffer buffertest; 
    tf2_ros::TransformListener tfListener_test(buffertest);   
    // 创建 TF 广播对象       
    tf2_ros::TransformBroadcaster buffer0_to_buffer1; 
    tf2_ros::TransformBroadcaster buffer2_to_buffer3; 
    tf2_ros::TransformBroadcaster buffer0_to_buffer3; 
    tf2_ros::TransformBroadcaster buffer0_to_buffer4;     
    tf2_ros::TransformBroadcaster buffer1_to_buffer2;
    tf2_ros::TransformBroadcaster buffer4_to_buffer3;                  
    // organize the data  
    geometry_msgs::TransformStamped T_0_1;  
    geometry_msgs::TransformStamped T_0_4;     
    geometry_msgs::TransformStamped T_2_3;  
    geometry_msgs::TransformStamped T_1_2; 
    geometry_msgs::TransformStamped T_0_3;  
    geometry_msgs::TransformStamped T_4_3;          
    // vins.header.frame_id = "hikrobot_camera";  
    // vins.header.stamp = ros::Time::now();  
    // vins.child_frame_id = "world";  

    T_0_1.header.frame_id = "base0";  
    T_0_1.header.stamp = ros::Time::now();  
    T_0_1.child_frame_id = "base1"; 
    // set translation 
    T_0_1.transform.translation.x = -1;  
    T_0_1.transform.translation.y = 1;  
    T_0_1.transform.translation.z = 0;  
    tf2::Quaternion qtn0;  
    qtn0.setRPY(0,0,0);  
    T_0_1.transform.rotation.w = qtn0.getW();  
    T_0_1.transform.rotation.x = qtn0.getX();  
    T_0_1.transform.rotation.y = qtn0.getY();  
    T_0_1.transform.rotation.z = qtn0.getZ();   
    T_0_1.header.stamp = ros::Time::now(); 
    buffer0_to_buffer1.sendTransform(T_0_1); 


    T_2_3.header.frame_id = "base2";  
    T_2_3.header.stamp = ros::Time::now();  
    T_2_3.child_frame_id = "base3"; 
    // set translation 
    T_2_3.transform.translation.x = 1;  
    T_2_3.transform.translation.y = 1;  
    T_2_3.transform.translation.z = 0;  
    tf2::Quaternion qtn1;  
    qtn1.setRPY(0,0,0);  
    T_2_3.transform.rotation.w = qtn1.getW();  
    T_2_3.transform.rotation.x = qtn1.getX();  
    T_2_3.transform.rotation.y = qtn1.getY();  
    T_2_3.transform.rotation.z = qtn1.getZ();     
    T_2_3.header.stamp = ros::Time::now(); 
    buffer2_to_buffer3.sendTransform(T_2_3); 


    T_0_4.header.frame_id = "base0";  
    T_0_4.header.stamp = ros::Time::now();  
    T_0_4.child_frame_id = "base4"; 
    // set translation 
    T_0_4.transform.translation.x = 0;  
    T_0_4.transform.translation.y = 4;  
    T_0_4.transform.translation.z = 0;  
    tf2::Quaternion qtn4;  
    qtn4.setRPY(0,0,0);  
    T_0_4.transform.rotation.w = qtn4.getW();  
    T_0_4.transform.rotation.x = qtn4.getX();  
    T_0_4.transform.rotation.y = qtn4.getY();  
    T_0_4.transform.rotation.z = qtn4.getZ();      
    T_0_4.header.stamp = ros::Time::now(); 
    buffer0_to_buffer3.sendTransform(T_0_4); 


    T_1_2.header.frame_id = "base1";  
    T_1_2.header.stamp = ros::Time::now();  
    T_1_2.child_frame_id = "base2"; 
    // set translation 
    T_1_2.transform.translation.x = 0;  
    T_1_2.transform.translation.y = 1;  
    T_1_2.transform.translation.z = 0;  
    tf2::Quaternion qtn3;  
    qtn3.setRPY(0,0,0);  
    T_1_2.transform.rotation.w = qtn3.getW();  
    T_1_2.transform.rotation.x = qtn3.getX();  
    T_1_2.transform.rotation.y = qtn3.getY();  
    T_1_2.transform.rotation.z = qtn3.getZ();      
    T_1_2.header.stamp = ros::Time::now(); 
    buffer1_to_buffer2.sendTransform(T_1_2); 

    T_4_3.header.frame_id = "base4";  
    T_4_3.header.stamp = ros::Time::now();  
    T_4_3.child_frame_id = "base3"; 
    // set translation 
    T_4_3.transform.translation.x = 0;  
    T_4_3.transform.translation.y = 1;  
    T_4_3.transform.translation.z = 0;  
    tf2::Quaternion qtn5;  
    qtn5.setRPY(0,0,0);  
    T_4_3.transform.rotation.w = qtn5.getW();  
    T_4_3.transform.rotation.x = qtn5.getX();  
    T_4_3.transform.rotation.y = qtn5.getY();  
    T_4_3.transform.rotation.z = qtn5.getZ();      
    T_4_3.header.stamp = ros::Time::now(); 
    // buffer4_to_buffer3.sendTransform(T_4_3); 

    while(ros::ok())
    {  

    try{
        T_0_1.header.stamp = ros::Time::now(); 
        T_2_3.header.stamp = ros::Time::now(); 
        // T_0_3.header.stamp = ros::Time::now(); 
        T_1_2.header.stamp = ros::Time::now(); 
        T_4_3.header.stamp = ros::Time::now(); 
        T_0_4.header.stamp = ros::Time::now();

        buffer0_to_buffer1.sendTransform(T_0_1); 
        buffer2_to_buffer3.sendTransform(T_2_3); 
        // buffer0_to_buffer3.sendTransform(T_0_3); 
        buffer1_to_buffer2.sendTransform(T_1_2); 
        //buffer4_to_buffer3.sendTransform(T_4_3); 
        buffer0_to_buffer4.sendTransform(T_0_4);

        T_0_3 = buffertest.lookupTransform("base3","base1",ros::Time(0));


        continue;
        // ros::Duration(1).sleep();
        
    }
        catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        ROS_INFO("异常信息:%s",e.what());

    }
    
        ros::spinOnce();
    } 
    
    return 0;
}