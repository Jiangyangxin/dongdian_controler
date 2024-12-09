#include "ros/ros.h"
#include "tf2_ros/buffer.h" 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h" 
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"  
#include "steer_track/kalman_filter.h"
#include <stdio.h>
#include <iostream>
#include <tf/tf.h>
#define gravity 9.8
#define init_NUM 10
int coordinate_init=init_NUM;//切换工位后的初始化标识符，后续会使用到
float body_last_x=0;
float body_last_y=0;
float vx_body=0;
float vy_body=0;
int First_init=0;
std::string tf_prefix="";
float VIO_MIX_bias[3]={0,0,0};

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "apriltag_to_vins_node");
    //创建节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle n("~");
    n.param("tf_prefix", tf_prefix, (std::string)"robot_1"); //获取tf_prefix
    std::cout<<"tf_prefix:"<<tf_prefix<<std::endl;
    //创建最终的位姿发布
    ros::Publisher final_pose_publisher= nh.advertise<geometry_msgs::PoseStamped>("final_pose",1);
    // ros::Publisher apriltag1_pose_publisher= nh.advertise<geometry_msgs::PoseStamped>("/apriltag1_pose",1); //已修改apriltag源码，无需此处再发.

    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener tfListener(buffer);
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer_c2_vins; 
    tf2_ros::TransformListener tfListener_c2_vins(buffer_c2_vins);
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer_c2_body; 
    tf2_ros::TransformListener tfListener_c2_body(buffer_c2_body);    
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer_c1_vins; 
    tf2_ros::TransformListener tfListener_c1_vins(buffer_c1_vins);
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer_a1_body; 
    tf2_ros::TransformListener tfListener_a1_body(buffer_a1_body);    
    // 创建 TF 订阅对象    
    tf2_ros::Buffer buffer2; 
    tf2_ros::TransformListener tfListener2(buffer2);
    // 创建 TF 广播对象       
    //tf2_ros::TransformBroadcaster apriltag_to_vins_pub; 
    tf2_ros::TransformBroadcaster T_A1_C2_pub; //只在初始化的时候发，初始化完成之后断掉 
    tf2_ros::TransformBroadcaster T_final_pub; //融合定位结果   
    tf2_ros::TransformBroadcaster T_C2_body_pub; //只在初始化的时候发，初始化完成之后断掉   
    tf2_ros::StaticTransformBroadcaster static_T_A1_C2_vins_pub; //固定发送 
    tf2_ros::StaticTransformBroadcaster static_T_C2_vins_pub; //固定发送       
    tf2_ros::StaticTransformBroadcaster static_T_C1_vins_pub; //固定发送
    tf2_ros::StaticTransformBroadcaster static_T_C1_A1_vins_pub; //固定发送   
    tf2_ros::StaticTransformBroadcaster T_vins_world_pub; //固定发送    
    tf2_ros::StaticTransformBroadcaster static_T_A1_body2_pub; //固定发送        

    // organize the data  
    geometry_msgs::TransformStamped T_A1_C2;
    geometry_msgs::TransformStamped T_A1_C2_2;      
    geometry_msgs::TransformStamped T_C2_body;
    geometry_msgs::TransformStamped T_C2_body_2;           
    geometry_msgs::TransformStamped T_C1_A1;
    geometry_msgs::TransformStamped T_C1_A1_2;  

    geometry_msgs::TransformStamped T_A1_body; 
    geometry_msgs::TransformStamped static_T_A1_body;               
    geometry_msgs::TransformStamped T_C2_Vins;
    geometry_msgs::TransformStamped T_C2_Vins_2;    
    geometry_msgs::TransformStamped static_T_C1_vins;
    geometry_msgs::TransformStamped T_C1_vins;
    geometry_msgs::TransformStamped T_vins_world; 
    geometry_msgs::TransformStamped T_final; 
    geometry_msgs::PoseStamped final_pose;

//需要运行biaoding.py，把结果T的值给输入进来
    T_A1_C2.header.frame_id = tf_prefix+"apriltag";  
    T_A1_C2.header.stamp = ros::Time::now();  
    T_A1_C2.child_frame_id = tf_prefix+"camera_apriltag"; 
    // set translation 
    T_A1_C2.transform.translation.x = -0.19438804;  //代表child_frame_id相对于frame_id的偏移量
    T_A1_C2.transform.translation.y = 0.15247693;  
    T_A1_C2.transform.translation.z = -0.02340984;  
    // set rotation  
    tf2::Quaternion qtn;  
    Eigen::Matrix3d Rotation_matrix;
    Rotation_matrix << -0.85467516,-0.51867734,-0.0224542,
    -0.51819623,0.85492354,-0.02405013,
    0.03167088,-0.00891937,-0.99945855 ;
    Eigen::Quaterniond q = Eigen::Quaterniond(Rotation_matrix); 
    T_A1_C2.transform.rotation.w = q.w();  
    T_A1_C2.transform.rotation.x = q.x();   
    T_A1_C2.transform.rotation.y = q.y();   
    T_A1_C2.transform.rotation.z = q.z();   

//机器人本体的相机与IMU的位姿转换关系，需要在运行 vins之后获取
    T_C2_body_2.header.frame_id=tf_prefix+"camera_apriltag"; 
    T_C2_body_2.child_frame_id=tf_prefix+"body_apriltag";
    T_C2_body_2.header.stamp = ros::Time::now(); 

    //kalman_filter
    int statesize=4;
    int measize=2;
    int usize=2;
    Kalman ka(statesize,measize,usize);//4,2,2
    Eigen::VectorXd x;
    Eigen::MatrixXd P0;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd H;
    Eigen::VectorXd z0;
    Eigen::VectorXd u_v;


    ros::Duration(1).sleep();
    // publish data  
    ros::Rate rate(60);  
    while(ros::ok())
    {  

    try{
       if(coordinate_init>0)//切换工位后，完成apriltag与vins的坐标系原点对齐
        {
            First_init=0;
            //cout<<coordinate_init<<endl;  
            if(!buffer_c2_vins.canTransform(tf_prefix+"camera",tf_prefix+"world",ros::Time(0),NULL)) //等待VIO数据的发布
            {
            cout<<"can not receive VIO's TF!"<<endl;
            coordinate_init=init_NUM;
            ros::Duration(1).sleep();
            continue;
            }

            if(!buffer.canTransform("hikrobot_camera",tf_prefix+"apriltag",ros::Time(0),NULL)) //等待apriltag数据的发布
            {
            cout<<"can not receive apriltag's TF!"<<endl;
            coordinate_init=init_NUM;
            ros::Duration(1).sleep();
            continue;
            }

            T_C2_Vins = buffer_c2_vins.lookupTransform(tf_prefix+"camera",tf_prefix+"world",ros::Time(0));
            T_C2_body = buffer_c2_body.lookupTransform(tf_prefix+"camera",tf_prefix+"body",ros::Time(0));
        
            T_A1_C2.header.stamp = ros::Time::now(); 
            T_A1_C2_pub.sendTransform(T_A1_C2);//"apriltag"->"camera_apriltag"
            
            T_C2_body_2.transform=T_C2_body.transform; 
            T_C2_body_2.header.stamp = ros::Time::now(); 
            T_C2_body_pub.sendTransform(T_C2_body_2);//"camera_apriltag"->"body_apriltag"


            
            T_C1_A1 = buffer.lookupTransform("hikrobot_camera",tf_prefix+"apriltag",ros::Time(0));
            T_C1_A1_2.header.frame_id= "hikrobot_camera";
            T_C1_A1_2.child_frame_id= tf_prefix+"apriltag2"; 
            T_C1_A1_2.header.stamp= ros::Time::now(); 
            T_C1_A1_2.transform = T_C1_A1.transform;
            static_T_C1_A1_vins_pub.sendTransform(T_C1_A1_2);

            T_A1_C2_2.header.frame_id=tf_prefix+"apriltag2";
            T_A1_C2_2.child_frame_id= tf_prefix+"camera2"; 
            T_A1_C2_2.header.stamp= ros::Time::now(); 
            T_A1_C2_2.transform = T_A1_C2.transform;
            static_T_A1_C2_vins_pub.sendTransform(T_A1_C2_2);

            T_C2_Vins_2.header.frame_id= tf_prefix+"camera2";
            T_C2_Vins_2.child_frame_id=tf_prefix+"world"; 
            T_C2_Vins_2.header.stamp= ros::Time::now(); 
            T_C2_Vins_2.transform = T_C2_Vins.transform;
            static_T_C2_vins_pub.sendTransform(T_C2_Vins_2);



            //kalman_filter init
            x.resize(statesize);
            x(0)=T_C1_A1.transform.translation.x;
            x(1)=0;
            x(2)=T_C1_A1.transform.translation.y;
            x(3)=0;

            P0.resize(statesize,statesize);       
            P0.setIdentity();

            Q.resize(statesize,statesize);
            // Q.setZero();
            Q << 0.5, 0, 0, 0,
            0, 0.5, 0, 0,
            0, 0, 0.5, 0,
            0, 0, 0, 0.5;

            R.resize(measize,measize);
            R << 1, 0,  
                0, 1; 


            A.resize(statesize,statesize);
            A << 1, 0, 0, 0,  
            0, 1, 0, 0,  
            0, 0, 1, 0,  
            0, 0, 0, 1; 
            //对角线为1的矩阵
            // 状态维度：4 (x, vx, y, vy)

            B.resize(statesize,usize);
            // B << 1, 0, 
            //     0, 0, 
            //     0, 1,
            //     0, 0;
            B << 0, 0, 
                0, 0, 
                0, 0,
                0, 0;

            H.resize(measize,statesize);
            H << 1, 0, 0, 0,
                0, 0, 1, 0;

            z0.resize(usize);
            z0 << 0, 0; // 初始观测为 (0, 0)

            u_v.resize(usize);

            ka.Init_Par(x,P0,R,Q,A,B,H,u_v);
            
            if(coordinate_init==1)
            {
                cout<<"TF initializing!"<<endl;
            }

            coordinate_init-=1;
            ros::Duration(0.1).sleep();
        } 
        
        else if(coordinate_init<=0) //完成初始化
        {
            // if(First_init<10)
            //  {
            //     First_init++;//最多到10，避免溢出
            //  }
             
            T_C2_body = buffer_c2_body.lookupTransform(tf_prefix+"camera",tf_prefix+"body",ros::Time(0));
            T_A1_C2.header.stamp = ros::Time::now(); 
            T_A1_C2_pub.sendTransform(T_A1_C2);//"apriltag"->"camera_apriltag"
            
            T_C2_body_2.transform=T_C2_body.transform; 
            T_C2_body_2.header.stamp = ros::Time::now(); 
            T_C2_body_pub.sendTransform(T_C2_body_2);//"camera_apriltag"->"body_apriltag"
    //
            geometry_msgs::TransformStamped tfs2;
            tfs2 = buffer2.lookupTransform("hikrobot_camera",tf_prefix+"body",ros::Time(0)); //通过VIO获取的机器人坐标 在海康相机坐标系下的坐标数据


            if(buffer.canTransform("hikrobot_camera",tf_prefix+"body_apriltag",ros::Time(0),NULL)) //收到了apriltag信息
            {
                geometry_msgs::TransformStamped tfs;
                tfs = buffer.lookupTransform("hikrobot_camera",tf_prefix+"body_apriltag",ros::Time(0));

                // if(First_init<2)
                // {   
                    
                //     x.resize(statesize);
                //     x(0)=tfs.transform.translation.x;
                //     x(1)=0;
                //     x(2)=tfs.transform.translation.y;
                //     x(3)=0;
                //     ka.Init_Par(x,P0,R,Q,A,B,H,u_v);                    
                //     cout<<"kalman init now!"<<endl;
                //     continue;
                // }

                vx_body= (tfs.transform.translation.x - body_last_x) ;     
                vy_body= (tfs.transform.translation.y - body_last_y) ;           
                body_last_x=tfs.transform.translation.x;
                body_last_y=tfs.transform.translation.y;       
                
                u_v(0)=vx_body;
                u_v(1)=vy_body;

                // ka.m_u=u_v;

                z0(0)=tfs2.transform.translation.x;
                z0(1)=tfs2.transform.translation.y;
                ka.Predict_State();
                ka.Predict_Cov();
                ka.Mea_Resd(z0);
                ka.Cal_Gain();
                ka.Update_State();
                ka.Update_Cov();//卡尔曼滤波更新

                // if(abs(tfs.transform.translation.z-tfs2.transform.translation.z)>1.0)//apriltag与VIO的差距过大 ,直接使用apriltag的数据
                // {
                //     // coordinate_init=init_NUM;
                //     // First_init=0;
                //     // ka.Clear_all();//重新初始化结构体
                //     // ros::Duration(0.1).sleep();
                //     // continue;//重新进行初始化

                //     cout<<"can not find apriltag, use VIO instead"<<endl;
                //     T_final.header.frame_id= "hikrobot_camera";
                //     T_final.child_frame_id= "final_body"; 
                //     T_final.header.stamp= ros::Time::now(); 
                //     T_final.transform.translation.x = tfs2.transform.translation.x;
                //     T_final.transform.translation.y = tfs2.transform.translation.y;
                //     T_final.transform.translation.z = tfs2.transform.translation.z;
                //     T_final.transform.rotation=tfs2.transform.rotation;
                //     T_final_pub.sendTransform(T_final);  

                //     // if(First_init!=1)
                //     //     cout<<"can not find apriltag, use VIO instead"<<endl;
                //     // ka.Clear_all();
                //     // First_init=0;
                // }
                 //发布apriltag定位消息
                // geometry_msgs::PoseStamped apriltag_pose;
                // apriltag_pose.header.stamp=ros::Time::now(); 
                // apriltag_pose.header.frame_id= "apriltag1_pose"; 
                // apriltag_pose.pose.position.x=tfs.transform.translation.x;
                // apriltag_pose.pose.position.y=tfs.transform.translation.y;
                // apriltag_pose.pose.position.z=tfs.transform.translation.z;
                // apriltag_pose.pose.orientation.w=tfs.transform.rotation.w;
                // apriltag_pose.pose.orientation.x=tfs.transform.rotation.x;
                // apriltag_pose.pose.orientation.y=tfs.transform.rotation.y;
                // apriltag_pose.pose.orientation.z=tfs.transform.rotation.z;
                // apriltag1_pose_publisher.publish(apriltag_pose);
                
                //发布融合定位消息
                T_final.header.frame_id= "hikrobot_camera"; 
                T_final.child_frame_id= tf_prefix+"final_body"; 
                T_final.header.stamp= ros::Time::now(); 
                T_final.transform.translation.x = ka.m_x(0);
                T_final.transform.translation.y = ka.m_x(2);
                T_final.transform.translation.z = tfs.transform.translation.z;
                T_final.transform.rotation=tfs.transform.rotation;
                T_final_pub.sendTransform(T_final);     

                VIO_MIX_bias[0]= T_final.transform.translation.x- tfs2.transform.translation.x; //VIO与融合定位的差值
                VIO_MIX_bias[1]= T_final.transform.translation.y- tfs2.transform.translation.y;

                // if(abs(T_final.transform.translation.x-tfs2.transform.translation.x)>0.1)//融合定位与VIO的差距过大
                // {
                //     ka.Clear_all();//重新初始化结构体
                //     coordinate_init=init_NUM;
                //     First_init=0;
                //     cout<<"kalman error, restart!"<<endl;
                //     ros::Duration(0.1).sleep();
                //     continue;//重新进行初始化
                // }                

                // cout<<"final position:"<<endl;
                // cout<<ka.m_x<<endl;
                // ros::Duration(1).sleep();


            }

            else if(!buffer.canTransform("hikrobot_camera","body_apriltag",ros::Time(0),NULL)) //若丢失apriltag信息，发布VIO的位置
            {
                T_final.header.frame_id= "hikrobot_camera";
                T_final.child_frame_id= tf_prefix+"final_body"; 
                T_final.header.stamp= ros::Time::now(); 
                T_final.transform.translation.x = tfs2.transform.translation.x + VIO_MIX_bias[0]; //加上VIO与融合定位的差值，保证连续性
                T_final.transform.translation.y = tfs2.transform.translation.y + VIO_MIX_bias[1];
                T_final.transform.translation.z = tfs2.transform.translation.z;
                T_final.transform.rotation=tfs2.transform.rotation;
                T_final_pub.sendTransform(T_final);  
                cout<<"can not find apriltag, use VIO instead"<<endl;

                // if(First_init!=1)
                //     cout<<"can not find apriltag, use VIO instead"<<endl;
                // ka.Clear_all();
                // First_init=0;
            }

            // else
            // {
            //     coordinate_init=init_NUM;
            //     continue;
            // }

            //发布最终的位姿消息

            final_pose.header.stamp=ros::Time::now(); 
            final_pose.header.frame_id= tf_prefix+"final_pose"; 
            final_pose.pose.position.x=T_final.transform.translation.x;
            final_pose.pose.position.y=T_final.transform.translation.y;
            final_pose.pose.position.z=T_final.transform.translation.z;
            final_pose.pose.orientation.w=T_final.transform.rotation.w;
            final_pose.pose.orientation.x=T_final.transform.rotation.x;
            final_pose.pose.orientation.y=T_final.transform.rotation.y;
            final_pose.pose.orientation.z=T_final.transform.rotation.z;
            final_pose_publisher.publish(final_pose);

        }
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