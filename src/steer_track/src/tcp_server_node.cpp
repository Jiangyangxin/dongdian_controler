#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "pthread.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <vector>
#include <signal.h>
#include <sstream>
#include "steer_track/motion_instruction.h"
#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

//Tcp服务器端口号
int SERVER_PORT = 9527;//9527->robot  9528->gongzhuang
std::vector<float> left_xy(4);
float left_dis;
float left_curlen; 
float left_linelen;
std::vector<float> right_xy(4);
float right_dis;
float right_curlen; 
float right_linelen;
std::vector<float> lidar_info_vec;

//服务器监听套接字
int listenfd;
int confd;
int max_fd;
//创建子线程
pthread_t tid;
//服务端select()函数准备  
fd_set read_fds;
struct timeval timeout;
//检测是否正确连接
int connnected = 0;
int ps_success =0;
std::string fan_data_str;


//运动指令消息发布器
ros::Publisher motion_instruction_pub;
ros::Subscriber fan_data_sub;
ros::Subscriber traj_data_sub;
ros::Subscriber motor_line_sub;
ros::Subscriber pose_data_sub;
ros::Subscriber lvban_pose_data_sub;
ros::Subscriber left_lidar_sub;
ros::Subscriber right_lidar_sub;
ros::Timer tcp_recv_timer;

// 位资数据
float pose_data[3] = {0};
geometry_msgs::PoseStamped lvban_pose;

//指令发布定时器
//子线程调用函数（发布运动指令）
void* InstructionPubCallback(void* arg);

/*!
 * \brief 字符串分割函数
 * 
 * \param return_data_vec 分割后的vector
 * \param str 传入待分割的字符串
 * \param pattern 分割字符
 */
void buf_split(std::vector<std::string> &return_data_vec, const std::string &str, const std::string &pattern);

/**
 * @description: "fan_data"话题风机数据回调函数
 * @param {const} std_msgs
 * @return {*}
 */
// void FanDataCallback(const std_msgs::StringConstPtr& msg);
void FanDataCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
void MotorlineDataCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
void left_LidarCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
void right_LidarCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
void TrajCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
/*!
*\brief 消息订阅线程回调函数
*/
void* MessageSubCallback(void* arg);

void PoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg);
void lvbanPoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg);

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "tcp_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    //消息发布器和订阅器建立
    motion_instruction_pub = nh.advertise<steer_track::motion_instruction>("motion_instruction", 1);
    fan_data_sub = nh.subscribe("fan_pwm_info", 1, FanDataCallback); 
    traj_data_sub = nh.subscribe("traj_data", 1, TrajCallback);
    motor_line_sub = nh.subscribe("motor_line_data", 1, MotorlineDataCallback);

    // pose_data_sub = nh.subscribe("/vrpn_client_node/steerRobot/pose", 1, PoseDataCallback); 
    // lvban_pose_data_sub = nh.subscribe("/vrpn_client_node/lvban/pose", 1, lvbanPoseDataCallback); 

    nh_private.param<int>("SERVER_PORT",SERVER_PORT,9527);
    printf("SERVER_PORT: %d \n",SERVER_PORT);

    if(SERVER_PORT!=9527) //工装端口 9528 
    {
        left_lidar_sub=nh.subscribe("/lidar_info_left",1 ,left_LidarCallback);
        right_lidar_sub=nh.subscribe("/lidar_info_right", 1,right_LidarCallback);    
    }

    //服务器sockaddr
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;//IPV4
    servaddr.sin_port = htons(SERVER_PORT); //确定端口号
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    //客户端IP
    char client_IP[1024];
    //客户端sockaddr
    struct sockaddr_in client_addr;
    //客户端sockaddr长度
    socklen_t client_addr_len;


    //创建ROS消息监听子线程
    // pthread_t tid_1;
    // int rett_1;
    // rett_1 = pthread_create(&tid_1, nullptr, MessageSubCallback, nullptr);
    // if(rett_1 != 0) {
    //     std::cerr << "thread creat error!" << std::endl;
    // }
    
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);
    

    //整体流程： socket()->bind()->listen()->accept->recv()或者sent()->close()

    //与客户端建立通信 socket->bind->listen->accept
    listenfd = socket(AF_INET, SOCK_STREAM, 0); //AF_INET代表IPV4,SOCK_STREAM代表TCP协议
    //设置socket的SO_REUSEADDR选项，避免端口占用
    int opt = 1;
    setsockopt(listenfd,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof( opt ));

    if (listenfd == -1)//出现错误
    {
        ROS_ERROR("creat socket erro: %s(errno: %d)", strerror(errno), errno);
        return 0;
    }

    // while(ros::ok()) 
    // {
        if ((bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr))) == -1)//bind将socket连接到网卡
        {
            ROS_ERROR("bind socket erro: %s(errno: %d)", strerror(errno), errno);
            //continue;
            return 0;
        }
    // }

    
    if ((listen(listenfd, 128)) == -1)//开始进入监听状态,等待客户端的连接
    {
        ROS_ERROR("listen socket erro: %s(errno: %d)", strerror(errno), errno);
        return 0;
    }

    // 初始化文件描述符集合
    FD_ZERO(&read_fds);
    FD_SET(listenfd, &read_fds);
    // 设置最大文件描述符和超时时间
    max_fd = listenfd;
    timeout.tv_sec = 15; // 5秒超时 如果 timeout 为 NULL，则 select 将阻塞，直到有一个文件描述符就绪（即可读、可写或出现异常条件）
    timeout.tv_usec = 0;

    ROS_INFO("======waiting for client's request======");
    ros::Rate r(10);
    while(ros::ok()) 
    {

        // // 复制文件描述符集合，因为 select 会修改它
        // fd_set read_fds_copy = read_fds;
        // // 调用 select 函数
        // // int ret = select(max_fd + 1, &read_fds_copy, NULL, NULL, &timeout);
        // int ret = select(max_fd + 1, &read_fds_copy, NULL, NULL, NULL);
        // if (ret < 0) { //表示调用失败。此时，errno 被设置为适当的错误码，以指示失败的原因。
        //     perror("select");
        //     close(listenfd);
        //     exit(EXIT_FAILURE);
        // } 
        // else if (ret == 0) { //如果 timeout 为 NULL，则这种情况不会发生（除非发生错误）。
        //     printf("Timeout occurred! No data...\n");
        //     continue; // 超时，没有数据可读，继续循环  
        // }
        // printf("OOKOKK");




        if(connnected==0) //未成功连接
        {


        // 复制文件描述符集合，因为 select 会修改它
        fd_set read_fds_copy = read_fds;
        // 调用 select 函数
        // int ret = select(max_fd + 1, &read_fds_copy, NULL, NULL, &timeout);
        int ret = select(max_fd + 1, &read_fds_copy, NULL, NULL, NULL);
        if (ret < 0) { //表示调用失败。此时，errno 被设置为适当的错误码，以指示失败的原因。
            perror("select");
            close(listenfd);
            exit(EXIT_FAILURE);
        } 
        else if (ret == 0) { //如果 timeout 为 NULL，则这种情况不会发生（除非发生错误）。
            printf("Timeout occurred! No data...\n");
            continue; // 超时，没有数据可读，继续循环  
        }

            // 检查监听 socket 是否有新的连接
            if (FD_ISSET(listenfd, &read_fds_copy)) 
            {
                client_addr_len = sizeof(client_addr);
                if((confd = accept(listenfd, (struct sockaddr*)&client_addr, &client_addr_len) )== -1) //accept默认会阻塞进程
                {   //accept接受任意客户端的连接，并返回一个新的socket：confd，以及客户端的IP地址 client_addr 
                    //新的socket：confd负责与客户端通信，而旧的socket，listenfd依旧用于服务端的监听
                    ROS_ERROR("accept socket erro: %s(errno: %d)", strerror(errno), errno); 
                    continue;//不断地accept，直到成功为止
                }
            
                
                // 将新的连接添加到文件描述符集合中
                FD_SET(confd, &read_fds);
                if (confd > max_fd) {
                    max_fd = confd; // 更新最大文件描述符
                }

                
                //显示客户端IP、端口号信息
                ROS_INFO("client_ip:%s port:%d", 
                        inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, client_IP, sizeof(client_IP)),
                        ntohs(client_addr.sin_port));
                connnected = 1;
            }
        }

        else if(connnected==1) //已经成功连接
        {
            if(ps_success==0)
            {
                int rett;
                rett = pthread_create(&tid, nullptr, InstructionPubCallback, nullptr);//创建子线程，通过套接字confd接收消息
                
                if(rett != 0) 
                {
                std::cerr << "thread creat error!" << std::endl;
                }
                else
                {
                    ps_success=1;//成功创建线程
                }
            }
  
        }
        if(!ros::ok())
        {
            close(confd);//关闭套接字
            close(listenfd);    // 关闭监听 socket
            pthread_cancel(tid);//关闭子线程，不写应该也可以，主线程结束后会退出进程，所以进程里的其他线程都会终止结束
            break;
        }

        // r.sleep();
        ros::spinOnce();

    }
    close(confd);//关闭套接字
    close(listenfd);
   
    char* s = NULL;
    pthread_join(tid,(void**)&s); //阻塞，保证在退出的时候退出线程
    pthread_cancel(tid);//关闭子线程，不写应该也可以，主线程结束后会退出进程，所以进程里的其他线程都会终止结束
    return 0;//若退出后端口依旧被占用，可以通过 netstat -anp | grep 9527 来查看进程号，然后kill掉
}


//控制指令发布回调函数
void* InstructionPubCallback(void* arg)
{
    while (ros::ok())
    {
        char buf[1024] = "";         //服务器读取字符串，最大1024个字节

         if(recv(confd, buf, sizeof(buf), 0) <= 0) 
         { // 客户端关闭连接或者出错
            if(connnected==1)
            {
                ROS_INFO("Socket disconnected!");
            }
            connnected = 0;
            //break;
            continue;
        };

        steer_track::motion_instruction motion_msg;//待发送的tcp指令消息

        std::string mode;
        std::vector<std::string> motion_instruction_str;
        buf_split(motion_instruction_str, buf, "\t");
        mode = motion_instruction_str.at(0);

        //根据传入字符判断并作出相应处理
        if(mode == "motion" || mode == "stop" || mode == "track" || \
        mode == "stcorr" || mode == "fans" || mode == "stopF" || \
        mode=="splineTrack" || mode=="getDeltaYaw" ||  mode=="RestAngle" || \
        mode=="CCWiRotate" || mode=="CWiRotate" || mode=="start_follow" || mode=="stop_follow" || mode=="set_zeroing")
        {
            motion_msg.mode = mode;
            motion_instruction_pub.publish(motion_msg);
        }
        else if(mode == "robotVel")
        {
            motion_msg.mode = mode;
            motion_msg.robot_vel = atof(motion_instruction_str[1].c_str());//字符串转化为浮点数
            motion_instruction_pub.publish(motion_msg);
        }
        else if(mode == "increment" || mode == "absolute")
        { 
            motion_msg.mode = mode;
            if (mode == "increment")
            {
                float dx = atof(motion_instruction_str[1].c_str());
                float dy = atof(motion_instruction_str[2].c_str());
                motion_msg.dx = dx;
                motion_msg.dy = dy;
            }
            else 
            {
                float x = atof(motion_instruction_str[1].c_str());
                float y = atof(motion_instruction_str[2].c_str());
                motion_msg.x = x;
                motion_msg.y = y;
            }
            motion_instruction_pub.publish(motion_msg);
        }
        else if(mode == "set_fan")
        {
            motion_msg.mode = mode;
            motion_msg.fan_1 = atof(motion_instruction_str[1].c_str());
            motion_msg.fan_2 = atof(motion_instruction_str[2].c_str());
            motion_msg.fan_3 = atof(motion_instruction_str[3].c_str());
            motion_instruction_pub.publish(motion_msg);
        }
        else if(mode == "set_polish")
        {
            motion_msg.mode = mode;
            motion_msg.polish_speed = atof(motion_instruction_str[1].c_str());
            motion_instruction_pub.publish(motion_msg);
        }
        else if(mode == "autotrack")
        {
            motion_msg.mode = mode;
            motion_msg.per_x = atof(motion_instruction_str[1].c_str());
            motion_msg.per_y = atof(motion_instruction_str[2].c_str());
            motion_msg.num_track = atof(motion_instruction_str[3].c_str());
            motion_msg.orien_track = atof(motion_instruction_str[4].c_str());
            motion_instruction_pub.publish(motion_msg);
        }
        else if(mode=="fzmotor")
        {
            motion_msg.mode = mode;
            motion_msg.motor_linelenth = atof(motion_instruction_str[1].c_str());
            motion_msg.motor_speed = atof(motion_instruction_str[2].c_str());
            motion_instruction_pub.publish(motion_msg);
        }


    }
    ps_success=0;
    close(confd);
    close(listenfd);
    pthread_exit(NULL);
}

void* MessageSubCallback(void* arg) {
    ros::spin();
    return 0;
}

void buf_split(std::vector<std::string> &return_data_vec, const std::string &str, const std::string &pattern)
{
    char * strc = new char[strlen(str.c_str())+1];
    strcpy(strc, str.c_str());   //string转换成C-string
    char* temp = strtok(strc, pattern.c_str());
    while(temp != nullptr)
    {
        return_data_vec.push_back(std::string(temp));
        temp = strtok(nullptr, pattern.c_str());
    }
    delete[] strc;
    return;
}

/**
 * @description: "fan_pwm_info"话题，风机数据回调函数
 * @param {const} std_msgs::Float32MultiArray
 * @return {*}
 */
void FanDataCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    static int countt=0;
    countt++;
    if(connnected == 1)
    {
        std::string fan_data_string;
        char* str;
        fan_data_string.append("fan");
        //上位机的格式：sprintf(dbgStr, "fan\t%.3f\t%.3f\t%.3f\r\n", param_sample[0], param_sample[1], param_sample[2]);
        for (size_t i = 0; i < msg->data.size()-3; i++) //将风机数据合并成一个字符串，msg的前三个是实际负压，msg的后三个是pwm，只取实际负压，因此要-3
        {
            fan_data_string.append("\t");
            sprintf(str,"%.3f",msg->data[i]);
            //fan_data_string.append(std::to_string(msg->data[i]));
            fan_data_string.append(str);
        }
        fan_data_string.append("\r\n");
        char* fan_data_char = new char[strlen(fan_data_string.c_str())+1];
        strcpy(fan_data_char,fan_data_string.c_str());//将string类型转化为C语言中的char*类型
        if(countt>10)//降低发送频率
        {
            // std::cout<<fan_data_char<<std::endl; //debug
            write(confd, fan_data_char, strlen(fan_data_char));
            countt=0;
            // std::cout<<"11111"<<std::endl;
        }
        // std::cout<<"fan now"<<std::endl;
        delete[] fan_data_char;
    }

    // std::cout<<"33333"<<std::endl;
    return;
}


/**
 * @description: "motor_line"话题，轨迹限制区域数据回调函数
 * @param {const} std_msgs
 * @return {*}
 */
void MotorlineDataCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    static int countt=0;
    countt++;
    if(connnected == 1)
    {
        std::string motor_data_string;
        char* str;
        motor_data_string.append("motor");
        //上位机的格式：sprintf(dbgStr, "motor\t%.3f\r\n", param_sample[0], param_sample[1], param_sample[2]);
        for (size_t i = 0; i < msg->data.size();i++) //将电机数据合并成一个字符串
        {
            motor_data_string.append("\t");
            sprintf(str,"%.3f",msg->data[i]);
            // std::cout<<msg->data[i]<<std::endl;
            //fan_data_string.append(std::to_string(msg->data[i]));
            motor_data_string.append(str);
        }
        motor_data_string.append("\r\n");
        char* motor_data_char = new char[strlen(motor_data_string.c_str())+1];
        strcpy(motor_data_char,motor_data_string.c_str());//将string类型转化为C语言中的char*类型
        if(countt>10)//降低发送频率
        {
            // std::cout<<fan_data_char<<std::endl; //debug
            write(confd, motor_data_char, strlen(motor_data_char));
            countt=0;
            // std::cout<<"11111"<<std::endl;
        }
        // std::cout<<"fan now"<<std::endl;
        delete[] motor_data_char;
    }

    // std::cout<<"33333"<<std::endl;
    return;
}


/**
 * @description: "traj_data"话题，风机数据回调函数
 * @param {const} std_msgs
 * @return {*}
 */
void TrajCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    static int countt=0;
    countt++;
    if(connnected == 1)
    {
        std::string traj_data_string;
        char* str;
        traj_data_string.append("traj");
        //上位机的格式：sprintf(dbgStr, "fan\t%.3f\t%.3f\t%.3f\r\n", param_sample[0], param_sample[1], param_sample[2]);
        for (size_t i = 0; i < msg->data.size(); i++) //将轨迹数据合并成一个字符串
        {
            traj_data_string.append("\t");
            sprintf(str,"%.3f",msg->data[i]);
            //fan_data_string.append(std::to_string(msg->data[i]));
            traj_data_string.append(str);
        }
        traj_data_string.append("\r\n");
        char* traj_data_char = new char[strlen(traj_data_string.c_str())+1];
        strcpy(traj_data_char,traj_data_string.c_str());//将string类型转化为C语言中的char*类型
        if(countt>10)//降低发送频率
        {
            // std::cout<<fan_data_char<<std::endl; //debug
            write(confd, traj_data_char, strlen(traj_data_char));
            countt=0;
        }

        delete[] traj_data_char;
    }
    return;
}


void left_LidarCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    static int count_left=0;
    count_left++;
    for(int i=0;i<4;i++)
    {
        left_xy[i]=msg->data[i];
    }
    left_dis=msg->data[4];
    left_curlen=msg->data[5]; 
    left_linelen=msg->data[6];
//将左侧雷达和右侧雷达的信息集成起来
    // lidar_info_vec.resize(left_xy.size()+right_xy.size()+3+3);
    std::copy(left_xy.begin(),left_xy.end(),std::back_inserter(lidar_info_vec));
    lidar_info_vec.push_back(left_dis);
    lidar_info_vec.push_back(left_curlen);
    lidar_info_vec.push_back(left_linelen);
    std::copy(right_xy.begin(),right_xy.end(),std::back_inserter(lidar_info_vec));
    lidar_info_vec.push_back(right_dis);
    lidar_info_vec.push_back(right_curlen);
    lidar_info_vec.push_back(right_linelen);
    // for(const auto &ch:lidar_info_vec)
    // {
    // std::cout<<" "<<ch<<std::endl;
    // }
    // std::cout<<"lidar_info_vec"<<lidar_info_vec.size()<<std::endl;

    if(connnected == 1)
    {
        std::string lidar_data_string;
        // char* str;
        lidar_data_string.append("lidar");
        //上位机的格式：sprintf(dbgStr, "motor\t%.3f\r\n", param_sample[0], param_sample[1], param_sample[2]);
        for (int i = 0; i < lidar_info_vec.size();i++) //将lidar数据合并成一个字符串
        {
            // std::cout<<lidar_data_string<<std::endl;
            lidar_data_string.append("\t");
            // sprintf(str,"%.3f",(double)lidar_info_vec[i]);
            std::ostringstream out;
            out<<std::fixed<<std::setprecision(3)<<lidar_info_vec[i];
            lidar_data_string += out.str();

            // std::cout<<" "<<lidar_info_vec[i]<<std::endl;
            // std::cout<<msg->data[i]<<std::endl;
            //fan_data_string.append(std::to_string(msg->data[i]));
            // lidar_data_string.append(str);
        }
        lidar_data_string.append("\r\n");
        // std::cout<<lidar_data_string<<std::endl;
        char* lidar_data_char = new char[strlen(lidar_data_string.c_str())+1];
        // std::cout<<lidar_data_string<<std::endl;
        strcpy(lidar_data_char,lidar_data_string.c_str());//将string类型转化为C语言中的char*类型
        if(count_left>10)//降低发送频率
        {
            // std::cout<<fan_data_char<<std::endl; //debug
            write(confd, lidar_data_char, strlen(lidar_data_char));
            count_left=0;
            // std::cout<<"11111"<<std::endl;
        }
        // std::cout<<"fan now"<<std::endl;
        delete[] lidar_data_char;
    }
    lidar_info_vec.clear();

    // std::cout<<"33333"<<std::endl;
    return;
}

void right_LidarCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    static int count_right=0;
    count_right++;
    for(int i=0;i<4;i++)
    {
        right_xy[i]=msg->data[i];
    }
    right_dis=msg->data[4];
    right_curlen=msg->data[5]; 
    right_linelen=msg->data[6];
}

/*以下为刘晓顺师兄写的风机反馈程序，现将std_msgs::String改为了std_msgs::Float32MultiArray*/
// 原本的底层上传的string类型为 "fan\t%.3f\t%.3f\t%.3f\r\n"，因此 msg 这个string中包含了三个风机的数据
// void FanDataCallback(const std_msgs::StringConstPtr& msg)
// {
//     if(connnected == 1)
//     {
//         char* fan_data_char = new char[strlen(msg->data.c_str())+1];
//         strcpy(fan_data_char, msg->data.c_str());
//         write(confd, fan_data_char, strlen(fan_data_char));
//         delete[] fan_data_char;
//     }
//     // char pose_data_char[1024];
//     // sprintf(pose_data_char, "pose\t%f\t%f\t%f\r\n", pose_data[0], pose_data[1], pose_data[2]);
//     // write(confd, pose_data_char, strlen(pose_data_char));
//     // memset(pose_data_char, 0, sizeof(pose_data_char));
    
//     return;
// }

/**
 * @description: "pose_vel_data"话题，位姿数据回调函数
 * @param {const} std_msgs
 * @return {*}
 */
void PoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    /*
    Eigen::Matrix4d t_r_l, t_lp_l, t_o_lp, t_r_o;
    // 求t_lp_l
    t_lp_l<<1, 0, 0, 0.005,
            0, 1, 0, 0.175,
            0, 0, 1, 0.38,
            0, 0, 0, 1;
    // 求t_o_lp
    Eigen::Quaterniond q_lp_o(lvban_pose.pose.orientation.w, lvban_pose.pose.orientation.x,
     lvban_pose.pose.orientation.y, lvban_pose.pose.orientation.z);
    Eigen::Matrix3d r_lp_o = q_lp_o.matrix();
    Eigen::Vector3d p_lp_o(lvban_pose.pose.position.x, lvban_pose.pose.position.y, lvban_pose.pose.position.z);
    Eigen::Matrix4d t_lp_o;
    t_lp_o<<r_lp_o, p_lp_o, 0, 0, 0, 1;
    t_o_lp = t_lp_o.inverse();
    // 求t_r_o
    Eigen::Quaterniond q_r_o(msg->pose.orientation.w, msg->pose.orientation.x,
     msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d r_r_o = q_r_o.matrix();
    Eigen::Vector3d p_r_o(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    t_r_o<<r_r_o, p_r_o, 0, 0, 0, 1;
    // 求t_r_l
    t_r_l = t_lp_l*t_o_lp*t_r_o;
    Eigen::Quaterniond q_r_l(t_r_l.block<3,3>(0, 0));
    Eigen::Vector3d p_r_l(t_r_l.block<3, 1>(0, 3));

    char pose_data_char[1024];
    sprintf(pose_data_char, "pose\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",
     p_r_l[0], p_r_l[1], p_r_l[2],
     q_r_l.w(), q_r_l.x(), q_r_l.y(), q_r_l.z());
    write(confd, pose_data_char, strlen(pose_data_char));
    memset(pose_data_char, 0, sizeof(pose_data_char));
    return;
*/
    // 求t_r_o
    if(connnected == 1)
    {
        Eigen::Quaterniond q_r_o(msg->pose.orientation.w, msg->pose.orientation.x,
        msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Matrix3d r_r_o = q_r_o.matrix();
        Eigen::Vector3d p_r_o(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        char pose_data_char[1024];
        sprintf(pose_data_char, "pose\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",
        p_r_o[2], p_r_o[1], p_r_o[0],
        q_r_o.w(), q_r_o.x(), q_r_o.y(), q_r_o.z());
        write(confd, pose_data_char, strlen(pose_data_char));
        memset(pose_data_char, 0, sizeof(pose_data_char));
    }
    return;
}


void lvbanPoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    lvban_pose = *msg;
}


