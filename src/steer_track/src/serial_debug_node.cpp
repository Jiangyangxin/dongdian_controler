
#include "steer_track/serial_debug_node.h"

using namespace std;

//串口对象
Serial_Debug serial_debug;
//串口接收定时器
ros::Timer uart_data_timer;
//电机信息队列
queue<motor_info> motor_infos;
queue<steer_track::task_node_debug> task_node_datas;
queue<steer_track::robot_pose_vel> pose_vel_datas;
int queue_max_size = 30;
ros::Subscriber task_node_debug_sub;
ros::Subscriber robot_pose_vel_sub;
ros::Publisher all_debug_pub;
ros::Publisher motor_info_pub;

string buffer;
string head_str, tail_str;

//rosbag record -O test /all_debug_data
int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "serial_debug_node");
    if (argc != 2)
    {
        ROS_INFO("Serial_Debug:Please set 1 serial port!");
        return 1;
    }

    //创建节点句柄
    ros::NodeHandle nh;

    //初始化串口
    serial_debug.SerialInit(argv[1]);

    //初始化订阅器、发布器
    task_node_debug_sub = nh.subscribe("/task_node_debug_data", 1, TaskNodeDebugDataCallback);
    robot_pose_vel_sub = nh.subscribe("/pose_vel_data", 3, RobotPoseVelCallback);
    all_debug_pub = nh.advertise<steer_track::all_debug_data>("/all_debug_data",1);
    motor_info_pub = nh.advertise<steer_track::motor_info>("/motor_info_data",1);
    

    head_str.append(2, 0x50);
    tail_str.append(1, '\r');
    tail_str.append(1, '\n');
    //command串口读取定时器(周期为5ms)
    uart_data_timer = nh.createTimer(ros::Duration(0.005), SerialDataCallback);

    ros::spin();
    
    
    return 0;
}

void SerialDataCallback(const ros::TimerEvent&)
{
    static int t=0,tok=0;
    buffer += serial_debug.SerialRead();
    if(buffer.size() > 500)
    {
        buffer.clear();
    }
    int st = 0,en = 0;
    st = buffer.find(head_str);
    if (st != std::string::npos)
    {
        en = buffer.find(tail_str,st);
        if (en != std::string::npos && en-st == 140)//下位机发送长度为140字节
        {
            string valid_payload = buffer.substr(st, en-st+2);
            buffer.erase(buffer.begin(),buffer.begin()+en+2);
            motor_info temp;//电机的类，内部包含电机结构体
            parase_motor_info(valid_payload, temp);//解码下位机发来的数据，具体看32代码trans_motor_info（）函数
            motor_infos.push(temp);
            if(motor_infos.size()>queue_max_size)
            {
                motor_infos.pop();
            }
            temp = motor_infos.front();
            steer_track::motor_info m_info_msg;
            temp.fill_msg(m_info_msg);//转化为自定义消息类型
            motor_info_pub.publish(m_info_msg);
            // temp.print_infos(_logger);
            tok++;
        }
    }
    t++;
    if(t >= 200)
    {
        static ros::Time last_time, now_time;
        now_time = ros::Time::now();
        // double d = (double)((now_time.toSec() - last_time.toSec())*1e9 + (now_time.toNSec() - last_time.toNSec()))*1e-9;
        double d = (double)(now_time.toSec() - last_time.toSec());
        // ROS_INFO_STREAM("d:"<<d<<"s, rate:"<<tok/d<<"%");
        last_time = ros::Time::now();
        tok = 0;
        t = 0;
    }
    return ;
}

void TaskNodeDebugDataCallback(const steer_track::task_node_debugConstPtr& msg)
{
    steer_track::task_node_debug temp;
    temp = *msg;
    task_node_datas.push(temp);
    if(task_node_datas.size()>queue_max_size)
    {
        task_node_datas.pop();
    }
}

void pop_this_queue(int i)
{
    if(i == 0)
    {
        pose_vel_datas.pop();
    }else if(i == 1)
    {
        motor_infos.pop();
    }else if(i == 2)
    {
        task_node_datas.pop();
    }
}
    
void RobotPoseVelCallback(const steer_track::robot_pose_velConstPtr& msg)
{
    steer_track::robot_pose_vel temp;
    temp = *msg;
    pose_vel_datas.push(temp);//消息队列
    if(pose_vel_datas.size()>queue_max_size)
    {
        pose_vel_datas.pop();
    }
    // LOG4CPLUS_INFO(_logger, LOG4CPLUS_TEXT("start while"));
    boost::format fmt_time;
    fmt_time = boost::format("\npose_vel_datas:\t%lf, %d\n"\
                             "motor_info:\t\t%lf, %d\n"\
                             "task_node:\t\t%lf, %d\n");
    while(pose_vel_datas.size()!=0 && task_node_datas.size()!=0 && motor_infos.size()!=0)
    {
        //0:pose_vel_datas, 1:motor_info, 2:task_node
        bool need_pop[3] = {0};
        double fs[3] = {0};
        fs[0] = pose_vel_datas.front().sec_f;
        fs[1] = motor_infos.front().sec_f;
        fs[2] = task_node_datas.front().sec_f;
        //debug
        string debug_string;
        fmt_time%fs[0]%pose_vel_datas.size()%fs[1]%motor_infos.size()%fs[2]%task_node_datas.size();
        debug_string = fmt_time.str();
        // LOG4CPLUS_INFO(_logger, LOG4CPLUS_TEXT(debug_string));
        if(abs(fs[0]-fs[1])<0.005 && abs(fs[0]-fs[2])<0.005 && abs(fs[1]-fs[2])<0.005)
        {
            steer_track::all_debug_data all_msg;
            all_msg.robot_pose_vel = pose_vel_datas.front();
            all_msg.task_node_data = task_node_datas.front();
            motor_infos.front().fill_msg(all_msg.m_info);
            all_msg.sec_f = pose_vel_datas.front().sec_f;
            all_debug_pub.publish(all_msg);
            pose_vel_datas.pop();
            task_node_datas.pop();
            motor_infos.pop();
        }else
        {
            int i = 0;
            int maxPosition = max_element(fs, fs+3) - fs;
            for(i=0; i<3; i++)
            {
                if(fs[maxPosition] - fs[i] > 0.005)
                {
                    pop_this_queue(i);
                }
            }
        }
    }
}

float hex2float(const char* h)
{
    float* f = (float*)h;
    return *f;
}
double hex2double(const char* h)
{
    double* f = (double*)h;
    return *f;
}


void parase_motor_info(const string s, motor_info& m)
{
    int i = 0;
    int j = 0;
    int index = 0;
    for(i=0; i<3; i++)
    {
        m.dr2.tar_th[i] = hex2float(s.c_str()+2+4*j++);//c_str()就是将C++的string转化为C的字符串数组
        m.dr2.now_th[i] = hex2float(s.c_str()+2+4*j++);//hex2float将字符串数据转为 float类型的浮点数
        m.dr2.now_vel[i] = hex2float(s.c_str()+2+4*j++);
        m.dr2.tar_force[i] = hex2float(s.c_str()+2+4*j++);
        m.dr2.now_force[i] = hex2float(s.c_str()+2+4*j++);
        m.dr1.now_pos[i] = hex2float(s.c_str()+2+4*j++);
        m.dr1.now_vel[i] = hex2float(s.c_str()+2+4*j++);
        m.dr1.tar_force[i] = hex2float(s.c_str()+2+4*j++);
        m.dr1.now_force[i] = hex2float(s.c_str()+2+4*j++);
        m.fan_pre[i] = hex2float(s.c_str()+2+4*j++);
    }//j = 30
    index = 2+4*j;
    index+=10;
    m.sec_f = hex2double(s.c_str()+index);
    index += 8;
}


void Serial_Debug::SerialInit(std::string port)
{
     //打开串口，波特率115200，串口为port
	try 
	{
		ros_ser_.setPort(port);             //串口端口设置
		ros_ser_.setBaudrate(230400);       //串口波特率设置
		serial::Timeout to = serial::Timeout(2, 4, 0, 25, 0);       //串口延时设置
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
         ROS_INFO_STREAM("Serial Port opened");
         ros_ser_.flushInput();
    } 
	else 
	{
        return;
    }
}

/*!
 * \brief 读取串口数据
 * 
 * \return 串口缓冲区数据
 */
std::string Serial_Debug::SerialRead()
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

void motor_info::fill_msg(steer_track::motor_info & msg)
{
    int i = 0;
    for(i = 0; i<3; i++)
    {
        msg.dr2_tar_th[i] = dr2.tar_th[i];
        msg.dr2_now_th[i] = dr2.now_th[i];
        msg.dr2_now_vel[i] = dr2.now_vel[i];
        msg.dr2_tar_force[i] = dr2.tar_force[i];
        msg.dr2_now_force[i] = dr2.now_force[i];
        msg.dr1_now_pos[i] = dr1.now_pos[i];
        msg.dr1_now_vel[i] = dr1.now_vel[i];
        msg.dr1_tar_force[i] = dr1.tar_force[i];
        msg.dr1_now_force[i] = dr1.now_force[i];
        msg.fan_pre[i] = fan_pre[i];
    }

    msg.sec_f = sec_f;
}