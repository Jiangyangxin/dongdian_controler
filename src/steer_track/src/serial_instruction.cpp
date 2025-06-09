#include "steer_track/serial_instruction.h"

/*! 
 * \brief 初始化串口
 * 
 * \param port Jetson nano串口的端口
 */
void Serial_Prototype::SerialInit(std::string port)
{
     //打开串口，波特率115200，串口为port
	try 
	{
		ros_ser_.setPort(port);             //串口端口设置
		ros_ser_.setBaudrate(115200);       //串口波特率设置
		serial::Timeout to = serial::Timeout::simpleTimeout(10);        //串口延时设置
		ros_ser_.setTimeout(to);
        ros_ser_.setStopbits(serial::stopbits_one);                     //一个停止位
        
		ros_ser_.open();        //打开串口
	} 
	catch (serial::IOException& e) 
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return ;
	}
	
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
 * \brief 关闭串口
 */
void Serial_Prototype::CloseSerial()
{
    ros_ser_.close();
    return;
}


/*!
 * \brief 读取串口数据
 * 
 * \return 串口缓冲区数据
 */
std::string Serial_Prototype::SerialRead()
{
    std::string data_str;
    size_t p = ros_ser_.available();

    //串口数据读取
    if (p != 0)
    {
        data_str = ros_ser_.read(p);
        ros_ser_.flushInput();
    } 
    return data_str;
}

size_t Serial_Prototype::SerialWrite(const std::string str)
{
    size_t n;
    n = ros_ser_.write(str);
    ROS_INFO("Tx: %s",(str).c_str());
    //ros_ser_.flushOutput();
    return n ;
}


/*!
 * \brief 风机停止
 */
void Serial_Instruction::StopFans(void)
{
    ROS_INFO("stop fan");
    ros_ser_.write("stopF\r\n");
    ros_ser_.flushOutput();
    return;
}


/*!
 * \brief 设置电机参数（速度值）
 *
 * \param vx x方向速度
 * \param vy y方向速度
 * \param vw 运动角速度
 */
void Serial_Instruction::SetMotorVec(float vx, float vy, float vw)
{
    //速度幅值限制
    SpeedLimit(vx, 0.3);
    SpeedLimit(vy, 0.3);
    SpeedLimit(vw, 0.2);

    //发送串口指令
    std::string set_motor_vec_str = "motor " + std::to_string(vx) + " " + std::to_string(vy) + 
                                    " " + std::to_string(vw) + "\r\n";
    ros_ser_.write(set_motor_vec_str);

    //清空串口发送缓存区
    ros_ser_.flushOutput();
    return;
}


/*!
 * \brief 设置风机参数（负压值）
 *
 * \param pre1 风机1负压值
 * \param pre2 风机2负压值
 * \param pre3 风机3负压值
 */
void Serial_Instruction::SetFan(float pre1, float pre2, float pre3)
{
    //风机负压幅值限制
    FanLimit(pre1, 7.0, 0.0);
    FanLimit(pre2, 7.0, 0.0);
    FanLimit(pre3, 7.0, 0.0);

    //发送串口指令
    std::string set_fan_str = "params pre1 " + std::to_string(pre1) + 
                                    " pre2 " + std::to_string(pre2) + 
                                    " pre3 " + std::to_string(pre3) + "\r\n";
    ros_ser_.write(set_fan_str);
    //ros_ser_.flushOutput();
    return;
}


/*!
 * \brief 限制速度指令最大值
 * 
 * \param input_value 速度值
 * \param max 速度值绝对值上限
 */
void Serial_Instruction::SpeedLimit(float& input_value, float max)
{
    input_value = (input_value > max) ? max : input_value;
    input_value = (input_value < -max) ? -max : input_value;
    return;
}


/*!
 * \brief 限制风机负压最大值
 * 
 * \param input_value 风机负压指令
 * \param max 风机默认负压最大值，默认为0kPa
 * \param min 风机的负压最小值，默认为-10kPa
 */
void Serial_Instruction::FanLimit(float& input_value, float max, float min)
{
    input_value = (input_value > max) ? max : input_value;
    input_value = (input_value < min) ? min : input_value;
    return;
}

void Serial_Instruction::float2hex(float f, char h[4])
{
    char *ph = (char*)&f;
    for(int i=0; i<4; i++)
    {
        h[i] = ph[i];
    }
}
void Serial_Instruction::double2hex(double d, char h[8])
{
    char *ph = (char*)&d;
    for(int i=0; i<8; i++)
    {
        h[i] = ph[i];
    }
}
/*** 
 * @Description: 发送舵轮的控制指令[th1,th2,th3][v1,v2,v3]
 * @param {float} *data
 * @return {*}
 */
void Serial_Instruction::setSteerTHV(const steer_track::steer_th_velConstPtr& msg)
{
    std::string set_steer_str;
    int i = 0;
    char data_array[64];
    //set_steer_str 一帧共7+24+12+8+2+1=54个字节
    set_steer_str += "theta_v";
    for(i = 0; i<6; i++)
    {
        float2hex(msg->th_vel[i], data_array+4*i);
    }//i=6
    for(i = 6; i<9; i++)
    {
        // float2hex(5, data_array+4*i);
        float2hex(msg->fan_pre[i-6], data_array+4*i);
    }//i=9
    double2hex(msg->sec_f, data_array+4*i);
    set_steer_str.append(data_array, 44);//舵轮数据+风机数据+时间戳一共44字节
    set_steer_str += "\r\n";
    uint8_t sum = 0;
    for(i=0; i<set_steer_str.size(); i++)
    {
        sum += set_steer_str[i];
    }
    set_steer_str.append(1, (char)sum);
    ros_ser_.write(set_steer_str);//通过串口发送字符串
    //ros_ser_.flushOutput();
    return;
}