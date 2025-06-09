#include<ros/ros.h>
#include<serial/serial.h>
#include<std_msgs/String.h>

#include<iostream>
#include<string>
#include<sstream>

using namespace std;

class ceju
{
    public:
    //创建一个serial类
    serial::Serial ser;
    std::string serial_port_;
    int baudrate_;
    string data_str;
    size_t p_size = 50;
    char *ucRxData;
    int result=0;
    bool SerialInit();
    int SerialRead();
    void Get_Value(char *ucRxData,int &data);

};

bool ceju::SerialInit()
{
    try
    {
        ser.setPort(serial_port_);
        ser.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.setStopbits(serial::stopbits_one);   //一个停止位
        ser.open();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");           //打开串口失败，打印信息
        return false;
    }

    //判断串口是否成功打开
    if( ser.isOpen() )
    { 
        ROS_INFO_STREAM("Serial Port "<< serial_port_<<" initialized");         //成功打开串口，打印信息
        return true;
    }
    else
    {
        return false;
    }

}

void ceju::Get_Value(char *ucRxData,int &data)
{
	unsigned char *p;
    int stateee=0;
	int Data=0;
    p=(unsigned char*)strstr((char*)ucRxData,"e;");	
    while(*p!=',')
    {
        if(*p>='0'&&*p<='9')
        {
            stateee=(int)(*p-'0'); //找到state的值
        }
        p++;		
    }    		
    
    p=(unsigned char*)strstr((char*)ucRxData,"d");
    while(*p!='m')
    {
        if(*p>='0'&&*p<='9')
        {
            Data=Data*10+(int)(*p-'0'); //算出距离值
        }
        p++;		
    }

    if(stateee==0)
    {
        data=Data;
    }
}

int ceju::SerialRead()
{

    data_str = ser.read(p_size);
    ucRxData = (char*)malloc((data_str.length()) * sizeof(char));
    data_str.copy(ucRxData, data_str.length(),0);
    Get_Value(ucRxData,result);
    // cout<<"接受数据："<<result<<endl;
    ser.flushInput();
    return result;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv,"ceju");
    ros::NodeHandle nh; 
    ros::NodeHandle nh_private("~");

    std::string serial_port;
    int baudrate;

    nh_private.param<std::string>("serial_port", serial_port,"/dev/ttyUSB0");
    nh_private.param<int>("baudrate", baudrate,115200);

    ceju ceju1;
    ceju ceju2;

    ceju1.baudrate_=baudrate;
    ceju1.serial_port_=serial_port;
    ceju1.SerialInit();

    while( ros::ok() )
    {
        int result=ceju1.SerialRead();
        cout<<result<<endl;
    }


}

