/*** 
 * @Author: lxs
 * @Date: 2021-07-09 15:29:48
 * @LastEditTime: 2021-07-10 16:25:39
 * @LastEditors: lxs
 * @Description: 
 * @FilePath: /steer_track_ws/src/steer_track/include/steer_track/serial_instruction.h
 * @jntm jntm jntm jntm jntm
 */
#ifndef CLIMBOT_VEL_CONTROL_SERIAL_INSTRUCTION
#define CLIMBOT_VEL_CONTROL_SERIAL_INSTRUCTION

#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include "steer_track/steer_th_vel.h"
class Serial_Prototype
{
protected:
    serial::Serial ros_ser_;
public:
    /*! 
    * \brief 初始化串口
    * 
    * \param port Jetson nano串口的端口
    */
    void SerialInit(std::string port);

    /*!
    * \brief 关闭串口
    */
    void CloseSerial();

    /*!
   * \brief 读取串口数据
   * 
   * \return 串口缓冲区数据
   */
    std::string SerialRead();

    /*** 
     * @brief: 
     * @param {const} std
     * @return {*}
     */    
    size_t SerialWrite(const std::string str);


};

class Serial_Instruction:public Serial_Prototype
{
public:
    /*!
    * \brief 风机停止
    */
    void StopFans(void);


   /*!
   * \brief 设置电机参数（速度值）
   *
   * \param vx x方向速度
   * \param vy y方向速度
   * \param vw 运动角速度
   */
    void SetMotorVec(float vx, float vy, float vw);


    /*!
    * \brief 设置风机参数（负压值）
    *
    * \param pre1 风机1负压值
    * \param pre2 风机2负压值
    * \param pre3 风机3负压值
    */
    void SetFan(float pre1, float pre2, float pre3);

    /*!
    * \brief 限制速度指令最大值
    * 
    * \param input_value 速度值
    * \param max 速度值绝对值上限
    */
    void SpeedLimit(float& input_value, float max);


    /*!
    * \brief 限制风机负压最大值
    * 
    * \param input_value 风机负压指令
    * \param max 风机默认负压最大值，默认为0kPa
    * \param min 风机的负压最小值，默认为-10kPa
    */
     void FanLimit(float& input_value, float max = 0.0, float min = -10.0);
     
     /*** 
      * @Description: 发送舵轮的控制指令[th1,th2,th3][v1,v2,v3]
      * @param {float} *data
      * @return {*}
      */
     void setSteerTHV(const steer_track::steer_th_velConstPtr& msg);

     void float2hex(float f, char h[4]);

     void double2hex(double d, char h[8]);
};

#endif
