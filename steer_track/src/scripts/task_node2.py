#!/usr/bin/env python
# coding:utf-8
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from bottom_control import BottomControl
from packInfo import packInfo
from steer_track.msg import steer_th_vel
import numpy as np
import time
import os


last_fanstate=0
countt=0
fan_info_array=Float32MultiArray()
fan_info_array.data=[0.0,0.0,0.0,0.0,0.0,0.0]
io_state=0

class Task():
    def __init__(self, frequency, L) -> None:
        self.frequency = frequency
        self.L = L
        #self.steer_angle_comp = [np.pi/12, -(2*np.pi/3 - np.pi/12), 2*np.pi/3 + np.pi/12]
        #self.steer_angle_comp = [np.pi/6, -np.pi/6-np.pi/12, np.pi/2+np.pi/12]
        self.steer_angle_comp = [np.pi/12+np.pi/3, -np.pi/2, np.pi/2+np.pi/12]
        self.steer_state = 0
        self.active_state = 0
        self.fan_state = 0
        
        # self.steer_info = packInfo()
        # self.fan_info = packInfo()
        self.fan_input = [5.0, 5.0, 5.0]
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.steer_v = [0.0, 0.0, 0.0]
        self.steer_angle = [0.0, 0.0, 0.0]
        
        self.fan_ctrl = []
        self.steer_ctrl = []
        self.io_ctrl = []
        self.steer_info = []
        self.fan_info = []
        self.io_info = []
        for i in range(3):
            # self.steer_ctrl.append(BottomControl(ip='192.168.0.10%d'%(i), type='1', frequency=self.frequency))
            # self.fan_ctrl.append(BottomControl(ip='192.168.0.10%d'%(i+3), type='2', frequency=self.frequency))
            self.steer_ctrl.append(BottomControl(ip='192.168.0.10%d'%(i), type='1', frequency=self.frequency))
            # self.fan_ctrl.append(BottomControl(ip='192.168.0.10%d'%(i+3), type='2', frequency=100))
            
            self.steer_info.append(packInfo())
            self.fan_info.append(packInfo())
        self.io_ctrl.append(BottomControl(ip='192.168.0.106', type='4', frequency=self.frequency))
        self.io_info.append(packInfo())
        self.control_timer = rospy.Timer(rospy.Duration(1 / self.frequency), self.taskCallback)
        self.recv_timer = rospy.Timer(rospy.Duration(1 / self.frequency), self.recvCallback)
        # self.velocity_input_sub = rospy.Subscriber('velocity_input', Twist, self.VelocityInputCallback, queue_size=1)#方式一、给机器人x和y方向的速度，后进行解算
        self.motor_instruction_sub = rospy.Subscriber('motor_instruction', steer_th_vel, self.MotionInputCallback, queue_size=10)#方式二、直接给三个舵轮的转角和转速
        self.fan_input_sub = rospy.Subscriber('fan_input', Float32MultiArray, self.FanInputCallback, queue_size=1)
        self.fan_info_pub = rospy.Publisher('fan_pwm_info', Float32MultiArray, queue_size=1)

    def taskCallback(self, event):
        global countt
        
        # print(countt)
        for i in range(3):

            # self.steer_info[i].SteerCmdName["dr1_tar_v"fan_info_pub].SteerValName["dr2_real_p"])
            if self.steer_state == 1:
                countt=0
                self.steer_info[i].SteerCmdName["steer_state"] = self.steer_state
                self.steer_info[i].SteerCmdName["dr2_tar_p"] = round(self.angle_in_pi(self.steer_angle[i] + self.steer_angle_comp[i]), 4)#保留四位小数
                
                if abs(self.angle_in_pi(self.steer_info[i].SteerValName["dr2_tar_p"])-self.angle_in_pi(self.steer_info[i].SteerValName["dr2_real_p"]))>np.pi/12:
                    self.steer_info[i].SteerCmdName["dr1_tar_v"] = round(float(0.0),4)#旋转的时候，舵轮速度为0
                else:
                    self.steer_info[i].SteerCmdName["dr1_tar_v"] = round(self.steer_v[i],4)
                # self.steer_info.SteerCmdName["dr2_tar_p"] = 1.0 #-pi -- pi
            elif self.steer_state == 3:
                countt=countt+1
                if countt<300:
                    self.steer_info[i].SteerCmdName["steer_state"] = 1 #先轮子速度给0，再复位舵角
                    self.steer_info[i].SteerCmdName["dr1_tar_v"] = round(float(0.0),4)
                    self.steer_info[i].SteerCmdName["dr2_tar_p"] = round(self.angle_in_pi(self.steer_angle[i] + self.steer_angle_comp[i]), 4)#保留四位小数
                else:
                    self.steer_info[i].SteerCmdName["steer_state"] = 3 #先轮子速度给0，再复位舵角
                    self.steer_info[i].SteerCmdName["dr1_tar_v"] = round(self.steer_v[i],4)
                    self.steer_info[i].SteerCmdName["dr2_tar_p"] = round(self.angle_in_pi(self.steer_angle[i] + self.steer_angle_comp[i]), 4)#保留四位小数
            else:
                self.steer_info[i].SteerCmdName["steer_state"] = self.steer_state
                countt=0
                
            self.fan_info[i].AdsorptionCmdName["active_state"] = self.active_state
            # print(io_state,"     ",self.fan_input[i])
            if io_state==3 :
                self.fan_info[i].FanCmdName["fan_tar_pre"] = round(self.fan_input[i],2)
                self.fan_info[i].FanCmdName["fan_state"] = self.fan_state
            elif io_state==0 :
                self.fan_info[i].FanCmdName["fan_tar_pre"] = round(0.0,2)   
                self.fan_info[i].FanCmdName["fan_state"] = 0
            
            print(self.steer_info[i].SteerCmdName["steer_state"]) 
            print(self.steer_info[i].SteerCmdName["dr1_tar_v"]) 
            
            self.steer_ctrl[i].sendTask(self.steer_info[i])
            # self.fan_ctrl[i].sendTask(self.fan_info[i])
         
    def recvCallback(self, event):
        global last_fanstate
        global fan_info_array
        global io_state
        self.io_ctrl[0].recvTask(self.io_info[0])
        io_state=self.io_info[0].IOValName["io_state"]
        # print(self.io_info[0].IOValName["io_state"])
        for i in range(3):
            # print(i)
            # print("steer")
            # self.steer_ctrl[i].recvTask(self.steer_info[i])
            # print("fan_input: ",self.fan_input[i])
            self.fan_ctrl[i].recvTask(self.fan_info[i])
            fan_info_array.data[i] = round((float)(self.fan_info[i].FanValName["fan_real_pre"]),4)
            fan_info_array.data[i+3] = (float)(self.fan_info[i].FanValName["fan_pwm"])
        # print(task_object.fan_info[0].FanValName["fan_real_pre"])
        # print(task_object.fan_info[1].FanValName["fan_real_pre"])
        # print(task_object.fan_info[2].FanValName["fan_real_pre"]) 
        if(self.fan_input[0]==0 and self.fan_input[1]==0 and self.fan_input[2]==0):
            self.fan_state = 0     
        else:
            self.fan_state = 1
        self.fan_info_pub.publish(fan_info_array) # 发布风机数据
              
    def VelocityInputCallback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        # self.input_v_to_steer_v()
        A = np.array([[1, 0, 0], [0, 1, self.L],
                     [1, 0, -self.L*np.cos(np.pi/6)], [0, 1, -self.L*np.sin(np.pi/6)],
                     [1, 0, self.L*np.cos(np.pi/6)], [0, 1, -self.L*np.sin(np.pi/6)]])
        input_v = np.array([self.vx, self.vy, self.wz])
        steer_v_projection = A@input_v
        for i in range(3):
            self.steer_v[i] = np.sqrt(steer_v_projection[2*i]**2 + steer_v_projection[2*i+1]**2)
            self.steer_angle[i] = np.arctan2(steer_v_projection[2*i+1], steer_v_projection[2*i])
            #print(self.steer_angle[i])
            # self.steer_v[i] = 0.5
            # self.steer_angle[i] = 0.0
    def MotionInputCallback(self, msg):
        # print(msg)
        self.steer_state=msg.steer_state
        if self.steer_state==3:
            for i in range(3):
                self.steer_v[i] = 0
                self.fan_input[i] = msg.fan_pre[i]
        else:        
            for i in range(3):
                self.steer_v[i] = msg.th_vel[i+3]
                self.steer_angle[i] = msg.th_vel[i]
                if msg.fan_pre[i]>=3:  #避免输入过小的负压
                    self.fan_input[i] = msg.fan_pre[i]
                else:
                    self.fan_input[i] = 3 

    def FanInputCallback(self, msg):
        for i in range(3):
            if msg.data[i]>3:
             self.fan_input[i] = msg.data[i]
            else:
             self.fan_input[i] = 3  
                    

                
    def input_v_to_steer_v(self,):
        A = np.array([1, 0, 0], [0, 1, self.L],
                     [1, 0, -self.L*np.cos(np.pi/6)], [0, 1, -self.L*np.sin(np.pi/6)],
                     [1, 0, self.L*np.cos(np.pi/6)], [0, 1, -self.L*np.sin(np.pi/6)])
        input_v = np.array([self.vx, self.vy, self.wz])
        steer_v_projection = A@input_v
        for i in range(3):
            self.steer_v[i] = np.sqrt(steer_v_projection[2*i]**2 + steer_v_projection[2*i+1]**2)
            self.steer_angle[i] = np.arctan2(steer_v_projection[2*i+1], steer_v_projection[2*i])
            
    def robot_reset(self,):
        # self.active_state = 3
        # self.steer_state = 3
        # self.fan_state = 0
        # self.steer_info.SteerCmdName["steer_state"] = 3
        # self.fan_info.AdsorptionCmdName["active_state"] = 3
        # self.fan_info.FanCmdName["fan_state"] = 0
        # for i in range(3):
        #     self.steer_ctrl[i].sendTask(self.steer_info)
        #     # rospy.Duration(0.1).sleep()
        #     time.sleep(0.1)
        #     self.fan_ctrl[i].sendTask(self.fan_info)
        
        # rospy.Duration(30.0).sleep()
        # time.sleep(10.0)
        # self.active_state = 0
        # self.fan_state = 2
        # self.fan_info.AdsorptionCmdName["active_state"] = 0
        # self.fan_info.FanCmdName["fan_state"] = 2
        # for i in range(3):
        #     self.fan_ctrl[i].sendTask(self.fan_info)
        
        # rospy.Duration(10.0).sleep()
        # time.sleep(3.0)
        self.active_state = 0
        self.fan_state = 0
        self.steer_state = 0
        time.sleep(1.0)
        # self.fan_info.AdsorptionCmdName["active_state"] = 0
        # self.fan_info.FanCmdName["fan_state"] = 0
        # for i in range(3):
        #     self.fan_ctrl[i].sendTask(self.fan_info)
    
    # def stop():
    #     for i in range(3):
    #         steer_ctrl[i].close()
    
    
    def robot_stop(self,):
        self.active_state = 0
        self.steer_state = 0
        self.fan_state = 1
           
    def start_control(self,):
        self.active_state = 0
        self.fan_state = 1
        self.steer_state = 1 
  
                 
    def angle_in_pi(self, angle) -> float:
        while angle > np.pi:
            angle = angle - 2*np.pi
        while angle < -np.pi:
            angle = angle + 2*np.pi
        return angle
        

if __name__ == "__main__":
    rospy.init_node("task_node", anonymous=False)
    task_object = Task(60.0, 0.370)
    # task_object.robot_reset()
    # rospy.Duration(30.0).sleep()
    task_object.start_control()
    while not rospy.is_shutdown():                
                 
        rospy.spin() 
    if(rospy.is_shutdown()):
        print("over  ")   
        for i in range(3):
            task_object.steer_ctrl[i].tcpCliSock.close() 
            task_object.fan_ctrl[i].tcpCliSock.close()
        time.sleep(1.0)
        os._exit(0)   
