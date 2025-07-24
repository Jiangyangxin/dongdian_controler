#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: jyx
Date: 2024-07-09 18:17:29
LastEditTime: 2024-10-03 02:14:38
LastEditors: jyx
Description: 
FilePath: /dongdian_controler/src/steer_track/src/task_node.py
jntm jntm jntm jntm jntm
'''
from numpy.core.records import array
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from steer_track.msg import motion_instruction
from steer_track.msg import task_node_debug
from steer_track.msg import robot_pose_vel
from steer_track.msg import steer_th_vel
import numpy as np
import math
from scipy.interpolate import BSpline
from dynamic_reconfigure.server import Server
from steer_track.cfg import task_node_paramsConfig
import logging
import time
import pickle
import os
import sys
import string
import tf.transformations as tf_trans
import threading

# 创建一个互斥锁
lock = threading.Lock()
# 状态机参数
# state: 0:正常运行状态；1：测量delta yaw状态
state = 0
# 状态1计数器
gdy_cnt = 0
leftlidar_cnt=0
rightlidar_cnt=0
# 测量起点
dy_st_point = [0, 0]
# optitrack传来的值
raw_yaw = 0
# 机器人真实舵角-optitrack传来的值
# delta_yaw = -0.009053085#新微星
# delta_yaw = -0.064477313#老微星
# delta_yaw = 0.031094868#e305老微星
# 重力方向上的误差积分
delta_yaw = 0
vy_i = 0

# 机器人工作区域范围限制,上界与下界
y_limit = [10, -10]
x_limit = [10, -10]
tar_y_limit=[10, -10]
tar_x_limit=[10, -10]
# 位置和速度信息全局变量
current_pose = [0.0, 0.0, math.pi/2]
last_pose = [0.0, 0.0, math.pi/2]

# 控制频率
frequency = 30.0
#最大速度
robot_standard_vel = 0.05
# 机器人的角速度，pi/6=0.5时对应轮子的线速度是 0.244*pi/6=0.127m/s
robot_standard_w = 0.05
# 单个舵轮的最大速度
maxstvel = 0.3
# 单个舵轮的最大、最小角速度
steer_max_w = 3.14
steer_min_w = 0.03
tar_fan_pre = [0.0,0.0,0.0]
polish_speed =0 
#路径点
trajectory_vector = [] #x，y位置和速度
target_follow=[]
#optitrack 超时计数器，用于排除异常点
optitrack_timeout_cnt = 0

turn_vel = 0.02
turn_r = 0.02
pose_pid_p = 4
pose_pid_vel_bia = 0.02
yaw_corr_pid_p = 2
yaw_corr_pid_i = 15

# 根据optitrack是否超时选择，开闭环控制
open_control = True
last_open_control = True

Lfc = 0.15  # 前视距离

global_start_time = 0
pose_time = 0#已经接收到的optitrack位姿的时间
follow_state=0#机器人跟随标志位
# imu相关
imu_yaw = math.pi/2
imu_time = 0
steer_state=1

# lidar相关
left_boundary=[0,0,0,0] #x1 y1 x2 y2
left_dis = 0
left_len = 0
left_Wind_blade_lenth= 0
right_boundary=[0,0,0,0]
right_dis = 0
right_len = 0
right_Wind_blade_lenth= 0
traj_data=Float32MultiArray()#xlimit ylimit k1 k2
'''
description: 获取两个角度值的距离（小于pi的），绝对值
param {*} a1
param {*} a2
return {*}
'''
def get_abs_angle(a1, a2):
    abs_d = abs(a1-a2)
    return min(abs_d, 2*math.pi-abs_d)
'''
description: 限制数值的最大最小值
param {*} min_value
param {*} max_value
return {*}
'''
def limit_number(number, min_value, max_value):
    return min(max(number, min_value), max_value)
'''
description: 获取小于pi的排除过正负pi影响的delta angle，带符号
param {*} tar
param {*} now
return {*}
'''
def get_good_d_angle(tar, now):
    d_angle = tar-now
    if(d_angle<3.14 and d_angle > -3.14):
        return d_angle
    elif(d_angle<-3.14):
        return d_angle+2*math.pi
    else:#d_angle>3.14
        return d_angle-2*math.pi

def get_short_angle(now, tar):
    d = tar - now
    if d > -math.pi/2 and d < math.pi/2:
        return d
    elif d <= -math.pi/2 and d > -math.pi:
        return d + math.pi
    elif d <= -math.pi and d > -math.pi/2*3:
        return d + math.pi
    elif d <= -math.pi/2*3:
        return d + 2*math.pi
    elif d >= math.pi/2 and d < math.pi:
        return d - math.pi
    elif d >= math.pi and d < math.pi/2*3:
        return d - math.pi
    elif d >= math.pi/2*3:
        return d - 2*math.pi

def get_pi_2_angle(now):
    if(now >=-math.pi/2 and now <= math.pi):
        d = math.pi/2-now
    else:
        d = -2*math.pi + math.pi/2 -now
    return d

        
class dynamicRotatePid:
    def __init__(self):
        self.PP = 0.5# dp = 0.1/p = 0.2 rad = 12°开始减速
        self.vmax = 0.05
    def Ptick(self, nowPose, tarPose):#v>0逆时针转动，v<0顺时针转动
        if(tarPose - nowPose >= -math.pi and tarPose - nowPose <= math.pi):
            p_error = tarPose - nowPose
        elif(tarPose - nowPose < -math.pi and tarPose - nowPose >= -2*math.pi):
            p_error = tarPose - nowPose + 2*math.pi
        elif(tarPose - nowPose > math.pi and tarPose - nowPose <= 2*math.pi):
            p_error = tarPose - nowPose - 2*math.pi
        else:
            p_error = 0
        v = self.PP * p_error# 目标速度
        if(v > self.vmax):
            v = self.vmax
        elif(v < -self.vmax):
            v = -self.vmax
        return v
myDRPID = dynamicRotatePid()

'''
@Description: “pose_vel_data”消息订阅回调函数
@param {*} msg “robot_pose_vel”数据类型
@return {*}
'''
def PoseVelSubCallback(robot_pose_vel_msg):
    global current_pose, raw_yaw, delta_yaw,last_pose, optitrack_timeout_cnt,pose_time, poseStamp,imu_yaw
    # if getd(last_pose[0:2], robot_pose_vel_msg.xy)>0.03 or abs(passpi(last_pose[2]-robot_pose_vel_msg.yaw))>0.2:
    if getd(last_pose[0:2], robot_pose_vel_msg.xy)>0.03:
        optitrack_timeout_cnt += 1
        if optitrack_timeout_cnt >= 100:
            optitrack_timeout_cnt = 0
            last_pose[0:2] = robot_pose_vel_msg.xy[:]
            last_pose[2] = robot_pose_vel_msg.yaw
    else:
        optitrack_timeout_cnt = 0
        current_pose[0:2] = robot_pose_vel_msg.xy[:]
        # raw_yaw = robot_pose_vel_msg.yaw  #仅仅在使用optitrack的时候采用yaw，否则yaw统一从imu获得
        raw_yaw=imu_yaw
        current_pose[2] = passpi(raw_yaw + delta_yaw)
        last_pose = current_pose[0:3]
        pose_time = robot_pose_vel_msg.sec_f
        poseStamp = robot_pose_vel_msg.optitrack_pose
        

def IMUyawCallback(yaw_msg):
    global imu_yaw, imu_time
    imu_time = rospy.get_time()
    imu_yaw = yaw_msg.data
    
def left_LidarCallback(left_msg):
    global left_boundary,left_dis,left_len,left_Wind_blade_lenth,leftlidar_cnt
    left_boundary=[0,0,0,0] #x1 y1 x2 y2
    for i in range(4):
        left_boundary[i]=left_msg.data[i]
    left_dis = left_msg.data[4]
    left_len = left_msg.data[5]
    left_Wind_blade_lenth= left_msg.data[6]
    leftlidar_cnt=0

def right_LidarCallback(right_msg):
    global right_boundary,right_dis,right_len,right_Wind_blade_lenth,rightlidar_cnt
    right_boundary=[0,0,0,0]
    for i in range(4):
        right_boundary[i]=right_msg.data[i]
    right_dis = right_msg.data[4]
    right_len = right_msg.data[5]
    right_Wind_blade_lenth= right_msg.data[6]
    rightlidar_cnt=0

def double_LidarCallback(msg):
    global left_boundary,left_dis,left_len,left_Wind_blade_lenth,right_boundary,right_dis,right_len,right_Wind_blade_lenth
    left_boundary=[0,0,0,0]
    for i in range(4):
        left_boundary[i]=msg.data[i]
    left_dis = msg.data[4]
    left_len = msg.data[5]
    left_Wind_blade_lenth= msg.data[6]
    
    right_boundary=[0,0,0,0]
    for i in range(4):
        right_boundary[i]=msg.data[i+7]
    right_dis = msg.data[4+7]
    right_len = msg.data[5+7]
    right_Wind_blade_lenth= msg.data[6+7]
'''
@Description: “motion_instruction”消息回调函数,接收控制指令
@param {*} motion_msg “motion_instruction”数据类型
@return {*}
'''    
def MotionInstructionCallback(motion_msg):
    global trajectory_vector, current_pose, state, dy_st_point, pose_pid_p,\
        pose_pid_vel_bia, trajectory_vector_lock, robot_standard_vel, tar_yaw, \
        open_control, imu_yaw, tar_fan_pre,steer_state,polish_speed,follow_state,target_follow,tar_y_limit,y_limit
    while(trajectory_vector_lock == 1):
        wait = 1
        # print("receive fail")
    if open_control:
        now_yaw = imu_yaw
        
    else:
        x,y,now_yaw = current_pose
    if(motion_msg.mode == 'increment'):
        # if(len(trajectory_vector)!=0):
        #     trajectory_vector = [trajectory_vector[0]]
        #     x = trajectory_vector[0][0] + motion_msg.dx
        #     y = trajectory_vector[0][1] + motion_msg.dy
        #     PlanningTrajectory(trajectory_vector[0], [x,y])
        # else:
        state = 0
        steer_state=1
        follow_state=0
        trajectory_vector = []
        x = current_pose[0] + motion_msg.dx
        y = current_pose[1] + motion_msg.dy
        tar_y_limit=y_limit

        ###机器人分区，限制机器人的移动区域，必须配合激光雷达使用########
        # min_x=-2
        # max_x=2
        # min_y=-left_Wind_blade_lenth/2
        # max_y=left_Wind_blade_lenth/2
        
        # if open_control != True:
        #     limit_number(x,min_x,max_x)
        #     if max_y!=0 and min_y!=0:
        #         limit_number(y,min_y,max_y)
        #####################################
        plan_staight_line(np.array(current_pose[0:2]), np.array([x, y]), 0.001, pose_pid_vel_bia, pose_pid_p) #step（0.001）越小，按一次走的越长
        
    elif(motion_msg.mode == 'absolute'):
        state = 0
        steer_state=1
        follow_state=0
        trajectory_vector = []
        tar_y_limit=y_limit
        plan_staight_line(np.array(current_pose[0:2]), np.array([motion_msg.x,motion_msg.y]), 0.001, pose_pid_vel_bia, pose_pid_p)
    elif(motion_msg.mode == 'start_follow'): #机器人跟随工装移动的指令，此指令需要QT上位机循环发送
        state = 0
        steer_state=1
        follow_state=1
        target_follow=np.array(current_pose[0:2])
        trajectory_vector = []
        #进入FollowCallback 函数中，执行跟随
    elif(motion_msg.mode == 'stop_follow'): #机器人跟随工装移动的指令，此指令需要QT上位机循环发送
        state = 0
        steer_state=1
        follow_state=0
        trajectory_vector = []
    elif(motion_msg.mode == 'set_zeroing'):   
        state = 0
        steer_state=1
        follow_state=1
        trajectory_vector = []
        tar_y_limit=y_limit
        plan_staight_line(np.array(current_pose[0:2]), np.array([-0.14, -0.23]), 0.001, pose_pid_vel_bia, pose_pid_p) 
                
    elif(motion_msg.mode == 'track'):
        state = 0
        steer_state=1
        follow_state=0
        trajectory_vector = []
        tar_y_limit=y_limit
        CreateTrajectoryFromFile()
    elif(motion_msg.mode == 'autotrack'):
        state = 0
        steer_state=1
        follow_state=0
        trajectory_vector = []
        tar_y_limit=y_limit
        CreateTrajectoryFromMotion(motion_msg.per_x,motion_msg.per_y,motion_msg.num_track,motion_msg.orien_track)
    elif(motion_msg.mode == 'stcorr'):
        state = 0
        steer_state=3
        follow_state=0
        # CreateCircleCornerTrajectoryFromFile()
    elif(motion_msg.mode == 'splineTrack'):
        state = 0
        steer_state=1
        follow_state=0
        trajectory_vector = []
        tar_y_limit=y_limit
        CreateSplineTrajectoryFromFile()
    elif(motion_msg.mode == 'robotVel'):
        follow_state=0
        if (motion_msg.robot_vel > 0 and motion_msg.robot_vel <= 0.4):
            robot_standard_vel = motion_msg.robot_vel
            rospy.loginfo('robot_standard_vel:{}'.format(robot_standard_vel))
        else:
            rospy.logwarn('invalid robot_standard_vel {}'.format(robot_standard_vel))
    elif(motion_msg.mode == 'getDeltaYaw'):
        state = 1
        steer_state=1
        follow_state=0
    elif(motion_msg.mode == "CCWiRotate"):
        state = 2
        steer_state=1
        follow_state=0
        tar_yaw = now_yaw + math.pi/30
    elif(motion_msg.mode == "CWiRotate"):
        state = 2
        steer_state=1
        follow_state=0
        tar_yaw = now_yaw - math.pi/30
    elif(motion_msg.mode == "RestAngle"):
        state = 2
        steer_state=1
        follow_state=0
        tar_yaw = math.pi/2
    elif(motion_msg.mode == "set_fan"):
        follow_state=0
        tar_fan_pre[0] = motion_msg.fan_1
        tar_fan_pre[1] = motion_msg.fan_2
        tar_fan_pre[2] = motion_msg.fan_3
    elif(motion_msg.mode == "set_polish"):
        polish_speed=motion_msg.polish_speed

    elif(motion_msg.mode == 'stop'):
        state = 0
        follow_state=0
        steer_state=1
        trajectory_vector = []



'''
@Description: 根据起始点和目标点细化轨迹点,间隔为1cm
@param {*} st start point起始点[x,y]
@param {*} tp target point目标点[x,y]
@return {*}
'''
def PlanningTrajectory(st, tp): #此函数没有用到
    global trajectory_vector, robot_standard_vel
    th = math.atan2(tp[1]-st[1],tp[0]-st[0])
    dx = 0.01*math.cos(th)
    dy = 0.01*math.sin(th)
    step = 0.005
    temp = st[:].extend([0])
    delta_d = getd(temp[0:2], tp)
    while delta_d > step:
        temp[2] = saturate(delta_d, robot_standard_vel, 0)
        trajectory_vector.append(temp[:])
        temp[0]+=dx
        temp[1]+=dy
        delta_d = getd(temp[0:2], tp)
    if(delta_d>0.001):#[***temp****0.001****tp]
        temp[2] = saturate(delta_d, robot_standard_vel, 0)
        trajectory_vector.append(temp[:])
        trajectory_vector.append([tp[:], robot_standard_vel/5])
    else:#[*******0.001**temp**tp]
        trajectory_vector.append([tp[:], robot_standard_vel/5])

'''
@Description: 从config/path.txt中读取路径角点，用PlanningTrajectory函数细化轨迹点
@param {*}
@return {*}
'''
def CreateTrajectoryFromFile():
    global current_pose, trajectory_vector, pose_pid_p, pose_pid_vel_bia
    # with open("/home/ddmp/dongdian_controler/src/steer_track/config/path.txt", 'r') as f:
    with open("../../config/path.txt", 'r') as f:
        line = f.readline()
        lxy = np.array(current_pose[0:2], dtype="float64") #在开环控制中，current_pose始终为[0.0, 0.0, 1.57]不变
        while line:
            temp = line.split() #temp为txt中每一行的数据，例如 temp: ['0.4', '0']
            # print("temp:",temp)
            # print("current_pose:",current_pose)
            x = float(temp[0]) + current_pose[0]
            y = float(temp[1]) + current_pose[1]
            xy =  np.array([x, y])#设定要前往的位置
            plan_staight_line(lxy, xy, 0.001, pose_pid_vel_bia, pose_pid_p)
            lxy = xy.copy() #当前位置转换为上一次的要前往的位置，以便前往下一个点
            line = f.readline()


'''
@Description: 根据距离信息，自动生成一段上下交替的工字形轨迹
@param {*} per_x为每次运动的x的距离,per_y为每次运动的y,num为运动的次数,orien为运动方向
@return {*}
'''
def CreateTrajectoryFromMotion(per_x=0.2,per_y=0.4,num=1,orien=1):
    global current_pose, trajectory_vector, pose_pid_p, pose_pid_vel_bia
    temp_xy =  np.array([0, 0])#设定要前往的位置
    now_xy = np.array(current_pose[0:2], dtype="float64") #在开环控制中，current_pose始终为[0.0, 0.0, 1.57]不变
    while num>0:
        temp_xy = np.array([0, per_y])+now_xy
        plan_staight_line(now_xy, temp_xy, 0.001, pose_pid_vel_bia, pose_pid_p)
        now_xy=temp_xy.copy()#当前位置转换为上一次的要前往的位置，以便前往下一个点

        temp_xy = np.array([orien*per_x, 0])+now_xy
        plan_staight_line(now_xy, temp_xy, 0.001, pose_pid_vel_bia, pose_pid_p)
        now_xy=temp_xy.copy()#当前位置转换为上一次的要前往的位置，以便前往下一个点

        temp_xy = np.array([0, -per_y])+now_xy
        plan_staight_line(now_xy, temp_xy, 0.001, pose_pid_vel_bia, pose_pid_p)
        now_xy=temp_xy.copy()#当前位置转换为上一次的要前往的位置，以便前往下一个点

        temp_xy = np.array([orien*per_x, 0])+now_xy
        plan_staight_line(now_xy, temp_xy, 0.001, pose_pid_vel_bia, pose_pid_p)
        now_xy=temp_xy.copy()#当前位置转换为上一次的要前往的位置，以便前往下一个点

        num=num-1




def plan_staight_line(st, tar, step, vel_bia, pid_p):
    # st表示当前的位置current_pose[0:2],tar 表示目标位置np.array([motion_msg.x,motion_msg.y]),step表示步长 0.001，为估算值，step越小，按一次走的越长
    global robot_standard_vel,y_limit
    temp=[0.0,0.0]
    temp_tar=[0.0,0.0] 
    temp_y=0
    
    temp = st.copy()
    temp_tar= tar.copy()
    #限制目标位置的y轴上限，目前暂时不限制x轴的上限
    # if open_control!=True:
        # temp_y=saturate(temp_tar[1],y_limit[0],y_limit[1])
        # temp_tar[1]=temp_y
        # print("temp_tar:",temp_tar) 
        # print("ylimit:",y_limit)  
    ########################################

    if np.linalg.norm(temp_tar - temp) != 0:
        unitVector = (temp_tar - temp)/np.linalg.norm(temp_tar - temp)#np.linalg.norm为求模长的函数，这一步将坐标差值的矩阵化为单位向量
    else:
        unitVector = np.array([0, 0])


    delta_d = getd(temp, temp_tar)#计算两个坐标间的距离
    while delta_d > step:
        temp_pv = np.append(temp,[saturate(delta_d*pid_p + vel_bia, robot_standard_vel, 0)], axis=0).tolist() #saturate为限制最大最小值，temp_pv被转换为列表形式       
        #temp_pv为一个向量，即根据设定点和反馈位置，生成了很多子轨迹，如 temp_pv: [0.7870000000000004, 0.8, 0.03]，0.03为速度
        trajectory_vector.append(temp_pv[:])
        temp += step*unitVector
        delta_d = getd(temp, temp_tar)
    if delta_d > step/10:#[***temp****0.001****tp]
        temp_pv = np.append(temp,[saturate(delta_d*pid_p + vel_bia, robot_standard_vel, 0)], axis=0).tolist()  
        trajectory_vector.append(temp_pv[:])
        temp_pv = np.append(temp_tar,[saturate(vel_bia, robot_standard_vel, 0)], axis=0).tolist()
        trajectory_vector.append(temp_pv[:])
    else:#[*******0.001**temp**tp]
        temp_pv = np.append(temp_tar, [saturate(vel_bia, robot_standard_vel, 0)], axis=0).tolist()
        trajectory_vector.append(temp_pv[:])

'''
@Description: 从config/path.txt中读取路径角点，规划圆角轨迹
@param {*}
@return {*}
'''
def CreateCircleCornerTrajectoryFromFile(): #此函数没有用到
    global current_pose, trajectory_vector, robot_standard_vel, turn_vel, turn_r, pose_pid_p
    cornerlist = []
    with open("../../config/path.txt", 'r') as f:
        line = f.readline()
        cornerlist.append(current_pose[0:2])
        while line:
            temp = line.split()
            x = float(temp[0]) + current_pose[0]
            y = float(temp[1]) + current_pose[1]
            cornerlist.append([x,y])
            line = f.readline()
    cornerlist = np.array(cornerlist)
    pos_pid_p = pose_pid_p
    circle_step = 0.001
    circle_vel = turn_vel#圆弧转弯速度
    step  = 0.005
    r = turn_r
    for i in range(0, len(cornerlist)):
        if i == 0:
            unitVector = (cornerlist[i+1, :] - cornerlist[i, :])/np.linalg.norm(cornerlist[i+1, :] - cornerlist[i, :])
            tar = cornerlist[i+1, :] - r*unitVector
            unitVector1 = (tar - cornerlist[i, :])/np.linalg.norm(tar - cornerlist[i, :])
            if (i != len(cornerlist) - 2):
                plan_staight_line(cornerlist[0,:].copy(), tar, step, circle_vel, pos_pid_p)
            else:
                plan_staight_line(cornerlist[0, :].copy(), tar, step, saturate(r*pos_pid_p, robot_standard_vel, 0), pos_pid_p)
        elif i != len(cornerlist)-1:
            unitVector1 = (cornerlist[i, :] - cornerlist[i-1, :])/np.linalg.norm(cornerlist[i, :] - cornerlist[i-1, :])
            unitVector2 = (cornerlist[i + 1, :] - cornerlist[i, :]) / np.linalg.norm(cornerlist[i + 1, :] - cornerlist[i, :])
            p1 = cornerlist[i, :] - r * unitVector1
            p2 = cornerlist[i, :] + r * unitVector2
            p3 = cornerlist[i, :].copy()
            unitVector1_3 = np.append(unitVector1, 0)
            unitVector2_3 = np.append(unitVector2, 0)
            th1 = math.atan2(unitVector1[1], unitVector1[0])
            th2 = math.atan2(unitVector2[1], unitVector2[0])
            th = [abs(th2-th1), 2*math.pi-abs(th2-th1)][abs(th2-th1)>math.pi]
            # 第一阶段的折线或圆弧
            if th < math.pi/18 or th > math.pi - math.pi/18:# 折线
                plan_staight_line(p1[:] + step*unitVector1, p3, step, circle_vel, 0)
                plan_staight_line(p3[:] + step*unitVector2, p2, step, circle_vel, 0)
            else:# 圆弧
                unitVector6 = np.cross(np.cross(unitVector1_3, unitVector2_3), unitVector1_3)[0:2]
                unitVector6 = unitVector6/np.linalg.norm(unitVector6)
                po = p1 + r/math.tan(th/2)*unitVector6
                unitVector4 = (p1 - po) / np.linalg.norm(p1 - po)
                unitVector5 = (p2 - po) / np.linalg.norm(p2 - po)
                th4 = math.atan2(unitVector4[1], unitVector4[0])
                th5 = math.atan2(unitVector5[1], unitVector5[0])
                R = r / math.tan(th / 2)
                dth = circle_step/R * [-1, 1][(th5-th4)>0]
                if abs(th5-th4) > math.pi:
                    dth = -dth
                tempth = th4 + dth
                temp = po + R*np.array([math.cos(tempth), math.sin(tempth)])
                while getd(temp, p2) > circle_step:
                    trajectory_vector.append(np.append(temp,[circle_vel],axis=0).tolist())
                    tempth = tempth + dth
                    temp = po + R * np.array([math.cos(tempth), math.sin(tempth)])
                if getd(temp, p2) > circle_step/10:
                    trajectory_vector.append(np.append(temp,[circle_vel],axis=0).tolist())
                    trajectory_vector.append(np.append(p2, [circle_vel], axis=0).tolist())
                else:
                    trajectory_vector.append(np.append(p2, [circle_vel], axis=0).tolist())
            # 第二阶段的直线
            tar = cornerlist[i+1, :]-r*unitVector2
            if np.linalg.norm(tar-p2) != 0:
                unitVector7 = (tar-p2)/np.linalg.norm(tar-p2)
            else:
                unitVector7 = np.array([0, 0])
            if(i != len(cornerlist)-2):
                plan_staight_line(p2 + unitVector7*step, tar, step, circle_vel, pos_pid_p)
            else:
                plan_staight_line(p2 + unitVector7*step, tar, step, saturate(r*pos_pid_p, robot_standard_vel, 0), pos_pid_p)
        else:
            unitVector = (cornerlist[i, :] - cornerlist[i-1, :])/np.linalg.norm(cornerlist[i, :] - cornerlist[i-1, :])
            st = cornerlist[i, :]-r*unitVector
            plan_staight_line(st + step*unitVector, cornerlist[i, :], step, 0, pos_pid_p)


def getPointnum(pointlist):
    sumd = 0.0
    for i in range(len(pointlist)-1):
        sumd += getd(pointlist[i], pointlist[i+1])
    x = sumd/0.01
    return int(x)

def CreateSplineTrajectoryFromFile():
    global current_pose, trajectory_vector, pid_control_timer
    pid_control_timer.shutdown()
    # 获取txt文件数据
    with open("../../config/curpath.txt", 'r') as f:
        line = f.readline()
        cornerlist = []
        cornerlist.append(current_pose[0:2])
        while line:
            temp = line.split()
            x = float(temp[0]) + current_pose[0]
            y = float(temp[1]) + current_pose[1]
            cornerlist.append([x,y])
            line = f.readline()
        cornor = np.array(cornerlist)
        x = cornor[:,0]
        y = cornor[:,1]
    # 计算样条曲线
    num = len(x)
    k = 3   #degree, k越大，曲线越逼近原始控制点
    t = []  #knots vector
    for i in range(num + k + 1):
        if i <= k:
            t.append(0)
        elif i >= num:
            t.append(num - k)
        else:
            t.append(i - k)
    # 生成B样条参数函数
    spl_x = BSpline(t, x, k)
    spl_y = BSpline(t, y, k)
    # 获取要分多少个点
    numofpoint = getPointnum(cornerlist)
    # 选择B样条区间
    xx = np.linspace(0, num-k, numofpoint)
    x_new = spl_x(xx)
    y_new = spl_y(xx)
    point_new = np.vstack((x_new,y_new)).T.tolist()
    trajectory_vector = point_new[:]
    # pid_control_timer = rospy.Timer(rospy.Duration(1 / frequency), PidControlCallback)

'''
@Description: 计算两个坐标间的距离
@param {*} a
@param {*} b
@return {*}
'''        
def getd(a ,b):
    a = np.array(a).T
    b = np.array(b).T
    return np.linalg.norm(a-b)

'''
@Description: 根据前视距离计算目标点
@param {*}
@return {*}
ind 最终目标点索引
toend 是否到了最后一个点还小于前视距离
'''
def calc_target_index():
    global current_pose,trajectory_vector
    ind = 0
    c = current_pose[0:2]
    while True:
        if ind + 1 >= len(trajectory_vector):
            toend = True#此时，ind是最后一个元素
            break
        toend = False
        cur = getd(c, trajectory_vector[ind][0:2])
        next = getd(c, trajectory_vector[ind+1][0:2])
        if cur>=next:
            ind += 1
        else:
            break
    return ind, toend

'''
@Description: 求阿克曼转向圆心坐标和半径
@param {*} pt 目标坐标point target
@return {*}
'''
def getCircle(pt):
    global current_pose
    x,y,yaw = current_pose
    tx,ty = pt
    k1 = math.tan(yaw + math.pi/2)
    tth = math.atan2(ty-y, tx-x)
    k2 = math.tan(tth + math.pi/2)
    x1 = x
    y1 = y
    xm = (x1+tx)/2
    ym = (y1+ty)/2
    a = np.array([[-k1, 1], [-k2, 1]])
    b = np.array([y1-k1*x1, ym-k2*xm]).T
    try:
        c = np.dot(np.linalg.inv(a),b)
    except:
        c = [k1, k1]
    cirx,ciry = c[0], c[1]
    r = math.sqrt(pow(cirx-x1,2)+pow(ciry-y1,2))
    return cirx, ciry, r

'''
@Description: 如该a不在[-pi,pi]之间,将a约束到[-pi,pi]之间
@param {*} a 输入角度
@return {*}
'''
def passpi(a):
    if a > math.pi:
        a -= 2*math.pi
    if a < -math.pi:
        a += 2*math.pi
    return a

'''
@Description: 限制单个舵轮的速度，同时保持差速比
@param {*} input
@return {*}
'''
def lmStVel(input):
    temp = [abs(i) for i in input]
    m = max(temp)
    if m > maxstvel:
        for i in range(3):
            input[i] = input[i]*maxstvel/m
    return input

def saturate(i, max, min):
    if i > max:
        i = max
    if i < min:
        i = min
    return i

'''
@description: 模拟th的变化，假定角速度为steer_max_w
@param {*} cth当前的转角
@param {*} tth更新后的转角
@return {*}
'''
def updateCurTh(cth, tth):
    global steer_max_w, frequency
    temp = cth
    if(abs(tth-cth) > steer_max_w/frequency):
        temp += (tth-cth)/abs(tth-cth)*steer_max_w/frequency
    else:
        temp = tth
    return temp

'''
@Description: 根据舵角与目标向量夹角是否大于90度，则轮子正转或反转,同时限制th未到位时轮子的转速
@param {*} v 输入速度
@param {*} tar 目标坐标
@return {*}
'''
last_direct = 1
cur_th = [0 , 0, 0]#模拟舵轮当前实际角度
def ForB(v, tar, th):
    global current_pose ,last_direct, cur_th
    # 使用反比例函数y = a/(x - b)求缩小倍速k
    b = math.pi*0.05/(0.05-1)
    a = -b*1
    for i in range(3):
        cur_th[i] = updateCurTh(cur_th[i], th[i])
    dth = [abs(th[i] - cur_th[i]) for i in range(3)]
    maxdth = max(dth)
    k = a/(maxdth - b)
    if k >1: k = 1
    v = [k*i for i in v]
    # 判断轮子正转或反转
    yaw = current_pose[2]
    tarth = math.atan2(tar[1]-current_pose[1], tar[0]-current_pose[0])
    dth = abs(tarth - yaw)
    # 使用迟滞比较器，防止夹角在90°附近时自激
    if(last_direct):
        if min([dth, 2*math.pi-dth]) > math.pi/2 + 5.0/180.0*3.14:
            v = [-i for i in v]
            last_direct = 0
        return v
    else:
        v = [-i for i in v]
        if min([dth, 2*math.pi-dth]) < math.pi/2 - 5.0/180.0*3.14:
            v = [-i for i in v]
            last_direct = 1
        return v

'''
Description: 重力补偿
param {*} th
param {*} v
return {*}
'''
def gravityCps(th, v):
    global trajectory_vector, current_pose
    if last_direct:
        yaw = current_pose[2]
    else:
        yaw = passpi(current_pose[2]+math.pi)
    # 原来的速度
    now_v = np.array([0.1*math.cos(yaw), 0.1*math.sin(yaw)]).T
    # 补偿速度
    g = np.array([0, 0.0247]).T
    # 补偿后的速度
    cps_v = g + now_v
    # 将补偿量反馈到轮子速度
    dth = passpi(math.atan2(cps_v[1], cps_v[0]) - math.atan2(now_v[1], now_v[0]))
    th = [passpi(i + dth) for i in th]
    k = np.linalg.norm(cps_v)/np.linalg.norm(now_v)
    v = [k*i for i in v]
    return th, v

'''
@Description: dynamic_reconfigure回调函数
@param {*} 
@return {*}
'''
def params_reconfigure_callback(config, level):
    global turn_vel, turn_r, pose_pid_p, yaw_corr_pid_p, yaw_corr_pid_i, pose_pid_vel_bia
    turn_vel = config["turn_vel"]
    turn_r = config["turn_r"]
    pose_pid_p = config["pose_pid_p"]
    yaw_corr_pid_p = config["yaw_corr_pid_p"]
    yaw_corr_pid_i = config["yaw_corr_pid_i"]
    pose_pid_vel_bia = config["pose_pid_vel_bia"]
    rospy.loginfo("""\
        \nReconfiugre Request\n\
turn_vel : {turn_vel}\n\
turn_r : {turn_r}\n\
pose_pid_p : {pose_pid_p}\n\
pose_pid_vel_bia : {pose_pid_vel_bia}\n\
yaw_corr_pid_p : {yaw_corr_pid_p}\n\
yaw_corr_pid_i : {yaw_corr_pid_i}\n""".format(**config))
    return config


def LidarCallback(event):
    global left_boundary,left_dis,left_len,left_Wind_blade_lenth,right_boundary,\
        right_dis,right_len,right_Wind_blade_lenth,y_limit,x_limit,current_pose,traj_data,traj_pub,leftlidar_cnt,rightlidar_cnt,leftlidarsub,rightlidarsub
    #left_boundary=[0,0,0,0] #x1 y1 x2 y2  y轴竖直向上，x轴垂直于叶片朝内,且 x1y1为上边界，x2y2为下边界
    #注意boundar中的xy是基于激光雷达坐标系的xy
    #根据激光雷达的测量值计算斜率
    lidar_distance=1.1 #两个激光雷达的x轴向的距离
    
    y_bias =  0.2 #激光雷达与相机在y轴上的偏置距离 0.2
    y_safty=  0.72               #安全距离，防止机器人超限
    k1_lidar= (right_boundary[1]-left_boundary[1]) / lidar_distance #上边界斜率
    k2_lidar= (right_boundary[3]-left_boundary[3]) / lidar_distance #下边界斜率
    #为简化计算难度，只考虑机器人工作区域最左边的上下边界获取，右侧边界由斜率推出
    y_left_limit= [left_boundary[1]-y_bias, left_boundary[3]-y_bias] #左侧激光雷达扫描叶片上下边界 在相机坐标系下的y轴坐标, y_bias为激光雷达与相机在y轴上的偏置距离
    x_left_lidar= 0.1 #左侧激光雷达扫描叶片边界 在相机坐标系下的x轴坐标
    
    #debug
    # k1_lidar=-0.5
    # k2_lidar=0
    # y_left_limit[0] = -0.1
    # y_left_limit[1] = -2.0
    # x_left_lidar = 0
    ############
    
    # y_limit[0]= y_left_limit[0] + (current_pose[0]-x_left_lidar) * k1_lidar  -y_safty  # y2= y1+k*(x2-x1) ，上界
    # y_limit[1]= y_left_limit[1] + (current_pose[0]-x_left_lidar) * k2_lidar  - 0       #下界
    
    
    # if right_boundary[1]==0 or left_boundary[1]==0:
    #     y_limit[0]= (right_boundary[1]+left_boundary[1]) -y_safty -y_bias
    #     y_limit[1]= (right_boundary[3]+left_boundary[3]) +0.65 -y_bias
    # else:
    #     y_limit[0]= (right_boundary[1]+left_boundary[1])/2 -y_safty -y_bias
    #     y_limit[1]= (right_boundary[3]+left_boundary[3])/2 +0.65 -y_bias
    # x_limit = [3, -3]
    y_limit[0]= (left_boundary[1]) -y_safty -y_bias
    y_limit[1]= (left_boundary[3]) +0.65 -y_bias

    ############ 
    #若收到的雷达信息超时，取消ylimit
    #lidar_cnt在回调函数会置0
    leftlidar_cnt+=1
    rightlidar_cnt+=1
    if leftlidar_cnt>30 or rightlidar_cnt>30:
        y_limit = [10, -10]
        lidar_reconnect()
    ##traj_data[]:  xlimit ylimit k
    traj_data.data.clear()
    traj_data.data.extend(x_limit)
    traj_data.data.extend(y_limit)
    traj_data.data.append(k1_lidar)
    traj_data.data.append(k2_lidar)
    traj_pub.publish(traj_data)
    # print(traj_data.data)
    # print("left_boundary[1]:",left_boundary[1])    
    # print("right_boundary[1]:",right_boundary[1])
    # print("ylimit:",y_limit)
    # print("ynow:",current_pose[1])
    # print("lidar_cnt:",leftlidar_cnt)

'''
@Description: 工装跟随函数
@param {*} event
@return {*}
'''
def FollowCallback(event):
    global current_pose, Lfc, trajectory_vector,\
         robot_standard_vel, lastlocwheelth, gdy_cnt, dy_st_point,\
             frequency, delta_yaw,state, debug_data_pub, raw_yaw , vy_i,\
            yaw_corr_pid_p, yaw_corr_pid_i, global_start_time, pose_time,\
            trajectory_vector_lock, open_control, imu_yaw, imu_time, last_open_control, steer_state,\
                 tar_fan_pre,polish_speed,follow_state,target_follow,tar_y_limit,y_limit
    while(trajectory_vector_lock == 1):
        wait = 1
        # print("receive fail")        
    if follow_state==1 and steer_state==1 and open_control==False: #开启工装跟随且机器人处于闭环状态
        trajectory_vector = []
        tar_y_limit=y_limit
        # tar_follow=np.array(current_pose[0:2])
        plan_staight_line(np.array(current_pose[0:2]),target_follow, 0.05, pose_pid_vel_bia, pose_pid_p)
        # print("target_follow:",target_follow)    
    else :
        
        pass
'''
@Description: 轨迹跟踪回调函数
@param {*} event
@return {*}
'''
lastlocwheelth = [0, 0, 0]
def PidControlCallback(event):
    global current_pose, Lfc, trajectory_vector,\
         robot_standard_vel, lastlocwheelth, gdy_cnt, dy_st_point,\
             frequency, delta_yaw,state, debug_data_pub, raw_yaw , vy_i,\
            yaw_corr_pid_p, yaw_corr_pid_i, global_start_time, pose_time,\
            trajectory_vector_lock, open_control, imu_yaw, imu_time, last_open_control, steer_state,\
                 tar_fan_pre,polish_speed,tar_y_limit,y_limit
    trajectory_vector_lock = 1 #之前设置 trajectory_vector_lock=1，这样会保证上一次的运动指令被执行，但也会存在失控风险
    with lock: 
        lock_pose_time = pose_time
        ###################################### 若只想开环，注释下面的东西
        if(rospy.get_time()-pose_time > 4):#optitrack超时
            if(last_open_control == False):
                trajectory_vector = []
            open_control = True
            current_pose = [0.0, 0.0, imu_yaw] #将反馈位置归0，避免控制指令出错

        else:                              #optitrack没有超时
            if(last_open_control == True):
                trajectory_vector = []
            open_control = False
        #####################################
        if last_open_control!=open_control and open_control==True:
            print("open_control now!")
        elif last_open_control!=open_control and open_control==False:
            print("close_control now!")  
        last_open_control = open_control
        # debug////////////
        debug_data_msg = task_node_debug()
        # print("opencontrol=",open_control)
        # print("pose_time",pose_time)
        # print(rospy.get_time()-pose_time)
        # /////////////
        if state == 0: 
            steer_fan_msg = steer_th_vel()#获取舵轮与风机的数据
            steer_fan_msg.th_vel = [0,0,0,0,0,0]
            a = 0.370#底盘半径
            D = math.sqrt(3) / 2 * a
            if open_control:#无optitrack时的开环控制，借用了之前的trajectory_vector
                yaw = imu_yaw
                if(len(trajectory_vector)>=3):#3,4,5应该都行
                    locwheelth = [0.0, 0.0, 0.0]#每个轮子相对于车子主轴（前进方向）的夹角
                    wheelv = [0.0, 0.0, 0.0]#每个轮子的转弯速度
                    tar_th = math.atan2(trajectory_vector[1][1]-trajectory_vector[0][1], trajectory_vector[1][0]-trajectory_vector[0][0])
                    robot_line_vel = robot_standard_vel*np.array([math.cos(tar_th), math.sin(tar_th)])#世界坐标系下的线速度
                    #重量补偿值
                    # adjust_vel = robot_standard_vel*np.array([0, 0])
                    adjust_vel = robot_standard_vel*np.array([0, math.tan(0.10)])#0.18为测量估算得到
                    robot_line_vel = robot_line_vel+adjust_vel
                    # 计算线速度分量(世界坐标系下)
                    T_R_W = np.array([[math.cos(yaw), math.sin(yaw)],
                                    [-math.sin(yaw), math.cos(yaw)]])
                    robot_line_vel = np.dot(T_R_W, robot_line_vel.T).T
                    # 计算角速度分量
                    if(yaw >= -math.pi/2 and yaw <= math.pi):
                        # 角度偏差
                        d = math.pi/2-yaw
                    else:
                        d = -2*math.pi + math.pi/2-yaw
                    # w_out = d*0.6*robot_standard_vel/0.1#robot_standard_w = 0.1的情况下，10度开始减速
                    w_out = d*0.4*robot_standard_vel/0.1
                    w_out = saturate(w_out, robot_standard_w, -robot_standard_w)
                    w = np.array([0, 0, w_out])
                    # 每个轮子的位置单位向量
                    # wheellocal = np.array([[a, 0, 0], [-a/2, D, 0], [-a/2, -D, 0]]) #轮子 1 2 3 的xy位置，x为机器人正前方，y为机器人右方
                    # wheellocal = np.array([[-a/2, -D, 0], [-a/2, D, 0], [a, 0, 0]])#老机器人 robot1
                    wheellocal = np.array([[a, 0, 0],[-a/2, -D, 0], [-a/2, D, 0]])#新机器人 robot2
                    # wheellocal = np.array([[a, 0, 0],[-a/2, -D, 0], [-a/2, D, 0]])#新机器人 robot5
                    # wheellocal = wheellocal/(wheellocal.sum(axis = 1)[:,None])#按行求单位向量
                    robot_vel = np.zeros((3,3))
                    for ii in range(3):
                        robot_vel[ii] = np.cross(w, wheellocal[ii]) + np.append(robot_line_vel, 0)
                    for ii in range(3):
                        locwheelth[ii] = math.atan2(robot_vel[ii][1], robot_vel[ii][0])
                        wheelv[ii] = math.sqrt(pow(robot_vel[ii][0], 2) + pow(robot_vel[ii][1], 2))
                    steer_fan_msg.th_vel = locwheelth + wheelv #每个轮子的角度,速度
                    lastlocwheelth = locwheelth[:]
                    trajectory_vector = trajectory_vector[1:]
                    # print(steer_fan_msg)
                else: #轨迹列表为0，速度归0
                    wheelv = [0.0, 0.0, 0.0]
                    locwheelth = lastlocwheelth[:]
                    steer_fan_msg.th_vel = locwheelth + wheelv
                    # print(steer_fan_msg)
                steer_fan_msg.fan_pre = tar_fan_pre
                steer_fan_msg.polish_speed=polish_speed
                steer_fan_msg.steer_state=steer_state
                steer_fan_msg.sec_f = 0
                # print(steer_fan_msg)
                motor_instruction_pub.publish(steer_fan_msg)#发布舵轮和风机数据
                
                
            
            
            else:#闭环控制，相应能收到定位数据的时的移动指令

                x,y,yaw = current_pose
                if(len(trajectory_vector)!=0):
                    tind, toend = calc_target_index()#返回局部最近点
                    trajectory_vector = trajectory_vector[tind:]#pop已经跟踪的点（局部最近点前的点认为已经被跟踪）
                    #限制y#############################
                    iii=0
                    while iii<len(trajectory_vector):
                        temp_y=saturate(trajectory_vector[iii][1],tar_y_limit[0],tar_y_limit[1]) #y_limit
                        trajectory_vector[iii][1]=temp_y 
                        iii= iii+1
                    # temp_y=saturate(trajectory_vector[0][1],y_limit[0],y_limit[1])
                    # trajectory_vector[0][1]=temp_y   
                    # temp_y=saturate(trajectory_vector[1][1],y_limit[0],y_limit[1])
                    # trajectory_vector[1][1]=temp_y
                    # print("max_y:",y_limit[0]) 
                    # print("target_x_y",trajectory_vector[0][0] ,trajectory_vector[0][1]) 
                    ####################################
                    if toend:
                        wheelv = [0.0, 0.0, 0.0]
                        locwheelth = lastlocwheelth[:]
                        msg = steer_th_vel()
                        msg.fan_pre = tar_fan_pre
                        msg.steer_state=steer_state
                        msg.polish_speed=polish_speed
                        msg.th_vel = locwheelth + wheelv
                        
                    else:
                        try:
                            # debug////////////
                            debug_data_msg.path_xy = trajectory_vector[0][0:2]
                            debug_data_msg.path_v = trajectory_vector[0][2] #正方向前进速度
                            debug_data_msg.cur_xy = current_pose[0:2]
                            debug_data_msg.cur_yaw = current_pose[2]
                            # /////////////////
                            locwheelth = [0.0, 0.0, 0.0]#每个轮子相对于车子主轴（前进方向）的夹角
                            wheelv = [0.0, 0.0, 0.0]#每个轮子的转弯速度
                            # 计算线速度分量
                            # 计算原生速度                   
                            tar_th = math.atan2(trajectory_vector[1][1]-trajectory_vector[0][1], trajectory_vector[1][0]-trajectory_vector[0][0])
                            orig_vel = trajectory_vector[0][2]*np.array([math.cos(tar_th), math.sin(tar_th)]) #x和y的速度
                            # 计算偏航矫正速度
                            vector1 = np.array([x-trajectory_vector[0][0], y-trajectory_vector[0][1], 0])
                            unit_vector2 = np.array([math.cos(tar_th), math.sin(tar_th), 0])
                            corr_vel = np.cross(np.cross(vector1, unit_vector2), unit_vector2)
                            # if corr_vel[1]<-0.002:
                            #     corr_vel /= 2
                            corr_vel_p_out = yaw_corr_pid_p*corr_vel[:]*trajectory_vector[0][2]/0.1#相当于0.5秒纠正偏差
                            # 计算重力方向偏航矫正速度积分
                            vy_i = vy_i + corr_vel[1]/yaw_corr_pid_i#控制频率为30hz时，0.5s能累加到corr_vel[1]
                            vy_i = saturate(vy_i, 0.1, -0.1)
                            vy_i_vector = np.array([0, vy_i, 0])
                            vy_i_out = np.cross(np.cross(unit_vector2, vy_i_vector), unit_vector2)*trajectory_vector[0][2]/0.1
                        except IndexError:
                            print("索引超出范围，但程序不会崩溃")
                            vy_i_out=np.array([0,0,0])
                            corr_vel_p_out=np.array([0,0,0])
                            orig_vel=np.array([0,0,0])
                        except Exception as e:
                            print("other error:",e)    
                            
                        # 求和
                        robot_line_vel = orig_vel[:] + corr_vel_p_out[0:2] + vy_i_out[0:2]
                        # 限制速度大小
                        if np.linalg.norm(robot_line_vel)>1.3*robot_standard_vel:
                            robot_line_vel = robot_line_vel/np.linalg.norm(robot_line_vel)*1.3*robot_standard_vel
                        # 计算线速度分量(世界坐标系下)
                        T_R_W = np.array([[math.cos(yaw), math.sin(yaw)],
                                        [-math.sin(yaw), math.cos(yaw)]])
                        # debug////////////
                        debug_data_msg.out_vel = robot_line_vel[0:2]
                        debug_data_msg.orig_vel = orig_vel[0:2]
                        debug_data_msg.corr_p_vel = corr_vel_p_out[0:2]
                        debug_data_msg.corr_i_vel = vy_i_out[0:2]
                        # /////////////////////
                        robot_line_vel = np.dot(T_R_W, robot_line_vel.T).T
                        # 计算角速度分量
                        if(yaw >= -math.pi/2 and yaw <= math.pi):
                            # 角度偏差
                            d = math.pi/2-yaw
                        else:
                            d = -2*math.pi + math.pi/2-yaw
                        # w_out = d*0.6*trajectory_vector[0][2]/0.1#robot_standard_w = 0.1的情况下，10度开始减速
                        try:
                            w_out = d*0.6*trajectory_vector[0][2]/0.1
                            w_out = saturate(w_out, robot_standard_w, -robot_standard_w)
                            w = np.array([0, 0, w_out])
                        except IndexError:
                            print("索引超出范围，但程序不会崩溃")
                            w_out = 0
                            w = np.array([0, 0, w_out])
                        except Exception as e:
                            print("other error:",e) 
                        # 每个轮子的位置单位向量
                        # wheellocal = np.array([[a, 0, 0], [-a/2, D, 0], [-a/2, -D, 0]]) #轮子 1 2 3 的xy位置，x为机器人正前方，y为机器人右方
                        # wheellocal = np.array([[-a/2, -D, 0], [-a/2, D, 0], [a, 0, 0]])#老机器人 robot1
                        wheellocal = np.array([[a, 0, 0],[-a/2, -D, 0], [-a/2, D, 0]])#新机器人 robot2
                        # wheellocal = np.array([[a, 0, 0],[-a/2, -D, 0], [-a/2, D, 0]])#新机器人 robot5
                        # wheellocal = wheellocal/(wheellocal.sum(axis = 1)[:,None])#按行求单位向量
                        robot_vel = np.zeros((3,3))
                        for ii in range(3):
                            robot_vel[ii] = np.cross(w, wheellocal[ii]) + np.append(robot_line_vel, 0) #robot_vel是机器人坐标系下沿着 xyz三个轴速度的向量
                        for ii in range(3):
                            locwheelth[ii] = math.atan2(robot_vel[ii][1], robot_vel[ii][0])
                            wheelv[ii] = math.sqrt(pow(robot_vel[ii][0], 2) + pow(robot_vel[ii][1], 2)) #pow 为求x的y次方
                        # debug
                        # logger.info("robot_line_vel:{}, wheelv:{}".format(robot_line_vel,wheelv))
                        msg = steer_th_vel()
                        msg.fan_pre = tar_fan_pre
                        msg.steer_state=steer_state
                        msg.polish_speed=polish_speed
                        msg.th_vel = locwheelth + wheelv
                        # debug//////////
                        debug_data_msg.sec_f = lock_pose_time
                        debug_data_msg.tar_th_vel = msg.th_vel
                        debug_data_msg.tar_fan_pre = msg.fan_pre
                        debug_data_pub.publish(debug_data_msg)
                        # ////////////
                        lastlocwheelth = locwheelth[:]
                else:
                    wheelv = [0.0, 0.0, 0.0]
                    locwheelth = lastlocwheelth[:]
                    msg = steer_th_vel()
                    msg.fan_pre = tar_fan_pre
                    msg.steer_state=steer_state
                    msg.polish_speed=polish_speed
                    msg.th_vel = locwheelth + wheelv
                # debug//////////
                # debug_data_msg.sec_f = lock_pose_time
                # debug_data_msg.tar_th_vel = msg.th_vel
                # debug_data_pub.publish(debug_data_msg)
                # ////////////
                msg.sec_f = lock_pose_time
                motor_instruction_pub.publish(msg) 
                # print("send close OK!") 
        
        trajectory_vector_lock = 0  #这里一定要设置trajectory_vector_lock=0，防止机器人失控      

def lidar_reconnect():
    global leftlidarsub,rightlidarsub
    if leftlidarsub is not None:
        leftlidarsub.unregister()
    leftlidarsub = rospy.Subscriber('/lidar_info_left', Float32MultiArray, left_LidarCallback, queue_size=10)
    if rightlidarsub is not None:
        rightlidarsub.unregister()
    rightlidarsub=rospy.Subscriber('/lidar_info_right', Float32MultiArray, right_LidarCallback, queue_size=10)
    # rospy.loginfo("Reconnected to lidarpublisher")


'''
@Description: 初始化函数
@param {*}
@return {*}
'''
def alltask():
    global frequency, robot_standard_vel, robot_standard_w,maxstvel, pid_control_timer, motor_instruction_pub, follow_control_timer, debug_data_pub,\
        turn_vel, turn_r, pose_pid_p, yaw_corr_pid_p, yaw_corr_pid_i, pose_pid_vel_bia, global_start_time,traj_pub,leftlidarsub,rightlidarsub
    # ROS节点初始化
    rospy.init_node('task_node_old', anonymous=True) #anonymous=True，表示后面定义相同的node名字时候，按照序号进行排列

    # 位资和速度订阅器,发布方为 get_pose_vel.cpp
    rospy.Subscriber('pose_vel_data', robot_pose_vel, PoseVelSubCallback, queue_size=1)
    # 指令订阅器
    rospy.Subscriber('motion_instruction', motion_instruction, MotionInstructionCallback, queue_size=1)
    # 指令订阅器
    # rosbag record -O test1 /imu_yaw
    rospy.Subscriber('imu_yaw', Float32, IMUyawCallback, queue_size=1)
    # 激光雷达订阅器
    leftlidarsub=rospy.Subscriber('/lidar_info_left', Float32MultiArray, left_LidarCallback, queue_size=10)
    rightlidarsub=rospy.Subscriber('/lidar_info_right', Float32MultiArray, right_LidarCallback, queue_size=10)
    # rospy.Subscriber('/lidar_info_double', Float32MultiArray, double_LidarCallback, queue_size=1)
    # 电机数据发布器
    motor_instruction_pub = rospy.Publisher('motor_instruction', steer_th_vel , queue_size=10)
    # 跟踪目标位置发布器
    # rosbag record -O test /task_node_debug_data /vrpn_client_node/steerRobot/pose
    debug_data_pub = rospy.Publisher('task_node_debug_data', task_node_debug, queue_size=1)
    traj_pub=rospy.Publisher('traj_data', Float32MultiArray, queue_size=1)
# #获取全局参数
# rospy.get_param(’/global_param_name’)
# #获取目前命名空间的参数
# rospy.get_param(‘param_name’)
# #获取私有命名空间参数
# rospy.get_param(’~private_param_name’)
# #获取参数，如果没有，使用默认值
# rospy.get_param(‘foo’,‘default_value’)


    # 控制频率,具体看steer_track/config/params.yaml
    frequency = rospy.get_param('task_node_old/frequency', 30.0)
    # 跟踪速度
    robot_standard_vel = rospy.get_param('task_node_old/target_vel', 0.045)
    # 单个舵轮的最大速度
    maxstvel = rospy.get_param('task_node_old/steer_max_vel', 0.1)
    # 单个舵轮的最大、最小角速度
    robot_standard_w = rospy.get_param('task_node_old/target_w', 0.1)
    
    # global_start_time = rospy.get_time()

    turn_vel         = rospy.get_param('task_node_old/trajectory/turn_vel', 0.02)
    turn_r           = rospy.get_param('task_node_old/trajectory/turn_r', 0.02)
    pose_pid_p       = rospy.get_param('task_node_old/trajectory/pose_pid_p', 4)
    pose_pid_vel_bia = rospy.get_param('task_node_old/trajectory/pose_pid_vel_bia', 0.03)
    yaw_corr_pid_p   = rospy.get_param('task_node_old/trajectory/yaw_corr_pid_p', 2)
    yaw_corr_pid_i   = rospy.get_param('task_node_old/trajectory/yaw_corr_pid_i', 30)
    print(robot_standard_vel)

    # 控制定时器
    print("frequency:",frequency)
    pid_control_timer = rospy.Timer(rospy.Duration(1 / frequency), PidControlCallback)
    follow_control_timer = rospy.Timer(rospy.Duration(3), FollowCallback) #1秒执行一次
    lidar_control_timer = rospy.Timer(rospy.Duration(0.1), LidarCallback) #0.1秒执行一次
    rospy.spin()

if __name__ == '__main__':
    alltask()

