#!/usr/bin/env python3
# coding:utf-8
import numpy as np  
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import os
import rospy
import tf2_ros
import tf2_geometry_msgs
import transforms3d as tfs
import cv2
# 标定机器人相机和Apriltag的代码，平时不调用

# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped
# from geometry_msgs.msg import PointStamped
from scipy.optimize import minimize  
  
# 定义损失函数（目标函数）  
def loss_function(X_flat, A, B):  
    X = X_flat.reshape(A.shape)  # 将一维数组重新整形为矩阵  
    return np.linalg.norm(X.dot(A).dot(X) - B, 'fro')  # 使用 Frobenius 范数作为损失  
  
# 定义梯度函数（对于损失函数关于 X 的梯度）  
# 注意：这里的梯度计算可能是近似的，需要更精确的数值微分或符号微分  
def gradient_function(X_flat, A, B):  
    X = X_flat.reshape(A.shape)  # 将一维数组重新整形为矩阵  
    dX = 2 * (X.dot(A).dot(X) - B).dot(A).dot(X)  
    return dX.flatten()  # 将梯度矩阵重新整形为一维数组  
  


n = 4
# A = np.random.rand(n, n)  # 假设 A 是一个 n x n 的方阵  
# B = np.random.rand(n, n)  # B 也是 n x n 的方阵  
# 定义旋转矩阵R和平移向量T
R1 = np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
R2 = np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
A = np.random.rand(n, n)  # 假设 A 是一个 n x n 的方阵  
B = np.random.rand(n, n)  # B 也是 n x n 的方阵  


if __name__ == "__main__":
    # global A,B,R1,R2
    rospy.init_node("biaoding_node", anonymous=False)
    
    buffer1 = tf2_ros.Buffer()
    listener1 = tf2_ros.TransformListener(buffer1)
    point_target1=PointStamped()

    
    buffer2 = tf2_ros.Buffer()
    listener2 = tf2_ros.TransformListener(buffer2)
    point_target2=PointStamped()
 
    buffer3 = tf2_ros.Buffer()
    listener3 = tf2_ros.TransformListener(buffer3)
    point_target3=PointStamped()
    
    rate = rospy.Rate(1)
    init=1
    
    while not rospy.is_shutdown():    
        try:
    #     5.调研订阅对象的 API 将 4 中的点坐标转换成相对于 world 的坐标
            if init>0:
                # buffer1.waitForTransform("hikrobot_camera","apriltag",rospy.Time(0),rospy.Duration(3.0))
                tfs1 = buffer1.lookup_transform("apriltag","hikrobot_camera",rospy.Time(0),rospy.Duration(3.0))
                tfs2=  buffer2.lookup_transform("hikrobot_camera","apriltag3",rospy.Time(0),rospy.Duration(3.0))
                tfs3 = buffer3.lookup_transform("apriltag2","pylon_camera",rospy.Time(0),rospy.Duration(3.0))

                # tfs1.transform.rotation.x
                # tfs1.transform.translation.x
                # 初始化 X（与 A 大小相同的单位矩阵）
                n = 4
                # A = np.random.rand(n, n)  # 假设 A 是一个 n x n 的方阵  
                # B = np.random.rand(n, n)  # B 也是 n x n 的方阵  
                # 定义旋转矩阵R和平移向量T
                R1 = np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
                T1 = np.asarray([tfs1.transform.translation.x,tfs1.transform.translation.y,tfs1.transform.translation.z])

                R2 = np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
                T2 = np.asarray([tfs2.transform.translation.x,tfs2.transform.translation.y,tfs2.transform.translation.z])      

                R3 = np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
                T3 = np.asarray([tfs3.transform.translation.x,tfs3.transform.translation.y,tfs3.transform.translation.z])   
                                
                R1 = tfs.quaternions.quat2mat([tfs1.transform.rotation.w,tfs1.transform.rotation.x,tfs1.transform.rotation.y,tfs1.transform.rotation.z])
                R2 = tfs.quaternions.quat2mat([tfs2.transform.rotation.w,tfs2.transform.rotation.x,tfs2.transform.rotation.y,tfs2.transform.rotation.z])
                R3 = tfs.quaternions.quat2mat([tfs3.transform.rotation.w,tfs3.transform.rotation.x,tfs3.transform.rotation.y,tfs3.transform.rotation.z])
                 
                A=tfs.affines.compose(T1,R1,[1,1,1])
                B=tfs.affines.compose(T2,R2,[1,1,1])
                C=tfs.affines.compose(T3,R3,[1,1,1])                
                print("A:",A)
                print("B:",B)
                print("C:",C)
                # print("B:",B)
                # A=np.array([[1,1,1,tfs1.transform.rotation.x],
                #             [1,1,1,tfs1.transform.rotation.y],
                #             [1,1,1,tfs1.transform.rotation.z],   
                #             [1,1,1,1],
                #             ])
                
                # X0 = np.eye(n).flatten()  # 初始化 X 为 n x n 的单位矩阵并转换为一维数组  
                # # print("X0:",X0)
                # # 约束条件：X 必须是方阵且元素无特殊限制（这里不添加显式约束）  
                
                # # 定义优化问题并求解  
                # # 注意：这里可能需要调整学习率、迭代次数等参数以改善收敛  
                # # result = minimize(loss_function, X0, args=(A, B), jac=gradient_function, method='Nelder-Mead',options={'maxiter':20000,'disp':True})  
                # result = minimize(loss_function, X0, args=(A, B), method='Nelder-Mead',options={'maxiter':20000,'disp':True})  
                # # 获取最优解并重新整形为矩阵  
                # X_opt = result.x.reshape(n, n)  
                
                # # finalR,finalT=cv2.calibrateHandEye(R1,T1,R2,T2)
                # # 检查解是否满足方程（由于数值误差，可能只是近似满足）  
                # print(np.linalg.norm(X_opt.dot(A).dot(X_opt) - B, 'fro'))
                # print(X_opt)
                finalT=A@B
                finalT=finalT@C
                # print('finalR:',finalR) 
                print('invfinalT:',np.linalg.inv(finalT))  
                print('finalT:',finalT)          
            else:
                exit(0) 
        except Exception as e:
            rospy.logerr("异常:%s",e)
            exit(0)
        
        init-=1    

    #     6.spin
        rate.sleep()
        

    if(rospy.is_shutdown()):
        time.sleep(1.0)
        os._exit(0)  