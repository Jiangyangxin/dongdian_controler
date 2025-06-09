#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: lxs
Date: 2021-08-18 19:39:31
LastEditTime: 2023-02-20 22:40:06
LastEditors: lxs
Description: 
FilePath: \steerPrototype-debug_motor_info\steer_track\src\imu_kalman.py
jntm jntm jntm jntm jntm
'''
import numpy as np
from rosgraph import xmlrpc
import rospy
from rospy.core import rospyinfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import tf
from scipy.spatial.transform import Rotation
import math

class EKF(object):
    def __init__(self, n, m, pval=0.1, qval=1e-4, rval=0.1):
        '''
        Creates a KF object with n states, m observables, and specified values for 
        prediction noise covariance pval, process noise covariance qval, and 
        measurement noise covariance rval.
        '''

        # No previous prediction noise covariance
        self.P_pre = None

        # Current state is zero, with diagonal noise covariance matrix
        self.x = np.array([1.0, 0.0, 0.0, 0.0])
        self.P_post = np.eye(n) * pval

        # Set up covariance matrices for process noise and measurement noise
        self.Q = np.eye(n) * qval
        self.R = np.eye(m) * rval
 
        # Identity matrix will be usefel later
        self.I = np.eye(n)

    def step(self, z, w, dt):
        '''
        Runs one step of the EKF on observations z, where z is a tuple of length M.
        Returns a NumPy array representing the updated state.
        '''

        # Predict ----------------------------------------------------

        # $\hat{x}_k = f(\hat{x}_{k-1})$
        self.x, F = self.f(self.x, w, dt)
        

        # $P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}$
        self.P_pre = F * self.P_post * F.T + self.Q

        # Update -----------------------------------------------------

        h, H = self.h(self.x)
        
        # $G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1}$
        G = np.dot(self.P_pre.dot(H.T), np.linalg.inv(H.dot(self.P_pre).dot(H.T) + self.R))

        x_cmp = np.dot(G, (np.array(z) - h.T).T)

        # $\hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k))$
        self.x += x_cmp

        # $P_k = (I - G_k H_k) P_k$
        self.P_post = np.dot(self.I - np.dot(G, H), self.P_pre)

        # log info
        # rospy.loginfo("x:" + str(np.linalg.norm(self.x)))
        # rospy.loginfo("h:"+str(np.linalg.norm(h)))
        # rospy.loginfo("z:"+str(np.linalg.norm(np.array(z))))
        # rospy.loginfo("R:"+str(np.linalg.det(H.dot(self.P_pre).dot(H.T))))
        # rospy.loginfo("Q:"+str(np.linalg.det(F * self.P_post * F.T)))
        # rospy.loginfo("G:" + str(G))
        # rospy.loginfo(self.P_post)

        # return self.x.asarray()
        return self.x

    def f(self, x, w, dt):
        '''
        Your implementing class should define this method for the state-transition function f(x).
        Your state-transition fucntion should return a NumPy array of n elements representing the
        new state, and a nXn NumPy array of elements representing the the Jacobian of the function
        with respect to the new state.  Typically this is just the identity
        function np.copy(x), so the Jacobian is just np.eye(len(x)).  '''
        # o = [[0, -w[0], -w[1], -w[2]],
        #      [w[0], 0, w[2], -w[1]],
        #      [w[1], -w[2], 0, w[0]],
        #      [w[2], w[1], -w[0], 0]]
        o = [[0, -w[0], -w[1], -w[2]],
             [w[0], 0, w[2], -w[1]],
             [w[1], -w[2], 0, w[0]],
             [w[2], w[1], -w[0], 0]]
        o = np.array(o)
        A = self.I + 0.5*dt*o
        # A = self.I
        # rospy.loginfo("f:" + str(np.linalg.det(A)))
        # rospy.loginfo(self.I)
        x = A.dot(x)
        return x, A

    def h(self, x):
        '''
        Your implementing class should define this method for the observation function h(x), returning
        a NumPy array of m elements, and a NumPy array of m x n elements representing the Jacobian matrix
        H of the observation function with respect to the observation. For
        example, your function might include a component that turns barometric
        pressure into altitude in meters.
        '''
        h = [2*x[1]*x[3]-2*x[0]*x[2], 
            2*x[0]*x[1]+2*x[2]*x[3],
            x[0]**2-x[1]**2-x[2]**2+x[3]**2]
        H = [[-2*x[2], 2*x[3], -2*x[0], 2*x[1]],
             [2*x[1], 2*x[0], 2*x[3], 2*x[2]],
             [2*x[0], -2*x[1], -2*x[2], 2*x[3]]]
        h = np.array(h)
        H = np.array(H)
        return h, H
        
ekf = EKF(4, 3, pval=0.1, qval=3.5e-2, rval=8e-1)
def imuRawDataSubCallback(msg):
    global time_last, ekf, x, yaw_data_pub
    dt = (rospy.Time.now() - time_last).to_sec()
    time_last  = rospy.Time.now()
    angle_w = msg.data[0:3]
    angle_w = [i*np.pi/180 for i in angle_w]
    gryo = msg.data[3:6]
    x = ekf.step(gryo, angle_w, dt)
    x = x/np.linalg.norm(x)
    br = tf.TransformBroadcaster()
    br.sendTransform((1, 1, 0),
                     (x[1], x[2], x[3], x[0]),
                     rospy.Time.now(),
                     "imu",
                     "world")
    # (r, p, y) = tf.transformations.euler_from_quaternion([x[1], x[2], x[3], x[0]])
    # rospy.loginfo(r)
    r_matrix = Rotation.from_quat((x[1], x[2], x[3], x[0]))
    Rm = r_matrix.as_matrix()
    x_world = -Rm[:,0]
    msg_yaw = Float32()
    msg_yaw.data = math.atan2(x_world[2], -x_world[0])
    yaw_data_pub.publish(msg_yaw)
    

def alltask():
    global time_last, yaw_data_pub
    # ROS节点初始化
    rospy.init_node('imu_kalman', anonymous=True)

    time_last = rospy.Time.now()

    # imu订阅器
    rospy.Subscriber('imu_data', Float32MultiArray, imuRawDataSubCallback, queue_size=1)

    # yaw角数据发布器
    yaw_data_pub = rospy.Publisher('yaw_data', Float32, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    alltask()