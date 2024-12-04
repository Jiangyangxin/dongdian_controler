#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import std_msgs.msg as msg
import scipy.signal as signal
from matplotlib.animation import FuncAnimation
from datetime import datetime
import time
import os
from std_msgs.msg import Float32MultiArray

# global Bdis,Blen,Bangle,Bboundary,count,times
# times=1
# count=0.
# Blen=0.
# Bangle=np.array([])
# Bboundary=np.array([])
# Bdis=np.array([])
# lidar_info_array=Float32MultiArray()

class Lidar_Bladelenth():
    def __init__(self,publisher) -> None:
        self.times=1
        self.count=0.
        self.Blen=0.
        self.Bangle=np.array([])
        self.Bboundary=np.array([])
        self.Bdis=np.array([])
        self.lidar_info_array=Float32MultiArray()
        self.publisher=publisher
    
    def find_sequence(arr, seq):#在arr中查找序列seq，每个序列都由0隔开，序列由0结尾
        seq = np.array(seq)
        arr = np.array(arr)
        n = seq.size
        count=0
        index=np.array([])
        for i in range(arr.size - n):#只需要遍历到索引l-n-1处
            if np.array_equal(arr[i:i + n], seq) and arr[i + n]==0:
                index=np.append(index,i)
                count=count+1
        return count,index#False index的数据类型是浮点
    def getlength(self,distance,angle_incre):#估计物体长度 distance是每个点的距离值
        distance=np.array(distance)
        distance=signal.medfilt(distance, kernel_size=5)#降噪
        L=np.array([])
        for i in range(distance.size-1):
            l1=distance[i]
            l2=distance[i+1]
            L=np.append(L,np.sqrt(l1**2+l2**2-2*l1*l2*np.cos(angle_incre)))
        return L.sum()
    def count_invalidpts(self,distance, i):  # 计算连续无效点的数目
        j = 0
        try:
            while distance[i] == 0:
                j = j + 1
                i = i + 1
        except IndexError:
            # print('索引超限')
            m = 1
        return j
    def repeated_remove(self,dis,angle,objcount):
        #去除所有重复序列，每个序列之间用0隔开，返回新序列，以及重复序列的个数
        dis=np.array(dis)
        angle=np.array(angle)
        bdis=dis.copy()
        bangle=angle.copy()
        dismax=np.max(dis)
        anglemax=np.max(angle)

        temp = np.array([])
        indices=np.array([])
        mask = np.ones(len(dis), dtype=bool)#全为True
        for i in range(dis.size):
            if bdis[i]!=dismax+1:
                num=dis[i]
                temp = np.append(temp, num)#从0之间获取序列
                if num == 0:
                    if temp.size <= 1:
                        temp = np.array([])
                        continue
                    temp=temp[:-1]
                    #查找这个序列
                    count, index=self.find_sequence(dis, temp)#temp是序列，去除了末尾的0
                    if count > 1:
                        # print('存在%d个重复序列，序列如下') %count
                        # print('//CHONGFU_SEQ_CHONGFU_SEQ')
                        # print(temp)
                        # print('CHONGFU_SEQ_CHONGFU_SEQ//')
                        for j in range(count-1):
                            #创建一个遮罩，标记所有存在重复序列的位置
                            j=j+1
                            indices=np.append(indices,np.array([index[j]+x for x in range(temp.size)]))
                            indices=indices.astype(np.int32)
                            mask[indices] = False
                            mask[indices[-1]+1]=False#标记重复的序列以及之后的0
                        bdis[~mask]=dismax+1
                        bangle[~mask]=anglemax+1
                    temp = np.array([])
        mask = bdis != (dismax + 1)
        newdis=bdis[mask]
        newangle=bangle[mask]
        objcount= newdis.size - np.count_nonzero(newdis)
        return newdis,newangle,objcount
    def each_relative_error(self,arr1,arr2):#arr1作为相对误差的基准
        error=np.abs(arr1-arr2)
        standard=np.abs(arr1/10)
        standard=np.where(standard<0.1,0.1,standard)
        error=error-np.abs(arr1/10)
        if (error<0).all():
            return True
        else:
            return False
    def update_Bdis(self,nowBdis,nowBangle,angle_incre):
        global Bdis,Bangle,count
        i = 0
        if nowBangle[0] <= Bangle[0]:  # B
            while abs(nowBangle[i] - Bangle[0]) >= angle_incre:
                i = i + 1
            if nowBangle.size - i >= Bangle.size:  # B1 OK
                print('B1')
                Bdis = Bdis * (count - 1) / count
                Bdis = Bdis + nowBdis[i:i + Bangle.size] / count
                if nowBangle.size - i > Bangle.size:
                    Bdis = np.append(Bdis, nowBdis[i + Bangle.size:])
                Bdis = np.concatenate((nowBdis[:i], Bdis))
                Bangle = nowBangle.copy()
            else:  # B2
                print('B2')
                indicies = np.arange(0, nowBangle.size - i, 1)
                np.multiply.at(Bdis, indicies, (count - 1) / count)
                np.add.at(Bdis, indicies, nowBdis[i:] / count)
                Bdis = np.concatenate((nowBdis[:i], Bdis))
                Bangle = np.concatenate((nowBangle[:i], Bangle))
        elif nowBangle[0] > Bangle[0]:  # A
            while abs(nowBangle[0] - Bangle[i]) >= angle_incre:
                i = i + 1
            if nowBangle.size >= (Bangle.size - i):  # A1  Bangle.size - i + 1是Bangle中，i到末尾的序列的长度
                print('A1')
                indicies = np.arange(i, Bangle.size, 1)
                np.multiply.at(Bdis, indicies, (count - 1) / count)#Bdis从i到末尾都与新获得值 取平均值
                np.add.at(Bdis, indicies, nowBdis[:Bdis.size - i] / count)
                if nowBangle.size > (Bangle.size - i):
                    Bdis = np.append(Bdis, nowBdis[Bangle.size - i + 1:])
                    Bangle = np.append(Bangle, nowBangle[Bangle.size - i + 1:])
            else:  # A2
                print('A2')
                indicies = np.arange(i, i + nowBangle.size,1)
                np.multiply.at(Bdis, indicies, (count - 1) / count)
                np.add.at(Bdis, indicies, nowBdis / count)
    def showobj_dis_coord(self,obj_dis,obj_angle,obj_count,angle_incre):
        global Bdis,Bboundary,Bangle,count,times,lidar_info_pub
        fig, (ax1,ax2,ax3) = plt.subplots(1, 3, subplot_kw={'projection': 'polar','aspect': 'equal'},figsize=(13, 6))
        plt.rcParams.update({'font.size': 12})
        plt.subplots_adjust(wspace=0.3)

        nowBdis=np.array([])
        nowBangle = np.array([])
        nowBlen = 0.
        dis=np.array([])#存储物体上每个点的距离数据
        angle = np.array([])
        tolerance=0.25
        i=0
        j=1

        while j<=obj_count:#获取每个物体的长度，边界坐标，并找到最大物体
            try:
                while obj_dis[i]!=0:
                    dis=np.append(dis,obj_dis[i])
                    angle=np.append(angle,obj_angle[i])
                    i=i+1
                obj_length = self.getlength(dis, angle_incre)
                if obj_length > nowBlen:
                    nowBdis = dis
                    nowBangle = angle
                    nowBlen = obj_length
                dis = np.array([])
                angle = np.array([])
                j = j + 1
                i = i + 1  # 把i移到下一个物体的距离数据开头处，但在处理完最后一个物体后这个操作会导致索引超限
            except IndexError:  # 处理到了最后一个物体
                pass
        x1 = nowBdis[0] * np.cos(nowBangle[0])
        x2 = nowBdis[-1] * np.cos(nowBangle[-1])
        y1 = nowBdis[0] * np.sin(nowBangle[0])
        y2 = nowBdis[-1] * np.sin(nowBangle[-1])
        nowBboundary=np.array([x1,y1,x2,y2])
        Wind_blade_lenth=abs(x1-x2)  #风电叶片的长度
        
        if count==0.:
            Bdis=nowBdis.copy()
            Bangle=nowBangle.copy()
            Blen=nowBlen
            Bboundary=nowBboundary
            count=1.
            str=" "
        else:#根据本次迭代的最大物体的长度，边界位置进行不同操作。
            Blen=self.getlength(Bdis, angle_incre)
            if abs(Blen-nowBlen)<tolerance*Blen and self.each_relative_error(Bboundary,nowBboundary):#如果长度差值和边界坐标的差值在限定范围内，
                count=count+1
                Bboundary=Bboundary*(count-1)/count#取平均值show
                Bboundary=Bboundary+nowBboundary/count

                self.update_Bdis(nowBdis, nowBangle,angle_incre)

                Blen=self.getlength(Bdis,angle_incre)
                print(("/***第%u次迭代,发现了相似物体,正常更新数据***/") % times)
                str="similar"
            elif abs(nowBlen-Blen)>tolerance*Blen:
                if nowBlen>Blen:
                    print("目前长度")
                    print(nowBlen)
                    print("之前长度")
                    print(Blen)
                    Bdis=nowBdis.copy()
                    Blen=nowBlen
                    Bangle=nowBangle.copy()
                    Bboundary=nowBboundary
                    count=1.
                    print(("/***第%u次迭代,发现了更大物体,重置最大物体***/") % times)
                    str = "bigger"
                else:#更小物体
                    print(("/***第%u次迭代,检测到更小物体,长度为%g m***/") %(times,nowBlen))
                    str = "shorter"
            elif abs(Blen-nowBlen)<tolerance*Blen and not self.each_relative_error(Bboundary,nowBboundary):
                #长度差值在限定范围内，但是边界坐标差值超过限度
                if Blen>nowBlen:
                    print(("/***第%u次迭代,发现了长度略短物体,但位置不同***/") % times)
                    str = "slightly shorter"
                elif nowBlen>Blen:
                    print(("/***第%u次迭代,发现了长度略长物体,但位置不同,重置最大物体***/") % times)
                    str = "slightly bigger"
                    Blen=nowBlen
                    Bangle = nowBangle.copy()
                    Bboundary = nowBboundary
                    Bdis=nowBdis.copy()
                    count = 1.
        print(("本次迭代后最长物体长度约为：%g m") %Blen)

        r,theta=self.object_segmentation(Bdis,Bangle,Blen,angle_incre,5+1)#r,theta分别保存着分割物体的点的角度和距离

        rlim = np.max(obj_dis)*1.05
        ax1.set_rlim(0, rlim)
        rlim = np.max(nowBdis)*1.05
        ax2.set_rlim(0, rlim)
        rlim = np.max(Bdis)*1.05
        ax3.set_rlim(0, rlim)

        ax1.scatter(obj_angle,obj_dis, s=4, alpha=0.75)
        ax1.text(-0.3,0,times)
        ax2.scatter(nowBangle,nowBdis,s=4,alpha=0.75)
        ax3.scatter(Bangle,Bdis,s=4,alpha=0.75)
        ax3.scatter(theta,r,s=4,alpha=1,c='red')

        ax1.set_title('angle&dis received', x=0.5, y=1.2)
        ax2.set_title('bigNOW' + str, x=0.5, y=1.2)
        ax3.set_title('bigALL', x=0.5, y=1.2)

        str = ("length:%.3fm") %Blen
        ax3.text(-0.1, -0.2, str, transform=ax3.transAxes, fontsize=15)
        str=("boundary:(%.3f,%.3f) (%.3f,%.3f)")%(Bboundary[0],Bboundary[1],Bboundary[2],Bboundary[3])
        ax3.text(-0.1, -0.3, str,transform=ax3.transAxes,fontsize=15)
        str=("distance: mean:%.3fm  min:%.3fm")%(np.mean(Bdis),np.min(Bdis))
        ax3.text(-0.1, -0.4, str,transform=ax3.transAxes,fontsize=15)

        for i in range(4):
            self.lidar_info_array.data.append(nowBboundary[i])
        self.lidar_info_array.data.append(float(np.mean(nowBdis))) #nowBdis 保存着最大物体扫描出的每个点的距离数据
        self.lidar_info_array.data.append(float(nowBlen)) #当前扫描出的最大物体的曲线长度
        self.lidar_info_array.data.append(float(Wind_blade_lenth)) #当前扫描出的最大物体的直线长度
        self.publisher.publish(self.lidar_info_array)  #发送雷达解算出来的消息
        self.lidar_info_array.data.clear()

        # plt.show(block=False)
        # plt.pause(0.3)
        plt.close('all')

        #需要修改路径
        # current_time = datetime.now().strftime('%m%d_%H:%M:%S')
        # filename = '/home/yyf/figs/ORIGINAL_P_%s.png' % current_time
        # plt.savefig(filename)
        # plt.close('all')

        times = times + 1
    def object_segmentation(self,r,theta,len,angle_incre,amount):#对物体按长度进行划分 例如amount=6,五等分
        final_indicies = np.array([])
        temp=0
        if r.size>60:#减少计算量
            blocks=60
        else:
            blocks=r.size
        indicies=np.array([np.floor((x+1)*r.size/blocks) for x in range(blocks)])
        indicies = indicies.astype(np.int32)
        lenbound=len/(amount-1.)
        for i in range(blocks):
            nowlen=self.getlength(r[:indicies[i]],angle_incre)
            if temp<=0 and nowlen-lenbound>0:
                lenbound=lenbound+len/(amount-1.)
                if abs(temp)<nowlen:
                    final_indicies=np.append(final_indicies,indicies[i-1])
                else:
                    final_indicies = np.append(final_indicies, indicies[i])
            temp=nowlen-lenbound
        final_indicies = final_indicies.astype(np.int32)
        finalr=r[final_indicies]
        finaltheta=theta[final_indicies]
        return finalr,finaltheta
    def neighbor_diff(dis,tolerance):#检测相邻有效点的差值是否超过测量误差，并进一步去除无效点
        for i in range(dis.size-1):
            if dis[i]!=0 and dis[i+1]!=0:
                if abs(dis[i]-dis[i+1])>tolerance:
                    dis[i+1]=dis[i]=0
        return dis

    def callback(self,msg):
        angle_range=np.array([180,360],dtype=float)
        angle_range=angle_range/360#屏蔽角度，以百分比表示，取msg中相应的点

        #初步处理msg.ranges
        rawdistance=np.array(msg.ranges)
        indicies = np.arange(0,rawdistance.size,1)
        start=np.floor(rawdistance.size*angle_range[0])
        end = np.floor(rawdistance.size * angle_range[1])
        selected = (indicies >= start) & (indicies <= end)
        rawdistance=rawdistance[selected]
        if rawdistance[-1]!=0:
            rawdistance[-1]=0.
        distance= np.where(rawdistance > 10,0, rawdistance)#无效距离赋0
        tolerance = 0.06  # 雷达测量误差是 ±3cm
        distance = self.neighbor_diff(distance, tolerance)  # 处理测量误差过大的点

        obj_dis=np.array([])  #存储每个物体上各点的距离，角度  不同物体之间用0隔开
        obj_angle=np.array([])
        angle_min = (msg.angle_max - msg.angle_min)*angle_range[0]
        angle_incre = msg.angle_increment   # 弧度间隔
        obj_minlen = 0.05     # 忽略长度大约小于5cm的物体
        obj_maxlen = 6.       #忽略长度大约大于6m的物体
        obj_count = 0.        # 有效物体个数
        invalidpts_count = 0  # 对连续出现的无效点进行计数
        validpts_count = 0
        temp = 0
        angles = np.linspace(angle_min,angle_min+(distance.size-1)*angle_incre,num=distance.size,endpoint=True)
        validpts_dis = np.array([])  # 存储每个物体的距离数据
        validpts_angle = np.array([])

        # fig, ax1 = plt.subplots(1, 1, subplot_kw={'projection': 'polar', 'aspect': 'equal'},figsize=(4, 4))
        # ax1.scatter(angles, distance, s=5, alpha=0.75)
        # ax1.set_title('original data')
        # plt.show(block=False)
        # plt.pause(0.3)
        # plt.close('all')

        #将获取的所有图片保存，需要修改路径
        # current_time = datetime.now().strftime('%m%d_%H:%M:%S')
        # filename = '/home/yyf/figs/ORIGINAL_%s.png' % current_time
        # plt.savefig(filename)
        # plt.close('all')

        for i in range(distance.size):#划分有效物体
            if distance[i]!=0:
                if temp!=0:
                    validpts_count=validpts_count+temp
                    temp=0
                validpts_count=validpts_count+1
                validpts_dis=np.append(validpts_dis,distance[i])
                validpts_angle=np.append(validpts_angle,angle_min+i*angle_incre)
            else:#遇到无效点
                if validpts_count>1:  #无效点之前是有效点，且有一个以上有效点
                    invalidpts_count = self.count_invalidpts(distance, i)
                    if self.getlength(validpts_dis,angle_incre)<obj_minlen or self.getlength(validpts_dis,angle_incre)>obj_maxlen:#忽略长度不在限度内的物体
                        validpts_dis=np.array([])
                        validpts_angle=np.array([])
                    else:
                        if (i + invalidpts_count)<=distance.size-1:#如果这组无效点之后的点存在
                            if invalidpts_count <= 3 and abs(distance[i - 1] - distance[i + invalidpts_count])<tolerance:
                                temp=validpts_count
                                validpts_count = 0
                                continue
                        obj_count=obj_count+1
                        validpts_dis=np.append(validpts_dis,0)
                        validpts_angle=np.append(validpts_angle,0)
                        obj_dis=np.append(obj_dis, validpts_dis)
                        obj_angle=np.append(obj_angle,validpts_angle)
                    validpts_count = 0
                else:#无效点之前是无效点或者之前的有效点只有一个
                    validpts_dis = np.array([])
                    validpts_angle = np.array([])
                    validpts_count = 0

        obj_dis,obj_angle,obj_count=self.repeated_remove(obj_dis, obj_angle,obj_count)
        self.showobj_dis_coord(obj_dis, obj_angle, obj_count, angle_incre)




def listener():
    global lidar_info_pub
    rospy.init_node('lasr_listener', anonymous=False)
    lidar_info_pub_left = rospy.Publisher('lidar_info_left', Float32MultiArray, queue_size=1)
    lidar_info_pub_right = rospy.Publisher('lidar_info_right', Float32MultiArray, queue_size=1)
    lidar_left=Lidar_Bladelenth(lidar_info_pub_left) 
    lidar_right=Lidar_Bladelenth(lidar_info_pub_right)
    rospy.Subscriber("/scan_left", LaserScan, lidar_left.callback, queue_size=1)
    rospy.Subscriber("/scan_right", LaserScan, lidar_right.callback, queue_size=1)
    
    ##此文件已作废，请使用distan_newest
    
    
    while not rospy.is_shutdown():                
        rospy.spin() 
    if(rospy.is_shutdown()):
        time.sleep(1.0)
        os._exit(0)   

if __name__ == '__main__':
    listener()

