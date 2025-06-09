#!/usr/bin/env python
# coding:utf-8
import rospy
import socket
import os
import time
from packInfo import packInfo
    

class BottomControl():
    def __init__(self, ip='127.0.0.1', port = 5000, type = '1', frequency = 200.0) -> None:
        self.ip = ip
        self.port = port
        self.type = type
        self.is_connect = False
        self.frequency = frequency
        self.BUFSIZE = 1024
        # 创建socket
        self.tcpCliSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 设置连接超时时间为3秒
        self.tcpCliSock.settimeout(10)

        # 启动socket长连接心跳
        self.tcpCliSock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)  # 在客户端开启心跳维护
        self.tcpCliSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)  # 在空闲5秒后激活
        self.tcpCliSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)  # 间隔1秒发送一次保活ping
        self.tcpCliSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 10)  # 在ping失败10次(Max_Ailures)或15秒后关闭连接
        # self.tcpCliSock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 1000, 1000))
        # 试图连接到服务端，默认使用5000端口   netstat -ntulp | grep 5000
        try:
            self.tcpCliSock.connect((ip, 5000))
            self.is_connect = True
        except socket.timeout:
            print(self.ip,":机器人初始化连接超时！")  
            self.tcpCliSock.close()
            exit(0)  
        except Exception as e:
            print(self.ip,":机器人连接失败，请重试！")
            self.tcpCliSock.close()
            print(e)
            exit(0)
            
        # 收发独立进行
        
        # 机器人控制指令订阅
        # self.velocity_input_sub = rospy.Subscriber('velocity_input', Twist, self.VelocityInputCallback, queue_size=1)
    
        # 数据反馈计时器
        # self.send_timer = rospy.Timer(rospy.Duration(1 / self.frequency), self.sendTask)
        # self.recv_timer = rospy.Timer(rospy.Duration(1 / self.frequency), self.recvTask)
        # self.send_timer.start()
        # self.recv_timer.start()
        
    def recvTask(self,myinfo):
        try:
            data_receive = self.tcpCliSock.recv(self.BUFSIZE)
            #print(data_receive.decode('utf-8'))
            if not data_receive:
                print('error :',self.ip)
            # 帧内、帧间数据均以空格进行分割
            infos = data_receive.decode('utf-8').split(' ')
            # 先根据控制板类型，确认是否存在多个反馈量
            valNameList = []
            vals = myinfo.boardType[self.type].split(' ')
            for val in vals:
                valName = getattr(myinfo, val + 'ValName')
                valNameList.append(valName)
            # 使用字典一对一存储反馈值
            for info in infos:
                if info != '' and info != '\r\n':
                    # 变量名和值以冒号进行分割
                    k = info.split(':')[0]
                    v = info.split(':')[1]
                    for valName in valNameList:
                        if k in valName.keys():
                            valName[k] = float(v)       
            # print('[client]: receive >>', valNameList)
        except socket.error as e:
            print(self.ip,"socket接收异常!:",e)  
            time.sleep(3)#每3秒重新连接一次
            self.create_connect()
        except KeyboardInterrupt:
            # 发起关闭
            self.tcpCliSock.close()
            os._exit(0)
        except Exception as err:
            print(self.ip,'其他接收异常！:', err)
    def sendTask(self, info):
        try:
            t = rospy.get_time()
            data_input = ''
            # 先根据控制板类型，确认是否存在多个控制量
            ctrls = info.boardType[self.type].split(' ')
            for ctrl in ctrls:
                cmdName = getattr(info, ctrl + 'CmdName')
                # 字典读值，以name:val 的方式拼接
                for (key, val) in cmdName.items():
                    cmdName['ts'] = round(t * 1000000)
                    data_input += key + ":" + str(val) + " "
            # 完成控制量遍历后发送帧
            # print('[server]: ' + data_input)
            # 客户端发送给服务端
            self.tcpCliSock.send(data_input.encode())

        except socket.error as e:
            print(self.ip,"socket发送异常!:",e) 
            time.sleep(3)#每3秒重新连接一次 
            self.create_connect()
        except KeyboardInterrupt:
            # 发起关闭
            self.tcpCliSock.close()
            print('send over')
            os._exit(0)
        except Exception as err:
            print('其他发送异常！',self.ip,err)



    def create_connect(self):
        try:
            self.tcpCliSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 设置连接超时时间为3秒
            self.tcpCliSock.settimeout(3)

            # 启动socket长连接心跳
            self.tcpCliSock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)  # 在客户端开启心跳维护
            self.tcpCliSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)  # 在空闲5秒后激活
            self.tcpCliSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)  # 间隔1秒发送一次保活ping
            self.tcpCliSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 10)  # 在ping失败10次(Max_Ailures)或15秒后关闭连接
            # self.tcpCliSock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 1000, 1000))
            # 试图连接到服务端，默认使用5000端口
            self.tcpCliSock.connect((self.ip, 5000))
            self.is_connect = True
        except socket.error as e:
            print(self.ip,"连接错误！:",e)  
            self.tcpCliSock.close()
            time.sleep(5)#每3秒重新连接一次
            self.create_connect() #递归，重新连接

# if __name__ == "__main__":
#     rospy.init_node("controller_node", anonymous=False)
#     bottom_controller = BottomControl('192.168.0.100', 5000, '1', 100)
#     info = packInfo()
#     while not rospy.is_shutdown():
#         info.SteerCmdName["steer_state"]=1
#         info.SteerCmdName["dr1_tar_v"] = round(float(0.0),4)#旋转的时候，舵轮速度为0
#         bottom_controller.sendTask(info=info)
#         # bottom_controller.recvTask(info)
#         # print(info.SteerValName["dr1_real_v"])
#         # print(info.SteerValName["dr2_real_p"])
#         # print(info.SteerValName["dr2_real_v"])
#         # print(info.SteerValName["ts"])
#         time.sleep(1.0)
    