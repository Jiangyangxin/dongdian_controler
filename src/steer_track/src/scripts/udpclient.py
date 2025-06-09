#!/usr/bin/env python
# coding:utf-8
import socket
import threading
import struct
import rospy
import numpy as np
import time
import os
from std_msgs.msg import Float32MultiArray


# 主机配置
HOST_IP = "0.0.0.0"  # 监听所有网络接口
HOST_PORT = 5000

# 定义float数组的长度（根据实际需求修改）
FLOAT_ARRAY_SIZE = 14  # 假设每个消息包含4个float值
input_data =[0,0,0,0,0,0,0,0,0,0,0,0,0,0]
data_array=Float32MultiArray()
# 从机配置列表（需要根据实际情况修改）
SLAVES = [
    {"ip": "192.168.1.12", "port": 5000},
    {"ip": "192.168.1.13", "port": 5000},
    {"ip": "192.168.1.14", "port": 5000},
    {"ip": "192.168.1.15", "port": 5000}
]


def receive_data(sock):
    global input_data,FLOAT_ARRAY_SIZE,float_array,data_pub
    """持续接收数据的线程函数"""
    while not rospy.is_shutdown():
        try:
            # 接收数据
            data, addr = sock.recvfrom(1024)
            # 使用struct解包数据
            float_array = struct.unpack(f'>{FLOAT_ARRAY_SIZE}f', data)  # 假设使用大端字节序
            # print(f"\n[Received from {addr[0]}:{addr[1]}] {float_array}")
            data_array.data = float_array
            data_pub.publish(data_array)
        except Exception as e:
            print(f"接收错误: {e}")
            break
        except KeyboardInterrupt:
            # 发起关闭
            os._exit(0)
    if(rospy.is_shutdown()):
        os._exit(0)   
def send_data(sock, slaves):
    global input_data,FLOAT_ARRAY_SIZE,float_array
    """发送数据的线程函数"""
    while not rospy.is_shutdown():   
        try:
            # 输入float数组数据
            float_list = list(input_data)

            # 使用struct打包数据
            packed_data = struct.pack(f'>{FLOAT_ARRAY_SIZE}f', *float_list)  # 假设使用大端字节序

            # 向所有从机发送数据
            for slave in slaves:
                sock.sendto(packed_data, (slave["ip"], slave["port"]))
                # print(f"[Sent to {slave['ip']}:{slave['port']}] {float_list}")

        except Exception as e:
            print(f"发送错误: {e}")
            break
        except KeyboardInterrupt:
            break
    if(rospy.is_shutdown()):
        os._exit(0)          
# def InputCallback(msg):
#     global input_data,FLOAT_ARRAY_SIZE
#     for i in range(len(msg.data)):
#         input_data[i]=msg.data[i]
#     FLOAT_ARRAY_SIZE=len(msg.data)
def left_LidarCallback(msg):
    global input_data
    for i in range(len(msg.data)):
        input_data[i]=msg.data[i]


def right_LidarCallback(msg):
    global input_data
    for i in range(len(msg.data)):
        input_data[i+6]=msg.data[i]



def main():
    global data_pub
    rospy.init_node("udpserver_node", anonymous=True)
    # rospy.Subscriber('fan_input', Float32MultiArray, InputCallback, queue_size=1)
    rospy.Subscriber("/lidar_info_left",Float32MultiArray ,left_LidarCallback,queue_size=1)
    rospy.Subscriber("/lidar_info_right",Float32MultiArray ,right_LidarCallback,queue_size=1)  
    data_pub=rospy.Publisher('/lidar_info', Float32MultiArray, queue_size=1)
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST_IP, HOST_PORT)) #bind是用来接收数据的
    # sock.settimeout(5)  # 设置socket超时（可选）

    print(f"主机已启动，监听 {HOST_IP}:{HOST_PORT}")

    # 创建接收线程
    recv_thread = threading.Thread(target=receive_data, args=(sock,))
    recv_thread.daemon = True

    # 创建发送线程
    send_thread = threading.Thread(target=send_data, args=(sock, SLAVES))

    # 启动线程
    recv_thread.start()
    send_thread.start()

    # 等待发送线程结束
    # send_thread.join()
    while not rospy.is_shutdown():                
                 
        rospy.spin() 
    if(rospy.is_shutdown()):
        sock.close()
        os._exit(0)   
    # 关闭socket
    sock.close()
    print("通信已结束")

if __name__ == "__main__":
    main()
    
    

# #!/usr/bin/env python
# # coding:utf-8
# import socket
# import threading
# import struct
# import rospy
# import numpy as np
# import time
# import os
# from std_msgs.msg import Float32MultiArray


# # 主机配置
# HOST_IP = "0.0.0.0"  # 监听所有网络接口
# HOST_PORT = 5000

# # 定义float数组的长度（根据实际需求修改）
# FLOAT_ARRAY_SIZE = 4  # 假设每个消息包含4个float值
# input_data =[1,2,3,4]
# data_array=Float32MultiArray()
# # 从机配置列表（需要根据实际情况修改）
# SLAVES = [
#     {"ip": "192.168.0.11", "port": 5000},
#     {"ip": "192.168.0.12", "port": 6001},
#     {"ip": "192.168.0.13", "port": 6001},
#     {"ip": "192.168.0.14", "port": 6001}
# ]


# def receive_data(sock):
#     global input_data,FLOAT_ARRAY_SIZE,float_array,data_pub
#     """持续接收数据的线程函数"""
#     while not rospy.is_shutdown():
#         try:
#             # 接收数据
#             data, addr = sock.recvfrom(1024)
#             # 使用struct解包数据
#             float_array = struct.unpack(f'>{FLOAT_ARRAY_SIZE}f', data)  # 假设使用大端字节序
#             # print(f"\n[Received from {addr[0]}:{addr[1]}] {float_array}")
#             data_array.data = float_array
#             data_pub.publish(data_array)
#         except Exception as e:
#             print(f"接收错误: {e}")
#             break
#         except KeyboardInterrupt:
#             # 发起关闭
#             os._exit(0)
#     if(rospy.is_shutdown()):
#         os._exit(0)   
# def send_data(sock, slaves):
#     global input_data,FLOAT_ARRAY_SIZE,float_array
#     """发送数据的线程函数"""
#     while not rospy.is_shutdown():   
#         try:
#             # 输入float数组数据
#             float_list = list(input_data)

#             # 使用struct打包数据
#             packed_data = struct.pack(f'>{FLOAT_ARRAY_SIZE}f', *float_list)  # 假设使用大端字节序

#             # 向所有从机发送数据
#             for slave in slaves:
#                 sock.sendto(packed_data, (slave["ip"], slave["port"]))
#                 # print(f"[Sent to {slave['ip']}:{slave['port']}] {float_list}")

#         except Exception as e:
#             print(f"发送错误: {e}")
#             break
#         except KeyboardInterrupt:
#             break
#     if(rospy.is_shutdown()):
#         os._exit(0)          
# def InputCallback(msg):
#     global input_data,FLOAT_ARRAY_SIZE
#     for i in range(len(msg.data)):
#         input_data[i]=msg.data[i]
#     FLOAT_ARRAY_SIZE=len(msg.data)

# def main():
#     global data_pub
#     rospy.init_node("udpserver_node", anonymous=True)
#     rospy.Subscriber('fan_input', Float32MultiArray, InputCallback, queue_size=1)
#     data_pub=rospy.Publisher('fan_pwm_info', Float32MultiArray, queue_size=1)
#     # 创建UDP socket
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.bind((HOST_IP, HOST_PORT)) #bind是用来接收数据的
#     # sock.settimeout(5)  # 设置socket超时（可选）

#     print(f"主机已启动，监听 {HOST_IP}:{HOST_PORT}")

#     # 创建接收线程
#     recv_thread = threading.Thread(target=receive_data, args=(sock,))
#     recv_thread.daemon = True

#     # 创建发送线程
#     send_thread = threading.Thread(target=send_data, args=(sock, SLAVES))

#     # 启动线程
#     recv_thread.start()
#     send_thread.start()

#     # 等待发送线程结束
#     # send_thread.join()
#     while not rospy.is_shutdown():                
                 
#         rospy.spin() 
#     if(rospy.is_shutdown()):
#         sock.close()
#         os._exit(0)   
#     # 关闭socket
#     sock.close()
#     print("通信已结束")

# if __name__ == "__main__":
#     main()