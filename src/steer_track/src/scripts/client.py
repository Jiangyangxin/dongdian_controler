# -*- coding: utf-8 -*-
import socket

import time
import os
import argparse
import serial.tools.list_ports

import csv

from datetime import datetime
from threading import Thread, Event

BUFSIZ = 2048

class packInfo:
    boardType = {'1':'Steer', '2':'Adsorption Fan', '3':'Fan', '4':'IO'}

    SteerCmdName = {"steer_state" : 3, "dr1_tar_v" : 0.0, "dr2_tar_p" : 0, "dr1_tar_i" : 0.0, "ts" : 0.00}
    SteerValName = {"steer_state" : 0, "dr1_tar_v" : 0.00, "dr1_real_v" : 0.00, "dr1_tar_i" : 0.00, 
                    "dr1_real_i" : 0.00, "dr2_tar_p" : 0.00, "dr2_real_p" : 0.00, "dr2_tar_v" : 0.00, 
                    "dr2_real_v" : 0.00, "dr2_tar_i" : 0.00, "dr2_real_i" : 0.00, "ts" : 0.00}

    AdsorptionCmdName = {
        "active_state" : 1, "s1_tar_p" : 10.00, "s2_tar_p" :10.00, "s3_tar_p" : 10.00,
        "plate_tar_x" : 0.00, "plate_tar_y" : 0.00, "plate_tar_z" : 0.00, "active_ts" : 0.00}
    AdsorptionValName = {
        "active_state" : 0.00, "s1_tar_p" : 0.00, "s2_tar_p" : 0.00, "s3_tar_p" : 0.00,
        "s1_real_p" : 0.00, "s2_real_p" : 0.00, "s3_real_p" : 0.00, "dr1_real_p" : 0.00,
        "dr2_real_p" : 0.00, "dr3_real_p" : 0.00, "plate_tar_x" : 0.00, "plate_real_x" : 0.00,
        "plate_tar_y" : 0.00, "plate_real_y" : 0.00, "plate_tar_z" : 0.00, "plate_real_z" : 0.00, "active_ts" : 0.00}

    FanCmdName = {"fan_state" : 2, "fan_tar_pre" : 0.00, "fan_ts" : 0.00}
    FanValName = {"fan_state" : 0, "fan_tar_pre" : 0.00, "fan_real_pre" : 0.00, "fan_pwm" : 0.00, "fan_speed" : 0.00, "fan_ts" : 0.00}

    IOCmdName = {"io_state" : 0, "ts" : 0}
    IOValName = {"io_state" : 0, "ts" : 0}

def recvTask(socket, event, logDir):
    path = logDir + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '.csv'
    valNameList = []
    headName = []
    vals = packInfo.boardType[type].split(' ')
    for val in vals:
        valName = getattr(packInfo, val + 'ValName')
        valNameList.append(valName)
        headName += list(valName.keys())
    with open(path, 'w') as f:
        csv_write = csv.writer(f)
        csv_head = headName
        print(headName)
        csv_write.writerow(csv_head)
    data_all = ''
    while not event.is_set():
        try:
            data_receive = socket.recv(BUFSIZ)
            if not data_receive:
                print('error')
            else:
                data_all += data_receive.decode('utf-8')
            index = data_all.find(" active_ts")
            # 帧内、帧间数据均以空格进行分割
            infos = data_all[0:index].split(' ')
            data_all = data_all[index:]
            # 先根据控制板类型，确认是否存在多个反馈量
            # 使用字典一对一存储反馈值
            idx = 1
            for info in infos:
                if info != '':
                    # 变量名和值以冒号进行分割
                    if len(info.split(':')) < 2:
                        print(info)
                    else:
                        k = info.split(':')[0]
                        v = info.split(':')[1]
                        for valName in valNameList:
                            if k in valName.keys():
                                valName[k] = float(v) 
            valAll = [] 
            for val in vals:
                valName = getattr(packInfo, val + 'ValName')
                valNameList.append(valName)
                valAll += list(valName.values())
            with open(path, "a+") as f:
                csv_write = csv.writer(f)
                csv_write.writerow(valAll)
            # if (int(time.time()*1000) % 100 == 0):
            #     print('[client]: receive >>', valNameList)
        except KeyboardInterrupt:
            event.set()
        except Exception as err:
            event.set()
            print('recv over', err)

def sendTask(socket, event):
    while not event.is_set():
        t = time.time()
        data_input = ''
        # 先根据控制板类型，确认是否存在多个控制量
        ctrls = packInfo.boardType[type].split(' ')
        for ctrl in ctrls:
            cmdName = getattr(packInfo, ctrl + 'CmdName')
            # 字典读值，以name:val 的方式拼接
            for (key, val) in list(cmdName.items()):
                cmdName['ts'] = round(t * 1000000)
                data_input += key + ":" + str(val) + " "
        # 完成控制量遍历后发送帧
        # print('[server]: ' + data_input)
        # 客户端发送给服务端
        socket.send(data_input.encode())

        time.sleep(0.01)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='192.168.0.199')
    parser.add_argument('--port', default='5000')
    parser.add_argument('-t', '--type', default='2')
    parser.add_argument('--virtual', default='1')
    parser.add_argument('-T', '--period', default='100')
    parser.add_argument('-s', '--serial', default='1')
    args = parser.parse_args()

    type = args.type
    ip = args.ip
    virtual = args.virtual
    period = args.period
    connectSerial = args.serial
    isConnect = False
    
    # 第一次上电使用需要进行串口配置，若已完成配置发起重连，则使用-s 0，避免卡死在串口等待中
    if connectSerial == '1':
        # 连接串口
        port_list = [port_info[0] for port_info in list(serial.tools.list_ports.comports())]
        comSerial = serial.Serial(port=port_list[0], baudrate=115200, timeout=0)
        connect_socket = True

        while connect_socket:
            ch = comSerial.read_all()
            if ch != b'':
                print(ch)
            if 'IP' in ch.decode('UTF-8'):
                connect_socket = False
            else:
                cmd = 'ip:' + ip + ' type:' + type + ' period:' + period + ' \r\n'
                comSerial.write(bytes(cmd, encoding='utf-8'))
            time.sleep(1)
            
    logDir = os.getcwd() + '/log/'
    if not os.path.exists(logDir):
        os.makedirs(logDir)
    else:
        files = os.listdir(logDir)
        for file in files:
            if os.path.getsize(logDir + file) == 0:
                os.remove(logDir + file)
    
    # 创建socket
    tcpCliSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 启动socket长连接心跳
    tcpCliSock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)  # 在客户端开启心跳维护
    tcpCliSock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 1000, 1000))
    # 试图连接到服务端，默认使用5000端口
    tcpCliSock.connect((ip, 5000))
    isConnect = True
    # 收发独立进行
    try:
        event = Event()
        t1 = Thread(target = sendTask, args=(tcpCliSock, event, ))
        t2 = Thread(target = recvTask, args=(tcpCliSock, event, logDir, ))
        t1.start()
        t2.start()
        event.wait()
    except KeyboardInterrupt:
        print("ctrl + c")
        event.set()
        # 发起关闭
        tcpCliSock.close()
        os._exit(0)
    
