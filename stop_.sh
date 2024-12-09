#!/bin/bash
###
 # @Author: lxs
 # @Date: 2022-11-11 03:52:08
 # @LastEditTime: 2024-12-05 01:48:45
 # @LastEditors: jyx
 # @Description: 
 # @FilePath: /dongdian_controler/climbRun.sh
 # Copyright (c) 2021 LXScience&Technology. All rights reserved.
### 
# sleep 1
# sudo chmod 777 /dev/ttyWCHUSB3
# sudo chmod 777 /dev/ttyUSB0 
#DIR=$(cd $(dirname $0);pwd) #获取当前目录
#source /opt/ros/noetic/setup.bash
#source ${DIR}/../devel/setup.bash
rosnode kill -a
sleep 1
ps -def | grep tcp_server | grep -v grep| cut -c 9-16 | xargs kill -9 #找到关键字“tcp_server” 然后杀死进程
ps -def | grep task_node | grep -v grep |cut -c 9-16 | xargs kill -9

killall climbRun.sh
#
