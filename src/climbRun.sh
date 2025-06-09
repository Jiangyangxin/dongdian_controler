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
DIR=$(cd $(dirname $0);pwd) #获取当前目录
source /opt/ros/noetic/setup.bash
source ${DIR}/../devel/setup.bash
roslaunch steer_track opencontrol.launch
