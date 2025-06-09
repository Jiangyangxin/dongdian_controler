#!/bin/bash
###
 # @Author: jyx
 # @Date: 2022-11-11 03:52:08
 # @LastEditTime: 2023-06-03 01:48:45
 # @LastEditors: jyx
 # @Description: 
 # @FilePath: /new_steer/climbRun.sh
 # Copyright (c) 2021 HUSTcience&Technology. All rights reserved.
### 
sleep 1 
DIR=$(cd $(dirname $0);pwd) #获取当前目录
source /opt/ros/noetic/setup.bash
source ${DIR}/devel/setup.bash
roslaunch steer_track apriltag_to_vins.launch
