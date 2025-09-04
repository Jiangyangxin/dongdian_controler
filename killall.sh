#!/bin/bash

# 关闭所有ROS节点的脚本

# 获取所有正在运行的ROS节点
nodes=$(rosnode list)

if [ -z "$nodes" ]; then
    echo "没有找到正在运行的ROS节点。"
    exit 0
fi

echo "正在运行的ROS节点:"
echo "$nodes"
echo ""

# 关闭每个节点
for node in $nodes; do
    echo "正在关闭节点: $node"
    rosnode kill $node
done

# 检查是否还有节点在运行
remaining_nodes=$(rosnode list)
if [ -n "$remaining_nodes" ]; then
    echo ""
    echo "警告: 以下节点未能正常关闭:"
    echo "$remaining_nodes"
else
    echo ""
    echo "所有ROS节点已成功关闭。"
fi

# 可选: 关闭roscore
#read -p "是否要关闭roscore? [y/N] " answer
#if [[ $answer =~ ^[Yy]$ ]]; then
    killall -9 roscore
    killall -9 rosmaster
#    echo "roscore已关闭。"
#fi

exit 0
