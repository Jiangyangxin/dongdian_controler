#!/bin/bash


#enx6c1ff7203c83   pylon and robot
#enx6c1ff7203c8c   computer
# 定义五台PC的SSH连接信息（用户名@IP）
PC2="mini@192.168.1.12"
PC3="mini2@192.168.1.13"
PC4="mini3@192.168.1.14"
PC5="mini4@192.168.1.15"

# SSH密码（所有PC相同）
PASSWORD="111"
# 目标目录（家目录下的相对路径）
TARGET_DIR="~/dongdian_newest"

# 检查是否安装了sshpass，如果没有则尝试安装
if ! command -v sshpass &> /dev/null; then
    echo "sshpass未安装，尝试安装..."
    sudo apt-get update && sudo apt-get install sshpass -y || { echo "安装sshpass失败"; exit 1; }
fi

# 检查是否安装了gnome-terminal
if ! command -v gnome-terminal &> /dev/null; then
    echo "gnome-terminal未安装，尝试安装..."
    sudo apt-get install gnome-terminal -y || { echo "安装gnome-terminal失败"; exit 1; }
fi

# SSH连接并进入目录的函数
run_ssh_with_cd() {
    local title="$1"
    local pc="$2"
    gnome-terminal --tab --title="$title" -- bash -c \
    "sshpass -p '$PASSWORD' ssh -t $pc \"cd $TARGET_DIR && echo '成功进入目录: \$(pwd)' && exec bash\"; exec bash"
}

# 并行连接五台PC
run_ssh_with_cd "SSH-PC3" "$PC3" &
run_ssh_with_cd "SSH-PC3" "$PC3" &
run_ssh_with_cd "SSH-PC3" "$PC3" &
run_ssh_with_cd "SSH-PC3" "$PC3" &

echo "已启动四个SSH连接窗口，正在进入各PC的 $TARGET_DIR 目录..."
