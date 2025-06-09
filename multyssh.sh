#!/bin/bash

# 定义4台PC的SSH连接信息
PCs=(
    "mini@192.168.1.12"
    "mini2@192.168.1.13"
    "mini3@192.168.1.14"
    "mini4@192.168.1.15"
)
PASSWORD="111"  # 统一密码
TARGET_DIR="~/dongdian_newest"  # 目标目录

# 检查依赖
if ! command -v sshpass &> /dev/null; then
    sudo apt update && sudo apt install -y sshpass
fi

if ! command -v gnome-terminal &> /dev/null; then
    sudo apt install -y gnome-terminal
fi

# 单个终端窗口+4个标签页函数
launch_terminal() {
    local pc="$1"
    local title_prefix="$2"
    
    # 创建新窗口并添加4个标签页
    gnome-terminal \
        --window --title "${title_prefix}-1" -- bash -c \
            "sshpass -p '$PASSWORD' ssh -t $pc \"cd $TARGET_DIR && exec bash\"" \
        --tab --title "${title_prefix}-2" -- bash -c \
            "sshpass -p '$PASSWORD' ssh -t $pc \"cd $TARGET_DIR && exec bash\"" \
        --tab --title "${title_prefix}-3" -- bash -c \
            "sshpass -p '$PASSWORD' ssh -t $pc \"cd $TARGET_DIR && exec bash\"" \
        --tab --title "${title_prefix}-4" -- bash -c \
            "sshpass -p '$PASSWORD' ssh -t $pc \"cd $TARGET_DIR && exec bash\""
}

# 为每台PC启动终端
for i in "${!PCs[@]}"; do
    launch_terminal "${PCs[$i]}" "PC$((i+1))" &
    sleep 0.5  # 防止窗口重叠
done

echo "已启动4个终端窗口，每个窗口4个SSH标签页"
