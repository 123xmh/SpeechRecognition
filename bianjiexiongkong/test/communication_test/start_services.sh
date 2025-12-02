#!/bin/bash

# 启动通信服务的脚本
# 解决nohup卡住的问题

echo "=== 启动通信服务 ==="

# 检查并停止现有进程
echo "检查现有进程..."
pkill -f communication_node 2>/dev/null
pkill -f vehicle_simulator 2>/dev/null
sleep 2

# 检查端口占用
echo "检查端口占用..."
netstat -tulpn | grep -E "(8887|8888|9091)" | while read line; do
    echo "端口被占用: $line"
done

# 启动communication_node
echo "启动communication_node..."
cd /home/jinshuxin/TK30V2/tk_chest_controller_ws
source install/setup.bash
ros2 launch communication communication.launch.py &
COMM_PID=$!
echo "communication_node PID: $COMM_PID"

# 等待communication_node启动
sleep 3

# 检查communication_node是否启动成功
if ps -p $COMM_PID > /dev/null; then
    echo "✓ communication_node启动成功"
else
    echo "✗ communication_node启动失败"
    exit 1
fi

# 启动vehicle_simulator
echo "启动vehicle_simulator..."
cd /home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test

# 使用screen或tmux来避免nohup卡住的问题
if command -v screen >/dev/null 2>&1; then
    echo "使用screen启动vehicle_simulator..."
    screen -dmS vehicle_simulator python3 src/vehicle_simulator.py
    sleep 2
    if screen -list | grep -q vehicle_simulator; then
        echo "✓ vehicle_simulator在screen中启动成功"
    else
        echo "✗ vehicle_simulator启动失败"
        exit 1
    fi
elif command -v tmux >/dev/null 2>&1; then
    echo "使用tmux启动vehicle_simulator..."
    tmux new-session -d -s vehicle_simulator 'python3 src/vehicle_simulator.py'
    sleep 2
    if tmux has-session -t vehicle_simulator 2>/dev/null; then
        echo "✓ vehicle_simulator在tmux中启动成功"
    else
        echo "✗ vehicle_simulator启动失败"
        exit 1
    fi
else
    echo "使用直接启动方式..."
    python3 src/vehicle_simulator.py &
    VEHICLE_PID=$!
    echo "vehicle_simulator PID: $VEHICLE_PID"
    sleep 2
    if ps -p $VEHICLE_PID > /dev/null; then
        echo "✓ vehicle_simulator启动成功"
    else
        echo "✗ vehicle_simulator启动失败"
        exit 1
    fi
fi

# 等待服务完全启动
echo "等待服务完全启动..."
sleep 3

# 检查服务状态
echo "=== 服务状态检查 ==="
echo "进程状态:"
ps aux | grep -E "(communication_node|vehicle_simulator)" | grep -v grep

echo ""
echo "端口监听状态:"
netstat -tulpn | grep -E "(8887|8888|9091)"

echo ""
echo "=== 访问地址 ==="
echo "Web界面: http://172.19.255.39:9091/"
echo "强制刷新测试: http://172.19.255.39:9091/force_refresh.html"

echo ""
echo "=== 服务管理命令 ==="
echo "查看vehicle_simulator日志:"
if command -v screen >/dev/null 2>&1; then
    echo "  screen -r vehicle_simulator"
elif command -v tmux >/dev/null 2>&1; then
    echo "  tmux attach -t vehicle_simulator"
fi
echo "停止所有服务:"
echo "  pkill -f communication_node"
echo "  pkill -f vehicle_simulator"
echo "  screen -S vehicle_simulator -X quit  # 如果使用screen"
echo "  tmux kill-session -t vehicle_simulator  # 如果使用tmux"

echo ""
echo "=== 启动完成 ==="
