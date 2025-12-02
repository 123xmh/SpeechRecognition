#!/bin/bash

echo "=== 启动 TK Chest Controller Web Monitor ==="

# 检查是否在WSL2环境中
if grep -q "microsoft" /proc/version; then
    echo "✓ 检测到WSL2环境"
    WSL_IP=$(hostname -I | awk '{print $1}')
    echo "✓ WSL2 IP地址: $WSL_IP"
else
    echo "⚠ 未检测到WSL2环境"
fi

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠ ROS2环境未激活，尝试激活..."
    source /opt/ros/humble/setup.bash 2>/dev/null || {
        echo "错误: 无法找到ROS2环境，请先激活ROS2"
        exit 1
    }
fi

echo "✓ ROS2环境: $ROS_DISTRO"

# 检查web_monitor包是否已构建
if [ ! -d "build/web_monitor" ] && [ ! -d "install/web_monitor" ]; then
    echo "⚠ web_monitor包未构建，正在构建..."
    colcon build --packages-select web_monitor
    if [ $? -ne 0 ]; then
        echo "错误: 构建失败"
        exit 1
    fi
fi

# 激活工作空间
source install/setup.bash

echo ""
echo "=== 启动Web Monitor ==="
echo "启动后可以通过以下方式访问:"
if [ ! -z "$WSL_IP" ]; then
    echo "1. WSL2内部: http://$WSL_IP:9090"
    echo "2. Windows浏览器: http://localhost:9090 (需要配置端口转发)"
    echo ""
    echo "如果Windows无法访问，请在Windows PowerShell中运行（管理员权限）:"
    echo "netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=$WSL_IP"
else
    echo "1. 本地访问: http://localhost:9090"
fi

echo ""
echo "按 Ctrl+C 停止服务"
echo ""

# 启动web_monitor
ros2 launch web_monitor web_monitor.launch.py
