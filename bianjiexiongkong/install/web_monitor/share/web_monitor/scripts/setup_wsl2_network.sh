#!/bin/bash

echo "=== WSL2 网络配置脚本 ==="
echo "此脚本将帮助配置WSL2环境下的网络访问"

# 检查是否在WSL2环境中
if ! grep -q "microsoft" /proc/version; then
    echo "警告: 这似乎不是WSL2环境"
    exit 1
fi

echo "✓ 检测到WSL2环境"

# 获取WSL2的IP地址
WSL_IP=$(hostname -I | awk '{print $1}')
echo "✓ WSL2 IP地址: $WSL_IP"

# 检查端口9090是否被占用
if netstat -tlnp 2>/dev/null | grep -q ":9090"; then
    echo "✓ 端口9090已被占用"
    echo "  当前占用进程:"
    netstat -tlnp 2>/dev/null | grep ":9090"
else
    echo "✓ 端口9090可用"
fi

echo ""
echo "=== 访问方式 ==="
echo "1. 在WSL2内部访问: http://$WSL_IP:9090"
echo "2. 在Windows中访问: http://localhost:9090 (需要配置端口转发)"
echo ""
echo "=== 配置Windows端口转发 ==="
echo "在Windows PowerShell中运行以下命令（需要管理员权限）:"
echo ""
echo "# 删除现有规则"
echo "netsh interface portproxy delete v4tov4 listenport=9090 listenaddress=0.0.0.0"
echo ""
echo "# 添加新规则"
echo "netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=$WSL_IP"
echo ""
echo "# 配置防火墙"
echo "netsh advfirewall firewall add rule name=\"WSL2 Web Monitor\" dir=in action=allow protocol=TCP localport=9090"
echo ""
echo "=== 测试连接 ==="
echo "配置完成后，可以通过以下方式测试:"
echo "1. 在WSL2中: curl http://localhost:9090"
echo "2. 在Windows中: 浏览器访问 http://localhost:9090"
echo ""
echo "如果仍然无法连接，请检查:"
echo "1. Windows防火墙设置"
echo "2. 防病毒软件是否阻止了连接"
echo "3. WSL2网络配置是否正确"
