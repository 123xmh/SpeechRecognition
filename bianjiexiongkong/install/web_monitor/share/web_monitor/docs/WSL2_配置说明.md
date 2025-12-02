# WSL2环境下Web Monitor配置说明

## 问题描述

在WSL2环境中，由于网络架构的特殊性，从Windows浏览器访问WSL2内运行的web服务可能会遇到连接问题。这是因为WSL2使用NAT网络，外部无法直接访问WSL2内的服务。

## 解决方案

### 方案1：配置Windows端口转发（推荐）

#### 步骤1：获取WSL2的IP地址
在WSL2终端中运行：
```bash
hostname -I
```
记下显示的IP地址（通常是172.x.x.x格式）。

#### 步骤2：配置Windows端口转发
在Windows PowerShell中运行（需要管理员权限）：
```powershell
# 删除现有规则（如果存在）
netsh interface portproxy delete v4tov4 listenport=9090 listenaddress=0.0.0.0

# 添加新规则（将WSL2_IP替换为实际的IP地址）
netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=WSL2_IP

# 配置Windows防火墙
netsh advfirewall firewall add rule name="WSL2 Web Monitor" dir=in action=allow protocol=TCP localport=9090
```

#### 步骤3：验证配置
```powershell
# 查看端口转发规则
netsh interface portproxy show all
```

### 方案2：使用提供的批处理脚本

1. 在Windows中双击运行 `setup_wsl2_port_forward.bat`
2. 脚本会自动获取WSL2 IP地址并配置端口转发
3. 按照提示完成配置

### 方案3：使用Linux脚本

在WSL2中运行：
```bash
chmod +x setup_wsl2_network.sh
./setup_wsl2_network.sh
```

## 访问方式

配置完成后，可以通过以下方式访问web_monitor：

1. **从Windows浏览器访问**：`http://localhost:9090`
2. **从WSL2内部访问**：`http://WSL2_IP:9090`

## 故障排除

### 常见问题1：端口转发后仍无法访问
- 检查Windows防火墙设置
- 确认防病毒软件没有阻止连接
- 验证端口转发规则是否正确配置

### 常见问题2：WSL2 IP地址变化
WSL2重启后IP地址可能会变化，需要重新配置端口转发。可以：
- 在WSL2中设置静态IP
- 使用脚本自动检测和配置

### 常见问题3：端口被占用
如果9090端口被占用，可以：
- 修改web_monitor使用其他端口
- 停止占用端口的进程
- 使用不同的端口进行转发

## 自动化脚本

### Windows批处理脚本
`setup_wsl2_port_forward.bat` - 自动配置Windows端口转发

### Linux脚本
`setup_wsl2_network.sh` - 检测WSL2环境并提供配置指导

### 启动脚本
`start_web_monitor.sh` - 启动web_monitor并显示连接信息

## 高级配置

### 设置WSL2静态IP
在Windows中创建或编辑 `%USERPROFILE%\.wslconfig` 文件：
```
[wsl2]
networkingMode=mirrored
dhcp=false
ip=192.168.50.2
gateway=192.168.50.1
```

### 使用WSL2的localhost转发
在Windows中启用WSL2的localhost转发功能：
```powershell
# 启用localhost转发
wsl --shutdown
wsl --update
```

## 测试连接

配置完成后，可以通过以下方式测试：

1. **在WSL2中测试**：
```bash
curl http://localhost:9090
```

2. **在Windows中测试**：
```bash
curl http://localhost:9090
```

3. **浏览器测试**：
在Windows浏览器中访问 `http://localhost:9090`

## 注意事项

1. 每次WSL2重启后，IP地址可能会变化，需要重新配置端口转发
2. 确保Windows防火墙允许9090端口的入站连接
3. 某些防病毒软件可能会阻止端口转发，需要添加例外规则
4. 建议将端口转发配置脚本保存，以便重复使用

## 技术支持

如果遇到问题，请检查：
1. WSL2版本是否最新
2. Windows版本是否支持相关功能
3. 网络配置是否正确
4. 防火墙和防病毒软件设置
