# 故障排除指南

## 概述

本文档提供了UDP通信测试系统的常见问题解决方案和故障排除方法。

## 常见问题分类

### 1. 环境问题

#### 问题：ROS2环境未设置
**症状**：
```
[ERROR] ROS2环境未设置，请先source ROS2环境
```

**解决方案**：
```bash
# 检查ROS2环境
echo $ROS_DISTRO

# 如果没有输出，需要设置ROS2环境
source /opt/ros/humble/setup.bash  # 根据您的ROS2版本调整

# 或者使用工作空间环境
cd /home/jinshuxin/TK30V2/tk_chest_controller_ws
source install/setup.bash
```

#### 问题：工作空间未编译
**症状**：
```
[ERROR] 请在ROS2工作空间根目录运行此脚本
```

**解决方案**：
```bash
# 确保在正确的工作空间目录
cd /home/jinshuxin/TK30V2/tk_chest_controller_ws

# 编译工作空间
colcon build

# 加载环境
source install/setup.bash
```

#### 问题：依赖包未找到
**症状**：
```
[ERROR] 包 udp_simulator 未找到，请先编译工作空间
```

**解决方案**：
```bash
# 检查包是否存在
ros2 pkg list | grep udp_simulator
ros2 pkg list | grep udp_receiver
ros2 pkg list | grep communication

# 如果包不存在，重新编译
colcon build --packages-select udp_simulator udp_receiver communication

# 重新加载环境
source install/setup.bash
```

### 2. 网络问题

#### 问题：端口被占用
**症状**：
```
[ERROR] Failed to bind receive socket to port 8888
```

**解决方案**：
```bash
# 检查端口占用
netstat -tulpn | grep 8888
netstat -tulpn | grep 8889

# 杀死占用端口的进程
sudo kill -9 $(lsof -t -i:8888)
sudo kill -9 $(lsof -t -i:8889)

# 或者使用不同的端口
ros2 launch udp_simulator udp_simulator.launch.py listen_port:=8880
```

#### 问题：UDP套接字创建失败
**症状**：
```
[ERROR] Failed to create send socket
[ERROR] Failed to create receive socket
```

**解决方案**：
```bash
# 检查系统资源
ulimit -n

# 增加文件描述符限制
ulimit -n 65536

# 检查系统内存
free -h

# 重启相关服务
sudo systemctl restart networking
```

#### 问题：网络连接超时
**症状**：
```
[WARNING] Failed to send vehicle state packet
[WARNING] Failed to send head tracking packet
```

**解决方案**：
```bash
# 检查网络接口
ip addr show

# 检查防火墙设置
sudo ufw status

# 临时关闭防火墙（仅用于测试）
sudo ufw disable

# 检查本地回环接口
ping 127.0.0.1
```

### 3. 进程问题

#### 问题：进程启动失败
**症状**：
```
[ERROR] 车辆模拟器启动失败
[ERROR] 头盔接收器启动失败
```

**解决方案**：
```bash
# 检查进程状态
ps aux | grep udp_simulator
ps aux | grep udp_receiver

# 杀死残留进程
pkill -f udp_simulator_node
pkill -f udp_receiver_node

# 检查系统资源
top
htop

# 重启测试
./test_scripts/run_communication_test.sh -c basic_communication
```

#### 问题：进程意外退出
**症状**：
```
[ERROR] 测试进程启动失败
```

**解决方案**：
```bash
# 查看详细错误信息
cat /tmp/udp_test_vehicle.log
cat /tmp/udp_test_helmet.log

# 检查系统日志
journalctl -f

# 使用调试模式运行
ros2 launch udp_simulator udp_simulator.launch.py log_level:=debug
```

### 4. 数据包问题

#### 问题：数据包格式错误
**症状**：
```
[WARNING] Failed to parse received data
[ERROR] Failed to parse UDP data
```

**解决方案**：
```bash
# 检查数据包格式
tcpdump -i lo -X udp port 8888

# 验证协议实现
./test_scripts/test_voice_commands.sh protocol_validation

# 检查字节序
# 确保使用网络字节序（大端模式）
```

#### 问题：CRC校验失败
**症状**：
```
[WARNING] CRC verification failed
```

**解决方案**：
```cpp
// 检查CRC计算算法
uint16_t calculateCrc(const std::vector<uint8_t>& data, size_t start, size_t end) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = start; i < end; i++) {
        crc ^= data[i];
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // 多项式 0x8005 的反向
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}
```

#### 问题：数据包丢失
**症状**：
```
[WARNING] 数据包丢失率过高
```

**解决方案**：
```bash
# 降低发送频率
ros2 launch udp_simulator udp_simulator.launch.py state_frequency:=10.0

# 检查网络延迟
ping -c 10 127.0.0.1

# 增加缓冲区大小
# 在代码中调整UDP缓冲区
```

### 5. 性能问题

#### 问题：CPU使用率过高
**症状**：
```
系统响应缓慢，CPU使用率接近100%
```

**解决方案**：
```bash
# 监控CPU使用率
top -p $(pgrep -f "udp_simulator_node\|udp_receiver_node")

# 降低发送频率
ros2 launch udp_simulator udp_simulator.launch.py state_frequency:=5.0

# 优化代码
# 减少不必要的计算和内存分配
```

#### 问题：内存使用过多
**症状**：
```
系统内存不足，进程被杀死
```

**解决方案**：
```bash
# 检查内存使用
free -h
ps aux --sort=-%mem | head -10

# 清理内存
sudo sync
sudo echo 3 > /proc/sys/vm/drop_caches

# 优化代码
# 及时释放不需要的内存
```

#### 问题：网络吞吐量低
**症状**：
```
数据包/秒 < 预期值
```

**解决方案**：
```bash
# 检查网络统计
cat /proc/net/udp

# 优化网络参数
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.wmem_max=16777216

# 使用更高效的网络库
```

### 6. 配置问题

#### 问题：参数配置错误
**症状**：
```
[ERROR] 无效的参数值
```

**解决方案**：
```bash
# 检查参数范围
# 频率范围：0.1-1000.0 Hz
# 端口范围：1024-65535
# IP地址格式：xxx.xxx.xxx.xxx

# 使用默认参数
ros2 launch udp_simulator udp_simulator.launch.py

# 或者指定有效参数
ros2 launch udp_simulator udp_simulator.launch.py state_frequency:=20.0
```

#### 问题：测试场景不存在
**症状**：
```
[ERROR] 未知的测试场景: invalid_scenario
```

**解决方案**：
```bash
# 查看可用场景
./test_scripts/run_communication_test.sh -l

# 使用正确的场景名称
./test_scripts/run_communication_test.sh normal
```

## 调试技巧

### 1. 启用详细日志
```bash
# 使用详细输出模式
./test_scripts/run_communication_test.sh -v basic_communication

# 或者直接设置日志级别
ros2 launch udp_simulator udp_simulator.launch.py log_level:=debug
```

### 2. 实时监控
```bash
# 监控日志文件
tail -f /tmp/udp_test_vehicle.log
tail -f /tmp/udp_test_helmet.log

# 监控网络流量
tcpdump -i lo udp port 8888 or udp port 8889

# 监控系统资源
htop
iotop
```

### 3. 网络诊断
```bash
# 检查网络连接
netstat -tulpn | grep 888

# 测试UDP连接
nc -u 127.0.0.1 8888

# 检查路由表
ip route show
```

### 4. 进程诊断
```bash
# 查看进程树
pstree -p $(pgrep -f udp_simulator)

# 查看进程状态
ps -ef | grep udp_simulator

# 查看进程资源使用
cat /proc/$(pgrep -f udp_simulator)/status
```

## 性能优化

### 1. 系统优化
```bash
# 优化网络参数
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.wmem_max=16777216
sudo sysctl -w net.core.rmem_default=262144
sudo sysctl -w net.core.wmem_default=262144

# 优化CPU调度
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 优化内存管理
sudo sysctl -w vm.swappiness=10
```

### 2. 应用优化
```cpp
// 使用内存池
class MemoryPool {
    std::vector<std::vector<uint8_t>> pool_;
    std::queue<size_t> available_;
public:
    std::vector<uint8_t>& acquire();
    void release(std::vector<uint8_t>& buffer);
};

// 使用无锁队列
#include <boost/lockfree/queue.hpp>
boost::lockfree::queue<Packet> packet_queue{1000};

// 批量处理
void processPacketsBatch(const std::vector<Packet>& packets) {
    for (const auto& packet : packets) {
        processPacket(packet);
    }
}
```

### 3. 网络优化
```cpp
// 设置套接字选项
int opt = 1;
setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

// 设置缓冲区大小
int rcvbuf = 1024 * 1024;
setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

// 使用非阻塞I/O
int flags = fcntl(socket_fd, F_GETFL, 0);
fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
```

## 故障恢复

### 1. 自动恢复机制
```cpp
class AutoRecovery {
private:
    std::atomic<bool> running_{true};
    std::thread recovery_thread_;
    
public:
    void startRecovery() {
        recovery_thread_ = std::thread([this]() {
            while (running_) {
                if (checkSystemHealth()) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                } else {
                    performRecovery();
                }
            }
        });
    }
    
    bool checkSystemHealth() {
        // 检查系统健康状态
        return true;
    }
    
    void performRecovery() {
        // 执行恢复操作
        restartServices();
        clearBuffers();
        resetConnections();
    }
};
```

### 2. 错误处理策略
```cpp
class ErrorHandler {
public:
    enum class ErrorType {
        NETWORK_ERROR,
        PROTOCOL_ERROR,
        SYSTEM_ERROR
    };
    
    void handleError(ErrorType type, const std::string& message) {
        switch (type) {
            case ErrorType::NETWORK_ERROR:
                handleNetworkError(message);
                break;
            case ErrorType::PROTOCOL_ERROR:
                handleProtocolError(message);
                break;
            case ErrorType::SYSTEM_ERROR:
                handleSystemError(message);
                break;
        }
    }
    
private:
    void handleNetworkError(const std::string& message) {
        // 重试连接
        // 切换备用网络
        // 记录错误日志
    }
    
    void handleProtocolError(const std::string& message) {
        // 丢弃错误数据包
        // 重置协议状态
        // 发送错误报告
    }
    
    void handleSystemError(const std::string& message) {
        // 释放系统资源
        // 重启相关服务
        // 发送告警通知
    }
};
```

## 预防措施

### 1. 定期维护
```bash
# 定期清理日志文件
find /tmp -name "udp_test_*.log" -mtime +7 -delete

# 定期检查系统资源
df -h
free -h

# 定期更新系统
sudo apt update && sudo apt upgrade
```

### 2. 监控告警
```bash
# 设置系统监控
# 使用监控工具如Prometheus + Grafana
# 设置告警规则
```

### 3. 备份恢复
```bash
# 备份配置文件
cp -r /home/jinshuxin/TK30V2/tk_chest_controller_ws/src/communication/test /backup/

# 备份测试数据
tar -czf test_data_backup.tar.gz test_data/
```

## 联系支持

如果以上方法都无法解决问题，请：

1. **收集信息**：
   - 错误日志文件
   - 系统配置信息
   - 复现步骤

2. **联系开发团队**：
   - 提供详细的错误描述
   - 附上相关的日志文件
   - 说明已尝试的解决方案

3. **提供环境信息**：
   - 操作系统版本
   - ROS2版本
   - 硬件配置

---

*最后更新: 2024年*

