# Communication节点测试指南

## 概述

本指南详细介绍了如何测试 `communication` 节点的UDP通信功能。测试系统通过发送符合头盔通讯协议的数据包来验证节点的接收、解析和响应能力。

## 测试架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Communication节点测试系统                 │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    UDP     ┌─────────────────┐        │
│  │   测试数据发送器 │◄──────────►│  Communication  │        │
│  │ (test_sender)   │   Port:8888 │     节点        │        │
│  │                 │   Port:8889 │                 │        │
│  └─────────────────┘            └─────────────────┘        │
├─────────────────────────────────────────────────────────────┤
│                    测试验证工具                              │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐ │
│  │   协议测试器    │ │   性能测试器    │ │   集成测试器    │ │
│  │test_protocol.sh │ │test_performance │ │run_all_tests.sh │ │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 快速开始

### 1. 环境准备

确保您已经：
- 安装了ROS2环境
- 编译了工作空间
- 所有依赖包已正确安装

```bash
# 检查ROS2环境
echo $ROS_DISTRO

# 编译工作空间
cd /home/jinshuxin/TK30V2/tk_chest_controller_ws
colcon build

# 加载环境
source install/setup.bash
```

### 2. 启动Communication节点

```bash
# 启动您的communication节点
ros2 run communication communication_node

# 或者使用launch文件
ros2 launch communication communication.launch.py
```

### 3. 运行测试

```bash
# 运行协议测试
./test_scripts/test_protocol.sh

# 运行性能测试
./test_scripts/test_performance.sh

# 运行所有测试
./test_scripts/run_all_tests.sh
```

## 测试类型详解

### 1. 协议测试 (test_protocol.sh)

**目的**: 验证communication节点对各种数据包格式的解析能力

**测试内容**:
- 车辆状态数据包 (0x55AE) - 59字节
- 头动跟踪数据包 (0x55AB) - 16字节
- 语音文本数据包 (0x55AC) - 可变长度
- 语音指令数据包 (0x55AA) - 21字节

**运行命令**:
```bash
# 测试所有协议类型
./test_scripts/test_protocol.sh

# 测试特定协议类型
./test_scripts/test_protocol.sh -t vehicle_state -f 5.0 -d 30

# 使用详细输出
./test_scripts/test_protocol.sh -v
```

**预期结果**:
- 数据包接收成功率 > 99%
- 协议解析正确率 = 100%
- 无CRC校验错误
- 无字节序转换错误

### 2. 性能测试 (test_performance.sh)

**目的**: 测试communication节点的性能表现

**测试内容**:
- 高频数据传输 (50Hz)
- 压力测试 (100Hz)
- 突发测试 (间歇性高频)

**运行命令**:
```bash
# 运行所有性能测试
./test_scripts/test_performance.sh

# 运行特定性能测试
./test_scripts/test_performance.sh -t high_freq -f 50.0 -d 30

# 使用详细输出
./test_scripts/test_performance.sh -v
```

**性能指标**:
- 吞吐量效率 > 95%
- 最大CPU使用率 < 50%
- 最大内存使用 < 100MB
- 无内存泄漏

### 3. 集成测试 (run_all_tests.sh)

**目的**: 运行完整的测试套件

**测试内容**:
- 协议测试
- 性能测试
- 系统集成验证
- ROS话题检查

**运行命令**:
```bash
# 运行所有测试
./test_scripts/run_all_tests.sh

# 生成测试报告
./test_scripts/run_all_tests.sh -r

# 跳过特定测试
./test_scripts/run_all_tests.sh -s performance

# 使用详细输出
./test_scripts/run_all_tests.sh -v
```

## 测试脚本说明

| 脚本文件 | 作用 | 使用场景 |
|---------|------|----------|
| `test_protocol.sh` | 协议格式验证 | 开发阶段验证协议实现 |
| `test_performance.sh` | 性能基准测试 | 性能优化和压力测试 |
| `run_all_tests.sh` | 运行所有测试 | 发布前完整验证 |

## 测试数据

测试系统会向您的communication节点发送以下类型的数据包：

### 1. 车辆状态数据包 (0x55AE)
```
帧头(2字节) + 数据长度(2字节) + 时间戳(4字节) + 平台标识(1字节) + 
经纬度(8字节) + 海拔高度(4字节) + 对地高度(4字节) + 航向角(4字节) + 
横滚角(4字节) + 俯仰角(4字节) + 速度(2字节) + 对地速度(2字节) + 
油量(1字节) + 电量(1字节) + 云台俯仰(4字节) + 云台方位(4字节) + 
云台激活(1字节) + 弹药类型(3字节) + 警告(2字节) + 校验和(2字节)
```

### 2. 头动跟踪数据包 (0x55AB)
```
帧头(2字节) + 数据长度(2字节) + 偏航角(4字节) + 俯仰角(4字节) + 
跟踪状态(1字节) + 置信度(1字节) + 校验和(2字节)
```

### 3. 语音文本数据包 (0x55AC)
```
帧头(2字节) + 数据长度(2字节) + 操作码(1字节) + 包信息(1字节) + 
文本数据(N字节) + 校验和(2字节)
```

### 4. 语音指令数据包 (0x55AA)
```
帧头(2字节) + 数据长度(2字节) + 数据类别(1字节) + 操作码(1字节) + 
命令ID(4字节) + 参数1(4字节) + 参数2(4字节) + 参数3(1字节) + 校验和(2字节)
```

## 验证方法

测试系统会验证：

1. **数据包接收** - communication节点是否正确接收UDP数据包
2. **协议解析** - 是否正确解析各种数据包格式
3. **ROS话题发布** - 是否发布相应的ROS消息
4. **错误处理** - 对错误数据包的处理是否正确
5. **性能指标** - 处理速度和资源使用情况

## 预期结果

测试通过的标准：
- 数据包接收成功率 > 99%
- 协议解析正确率 = 100%
- ROS话题发布正常
- 无内存泄漏
- CPU使用率 < 50%

## 故障排除

### 常见问题

1. **communication节点未运行**
   ```
   [ERROR] communication节点未运行，请先启动communication节点
   ```
   **解决方案**: 先启动communication节点
   ```bash
   ros2 run communication communication_node
   ```

2. **test_sender包未找到**
   ```
   [ERROR] test_sender包未找到，请先编译工作空间
   ```
   **解决方案**: 编译工作空间
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **数据包发送失败**
   ```
   [WARNING] Failed to send packet
   ```
   **解决方案**: 检查网络配置和端口占用

### 调试技巧

1. **启用详细日志**
   ```bash
   ./test_scripts/test_protocol.sh -v
   ```

2. **查看实时日志**
   ```bash
   tail -f /tmp/protocol_test.log
   ```

3. **监控系统资源**
   ```bash
   htop
   ```

## 测试报告

### 生成测试报告

```bash
# 生成HTML报告
./test_scripts/run_all_tests.sh -r

# 查看报告
firefox /tmp/test_report_*.html
```

### 报告内容

- 测试概述
- 测试结果统计
- 性能指标
- 错误分析
- 建议改进

## 最佳实践

1. **测试前准备**
   - 确保communication节点正在运行
   - 检查网络配置
   - 清理临时文件

2. **测试执行**
   - 按顺序运行测试
   - 记录测试结果
   - 及时处理错误

3. **结果分析**
   - 对比预期结果
   - 分析性能指标
   - 识别问题模式

4. **持续改进**
   - 更新测试用例
   - 优化测试脚本
   - 完善文档

## 联系支持

如果您在使用测试系统时遇到问题，请：

1. 查看本文档的故障排除部分
2. 检查日志文件中的错误信息
3. 参考协议文档验证数据格式
4. 联系开发团队获取技术支持

---

*最后更新: 2024年*