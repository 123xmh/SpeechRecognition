# Communication节点测试系统

## 概述

本测试系统用于验证 `communication` 节点的UDP通信功能，通过发送符合头盔通讯协议的数据包来测试节点的接收、解析和响应能力。

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
│  │protocol_tester  │ │performance_test │ │integration_test │ │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 快速开始

### 1. 启动Communication节点

```bash
# 启动您的communication节点
ros2 run communication communication_node

# 或者使用launch文件
ros2 launch communication communication.launch.py
```

### 2. 运行测试

```bash
# 运行基础协议测试
./test_scripts/test_protocol.sh

# 运行性能测试
./test_scripts/test_performance.sh

# 运行完整测试套件
./test_scripts/run_all_tests.sh
```

## 文件结构

```
communication/test/
├── README.md                          # 本文件
├── test_sender/                       # 测试数据发送器
│   ├── CMakeLists.txt & package.xml
│   ├── src/test_sender_node.cpp
│   └── launch/test_sender.launch.py
├── test_scripts/                      # 测试脚本
│   ├── test_protocol.sh              # 协议测试
│   ├── test_performance.sh           # 性能测试
│   ├── test_integration.sh           # 集成测试
│   └── run_all_tests.sh              # 运行所有测试
├── test_data/                         # 测试数据
│   ├── vehicle_states.json
│   ├── head_tracking_data.json
│   ├── voice_commands.json
│   └── test_scenarios.json
└── docs/                              # 文档
    ├── test_guide.md                  # 测试指南
    └── troubleshooting.md             # 故障排除
```

## 测试类型

### 1. 协议测试 (test_protocol.sh)
- 验证数据包格式正确性
- 测试CRC校验
- 验证字节序转换
- 测试各种数据包类型

### 2. 性能测试 (test_performance.sh)
- 测试数据包处理速度
- 验证高频数据传输
- 测试内存使用情况
- 验证CPU使用率

### 3. 集成测试 (test_integration.sh)
- 测试完整的通信流程
- 验证ROS话题发布
- 测试错误处理
- 验证日志输出

## 使用方法

### 启动测试环境

```bash
# 1. 启动communication节点
ros2 run communication communication_node

# 2. 在另一个终端运行测试
./test_scripts/test_protocol.sh
```

### 测试脚本说明

| 脚本文件 | 作用 | 使用场景 |
|---------|------|----------|
| `test_protocol.sh` | 协议格式验证 | 开发阶段验证协议实现 |
| `test_performance.sh` | 性能基准测试 | 性能优化和压力测试 |
| `test_integration.sh` | 系统集成测试 | 完整功能验证 |
| `run_all_tests.sh` | 运行所有测试 | 发布前完整验证 |

## 测试数据

测试系统会向您的communication节点发送以下类型的数据包：

1. **车辆状态数据包 (0x55AE)** - 59字节
2. **头动跟踪数据包 (0x55AB)** - 16字节  
3. **语音文本数据包 (0x55AC)** - 可变长度
4. **语音指令数据包 (0x55AA)** - 21字节
5. **确认包 (0x55AD)** - 9字节

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

---

*最后更新: 2024年*