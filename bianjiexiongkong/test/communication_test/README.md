# 车辆节点模拟器

这是一个用于测试头盔通讯协议的车辆节点模拟器，包含UDP通信节点和Web监控界面。

## 功能特性

- **UDP通信节点**：与communication模块进行UDP通信
- **Web监控界面**：实时监控车辆状态和通信数据
- **协议支持**：完整实现头盔通讯协议
  - 车辆状态数据包 (0x55AE)
  - 头动跟踪数据包 (0x55AB)
  - 语音文本数据包 (0x55AC)
  - 语音指令数据包 (0x55AA)
  - 确认包 (0x55AD)

## 目录结构

```
communication_test/
├── src/
│   └── vehicle_simulator.py    # 主程序
├── config/
│   └── vehicle_simulator_config.yaml  # 配置文件
├── web/
│   └── static/
│       ├── vehicle_index.html  # 主页面
│       ├── css/               # 样式文件
│       └── js/                # JavaScript文件
├── test_protocol.py           # 协议测试脚本
├── start_services.sh          # 启动脚本
└── README.md                  # 说明文档
```

## 快速开始

### 1. 启动车辆模拟器

```bash
# 方法1：直接启动
cd /home/jinshuxin/TK30V2/tk_chest_controller_ws/test/communication_test
python3 src/vehicle_simulator.py &

# 方法2：使用启动脚本
./start_services.sh
```

### 2. 访问Web界面

打开浏览器访问：http://172.19.255.39:9091/

### 3. 测试协议功能

```bash
python3 test_protocol.py
```

## 配置说明

配置文件：`config/vehicle_simulator_config.yaml`

```yaml
vehicle_simulator:
  network:
    vehicle_port: 8887        # 车辆节点UDP端口
    helmet:
      ip: "127.0.0.1"         # 头盔节点IP
      port: 8888              # 头盔节点端口
    web:
      host: "0.0.0.0"         # Web服务器监听地址
      port: 9091              # Web服务器端口
```

## 网络端口

- **8887**: 车辆节点UDP监听端口
- **8888**: 头盔节点UDP端口 (communication模块)
- **9091**: Web服务器HTTP端口

## 依赖要求

- Python 3.8+
- aiohttp
- PyYAML

## 使用说明

1. 确保communication模块正在运行
2. 启动车辆模拟器
3. 通过Web界面监控通信状态
4. 可以手动控制车辆状态发送
5. 查看接收到的头动跟踪、语音文本和语音指令数据

## 故障排除

- 如果端口被占用，使用 `pkill -f vehicle_simulator` 终止进程
- 检查防火墙设置，确保端口9091可访问
- 查看控制台输出获取详细日志信息