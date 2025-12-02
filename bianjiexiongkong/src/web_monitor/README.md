# Web Monitor Package

基于Web的ROS2监控系统，用于TK Chest Controller的实时状态监控。

## 功能特性

- 实时监控ROS2话题数据
- WebSocket实时数据推送
- 响应式Web界面
- 支持WSL2环境

## 包结构

```
web_monitor/
├── web_monitor/              # Python包源码
│   ├── __init__.py
│   ├── web_monitor_node.py   # 主节点
│   └── static/               # 静态文件
│       ├── index.html        # 主页面
│       ├── css/              # 样式文件
│       └── js/               # JavaScript文件
├── launch/                   # 启动文件
│   └── web_monitor.launch.py
├── scripts/                  # Linux脚本
│   ├── setup_wsl2_network.sh    # WSL2网络配置
│   └── start_web_monitor.sh     # 启动脚本
├── tools/                    # Windows工具
│   └── setup_wsl2_port_forward.bat  # 端口转发配置
├── docs/                     # 文档
│   └── WSL2_配置说明.md      # WSL2配置指南
├── resource/                 # 包资源
├── package.xml               # 包定义
├── setup.py                  # 安装配置
└── README.md                 # 本文件
```

## 快速开始

### 1. 构建包
```bash
colcon build --packages-select web_monitor
```

### 2. 激活工作空间
```bash
source install/setup.bash
```

### 3. 启动服务
```bash
# 使用启动文件
ros2 launch web_monitor web_monitor.launch.py

# 或使用启动脚本（推荐）
./src/web_monitor/scripts/start_web_monitor.sh
```

## WSL2环境配置

如果您在WSL2环境中使用，请参考 [WSL2配置说明](docs/WSL2_配置说明.md)。

### 快速配置
1. **Windows端**：运行 `tools/setup_wsl2_port_forward.bat`
2. **WSL2端**：运行 `scripts/setup_wsl2_network.sh`

## 访问方式

- **WSL2内部**：`http://WSL2_IP:9090`
- **Windows浏览器**：`http://localhost:9090`（需要配置端口转发）

## 依赖项

- ROS2 Humble
- Python 3.8+
- aiohttp
- websocket-server

## 许可证

Apache License 2.0

