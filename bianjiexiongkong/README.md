# TK Chest Controller Workspace

TK Chest Controller的ROS2工作空间，包含多个功能模块。

## 工作空间结构

```
tk_chest_controller_ws/
├── src/                      # 源代码包
│   ├── web_monitor/         # 网页监控包
│   │   ├── web_monitor/     # Python源码
│   │   ├── launch/          # 启动文件
│   │   ├── scripts/         # Linux脚本
│   │   ├── tools/           # Windows工具
│   │   ├── docs/            # 文档
│   │   └── README.md        # 包说明
│   ├── chest_interfaces/    # 消息接口包
│   ├── head_tracking/       # 头部跟踪包
│   ├── voice_processing/    # 语音处理包
│   ├── ar_display/          # AR显示包
│   ├── communication/       # 通信包
│   ├── controller_display/  # 控制器显示包
│   ├── button_simulator/    # 按钮模拟器包
│   └── video_source_loader/ # 视频源加载器包
├── build/                    # 构建目录
├── install/                  # 安装目录
├── log/                      # 日志目录
├── 说明文档/                 # 项目说明文档
└── README.md                 # 本文件
```

## 快速开始

### 1. 构建工作空间
```bash
colcon build
```

### 2. 激活工作空间
```bash
source install/setup.bash
```

### 3. 启动Web监控
```bash
# 进入web_monitor包目录
cd src/web_monitor

# 使用启动脚本（推荐）
./scripts/start_web_monitor.sh

# 或使用ROS2启动文件
ros2 launch web_monitor web_monitor.launch.py
```

## WSL2环境支持

如果您在WSL2环境中使用，web_monitor包提供了完整的WSL2支持：

1. **自动环境检测**
2. **网络配置脚本**
3. **端口转发工具**
4. **详细配置文档**

详细配置请参考：[web_monitor WSL2配置说明](src/web_monitor/docs/WSL2_配置说明.md)

## 包说明

### web_monitor
基于Web的ROS2监控系统，提供实时状态监控界面。
- 支持WSL2环境
- WebSocket实时数据推送
- 响应式Web界面

### chest_interfaces
定义所有模块间通信的消息接口。

### 其他模块
- head_tracking: 头部跟踪功能
- voice_processing: 语音处理功能
- ar_display: AR显示功能
- communication: 通信功能
- controller_display: 控制器显示功能
- button_simulator: 按钮模拟器
- video_source_loader: 视频源加载器

## 依赖项

- ROS2 Humble
- Python 3.8+
- 相关Python包（见各包的package.xml）

## 许可证

Apache License 2.0

