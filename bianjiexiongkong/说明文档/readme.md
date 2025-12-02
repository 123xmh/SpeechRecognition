# 简易文件结构

tk_chest_controller_ws/
└── src/
    ├── chest_interfaces/             # 自定义接口包
    ├── head_tracking/               # 头动解算模块
    ├── voice_processing/            # 语音处理模块
    ├── ar_display/                  # AR显示模块
    ├── controller_display/          # 小屏幕显示模块
    ├── button_simulator/            # 按键模拟器
    ├── video_source_loader/         # 视频源加载模块
    ├── communication/               # 通讯模块
    └── launch/                      # 新增：顶层启动文件目录
        └── all.launch.py

# 详细文件结构

tk_chest_controller_ws/
└── src/
    ├── chest_interfaces/             # 自定义接口包
    │   ├── msg/
    │   │   ├── HeadTracking.msg     # 头动信息
    │   │   ├── VoiceCommand.msg     # 语音指令
    │   │   ├── VoiceText.msg        # 语音文本   
    │   │   ├── VideoSource.msg      # 视频源信息
    │   │   ├── EquipmentStatus.msg  # 设备状态
    │   │   └── ButtonState.msg      # 按键状态
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── head_tracking/               # 头动解算模块
    │   ├── include/head_tracking/
    │   │   └── kalman_filter.hpp
    │   ├── src/
    │   │   ├── head_tracking_node.cpp
    │   │   └── kalman_filter.cpp    # 卡尔曼滤波实现
    │   ├── launch/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── voice_processing/            # 语音处理模块
    │   ├── include/voice_processing/
    │   │   └── keyword_matcher.hpp
    │   ├── src/
    │   │   ├── voice_processing_node.cpp
    │   │   └── keyword_matcher.cpp  # 关键词匹配
    │   ├── launch/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── ar_display/                  # AR显示模块
    │   ├── include/
    │   ├── src/
    │   │   ├── ar_display_node.cpp  # ROS节点
    │   │   └── video_processor.cpp  # 视频处理（独立线程）
    │   ├── launch/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├──controller_display/                  # 小屏幕显示模块
    │   ├── include/
    │   ├── src/
    │   │   └── controller_display_node.cpp  # ROS节点
    │   ├── launch/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │       
    ├── button_simulator/            # 按键模拟器
    │   ├── src/
    │   │   └── button_simulator_node.cpp
    │   ├── launch/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── video_source_loader/         # 视频源加载模块
    │   ├── include/
    │   │   └── video_source_loader/
    │   │       └── video_source_loader.hpp
    │   ├── src/
    │   │   ├── video_source_loader_node.cpp  # 主节点文件
    │   │   └── video_config_parser.cpp       # 配置文件解析器
    │   ├── launch/
    │   │   └── video_source_loader.launch.py # 启动文件
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── communication/               # 通讯模块
    │   ├── include/
    │   │   └── communication/
    │   │       └── protocol_parser.hpp  # 协议解析
    │   ├── src/
    │   │   ├── communication_node.cpp   # ROS节点
    │   │   └── protocol_parser.cpp      # 协议实现
    │   ├── launch/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    │
    ├── web_monitor/                  # 网页监控包
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/web_monitor
    │   ├── web_monitor
    │   │   ├── __init__.py
    │   │   ├── web_monitor_node.py   # 主节点，启动Web服务器和ROS节点
    │   │   ├── static/               # 存放静态文件（HTML, JS, CSS）
    │   │   │   ├── index.html
    │   │   │   ├── js
    │   │   │   │   └── app.js
    │   │   │   └── css
    │   │   │       └── style.css
    │   │   └── templates/            # 如果需要使用模板（这里可能不需要，因为单页应用）
    │   ├── launch
    │   │   └── web_monitor.launch.py # 启动文件
    │   └── test/
    │
    └── launch/                      # 新增：顶层启动文件目录
        └── all.launch.py
        
# 模块间通信关系

# 通信协议规范

## 话题列表

| 话题名 | 消息类型 | 发布者 | 订阅者 | 描述 |
|--------|----------|--------|--------|------|
| `/tk_chest/head_tracking/state` | `HeadTracking` | `head_tracking` | `ar_display`, `communication`, `controller_display` | 头部跟踪状态 |
| `/tk_chest/voice_processing/raw_text` | `VoiceText` | `voice_processing` | `ar_display`, `controller_display` | 语音识别文本 |
| `/tk_chest/voice_processing/command` | `VoiceCommand` | `voice_processing` | `communication` | 语音指令 |
| `/tk_chest/equipment/status` | `EquipmentState` | `communication` | `ar_display`, `voice_processing`, `controller_display` | 设备状态 |
| `/tk_chest/video/source/selected` | `VideoSource` | `video_source_loader` | `ar_display`, `controller_display` | 选择的视频源信息 |
| `/tk_chest/button/state` | `ButtonState` | `button_simulator` | `head_tracking`, `ar_display`, `voice_processing`, `video_source_loader`, `controller_display` | 按钮状态 |
| `/tk_chest/sensors/imu` | `sensor_msgs/Imu` | `head_tracking` (模拟) | `head_tracking` | IMU传感器数据 |
| `/tk_chest/microphone/transcript` | `std_msgs/String` | 外部语音输入 | `voice_processing` | 麦克风转录文本 |


## 消息格式说明

[此处详细说明每个消息字段的含义和用法]