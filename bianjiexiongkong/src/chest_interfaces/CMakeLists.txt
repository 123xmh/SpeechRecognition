cmake_minimum_required(VERSION 3.8)
project(chest_interfaces)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ButtonState.msg"
  "msg/EquipmentState.msg"  
  "msg/HeadTracking.msg"
  "msg/VideoSource.msg"
  "msg/VoiceCommand.msg"
  "msg/VoiceText.msg"
  DEPENDENCIES std_msgs
)

# 关键修复：显式指定头文件安装路径
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/chest_interfaces/
  DESTINATION include/chest_interfaces
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

# 修正：移除不正确的导出指令
# ament_export_include_directories(include)  # 删除或注释掉这一行

ament_package()