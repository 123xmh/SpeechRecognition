cmake_minimum_required(VERSION 3.8)
project(head_tracking)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(chest_interfaces REQUIRED)

# 添加头文件目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 创建卡尔曼滤波库
add_library(kalman_filter SHARED
  src/kalman_filter.cpp
)

# 设置目标属性
target_include_directories(kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 创建头动跟踪节点
add_executable(head_tracking_node
  src/head_tracking_node.cpp
)

# 链接库和依赖
target_link_libraries(head_tracking_node
  kalman_filter
)

ament_target_dependencies(head_tracking_node
  rclcpp
  sensor_msgs
  chest_interfaces
)

# 安装目标
install(TARGETS 
  head_tracking_node
  kalman_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()