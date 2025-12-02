cmake_minimum_required(VERSION 3.8)
project(ar_display)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chest_interfaces REQUIRED)

# 修改OpenCV依赖，添加freetype模块
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc freetype)

find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSESW REQUIRED ncursesw)

# 添加头文件目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${NCURSESW_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(ar_display_node
  src/ar_display_node.cpp
  src/video_processor.cpp
)

# 链接库 - 确保链接freetype模块
target_link_libraries(ar_display_node
  ${OpenCV_LIBS}
  ${NCURSESW_LIBRARIES}
)

ament_target_dependencies(ar_display_node
  rclcpp
  chest_interfaces
)

# 安装目标
install(TARGETS ar_display_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  chest_interfaces
)

ament_package()