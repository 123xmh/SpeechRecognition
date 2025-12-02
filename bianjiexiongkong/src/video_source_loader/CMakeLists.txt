cmake_minimum_required(VERSION 3.8)
project(video_source_loader)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(chest_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# 添加编译定义
add_definitions(-DYAML_CPP_STATIC_DEFINE)

# 包含目录
include_directories(
  include
  ${yaml-cpp_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(video_source_loader_node
  src/video_config_parser.cpp
  src/video_source_loader_node.cpp
)

# 链接依赖项
target_link_libraries(video_source_loader_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${chest_interfaces_LIBRARIES}
  yaml-cpp
)

# 添加目标依赖
ament_target_dependencies(video_source_loader_node
  rclcpp
  rclcpp_components
  std_msgs
  chest_interfaces
)

# 安装可执行文件
install(TARGETS
  video_source_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件目录
install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件目录
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 导出依赖
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  chest_interfaces
  yaml-cpp
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()