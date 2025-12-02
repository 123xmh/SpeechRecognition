cmake_minimum_required(VERSION 3.8)
project(controller_display)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS 2核心依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 查找自定义消息接口
find_package(chest_interfaces REQUIRED)

# 查找ncurses库
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSESW REQUIRED ncursesw)

# 自动包含依赖
ament_auto_find_build_dependencies()

# 添加可执行文件
add_executable(controller_display_node
  src/controller_display_node.cpp
)

# 包含头文件目录
target_include_directories(controller_display_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${chest_interfaces_INCLUDE_DIRS}
  ${NCURSESW_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(controller_display_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${chest_interfaces_LIBRARIES}
  ${NCURSESW_LIBRARIES}
)

# 安装目标
install(TARGETS controller_display_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  chest_interfaces
)

# 添加ament编译目标
ament_target_dependencies(controller_display_node
  rclcpp
  std_msgs
  chest_interfaces
)

# 添加安装目标
ament_export_libraries(controller_display_node)

# 添加编译测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()