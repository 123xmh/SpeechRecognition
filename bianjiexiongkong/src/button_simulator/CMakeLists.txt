cmake_minimum_required(VERSION 3.8)
project(button_simulator)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chest_interfaces REQUIRED)

# 查找 ncursesw 库
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES REQUIRED ncursesw)

# 添加宽字符支持编译选项
add_compile_definitions(_XOPEN_SOURCE_EXTENDED)
add_compile_options(-D_XOPEN_SOURCE_EXTENDED)

# 添加头文件目录
include_directories(
  include
  ${NCURSES_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(button_simulator_node
  src/button_simulator_node.cpp
)

# 链接库 - 关键修改：添加 -lncursesw
target_link_libraries(button_simulator_node
  ${rclcpp_LIBRARIES}
  ${chest_interfaces_LIBRARIES}
  ${NCURSES_LIBRARIES}
  -lncursesw
)

# 添加 ncurses 头文件路径
target_include_directories(button_simulator_node PRIVATE
  ${NCURSES_INCLUDE_DIRS}
)

ament_target_dependencies(button_simulator_node
  rclcpp
  chest_interfaces
)

# 安装目标
install(TARGETS button_simulator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()