cmake_minimum_required(VERSION 3.8)
project(communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(chest_interfaces REQUIRED)

# Add protocol parser library
add_library(protocol_parser
  src/protocol_parser.cpp
)
target_include_directories(protocol_parser PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(protocol_parser
  rclcpp
  std_msgs
  chest_interfaces
)

# Add communication node
add_executable(communication_node
  src/communication_node.cpp
)
target_link_libraries(communication_node
  protocol_parser
)
ament_target_dependencies(communication_node
  rclcpp
  std_msgs
  chest_interfaces
)

# Install targets
install(TARGETS
  communication_node
  protocol_parser
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()