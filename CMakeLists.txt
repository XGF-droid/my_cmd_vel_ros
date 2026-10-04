cmake_minimum_required(VERSION 3.8)
project(my_cmd_vel_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
# 2. 定义可执行文件（注意文件名必须和实际文件完全一致）
# 发布节点
add_executable(my_cmd_vel_ros_publisher src/my_cmd_vel_ros_publisher.cpp)
ament_target_dependencies(my_cmd_vel_ros_publisher rclcpp geometry_msgs std_msgs rcl_interfaces)

# 订阅节点
add_executable(cmd_vel_subscriber src/subscriber.cpp)
ament_target_dependencies(cmd_vel_subscriber rclcpp geometry_msgs std_msgs)

# 状态发布节点（如果这个文件还在你的 src 文件夹里，取消下面两行的注释）
# add_executable(robot_status_publisher src/robot_status_publisher.cpp)
# ament_target_dependencies(robot_status_publisher rclcpp geometry_msgs std_msgs)

# 3. 安装可执行文件
install(TARGETS
  my_cmd_vel_ros_publisher
  cmd_vel_subscriber
  # robot_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 安装 launch 和 config 文件夹
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()