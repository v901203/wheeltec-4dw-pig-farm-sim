cmake_minimum_required(VERSION 3.8)
project(turn_on_wheeltec_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)

include_directories(include)

add_executable(wheeltec_robot_node
  src/wheeltec_robot.cpp
  src/Quaternion_Solution.cpp
)

ament_target_dependencies(wheeltec_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial_driver
  io_context
)

install(TARGETS
  wheeltec_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# urdf/launch/map/params_costmap_car 是原本 package 就有的資料夾，
# 這裡只是安裝進 install space 方便之後用 share 路徑找到；OPTIONAL 避免資料夾不存在時 build 失敗。
install(DIRECTORY
  urdf
  launch
  map
  params_costmap_car
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
