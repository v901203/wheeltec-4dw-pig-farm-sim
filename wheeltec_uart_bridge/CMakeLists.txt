cmake_minimum_required(VERSION 3.8)
project(wheeltec_uart_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(wheeltec_packet src/packet.cpp)
target_include_directories(wheeltec_packet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(wheeltec_packet PRIVATE -Wall -Wextra -Wpedantic)

add_executable(wheeltec_uart_bridge_node src/wheeltec_uart_bridge_node.cpp)
target_link_libraries(wheeltec_uart_bridge_node wheeltec_packet)
ament_target_dependencies(wheeltec_uart_bridge_node geometry_msgs rclcpp)
target_compile_options(wheeltec_uart_bridge_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS wheeltec_packet wheeltec_uart_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_packet test/test_packet.cpp)
  target_link_libraries(test_packet wheeltec_packet)
  target_include_directories(test_packet PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(wheeltec_packet)
ament_package()
