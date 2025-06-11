cmake_minimum_required(VERSION 3.5)
project(turtlebot3_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(turtlebot3_hardware SHARED
  src/turtlebot3_hardware.cpp
)

ament_target_dependencies(turtlebot3_hardware
  rclcpp
  hardware_interface
  pluginlib
)

target_include_directories(turtlebot3_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  turtlebot3_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface turtlebot3_hardware.xml)

ament_package()
