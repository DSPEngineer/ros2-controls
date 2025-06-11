cmake_minimum_required(VERSION 3.8)
project(turtlebot3_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# create the hardware interface library
add_library(turtlebot3_hardware SHARED
  src/turtlebot3_hardware.cpp
)
target_include_directories(turtlebot3_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(turtlebot3_hardware PUBLIC
  rclcpp
  hardware_interface
  pluginlib
)

# create the plugin
pluginlib_export_plugin_description_file(hardware_interface src/turtlebot3_hardware.xml)

install(TARGETS
  turtlebot3_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
