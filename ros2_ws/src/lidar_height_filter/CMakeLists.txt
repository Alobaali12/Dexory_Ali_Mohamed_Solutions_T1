cmake_minimum_required(VERSION 3.5)
project(lidar_height_filter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(height_filter_node src/height_filter_node.cpp)

ament_target_dependencies(height_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(height_filter_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  height_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
