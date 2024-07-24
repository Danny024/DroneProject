cmake_minimum_required(VERSION 3.5)
project(depth_distance_processor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(depth_distance_processor src/depth_distance_processor.cpp)

ament_target_dependencies(depth_distance_processor rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  depth_distance_processor
  DESTINATION lib/${PROJECT_NAME})

ament_package()

