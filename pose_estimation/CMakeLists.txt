cmake_minimum_required(VERSION 3.8)
project(pose_estimation)

# General settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(msgs_interfaces REQUIRED)

# Define the main executable
add_executable(aruco_poses_publisher src/ArucoPoseEstimation.cpp)

# Depend on ROS 2 and other necessary libraries
ament_target_dependencies(aruco_poses_publisher
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "cv_bridge"
  "tf2"
  "msgs_interfaces"
)

ament_export_dependencies(msgs_interfaces)

# Install targets and directories
install(TARGETS aruco_poses_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()