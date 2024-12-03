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
find_package(tf2_geometry_msgs REQUIRED)
find_package(msgs_interfaces REQUIRED)

include_directories(include)

# Define the main executable
add_executable(aruco_pose_node src/aruco_pose_estimation.cpp)

# Depend on ROS 2 and other necessary libraries
ament_target_dependencies(aruco_pose_node
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "cv_bridge"
  "tf2"
  "tf2_geometry_msgs"
  "msgs_interfaces"
)

ament_export_dependencies(msgs_interfaces)
ament_export_include_directories(include)

# Install targets and directories
install(TARGETS aruco_pose_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()