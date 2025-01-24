cmake_minimum_required(VERSION 3.16)
project(optimizer)

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(tf2 REQUIRED)
find_package(msgs_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# General settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include)

# Define the main executable
add_executable(optimizer_node src/main.cpp)

# Depend on ROS 2 and other necessary libraries
ament_target_dependencies(optimizer_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "gazebo_msgs"
  "nav_msgs"
  "visualization_msgs"
  "GTSAM"
  "tf2"
  "msgs_interfaces"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
)

# Link to the third-party libraries with plain style
target_link_libraries(optimizer_node gtsam)

# Install targets and directories
install(TARGETS optimizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Mark the package for ament
ament_package()