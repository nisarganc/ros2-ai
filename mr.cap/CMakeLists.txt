cmake_minimum_required(VERSION 3.16)
project(mrcap)

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(tf2 REQUIRED)
find_package(msgs_interfaces REQUIRED)

# General settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include)

# Add subprojects for third-party libraries
add_subdirectory(lib/imgui)
add_subdirectory(lib/implot)

# Define the main executable
add_executable(mrcap_node src/main.cpp)

# Depend on ROS 2 and other necessary libraries
ament_target_dependencies(mrcap_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "gazebo_msgs"
  "nav_msgs"
  "visualization_msgs"
  "GTSAM"
  "tf2"
  "msgs_interfaces"
)

# Link to the third-party libraries with plain style
target_link_libraries(mrcap_node gtsam imgui implot)

# Install targets and directories
install(TARGETS mrcap_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Mark the package for ament
ament_package()