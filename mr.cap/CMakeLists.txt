cmake_minimum_required(VERSION 3.16)
project(mrcap)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(lib/imgui)
add_subdirectory(lib/implot)

add_subdirectory(src)