cmake_minimum_required(VERSION 3.16)
project(msgs_interfaces)

# General settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

# To convert the interfaces you defined into language-specific code
# (like C++ and Python) so that they can be used in those languages
# ROS Interfaces Settings:https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChatGPT.msg"
  "msg/MarkerPose.msg"
  "msg/MarkerPoseArray.msg"

  "srv/ChatGPT.srv"
  DEPENDENCIES geometry_msgs std_msgs 
 )

ament_package()
