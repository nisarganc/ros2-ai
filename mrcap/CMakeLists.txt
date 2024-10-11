cmake_minimum_required(VERSION 3.16)
project(mrcap)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs)
# find_package(gazebo_msgs)

include_directories(
  include
  )

add_executable(mrcap_node src/main.cpp)
target_link_libraries(mrcap_node 
rclcpp)

install(TARGETS
  mrcap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
