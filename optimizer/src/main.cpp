#include <stdio.h>
#include <iostream>
#include <optimizer/Optimizer.h>

#include <thread>
#include <ros2nodes/ArucoPoseEstimation.h>
#include <ros2nodes/RealTrajPublisher.h>
#include <ros2nodes/VelocityPublisher.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <stdio.h>

using std::placeholders::_1;

int main(int argc, char * argv[])
{  
  std::cout << "Start" << std::endl;

  // start velocity publisher
  rclcpp::init(argc, argv);
  auto aruco_pose = std::make_shared<ArucoPoseEstimation>();
  auto turtle2_vel = std::make_shared<VelocityPublisher>("turtle2");
  auto turtle4_vel = std::make_shared<VelocityPublisher>("turtle4");
  // auto turtle6_vel = std::make_shared<VelocityPublisher>("turtle6");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(aruco_pose);
  executor.add_node(turtle2_vel);
  executor.add_node(turtle4_vel);
  // executor.add_node(turtle6_vel);

  // start optimizer
  Optimizer opt;
  opt.startfg();

  executor.spin();

  // Shutdown
  rclcpp::shutdown();
  return 0;

}