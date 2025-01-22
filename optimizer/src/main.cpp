#include <stdio.h>
#include <iostream>
#include <optimizer/Optimizer.h>
#include <ros2nodes/RealTrajPublisher.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <stdio.h>

using std::placeholders::_1;

int main(int argc, char * argv[])
{  
  std::cout << "Start" << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoSubscriber>());
  rclcpp::shutdown();

  Optimizer opt;
  opt.startfg();

  return 0;

}