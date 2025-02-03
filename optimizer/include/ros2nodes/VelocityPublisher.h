
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>

using std::placeholders::_1;

int traj_mode=0; // stop is 0, push is 1, go back is 2, adjust position is 3

using namespace std::chrono_literals;
class VelocityPublisher : public rclcpp::Node {
public:
  VelocityPublisher(const std::string& robot)
    : Node("velocity_publisher_"+ robot), robot_(robot)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>( "/" +robot_ + "/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&VelocityPublisher::publish, this));
  }

  void publish()
  {
    geometry_msgs::msg::Twist twist;
    if (traj_mode == 0){
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    }
    else if (traj_mode == 1){
      twist.linear.x = 0.05;
      twist.angular.z = 0.0;
    }
    else if (traj_mode == 2){
      twist.linear.x = -0.1;
      twist.angular.z = 0.0;
    }
    else if (traj_mode == 3){
      twist.linear.x = 0.05;
      twist.angular.z = 0.0;
    }
      publisher_->publish(twist);
  }
  private:
  std::string robot_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};