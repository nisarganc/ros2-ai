// 1. Publishes the real trajectory of the centroid object

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>

#include <ros2nodes/ArucoPoseSubscriber.h>

using std::placeholders::_1;

class RealTrajPublisher : public rclcpp::Node {
public:
  RealTrajPublisher(): Node("ref_trajectory_publisher")
  {
    std::string topic = "aruco_poses";

    real_trajectory_msg_.header.frame_id = "aruco/world"; 
    real_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("real_traj", 10);

  }

private:
  void publish() {

    geometry_msgs::msg::PoseStamped pose;

    pose.header.stamp = this->now();
    pose.header.frame_id = "aruco/world"; 
    pose.pose.position.x = centroid_pose.x;
    pose.pose.position.y = centroid_pose.y;
    pose.pose.position.z = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, centroid_pose.yaw);
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    real_trajectory_msg_.poses.push_back(pose);
    real_trajectory_pub_->publish(real_trajectory_msg_);

    }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_trajectory_pub_;
  nav_msgs::msg::Path real_trajectory_msg_;

};