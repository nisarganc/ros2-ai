// 1. Publishes the latest planned trajectory of the object from GTSAM

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>

#include "datatypes/RobotData.h"

using std::placeholders::_1;

class RefTrajectoryPublisher : public rclcpp::Node {
  public:
  RefTrajectoryPublisher(const CentroidData& centroid)
    : Node("ref_trajectory_publisher"), centroid_(centroid)
    {
    ref_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("ref_traj", 10);
    ref_trajectory_msg_.header.frame_id = "aruco/world";
  }

  void publish()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = ref_trajectory_msg_.header.frame_id;
    for (int i = 0; i < centroid_.X_k_fg.size(); i++)
    {
      pose.pose.position.x = centroid_.X_k_fg[i].x();
      pose.pose.position.y = centroid_.X_k_fg[i].y();
      pose.pose.position.z = 0;
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, centroid_.X_k_fg[i].theta());
      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();
      ref_trajectory_msg_.poses.push_back(pose);
    }
    ref_trajectory_pub_->publish(ref_trajectory_msg_);
  }
  private:
  CentroidData centroid_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_trajectory_pub_;
  nav_msgs::msg::Path ref_trajectory_msg_;
};