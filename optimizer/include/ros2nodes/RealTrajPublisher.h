// 1. Stores latest poses of turtles, object, and Image
// 2. Publishes the real trajectory of the object

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>

#include <msgs_interfaces/msg/marker_pose.hpp>
#include <msgs_interfaces/msg/marker_pose_array.hpp>

#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

struct ROSPose {
  double x;
  double y;
  double yaw;
};

ROSPose turtle2, turtle4, turtle6;
ROSPose centroid_pose;
sensor_msgs::msg::Image aruco_image_msg;

class ArucoSubscriber : public rclcpp::Node {
public:
  ArucoSubscriber()
    : Node("aruco_subscriber")
  {
    std::string topic = "aruco_poses";

    real_trajectory_msg_.header.frame_id = "aruco/world"; 
    real_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("real_traj", 10);

    subscription_ = this->create_subscription<msgs_interfaces::msg::MarkerPoseArray>(
    topic, 10, std::bind(&ArucoSubscriber::topicCallback, this, std::placeholders::_1));
  }

private:
  void topicCallback(msgs_interfaces::msg::MarkerPoseArray::SharedPtr msg) {

    // read image 
    aruco_image_msg = msg->image;

    geometry_msgs::msg::PoseStamped pose;
    for (int i = 0; i < msg->poses.size(); i++)
    {      
      if (msg->poses[i].id == 40)
      {
        centroid_pose.x = msg->poses[i].x;
        centroid_pose.y = msg->poses[i].y;
        centroid_pose.yaw = msg->poses[i].theta;
        // log centroid pose
        RCLCPP_INFO(this->get_logger(), "Centroid: x=%f, y=%f, yaw=%f", centroid_pose.x, centroid_pose.y, centroid_pose.yaw);

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
      else if (msg->poses[i].id == 10)
      {
        turtle2.x = msg->poses[i].x;
        turtle2.y = msg->poses[i].y;
        turtle2.yaw = msg->poses[i].theta;
        // log turtle2 pose
        RCLCPP_INFO(this->get_logger(), "Turtle2: x=%f, y=%f, yaw=%f", turtle2.x, turtle2.y, turtle2.yaw);
      }
      else if (msg->poses[i].id == 20)
      {
        turtle4.x = msg->poses[i].x;
        turtle4.y = msg->poses[i].y;
        turtle4.yaw = msg->poses[i].theta;
        // log turtle4 pose
        RCLCPP_INFO(this->get_logger(), "Turtle4: x=%f, y=%f, yaw=%f", turtle4.x, turtle4.y, turtle4.yaw);
      }
      else if (msg->poses[i].id == 30)
      {
        turtle6.x = msg->poses[i].x;
        turtle6.y = msg->poses[i].y;
        turtle6.yaw = msg->poses[i].theta;
        // log turtle6 pose
        RCLCPP_INFO(this->get_logger(), "Turtle6: x=%f, y=%f, yaw=%f", turtle6.x, turtle6.y, turtle6.yaw);
      }
      }
    }

  rclcpp::Subscription<msgs_interfaces::msg::MarkerPoseArray>::SharedPtr subscription_;
  std::string robot_id_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_trajectory_pub_;
  nav_msgs::msg::Path real_trajectory_msg_;

};