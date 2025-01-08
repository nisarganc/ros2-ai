#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <msgs_interfaces/msg/marker_pose.hpp>
#include <msgs_interfaces/msg/marker_pose_array.hpp>
#include <msgs_interfaces/srv/gpt.hpp>

#include "DataTypes/RobotData.h"

#include <sensor_msgs/msg/image.hpp>


// ==================================== ROS2 subscriber and publisher =========================================

using std::placeholders::_1;

struct ROSPose {
  double x;
  double y;
  double yaw;
};

std::vector<ROSPose> robot_poses(3);
ROSPose centroid_pose;

ROSPose turtle2, turtle4, turtle6;
sensor_msgs::msg::Image aruco_image_msg;



////////////////////////////////////////////// ROS2 Trajectory Publisher //////////////////////////////////////////////////
class RefTrajectoryPublisher : public rclcpp::Node {
  public:
  RefTrajectoryPublisher(const CentroidData& centroid, const bool is_simulation)
    : Node("ref_trajectory_publisher"), centroid_(centroid)
  {
    ref_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("ref_traj", 10);

    if (is_simulation) {
        ref_trajectory_msg_.header.frame_id = "map"; // Change to your frame ID if needed
    } else {
        ref_trajectory_msg_.header.frame_id = "aruco/world"; // Change to your frame ID if needed
    }
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_trajectory_pub_;
  nav_msgs::msg::Path ref_trajectory_msg_;
};


////////////////////////////////////////////// ROS2 aruco robot pose subscriber //////////////////////////////////////////////////
class ArucoSubscriber : public rclcpp::Node {
public:
  ArucoSubscriber(const std::string& robot_id)
    : Node("aruco_subscriber_" + robot_id),  robot_id_(robot_id)
  {
    std::string topic = "aruco_poses";
    subscription_ = this->create_subscription<msgs_interfaces::msg::MarkerPoseArray>(
      topic, 10, std::bind(&ArucoSubscriber::topicCallback, this, std::placeholders::_1));
    real_trajectory_msg_.header.frame_id = "aruco/world"; // Change to your frame ID if needed
    real_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("real_traj", 10);
  }

private:
  void topicCallback(msgs_interfaces::msg::MarkerPoseArray::SharedPtr msg) {
    int robot_id = std::stoi(robot_id_);

    // read image 
    aruco_image_msg = msg->image;
    for (int i = 0; i < msg->poses.size(); i++)
    {      
      if (msg->poses[i].id == robot_id && robot_id == 40)
      {
        centroid_pose.x = msg->poses[i].x;
        centroid_pose.y = msg->poses[i].y;
        centroid_pose.yaw = msg->poses[i].theta;
      }
      else{
        robot_poses[robot_id-1].x = msg->poses[i].x;
        robot_poses[robot_id-1].y = msg->poses[i].y;
        robot_poses[robot_id-1].yaw = msg->poses[i].theta;
      }

      // Visualize the trajectory
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "aruco/world"; // Change to your frame ID if needed
      if(msg->poses[i].id == robot_id && robot_id == 40)
        {
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
      }
    }

  rclcpp::Subscription<msgs_interfaces::msg::MarkerPoseArray>::SharedPtr subscription_;
  std::string robot_id_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_trajectory_pub_;
  nav_msgs::msg::Path real_trajectory_msg_;

};


////////////////////////////////////////////// ROS2 Velocity Publisher //////////////////////////////////////////////////
using namespace std::chrono_literals;
class VelocityPublisher : public rclcpp::Node {
public:
  VelocityPublisher(const std::string& robot_id)
    : Node("publisher_" + robot_id), robot_id_(robot_id)
  {
    int turtle_id = 0;
    if (std::stoi(robot_id_) == 1) {
        turtle_id = 2;
    } else if (std::stoi(robot_id_) == 2) {
        turtle_id = 4;
    } else if (std::stoi(robot_id_) == 3) {
        turtle_id = 6;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid robot_id: %s", robot_id_.c_str());
    }
    std::string turtle_id_ = std::to_string(turtle_id);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle" + turtle_id_ + "/cmd_vel", 10);
  }

  void publish(double cmd_vel_x, double cmd_vel_w)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd_vel_x;
    twist.angular.z = cmd_vel_w;
    if((cmd_vel_x >0) || (cmd_vel_w > 0)){
    // RCLCPP_INFO(this->get_logger(), "Publishing (robot_id=%s): (v, w) ('%f', '%f')", robot_id_.c_str(), cmd_vel_x, cmd_vel_w);
    }
    publisher_->publish(twist);
  }
  private:
  std::string robot_id_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

class VLMServiceClient : public rclcpp::Node {
public:
  VLMServiceClient(): Node("VLM_service_client")
  {
    client_ = this->create_client<msgs_interfaces::srv::GPT>("GPT_service");

    // Wait for the service to be available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

  }

  void send_request(std::vector<RobotData> robots_poses, ROSPose centroid_pose, gtsam::Pose2 next_centroid_pose)
  {
    auto request = std::make_shared<msgs_interfaces::srv::GPT::Request>();
    for (auto &&robot : robots_poses) {
      if (robot.robot_id == 1) {
        turtle2.x = robot.X_k_real.back().x();
        turtle2.y = robot.X_k_real.back().y();
        turtle2.yaw = robot.X_k_real.back().theta();
      } else if (robot.robot_id == 2) {
        turtle4.x = robot.X_k_real.back().x();
        turtle4.y = robot.X_k_real.back().y();
        turtle4.yaw = robot.X_k_real.back().theta();
      } else if (robot.robot_id == 3) {
        turtle6.x = robot.X_k_real.back().x();
        turtle6.y = robot.X_k_real.back().y();
        turtle6.yaw = robot.X_k_real.back().theta();
      } 
      
    }
    // request->poses_text = "Note that all poses follow this syntax (x, y, yaw) and yaw angle of robots can be between -pi to pi according to right hand tumb rule. The pose of turtle2 with marker-id 10 is ({}, {}, {}), turtle4 with marker-id 20 is ({}, {}, {}), and turtle6 with marker-id 30 is ({}, {}, {}). The pose of manipulating object with marker-id 40 is ({}, {}, {}). Please return control commands of all turtles such that next pose of manipulating object with marker-id 40 is ({}, {}, {}). "; 
    std::ostringstream oss;
    oss << "Note that all poses follow this syntax (x, y, yaw) and yaw angle of robots can be between -pi to pi according to right hand thumb rule. "
        << "The pose of turtle2 with marker-id 10 is (" << turtle2.x << ", " << turtle2.y << ", " << turtle2.yaw << "), "
        << "turtle4 with marker-id 20 is (" << turtle4.x << ", " << turtle4.y << ", " << turtle4.yaw << "), "
        << "and turtle6 with marker-id 30 is (" << turtle6.x << ", " << turtle6.y << ", " << turtle6.yaw << "). "
        << "The pose of manipulating object with marker-id 40 is (" << centroid_pose.x << ", " << centroid_pose.y << ", " << centroid_pose.yaw << "). "
        << "Please return control commands of all turtles such that next pose of manipulating object with marker-id 40 is ("
        << next_centroid_pose.x() << ", " << next_centroid_pose.y() << ", " << next_centroid_pose.theta() << ").";

    request->poses_text = oss.str();
    request->image = aruco_image_msg;

    auto result = client_->async_send_request(request);
    
    try{
      auto response = result.get();
      RCLCPP_INFO(this->get_logger(), "Result of GPT: %s", response->response_text.c_str());
    } 
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

  private:
  rclcpp::Client<msgs_interfaces::srv::GPT>::SharedPtr client_;
};
