#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <msgs_interfaces/srv/gpt.hpp>
#include "datatypes/RobotData.h"
#include "ArucoPoseSubscriber.h"

using std::placeholders::_1;

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

  void send_request(gtsam::Pose2 next_centroid_pose)
  {
    auto request = std::make_shared<msgs_interfaces::srv::GPT::Request>();

    std::ostringstream oss;
    oss << "Note that all poses follow this syntax (x, y, yaw) and yaw angle of robots can be between -pi to pi according to right hand thumb rule. "
        << "The pose of turtle2 with marker-id 10 is (" << turtle2.x << ", " << turtle2.y << ", " << turtle2.yaw << "), "
        << "turtle4 with marker-id 20 is (" << turtle4.x << ", " << turtle4.y << ", " << turtle4.yaw << "), "
        << "The pose of manipulating object with marker-id 40 is (" << centroid_pose.x << ", " << centroid_pose.y << ", " << centroid_pose.yaw << "). "
        << "Please return 'linear_x' and 'angular_z' for all turtles such that next pose of manipulating object with marker-id 40 is ("
        << next_centroid_pose.x() << ", " << next_centroid_pose.y() << ", " << next_centroid_pose.theta() << ").";

    request->poses_text = oss.str();
    request->image = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    auto result = client_->async_send_request(request);
    
    try{
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
          auto response = result.get();

          double linear_x1, linear_x2;
          double angular_z1, angular_z2;

          linear_x1 = response->response_linearx1;
          angular_z1 = response->response_angularz1;
          linear_x2 = response->response_linearx2;
          angular_z2 = response->response_angularz2;

          // log velocities
          RCLCPP_INFO(this->get_logger(), "Turtle2: linear_x=%f, angular_z=%f", linear_x1, angular_z1);
          RCLCPP_INFO(this->get_logger(), "Turtle4: linear_x=%f, angular_z=%f", linear_x2, angular_z2);

        }
        else {
          RCLCPP_ERROR(this->get_logger(), "Failed to get response");
        }
      } 
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

  private:
  rclcpp::Client<msgs_interfaces::srv::GPT>::SharedPtr client_;
};
