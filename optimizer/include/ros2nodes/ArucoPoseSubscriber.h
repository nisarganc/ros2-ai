
// 1. Reads aruco_poses msg and stores them in turtle2, turtle4, turtle6, centroid_pose, and frame.
// 2. Draws the trajectory on the frame if traj_generated is true.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <iostream>
#include <cmath>
#include <map>
#include <string>

#include <msgs_interfaces/msg/marker_pose.hpp>
#include <msgs_interfaces/msg/marker_pose_array.hpp>

using std::placeholders::_1;
const double ARROW_LENGTH = 0.1;

struct ROSPose {
  double x;
  double y;
  double yaw;
};

ROSPose turtle2, turtle4, turtle6, centroid_pose;
cv::Mat frame;
bool traj_generated = false;
std::vector<std::tuple<double, double, double>> traj_pose_2d;

class ArucoPoseSubscriber : public rclcpp::Node {
    public:
        ArucoPoseSubscriber() : Node("aruco_pose_subscriber") {
            // create subscriber to the marker poses
            marker_pose_subscriber_ = this->create_subscription<msgs_interfaces::msg::MarkerPoseArray>(
                "aruco_poses", 10, std::bind(&ArucoPoseSubscriber::MarkerPosesCallback, this, _1));

            double fx = 1019.66062; 
            double cx = 944.551199; 
            double fy = 1021.42301; 
            double cy = 460.701976; 
            cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
            distCoeffs = cv::Mat::zeros(1, 5, CV_32F);

            // T0 values: 0.119024, -0.986425, 0.113131, 1.815938, -0.977355, -0.136476, -0.161713, 1.067014, 0.174957, -0.091322, -0.980332, 2.099810, 0.000000, 0.000000, 0.000000, 1.000000
            T0 = (cv::Mat_<double>(4, 4) << 0.119024, -0.986425, 0.113131, 1.815938, -0.977355, -0.136476, -0.161713, 1.067014, 0.174957, -0.091322, -0.980332, 2.099810, 0.000000, 0.000000, 0.000000, 1.000000);
             
        }

    private:
        rclcpp::Subscription<msgs_interfaces::msg::MarkerPoseArray>::SharedPtr marker_pose_subscriber_;
        cv::Mat T0, cameraMatrix, distCoeffs;

        void MarkerPosesCallback(const msgs_interfaces::msg::MarkerPoseArray::SharedPtr msg) {

            for (int i = 0; i < msg->poses.size(); i++) {
                if (msg->poses[i].id == 10) {
                    turtle2.x = msg->poses[i].x;
                    turtle2.y = msg->poses[i].y;
                    turtle2.yaw = msg->poses[i].theta;
                }
                else if (msg->poses[i].id == 20) {
                    turtle4.x = msg->poses[i].x;
                    turtle4.y = msg->poses[i].y;
                    turtle4.yaw = msg->poses[i].theta;
                }
                else if (msg->poses[i].id == 30) {
                    turtle6.x = msg->poses[i].x;
                    turtle6.y = msg->poses[i].y;
                    turtle6.yaw = msg->poses[i].theta;
                }
                else if (msg->poses[i].id == 40) {
                    centroid_pose.x = msg->poses[i].x;
                    centroid_pose.y = msg->poses[i].y;
                    centroid_pose.yaw = msg->poses[i].theta;
                }
            }

            // read frame from sensor msg
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
            frame = cv_ptr->image;

            if (traj_generated) {
                std::cout << "Drawing trajectory" << std::endl;
                for (const auto& [x, y, yaw] : traj_pose_2d) {
                    cv::Mat trajPointHomogeneous = cv::Mat::ones(4, 1, CV_64F);
                    trajPointHomogeneous.at<double>(0) = x;
                    trajPointHomogeneous.at<double>(1) = y;
                    trajPointHomogeneous.at<double>(2) = 0;  // z

                    cv::Mat trajEndPointHomogeneous = cv::Mat::ones(4, 1, CV_64F);
                    trajEndPointHomogeneous.at<double>(0) = x + ARROW_LENGTH * cos(yaw);
                    trajEndPointHomogeneous.at<double>(1) = y + ARROW_LENGTH * sin(yaw);
                    trajEndPointHomogeneous.at<double>(2) = 0;

                    cv::Mat trajPointCamera = T0 * trajPointHomogeneous;
                    cv::Mat trajEndPointCamera = T0 * trajEndPointHomogeneous;
                    cv::Mat projectedTrajPoint2D, projectedTrajEndPoint2D;

                    cv::projectPoints(trajPointCamera.rowRange(0, 3).t(), cv::Mat::zeros(3, 1, CV_64F),
                                    cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, projectedTrajPoint2D);
                    cv::projectPoints(trajEndPointCamera.rowRange(0, 3).t(), cv::Mat::zeros(3, 1, CV_64F),
                                    cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, projectedTrajEndPoint2D); 

                    cv::Point trajPoint2D(
                        static_cast<int>(projectedTrajPoint2D.at<double>(0)),
                        static_cast<int>(projectedTrajPoint2D.at<double>(1))
                    );
                    cv::Point trajEndPoint2D(
                        static_cast<int>(projectedTrajEndPoint2D.at<double>(0)),
                        static_cast<int>(projectedTrajEndPoint2D.at<double>(1))
                    );

                    cv::circle(frame, trajPoint2D, 2, cv::Scalar(255, 0, 0), 6);
                    cv::arrowedLine(frame, trajPoint2D, trajEndPoint2D, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);   
                }
            }

            cv::imshow("Aruco Markers", frame);
            cv::waitKey(0);
        }
};
