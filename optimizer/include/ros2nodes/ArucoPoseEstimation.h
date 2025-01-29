
// 1. Reads ArUco poses from the camera and stores them in turtle2, turtle4, turtle6, and centroid_pose.
// 2. Projects the goal pose into the camera and stroes it in frame.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/string.hpp"

#include <vector>
#include <iostream>
#include <cmath>
#include <map>
#include <string>

using std::placeholders::_1;
const double ARROW_LENGTH = 0.1;

std::map<int, std::string> aruco_turtle;
struct ROSPose {
  double x;
  double y;
  double yaw;
};

ROSPose turtle2, turtle4, turtle6, centroid_pose;
cv::Mat goal_pose, frame;
bool traj_generated = false;
std::vector<std::tuple<double, double, double>> traj_pose_2d;

class ArucoPoseEstimation : public rclcpp::Node {
    public:
        ArucoPoseEstimation() : Node("aruco_pose_node") {

            aruco_turtle[0] = "origin";
            aruco_turtle[10] = "turtle2";
            aruco_turtle[20] = "turtle4";
            aruco_turtle[30] = "turtle6";
            aruco_turtle[40] = "object";

            cap = std::make_shared<cv::VideoCapture>(4);
            // get fps of the camera
            double fps = cap->get(cv::CAP_PROP_FPS);
            RCLCPP_INFO(this->get_logger(), "Camera FPS: %f", fps);

            if (!cap->isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
                return;
            }

            marker_pose_publisher_ = this->create_publisher<std_msgs::msg::String>(
                "aruco_poses_stamp", 10);
            // timer_ = this->create_wall_timer(
            //     std::chrono::milliseconds(100), 
            //     std::bind(&ArucoPoseEstimation::PosesCallback, this));

            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

            int image_width = 1920;
            int image_height = 1080;

            double fx = 1019.66062; 
            double cx = 944.551199; 
            double fy = 1021.42301; 
            double cy = 460.701976; 
            cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
            distCoeffs = cv::Mat::zeros(1, 5, CV_32F);
            // distCoeffs.at<double>(0) = 0.08621978;
            // distCoeffs.at<double>(1) = 0.08457004;
            // distCoeffs.at<double>(2) = 0.00429467;
            // distCoeffs.at<double>(3) = -0.10166391;
            // distCoeffs.at<double>(4) = -0.06502892;

            bool success = false;
            while (!success) {
                RCLCPP_INFO(this->get_logger(), "Attempting World Frame Origin Detection");
                success = WorldFrame();
            }

            ProjectGoalPose(1.0, 3.0, -1.5);

        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marker_pose_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<cv::VideoCapture> cap;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Mat cameraMatrix, distCoeffs, tvec0, rvec0, T0, rvec, tvec, Ri, Ti, T_rel, rvec_rel, tvec_rel;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::Mat projectedGoalPoint2D, projectedGoalEndPoint2D;

        bool WorldFrame() {

            cv::Mat R0;

            if (!cap->read(frame)) { 
                RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
                return false;
            }

            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
            
            if (!markerIds.empty()) {

                cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.16, cameraMatrix, distCoeffs, rvecs, tvecs);                

                for (size_t i = 0; i < markerIds.size(); ++i) {
                    if (markerIds[i] == 0) {
                        rvec0 = cv::Mat(rvecs[i]);
                        tvec0 = cv::Mat(tvecs[i]);

                        // compute rotation matrix
                        cv::Rodrigues(rvec0, R0);

                        // create 4*4 transformation matrix
                        T0 = cv::Mat::eye(4, 4, CV_64F);

                        // Fill in rotation and translation for m0Xc
                        R0.copyTo(T0.rowRange(0, 3).colRange(0, 3));
                        tvec0.copyTo(T0.rowRange(0, 3).col(3));
                        T0.at<double>(3, 0) = 0;
                        T0.at<double>(3, 1) = 0;
                        T0.at<double>(3, 2) = 0;

                        // initialize matrices
                        rvec = cv::Mat::zeros(3, 1, CV_64F);
                        tvec = cv::Mat::zeros(3, 1, CV_64F);            
                        Ri = cv::Mat::zeros(3, 3, CV_64F);
                        Ti = cv::Mat::eye(4, 4, CV_64F);
                        T_rel = cv::Mat::eye(4, 4, CV_64F);
                        rvec_rel = cv::Mat::zeros(3, 1, CV_64F);
                        tvec_rel = cv::Mat::zeros(3, 1, CV_64F);

                        RCLCPP_INFO(this->get_logger(), "World Frame Found");

                        return true; 
                    }
                }
            }

            return false;
        }

        void ProjectGoalPose(double x, double y, double yaw) {

            // Convert goal pose to homogeneous coordinates
            cv::Mat goalPoseHomogeneous = cv::Mat::ones(4, 1, CV_64F);
            goalPoseHomogeneous.at<double>(0) = x;
            goalPoseHomogeneous.at<double>(1) = y;
            goalPoseHomogeneous.at<double>(2) = 0; // z

            // Calculate the arrow's end in the world frame
            cv::Mat goalEndHomogeneous = cv::Mat::ones(4, 1, CV_64F);
            goalEndHomogeneous.at<double>(0) = x + ARROW_LENGTH * cos(yaw);
            goalEndHomogeneous.at<double>(1) = y + ARROW_LENGTH * sin(yaw);
            goalEndHomogeneous.at<double>(2) = 0;

            // Transform the goal pose and arrow end into the camera coordinate system
            cv::Mat goalPoseCamera = T0 * goalPoseHomogeneous;
            cv::Mat goalEndCamera = T0 * goalEndHomogeneous;

            // Project these points into the image
            cv::projectPoints(goalPoseCamera.rowRange(0, 3).t(), cv::Mat::zeros(3, 1, CV_64F), 
                            cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, projectedGoalPoint2D);
            cv::projectPoints(goalEndCamera.rowRange(0, 3).t(), cv::Mat::zeros(3, 1, CV_64F), 
                            cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, projectedGoalEndPoint2D);
        }


        void PosesCallback() {

            if (!cap->read(frame)) {
                RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
                return;
            }

            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
            
            if (!markerIds.empty()) {

                cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.16, cameraMatrix, distCoeffs, rvecs, tvecs);

                for (int i = 0; i < markerIds.size(); ++i) {
                    if (markerIds[i] != 0) {

                        rvec = cv::Mat(rvecs[i]);
                        tvec = cv::Mat(tvecs[i]);

                        // Compute rotation matrix
                        cv::Rodrigues(rvec, Ri);

                        // Fill in rotation and translation for m0Xc and m1Xc
                        Ri.copyTo(Ti.rowRange(0, 3).colRange(0, 3));
                        tvec.copyTo(Ti.rowRange(0, 3).col(3));
                        Ti.at<double>(3, 0) = 0;
                        Ti.at<double>(3, 1) = 0;
                        Ti.at<double>(3, 2) = 0;

                        // Compute relative transformation
                        T_rel = T0.inv() * Ti;  

                        cv::Rodrigues(T_rel.rowRange(0, 3).colRange(0, 3), rvec_rel);
                        tvec_rel = T_rel.rowRange(0, 3).col(3);
                        if (markerIds[i] == 10) {
                            turtle2.x = tvec_rel.at<double>(0);
                            turtle2.y = tvec_rel.at<double>(1);
                            turtle2.yaw = rvec_rel.at<double>(2);
                        } else if (markerIds[i] == 20) {
                            turtle4.x = tvec_rel.at<double>(0);
                            turtle4.y = tvec_rel.at<double>(1);
                            turtle4.yaw = rvec_rel.at<double>(2);
                        } else if (markerIds[i] == 30) {
                            turtle6.x = tvec_rel.at<double>(0);
                            turtle6.y = tvec_rel.at<double>(1);
                            turtle6.yaw = rvec_rel.at<double>(2);
                        } else if (markerIds[i] == 40) {
                            centroid_pose.x = tvec_rel.at<double>(0);
                            centroid_pose.y = tvec_rel.at<double>(1);
                            centroid_pose.yaw = rvec_rel.at<double>(2);
                        }
                        RCLCPP_INFO(this->get_logger(), "marker: %d %f %f %f", markerIds[i], tvec_rel.at<double>(0), tvec_rel.at<double>(1), rvec_rel.at<double>(2));
                    }
                
                    // cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                    // cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                    // custom detection marking
                    cv::Point2f redPoint = markerCorners[i][3]; 
                    cv::Point2f cornerPointx = markerCorners[i][2]; 
                    cv::Point2f cornerPointy = markerCorners[i][0]; 
                    cv::Point2f otherPoint = markerCorners[i][1];

                    // calculate the centre point the border redpoint, cornerPointx, cornerPointy, otherPoint
                    cv::Point2f centrePoint = (redPoint + cornerPointx + cornerPointy + otherPoint) / 4;
                    cv::Point2f midy = (cornerPointy + otherPoint) / 2;
                    cv::Point2f midx = (cornerPointx + otherPoint) / 2;

                    // draw a red x, green y line, and a blue point for aruco markers 
                    cv::arrowedLine(frame, centrePoint, midx, cv::Scalar(0, 0, 255), 3);
                    // cv::arrowedLine(frame, centrePoint, midy, cv::Scalar(0, 255, 0), 3);
                    cv::circle(frame, centrePoint, 2, cv::Scalar(255, 0, 0), 6);
                    cv::putText(frame, aruco_turtle[markerIds[i]], cornerPointy, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

                    // Draw the goal pose and yaw arrow
                    cv::Point goalPoint2D(
                        static_cast<int>(projectedGoalPoint2D.at<double>(0)),
                        static_cast<int>(projectedGoalPoint2D.at<double>(1))
                    );
                    cv::Point goalEndPoint2D(
                        static_cast<int>(projectedGoalEndPoint2D.at<double>(0)),
                        static_cast<int>(projectedGoalEndPoint2D.at<double>(1))
                    );
                    cv::arrowedLine(frame, goalPoint2D, goalEndPoint2D, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);
                    cv::putText(frame, "Goal", goalEndPoint2D + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 1,
                                cv::Scalar(0, 0, 255), 2);

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
                }

            }

            cv::imshow("Aruco Markers", frame);
            cv::waitKey(0);
        }
};
