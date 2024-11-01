#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <iostream>
#include <cmath>

#include <msgs_interfaces/msg/marker_pose.hpp>
#include <msgs_interfaces/msg/marker_pose_array.hpp>

class ArucoDetectorNode : public rclcpp::Node {
    public:
        ArucoDetectorNode() : Node("aruco_detector_node") {
            // log node start
            RCLCPP_INFO(this->get_logger(), "Aruco detector node started");
            cap = std::make_shared<cv::VideoCapture>(0);
            if (!cap->isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
                return;
            }

            marker_pose_publisher_ = this->create_publisher<msgs_interfaces::msg::MarkerPoseArray>("aruco_poses", 10);

            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
            int image_width = 1920;
            int image_height = 1080;
            float fovx = 47.5;
            float fovy = 47.5;
            float fx = image_width / (2.0f * std::tan(fovx * M_PI / 360.0f));
            float cx = image_width / 2.0f;
            float fy = image_height / (2.0f * std::tan(fovy * M_PI / 360.0f));
            float cy = image_height / 2.0f;
            cameraMatrix = (cv::Mat_<float>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0);
            distCoeffs = cv::Mat::zeros(1, 5, CV_32F);

            posesLoop();  // Begin the loop
        }

    private:
        rclcpp::Publisher<msgs_interfaces::msg::MarkerPoseArray>::SharedPtr marker_pose_publisher_;
        std::shared_ptr<cv::VideoCapture> cap;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Mat cameraMatrix, distCoeffs;

        void posesLoop() {
            while (rclcpp::ok()) {
                cv::Mat frame;
                if (!cap->read(frame)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
                    continue;
                }

                std::vector<int> markerIds;
                std::vector<std::vector<cv::Point2f>> markerCorners;

                // Detection
                cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);

                // Estimation
                std::vector<cv::Vec3d> rvecs, tvecs;
                if (!markerIds.empty()) {
                    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

                    msgs_interfaces::msg::MarkerPoseArray marker_pose_array_msg;

                    for (size_t i = 0; i < markerIds.size(); ++i) {
                        msgs_interfaces::msg::MarkerPose marker_pose;
                        marker_pose.id = markerIds[i];
                        marker_pose.pose.position.x = tvecs[i][0];
                        marker_pose.pose.position.y = tvecs[i][1];
                        marker_pose.pose.position.z = tvecs[i][2];
                        tf2::Quaternion q;
                        q.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
                        marker_pose.pose.orientation = tf2::toMsg(q);
                        marker_pose_array_msg.poses.push_back(marker_pose);

                        cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                    }

                    marker_pose_publisher_->publish(marker_pose_array_msg);
                }

                cv::imshow("Aruco Markers", frame);
                cv::waitKey(1);  
            }
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}