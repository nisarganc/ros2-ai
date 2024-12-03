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

std::map<int, int> RobotMap;

class ArucoPoseEstimation : public rclcpp::Node {
    public:
        ArucoPoseEstimation() : Node("aruco_pose_node") {

            RobotMap[10] = 1;
            RobotMap[20] = 2;
            RobotMap[30] = 3;
            RobotMap[40] = 40;

            cap = std::make_shared<cv::VideoCapture>(4);
            if (!cap->isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
                return;
            }

            marker_pose_publisher_ = this->create_publisher<msgs_interfaces::msg::MarkerPoseArray>(
                "aruco_poses", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), 
                std::bind(&ArucoPoseEstimation::PosesCallback, this));

            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

            int image_width = 1920;
            int image_height = 1080;
            // float fov = 95/2;
            // float fovx = (image_width / image_height) * fov;
            // float fovy = fov;
            double fx = 1019.66062; //image_width / (2.0f * std::tan(fovx * M_PI / 360.0f));
            double cx = 944.551199; //image_width / 2.0f;
            double fy = 1021.42301; //image_height / (2.0f * std::tan(fovy * M_PI / 360.0f));
            double cy = 460.701976; //image_height / 2.0f;
            cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
            distCoeffs = cv::Mat::zeros(1, 5, CV_32F);
            // distCoeffs.at<double>(0) = 0.08621978;
            // distCoeffs.at<double>(1) = 0.08457004;
            // distCoeffs.at<double>(2) = 0.00429467;
            // distCoeffs.at<double>(3) = -0.10166391;
            // distCoeffs.at<double>(4) = -0.06502892;

            //rvec0_tvec0 = WorldFrame();

        }

    private:
        rclcpp::Publisher<msgs_interfaces::msg::MarkerPoseArray>::SharedPtr marker_pose_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<cv::VideoCapture> cap;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Mat cameraMatrix, distCoeffs;
        //std::pair<cv::Mat, cv::Mat> rvec0_tvec0;

        std::pair<cv::Mat, cv::Mat> WorldFrame() {
            cv::Mat frame;
            if (!cap->read(frame)) 
                RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
            
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            std::vector<cv::Vec3d> rvecs, tvecs; 
            cv::Mat rvec0, tvec0;

            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
            if (!markerIds.empty()) {
                cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.16, cameraMatrix, distCoeffs, rvecs, tvecs);                

                for (size_t i = 0; i < markerIds.size(); ++i) {
                    if (markerIds[i] == 0) {
                        rvec0 = cv::Mat(rvecs[i]);
                        tvec0 = cv::Mat(tvecs[i]);
                        return std::make_pair(rvec0, tvec0);
                    }
                }
            }
            return std::make_pair(cv::Mat(), cv::Mat());
        }

        void PosesCallback() {
            cv::Mat frame;
            if (!cap->read(frame)) {
                RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
                return;
            }

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::Mat rvec0, tvec0;
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
            
            if (!markerIds.empty()) {

                cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.16, cameraMatrix, distCoeffs, rvecs, tvecs);
                msgs_interfaces::msg::MarkerPoseArray marker_pose_array_msg;

                for (int i = 0; i < markerIds.size(); ++i) {
                    if (markerIds[i] == 0) {
                            rvec0 = cv::Mat(rvecs[i]);
                            tvec0 = cv::Mat(tvecs[i]);
                             cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                    }
                }

                for (int i = 0; i < markerIds.size(); ++i) {
                    if (markerIds[i] == 0) {
                        continue;
                    }
                    cv::Mat rvec = cv::Mat(rvecs[i]);
                    cv::Mat tvec = cv::Mat(tvecs[i]);

                    // Compute rotation matrix
                    cv::Mat R0, Ri;
                    cv::Rodrigues(rvec0, R0);
                    cv::Rodrigues(rvec, Ri);

                    // create 4*4 transformation matrix
                    cv::Mat T0 = cv::Mat::eye(4, 4, CV_64F);
                    cv::Mat Ti = cv::Mat::eye(4, 4, CV_64F);
                    cv::Mat T_rel = cv::Mat::eye(4, 4, CV_64F);

                    // Fill in rotation and translation for m0Xc and m1Xc
                    R0.copyTo(T0.rowRange(0, 3).colRange(0, 3));
                    tvec0.copyTo(T0.rowRange(0, 3).col(3));
                    T0.at<double>(3, 0) = 0;
                    T0.at<double>(3, 1) = 0;
                    T0.at<double>(3, 2) = 0;
                    Ri.copyTo(Ti.rowRange(0, 3).colRange(0, 3));
                    tvec.copyTo(Ti.rowRange(0, 3).col(3));
                    Ti.at<double>(3, 0) = 0;
                    Ti.at<double>(3, 1) = 0;
                    Ti.at<double>(3, 2) = 0;

                    // Compute relative transformation
                    T_rel = T0.inv() * Ti;                

                    cv::Mat rvec_rel, tvec_rel;
                    cv::Rodrigues(T_rel.rowRange(0, 3).colRange(0, 3), rvec_rel);
                    tvec_rel = T_rel.rowRange(0, 3).col(3);
                    msgs_interfaces::msg::MarkerPose marker_pose;
                    marker_pose.id = RobotMap[markerIds[i]];
                    marker_pose.pose.position.x = tvec_rel.at<double>(0);
                    marker_pose.pose.position.y = tvec_rel.at<double>(1);
                    marker_pose.pose.position.z = tvec_rel.at<double>(2);

                    // RCLCPP_INFO(this->get_logger(), "t_rel: %f %f %f", tvec_rel.at<double>(0), tvec_rel.at<double>(1), tvec_rel.at<double>(2));
                    // RCLCPP_INFO(this->get_logger(), "rvec_rel: %f %f %f", rvec_rel.at<double>(0), rvec_rel.at<double>(1), rvec_rel.at<double>(2));

                    tf2::Quaternion q;
                    q.setRPY(rvec_rel.at<double>(0), rvec_rel.at<double>(1), rvec_rel.at<double>(2));
                    marker_pose.pose.orientation = tf2::toMsg(q);
                    marker_pose_array_msg.poses.push_back(marker_pose);
                
                    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                    cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                }

                marker_pose_array_msg.image = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

                marker_pose_publisher_->publish(marker_pose_array_msg);
            }

            cv::imshow("Aruco Markers", frame);
            cv::waitKey(2);
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoPoseEstimation>());
    rclcpp::shutdown();
    return 0;
}
