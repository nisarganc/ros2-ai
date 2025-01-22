#pragma once
#include <gtsam/geometry/Pose2.h>

struct RobotData {
    int robot_id;
    double target_orientation;
    std::vector<gtsam::Pose2> X_k_real; //pose from robot aruco
    std::vector<gtsam::Pose2> X_k_modelled; //pose from robot aruco
    std::vector<gtsam::Vector2> U_k_fg_CentroidTranslation_RobotRotation;
    std::vector<gtsam::Vector2> U_k_fg_CentroidTranslation_RobotTranslation;
    std::vector<gtsam::Vector2> U_k_fg_CentroidRotation_RobotRotation;
    std::vector<gtsam::Vector2> U_k_fg_CentroidRotation_RobotArc;
    std::vector<gtsam::Pose2> velocity_pose_CentroidTranslation_RobotRotation;
    std::vector<gtsam::Pose2> velocity_pose_CentroidTranslation_RobotTranslation;
    std::vector<gtsam::Pose2> velocity_pose_CentroidRotation_RobotRotation;
    std::vector<gtsam::Pose2> velocity_pose_CentroidRotation_RobotArc;
};
    
struct CentroidData {
    std::vector<gtsam::Pose2> X_k_ref; // reference poses for simulation
    std::vector<gtsam::Pose2> U_k_ref; // reference velocities for simulation

    std::vector<gtsam::Pose2> X_k_real; // pose from object aruco
    std::vector<gtsam::Pose2> X_k_modelled; // pose from object aruco

    std::vector<gtsam::Pose2> X_k_fg; // estimated centroid poses from gtsam
    std::vector<gtsam::Pose2> U_k_fg; // estimated centroid velocities from gtsam

    std::vector<gtsam::Pose2> prev_X_k_optimized; // ref poses
    std::vector<gtsam::Pose2> prev_U_k_optimized; // ref velocities
    std::vector<std::vector<gtsam::Pose2>> all_fg_poses; // vector of centroid poses from gtsam for all iteractions
    std::vector<std::vector<gtsam::Pose2>> all_fg_velocities; // vector of centroid velocities from gtsam for all iteractions
    std::vector<std::vector<gtsam::Pose2>> X_k_ref_vector;
    bool collision;
    double dist_to_goal;
    double time;
    double deviation;
    double path_length;

};