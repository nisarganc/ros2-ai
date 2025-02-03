#pragma once
#include <gtsam/geometry/Pose2.h>
    
struct CentroidData {
    gtsam::Pose2 X_k_real; // pose from object aruco
    std::vector<gtsam::Pose2> X_k_ref; // reference poses for simulation
    std::vector<gtsam::Pose2> U_k_ref; // reference velocities for simulation

    std::vector<gtsam::Pose2> X_k_fg; // estimated centroid poses from gtsam
    std::vector<gtsam::Pose2> U_k_fg; // estimated centroid velocities from gtsam
    bool collision;
    double dist_to_goal;
    double time;
    double deviation;
    double path_length;

};