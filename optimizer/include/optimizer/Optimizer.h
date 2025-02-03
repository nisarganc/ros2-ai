#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Vector.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
// between factor
#include <gtsam/slam/BetweenFactor.h>

// inference
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Ordering.h>

// headers
#include "SDF.h"
#include "CustomFactor.h"
#include "Trajectory.h"
#include "datatypes/RobotData.h"
#include "FactorGraph.h"

#include "ros2nodes/ArucoPoseSubscriber.h"

#include "datatypes/CovarianceInfo.h"
// #include "DataTypes/Atan2LUT.h"
// std classes
#include <fstream>
#include <vector>
#include <tuple>
#include <random>

// ROS2
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <thread>
#include <mutex>

void Optimizer()
{
    // gtsam variables
    gtsam::Values init_values;
    std::vector<double> solver_parameters = {2000, 0.1, 0.1, 0.1, 3};

    std::vector<CovariancePreset> covariancePresets = {
    //                        Name     x0,x1,x2, t0,t1,t2, o0,o1,o2, p0,p1,p2, u0,u1   
        CovariancePreset("Default", 1,1,0.2, 0.0002,0.0002,0.002, 0.0001,0.0001,0.0001, 0.1,0.1,1, 0.1,0.1),  
        // Add more presets as required.
    };
    double covariance_X[3] = {covariancePresets[0].covX[0], covariancePresets[0].covX[1], covariancePresets[0].covX[2]};
    double covariance_ternary[3] = {covariancePresets[0].covTernary[0], covariancePresets[0].covTernary[1], covariancePresets[0].covTernary[2]};
    double covariance_obs[3] = {covariancePresets[0].covObs[0], covariancePresets[0].covObs[1], covariancePresets[0].covObs[2]};
    double covariance_priors[3] = {covariancePresets[0].covPriors[0], covariancePresets[0].covPriors[1], covariancePresets[0].covPriors[2]};
    double covariance_U[2] = {covariancePresets[0].covU[0], covariancePresets[0].covU[1]};

    gtsam::Vector3 covariance_X_vector(covariance_X[0], covariance_X[1], covariance_X[2]);
    gtsam::Vector3 covariance_U_vector(covariance_U[0], covariance_U[1], 0.01);
    gtsam::Vector3 covariance_ternary_vector(covariance_ternary[0], covariance_ternary[1], covariance_ternary[2]);
    gtsam::Vector3 covariance_obs_vector(covariance_obs[0], 0, 0);
    gtsam::Vector3 covariance_priors_vector(covariance_priors[0], covariance_priors[1], covariance_priors[2]);

    std::vector<gtsam::Vector3> covariance_information;
    covariance_information.push_back(covariance_X_vector);
    covariance_information.push_back(covariance_U_vector);
    covariance_information.push_back(covariance_ternary_vector);
    covariance_information.push_back(covariance_obs_vector);
    covariance_information.push_back(covariance_priors_vector);

    gtsam::Vector2 covariance_ternary_vec_2D(covariance_information[2][0], covariance_information[2][1]);
    gtsam::Vector1 covariance_obs_vec_1D(covariance_information[3][0]);
    gtsam::Vector2 covariance_obs_vec_2D(covariance_information[3][0], covariance_information[3][1]);
    gtsam::Vector2 covariance_speed_vec_2D(covariance_information[1][0], covariance_information[1][1]);
    gtsam::Vector1 covariance_speed_vec_1D(covariance_information[1][0]);

    NoiseModels noise_models;
    noise_models = {
        noiseModel::Diagonal::Sigmas(covariance_information[0]),
        // noiseModel::Diagonal::Sigmas(covariance_speed_vec_1D),
        noiseModel::Diagonal::Sigmas(covariance_information[1]),
        noiseModel::Diagonal::Sigmas(covariance_ternary_vec_2D),
        noiseModel::Diagonal::Sigmas(covariance_obs_vec_1D),
        noiseModel::Diagonal::Sigmas(covariance_information[4]),
    };

    double lambdaFactor = 10;
    double lambdaInitial = 1e-5;
    double lambdaUpperBound = 1000000;
    double lambdaLowerBound = 0.0;
    bool useFixedLambdaFactor = true;
    bool diagonalDamping = false;

    CentroidData centroid;

    // experimental variables
    int max_states = 10;
    int nr_of_robots = 2;
    gtsam::Pose2 ref_traj_end_pose(1.0, 3.0, -1.5);

    // obstacles information 
    int nr_of_obstacles = 1;

    // Environment Information
    SDF_s sdf_s;
    sdf_s.obstacles.reserve(nr_of_obstacles);
    sdf_s.obstacles.push_back(obstacle(0.70, 2.50, 0));
    sdf_s.system_radius = 0.2; //ToDO: Estimate the centroid system radius
    sdf_s.inv_system_radius = 1.0 / sdf_s.system_radius;
    sdf_s.system_radius_squared = sdf_s.system_radius * sdf_s.system_radius;
    sdf_s.safety_radius = 0.1;
    sdf_s.sys_radius_safety_radius = sdf_s.system_radius + sdf_s.safety_radius;
    sdf_s.sys_radius_safety_radius_squared = sdf_s.sys_radius_safety_radius * sdf_s.sys_radius_safety_radius;
    sdf_s.inv_sys_radius_safety_radius = 1.0 / sdf_s.sys_radius_safety_radius;

    // straight line centroid refernce trajectory
    Trajectory::ref_trajv X_ref(max_states); 

    // initialization of the ROS2 node
    rclcpp::init(0, nullptr);
    auto aruco_snode = std::make_shared<ArucoPoseSubscriber>();
    while (centroid_pose.x == 0.0 && centroid_pose.y == 0.0 && centroid_pose.yaw == 0.0) {
        rclcpp::spin_some(aruco_snode);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // print start/centroid_pose
    std::cout << "centroid position: " << centroid_pose.x << ", " << centroid_pose.y << ", " << centroid_pose.yaw << std::endl;
    centroid.X_k_real = gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw);
    
    // print the obstacles
    for (auto &&obstacle : sdf_s.obstacles) {
        std::cout << "Obstacle: " << obstacle.x << ", " << obstacle.y << std::endl;
    }

    // print goal pose
    std::cout << "Goal pose: " << ref_traj_end_pose.x() << ", " << ref_traj_end_pose.y() << ", " << ref_traj_end_pose.theta() << std::endl;

    // computing centroid reference trajectory
    gtsam::Pose2 ref_traj_start_pose; 
    ref_traj_start_pose = gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw);
    X_ref.set_initial(ref_traj_start_pose); 
    X_ref.set_final(ref_traj_end_pose); 
    X_ref.set_overall_dist(); 
    X_ref.set_overall_angle();
    X_ref.fill(); //computes the reference trajectory of object

    // assign the reference trajectory to object centroid
    for (int k = 0; k <= max_states; k++) {
        centroid.X_k_ref.push_back(X_ref.get_ref_pose(k));
    }

    // Centroid reference control 
    for (int k = 0; k < max_states; k++) { 
        gtsam::Pose2 ref_control_input = MotionModelArc::centroid_solveForControl(centroid.X_k_ref[k], centroid.X_k_ref[k + 1]);
        // print the reference control input
        std::cout << "Reference Control " << k <<": " << ref_control_input.x() << ", " << ref_control_input.y() << ", " << ref_control_input.theta() << std::endl;
        centroid.U_k_ref.push_back(ref_control_input);
    }

    std::vector<std::string> solverTypes = {
                    "MULTIFRONTAL_CHOLESKY",
                    "MULTIFRONTAL_QR",
                    "SEQUENTIAL_CHOLESKY",
                    "SEQUENTIAL_QR",
                    "CHOLMOD"};
    gtsam::LevenbergMarquardtParams parameters;
    // gtsam::GaussNewtonParams parameters;
    // gtsam::DoglegParams parameters;
    // gtsam::NonlinearOptimizerParams parameters;
    parameters.setMaxIterations(static_cast<int>(solver_parameters[0]));
    parameters.setRelativeErrorTol(solver_parameters[1]);
    parameters.setAbsoluteErrorTol(solver_parameters[2]);
    parameters.setErrorTol(solver_parameters[3]);

    int index = static_cast<int>(solver_parameters[4]);
    parameters.setLinearSolverType(index >= 0 && index < solverTypes.size() ? solverTypes[index] : "CHOLMOD");
    parameters.setOrderingType("METIS");
    // set LM params
    parameters.setlambdaUpperBound(lambdaUpperBound);
    parameters.setlambdaLowerBound(lambdaLowerBound);
    parameters.setlambdaInitial(lambdaInitial);
    parameters.setlambdaFactor(lambdaFactor);
    parameters.setUseFixedLambdaFactor(useFixedLambdaFactor);
    parameters.setDiagonalDamping(diagonalDamping);
    parameters.setVerbosityLM("SUMMARY");

    std::cout << "\n START FACTOR GRAPHS" << std::endl;

    NonlinearFactorGraph graph;
    
    std::tie(graph, init_values) = CentroidFG(max_states, centroid, noise_models, sdf_s);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
    // gtsam::GaussNewtonOptimizer optimizer(graph, init_values, parameters);
    // gtsam::DoglegOptimizer optimizer(graph, init_values, parameters);
    optimizer.optimize();
    std::cout << "\n OPTIMIZATION COMPLETED" << std::endl;
    gtsam::Values result = optimizer.values();

    // Conversion and assignment: LOOK AT THE CALCULATION
    centroid.X_k_fg = Utils::valuesToPose_mod(result, max_states);

    // compute difference between the reference and the optimized trajectory
    for (int k = 0; k <= max_states; k++) {
        gtsam::Pose2 error = centroid.X_k_fg[k].between(centroid.X_k_ref[k]);
        std::cout << "Difference "<< k << ": " << error.x() << ", " << error.y() << ", " << error.theta() << std::endl;
    }

    std::cout << "Centroid gtsam trajectory:" << std::endl;
    for (size_t i = 0; i < centroid.X_k_fg.size(); ++i) {
        traj_pose_2d.push_back(std::make_tuple(centroid.X_k_fg[i].x(), centroid.X_k_fg[i].y(), centroid.X_k_fg[i].theta()));
    }
    traj_generated = true;
    rclcpp::spin_some(aruco_snode);

    //clear traj_pose_2d
    traj_pose_2d.clear();

    // centroid.U_k_fg = Utils::valuesToVelocity_mod(result, max_states);

    // publish updated reference centroid trajectory
    // ref_traj_publisher_node->publish();
    
    result.clear();

    // vlm_service_client_node->send_request(robots, centroid_pose, centroid.X_k_fg[k+1], publishers);

    // for (auto &&subscriber : subscribers) {
    //         rclcpp::spin_some(subscriber);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(5));
    //     }
    //     rclcpp::spin_some(rc_subscriber_node);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
    //     for (auto &&robot : robots) {
    //         robot.X_k_real.push_back(gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw));
    //     }

    rclcpp::shutdown();
    init_values.clear(); 
}