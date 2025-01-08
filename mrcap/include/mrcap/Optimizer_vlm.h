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
#include "customFactor.h"
#include "Trajectory.h"
#include "DataTypes/RobotData.h"
#include "FactorGraph.h"
#include "ROSCommunications.h"
#include "Geometry.h"
// #include "DataTypes/Atan2LUT.h"
// std classes
#include <fstream>
#include <vector>
#include <tuple>
#include <random>

// plotting
#include "../implot/implot.h"
#include "../implot/implot_internal.h"
// ROS2
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <thread>
#include <mutex>
// Logger
extern ImGuiLogger logger;
// extern Atan2LUT lut;
std::mutex vicon_mutex;
int robot_failed;


// ==================================== PointMotion Function =====================================
std::pair<std::vector<RobotData>, CentroidData> PointMotion_vlm(Optimization_parameter &optimization_parameter, Geometry_information geometry_information, std::vector<gtsam::Vector3> covariance_info,
                                                            Utils::Disturbance disturbance_info, std::vector<double> solverer_params, SDF_s sdf_s) {

    gtsam::Values init_values;

    // create arrays to store reference poses: change Trajectory.h
    Trajectory::ref_trajv X_ref(optimization_parameter, geometry_information);

    // optimization parameter
    int Ts_translation = optimization_parameter.time_for_translation;
    int Ts_robot_rotation = optimization_parameter.time_for_robot_rotation;
    int Ts_centroid_rotation = optimization_parameter.time_for_centroid_rotation;
    int max_states = optimization_parameter.nr_of_steps;
    int nr_of_robots = optimization_parameter.nr_of_robots;
    double wheel_speed_limit = optimization_parameter.wheel_speed_limit;
    double proportional_gain = optimization_parameter.proportional_gain;
    double integral_gain = optimization_parameter.integral_gain;
    double derivative_gain = optimization_parameter.derivative_gain;

    auto ROS_Enabled = optimization_parameter.run_ros_nodes;
    int use_sdf = optimization_parameter.obstacle_avoidance;
    int adjust_centroid_orientation = optimization_parameter.adjust_centroid_orientation;
    int use_custom_trajectory = optimization_parameter.use_custom_trajectory;

    auto print_fg_factors = optimization_parameter.print_fg_factors;
    auto print_fg_initial_values = optimization_parameter.print_fg_initial_values;
    auto print_fg_iterated_results = optimization_parameter.print_fg_iterated_results;
    auto print_ref_traj = optimization_parameter.print_ref_traj;
    auto print_modelled_traj = optimization_parameter.print_modelled_traj;
    auto use_gazebo = optimization_parameter.gazebo;
    gtsam::Pose2 ref_traj_end_pose = optimization_parameter.ref_traj_end_pose;
    gtsam::Pose2 ref_traj_center_of_rotation = optimization_parameter.ref_traj_center_of_rotation;
    double ref_traj_angular_frequency = optimization_parameter.ref_traj_angular_frequency;
    double ref_traj_amplitude = optimization_parameter.ref_traj_amplitude;
    double pid_error_threshold = optimization_parameter.pid_error_threshold;
    gtsam::Pose2 custom_reference_trajectory[21];
    for (int i = 0; i <= 20; i++) {
        custom_reference_trajectory[i] = optimization_parameter.custom_reference_trajectory[i];
    }
    robot_failed = 10;

    // geometry
    double r = geometry_information.robot_wheel_radius;
    double L = geometry_information.robot_width;

    int dummy = std::system("clear");

    std::vector<RobotData> robots(nr_of_robots);
    CentroidData centroid;

    int k;

    // ROS2 init
    rclcpp::init(0, nullptr);
    if (ROS_Enabled == true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    }

    std::vector<std::shared_ptr<VelocityPublisher>> publishers;
    std::vector<std::shared_ptr<ArucoSubscriber>> subscribers;
    auto vlm_service_client_node = std::make_shared<VLMServiceClient>();
    auto ref_traj_publisher_node = std::make_shared<RefTrajectoryPublisher>(centroid, use_gazebo);
    auto rc_subscriber_node = std::make_shared<ArucoSubscriber>("40");

    int i = 0;
    for (auto &&robot : robots) {
        // subscribers
        robot.robot_id = i + 1;
        int robot_id_sub = robot.robot_id;
        auto subscriber_node = std::make_shared<ArucoSubscriber>(std::to_string(robot_id_sub));
        subscribers.push_back(subscriber_node);
        i = i + 1;
    }

    for (auto &&robot : robots) {
        // publishers
        int robot_id_sub = robot.robot_id;
        auto publisher_node = std::make_shared<VelocityPublisher>(std::to_string(robot_id_sub));
        publishers.push_back(publisher_node);
    }

    for (auto &&subscriber : subscribers) {
        rclcpp::spin_some(subscriber);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    rclcpp::spin_some(rc_subscriber_node);

    // ROS2 calls
    std::cout << "Subscribing to initial robot positions. . . " << std::endl;
    for (int i = 0; i < robot_poses.size(); i++) {
        while (robot_poses[i].x == 0 && robot_poses[i].y == 0 && robot_poses[i].yaw == 0) {
            rclcpp::spin_some(subscribers[i]);
            rclcpp::spin_some(rc_subscriber_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        robots[i].X_k_real.push_back(gtsam::Pose2(robot_poses[i].x, robot_poses[i].y, robot_poses[i].yaw));
        robots[i].X_k_modelled.push_back(gtsam::Pose2(robot_poses[i].x, robot_poses[i].y, robot_poses[i].yaw));
        std::cout << "robot " << i + 1 << " position: " << robot_poses[i].x << ", " << robot_poses[i].y << ", " << robot_poses[i].yaw << std::endl;
    }

    while(centroid_pose.x == 0 && centroid_pose.y == 0 && centroid_pose.yaw == 0) {
        rclcpp::spin_some(rc_subscriber_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    centroid.X_k_real.push_back(gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw));
    centroid.X_k_modelled.push_back(gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw));
    std::cout << "centroid position: " << centroid_pose.x << ", " << centroid_pose.y << ", " << centroid_pose.yaw << std::endl;

    rclcpp::spin_some(vlm_service_client_node);

    
    // computing centroid reference trajectory
    gtsam::Pose2 ref_traj_start_pose; 
    ref_traj_start_pose = gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw);
    Utils::calcualte_R_and_theta_vectors(centroid, robots, geometry_information);

    X_ref.set_initial(ref_traj_start_pose); //initial pose of the object
    X_ref.set_final(ref_traj_end_pose);
    X_ref.set_overall_dist();
    X_ref.fill(); //computes the reference trajectory of object

    gtsam::Vector2 covariance_ternary_vec_2D(covariance_info[2][0], covariance_info[2][1]);
    gtsam::Vector1 covariance_obs_vec_1D(covariance_info[3][0]);
    gtsam::Vector2 covariance_obs_vec_2D(covariance_info[3][0], covariance_info[3][1]);
    gtsam::Vector2 covariance_speed_vec_2D(covariance_info[1][0], covariance_info[1][1]);
    gtsam::Vector1 covariance_speed_vec_1D(covariance_info[1][0]);

    NoiseModels noise_models;
    noise_models = {
        noiseModel::Diagonal::Sigmas(covariance_info[0]),
        // noiseModel::Diagonal::Sigmas(covariance_speed_vec_1D),
        noiseModel::Diagonal::Sigmas(covariance_info[1]),
        noiseModel::Diagonal::Sigmas(covariance_ternary_vec_2D),
        noiseModel::Diagonal::Sigmas(covariance_obs_vec_1D),
        noiseModel::Diagonal::Sigmas(covariance_info[4]),
    };

    std::vector<gtsam::Pose2> custom_reference_trajectory_vec = optimization_parameter.custom_reference_trajectory_vector;
    for (int k = 0; k <= max_states; k++) {
        gtsam::Pose2 pose_ref;
        if (use_custom_trajectory) {
            pose_ref = custom_reference_trajectory_vec[k];
        } else {
            pose_ref = X_ref.get_ref_pose(k);
        }
        centroid.X_k_ref.push_back(pose_ref);
    }


    std::cout << "\nCentroid Reference Trajectory" << std::endl;
    Utils::printPoseVector(centroid.X_k_ref);


    // Centroid control 
    for (int k = 0; k < max_states; k++) { // centroid rotation is ignored
        gtsam::Pose2 ref_control_input = MotionModelArc::centroid_solveForControl(centroid.X_k_ref[k], centroid.X_k_ref[k + 1], optimization_parameter, geometry_information);
        centroid.U_k_ref.push_back(ref_control_input);
    }

    // initialize robot positions and speeds
    for (auto &&robot : robots) {
        double R_i = geometry_information.distance_to_robot[robot.robot_id - 1];
        double th_i = geometry_information.angle_to_robot[robot.robot_id - 1];
    }
 
    centroid.prev_X_k_optimized = centroid.X_k_ref;
    centroid.prev_U_k_optimized = centroid.U_k_ref;

    Benchmark benchmark;

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
    parameters.setMaxIterations(static_cast<int>(solverer_params[0]));
    parameters.setRelativeErrorTol(solverer_params[1]);
    parameters.setAbsoluteErrorTol(solverer_params[2]);
    parameters.setErrorTol(solverer_params[3]);

    int index = static_cast<int>(solverer_params[4]);
    parameters.setLinearSolverType(index >= 0 && index < solverTypes.size() ? solverTypes[index] : "CHOLMOD");
    parameters.setOrderingType("METIS");
    // set LM params
    parameters.setlambdaUpperBound(optimization_parameter.lambdaUpperBound);
    parameters.setlambdaLowerBound(optimization_parameter.lambdaLowerBound);
    parameters.setlambdaInitial(optimization_parameter.lambdaInitial);
    parameters.setlambdaFactor(optimization_parameter.lambdaFactor);
    parameters.setUseFixedLambdaFactor(optimization_parameter.useFixedLambdaFactor);
    parameters.setDiagonalDamping(optimization_parameter.diagonalDamping);

    // print parameters
    std::cout << "=========================================" << std::endl;
    std::cout << "Factor Graph Optimization" << std::endl;
    std::cout << "=========================================" << std::endl;

    benchmark.start();
    for (k = 0; k < max_states; k++) {

        std::cout << "\n---------------------------------" << std::endl;
        std::cout << "iteration " << k << std::endl;

        NonlinearFactorGraph graph;
        
        std::tie(graph, init_values) = MultiRobotFG(k, optimization_parameter, robots, centroid, noise_models, geometry_information, sdf_s);

        gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
        // gtsam::GaussNewtonOptimizer optimizer(graph, init_values, parameters);
        // gtsam::DoglegOptimizer optimizer(graph, init_values, parameters);
        optimizer.optimize();
        gtsam::Values result = optimizer.values();

        // Conversion and assignment
        centroid.X_k_fg = Utils::valuesToPose_mod(result, k, optimization_parameter, 0);

        std::cout << "Returned Trajectory:" << std::endl;
        for (size_t i = 0; i < centroid.X_k_fg.size(); ++i) {
            const auto& pose = centroid.X_k_fg[i];
            std::cout << "Pose " << i + 1 << ": ";
            std::cout << "x = " << pose.x() << ", ";
            std::cout << "y = " << pose.y() << ", ";
            std::cout << "theta = " << pose.theta() << std::endl;
        }

        centroid.all_fg_poses.push_back(centroid.X_k_fg);
        centroid.U_k_fg = Utils::valuesToVelocity_mod(result, k, optimization_parameter, 0);

        // print centroid.U_k_fg
        std::cout << "Returned Velocity:" << std::endl;
        for (size_t i = 0; i < centroid.U_k_fg.size(); ++i) {
            const auto& velocity = centroid.U_k_fg[i];
            std::cout << "Velocity " << i + 1 << ": ";
            std::cout << "x = " << velocity.x() << ", ";
            std::cout << "y = " << velocity.y() << ", ";
            std::cout << "theta = " << velocity.theta() << std::endl;
        }

        // publish updated reference centroid trajectory
        if (ROS_Enabled) {
            ref_traj_publisher_node->publish();
        }

        result.clear();


        //////////////////////////////////////// Multi-Robots Control ////////////////////////////////////////

        vlm_service_client_node->send_request(robots, centroid_pose, centroid.X_k_fg[k+1]);

        // update
        
        
    } // end of main loop

    // ====================================================================================================

    benchmark.end();

   
    if (print_modelled_traj) {
        std::cout << "\nCentroid Modelled Trajectory" << std::endl;
        Utils::printPoseVector(centroid.X_k_modelled);

        for (auto &&robot : robots) {
            std::cout << "\nRobot " << robot.robot_id << " Modelled Trajectory" << std::endl;
            Utils::printPoseVector(robot.X_k_modelled);
        }
    }
    rclcpp::shutdown();
    benchmark.get_error(centroid);
    centroid.time = benchmark.print_results();
    centroid.collision = false;
    std::pair<bool,std::pair<int, double>> collision = benchmark.check_if_collides(centroid.X_k_real, sdf_s.obstacles, sdf_s.system_radius);
    std::ostringstream oss;
    if (collision.first) {
        oss << "Collision at State: " << std::to_string(collision.second.first) << " (" << centroid.X_k_real[collision.second.first].x() << ", " << centroid.X_k_real[collision.second.first].y() << ") distance to obstacle: " << sqrt(collision.second.second) << "";
        logger.AddLog(LOG_INFO, "%s", oss.str().c_str());
        centroid.collision = true;
    }
    // optimization_parameter.collision_flag = collision.first;

    init_values.clear();
    return std::pair(robots, centroid);
}