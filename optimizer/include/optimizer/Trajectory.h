#pragma once

#include "MotionModelArc.h"

namespace Trajectory
{
    class ref_trajv
    {
    private:
        int nr_of_robots;                           
        int last; // the number of iterations   
                                
        double overall_angle; // the angle formed by start and end positions
        std::vector<double> overall_dist; // distance between start and end positions in the x-direction, y-direction, and in length

        gtsam::Pose2 start_pose;  // starting pose of the centroid
        gtsam::Pose2 end_pose; // ending pose of the centroid
        std::vector<gtsam::Pose2> reference_trajectory; // a vector storing the reference trajectory of the target

    public:
        ref_trajv(int nr_of_steps, int nr_of_robots)
        {
            
            last = nr_of_steps;
            nr_of_robots = nr_of_robots;
            overall_angle = 0;

            gtsam::Pose2 empty_pose(0, 0, 0);
            start_pose = empty_pose;
            end_pose = empty_pose;            
        }

        /**
         * @brief Sets the initial pose of the robot
         * 
         * @param initial_pose Initial pose
         */
        void set_initial(gtsam::Pose2 initial_pose)
        {
            start_pose = initial_pose;
        }

        /**
         * @brief Sets the final pose
         * 
         * @param final_pose Final pose
         */
        void set_final(gtsam::Pose2 final_pose)
        {
            end_pose = final_pose;
        }

        void set_overall_angle()
        {
            // ToDo: check the angle normalization
            overall_angle = Utils::angle_between_points(start_pose, end_pose);
        }

        /**
         * @brief Gets the desired gtsam::Pose2 value stored in the reference trajectory vector
         * 
         * @param state_number The desired gtsam::Pose2 index
         * @returns The value stored in the designated state
        */
        gtsam::Pose2 get_ref_pose(int state_number)
        {
            return reference_trajectory[state_number];
        }

        /**
         * @brief Computes the difference in x, y, annd theta between 2 arbitrary states
         * 
         * @param state_start 1st state
         * @param state_end 2nd state
         * @return A gtsam::Pose2 that stores the differences {diff_x, diff_Y, diff_theta}
         */
        gtsam::Pose2 compute_diff(int state_start, int state_end)
        {
            double diff_x, diff_y, diff_theta;
            
            diff_x = reference_trajectory[state_end].x() - reference_trajectory[state_start].x(); 
            diff_y = reference_trajectory[state_end].y() - reference_trajectory[state_start].y(); 
            diff_theta = reference_trajectory[state_end].theta() - reference_trajectory[state_start].theta(); 
            
            gtsam::Pose2 diff(diff_x, diff_y, diff_theta);
            return (diff);
        }

        /**
         * @brief Computes the linear distance between 2 arbitrary states
         * 
         * @param state_start 1st state
         * @param state_end 2nd state
         * @return The linear distance calculated
         */
        double compute_dist(int state_start, int state_end)
        {
            Eigen::Vector2f vector_AToB = Utils::make_vector(reference_trajectory[state_start], reference_trajectory[state_end]);
            double dist = vector_AToB.norm();
            return (dist);
        }

        /**
         * @brief Calculates the difference in the x and y component between X_start and X_final, and the linear distance between X_start and X_final
         * 
        */
        void set_overall_dist()
        {
            double x_overall_length = end_pose.x() - start_pose.x();
            double y_overall_length = end_pose.y() - start_pose.y();
            double overall_length = sqrt(pow(x_overall_length, 2) + pow(y_overall_length, 2));
            overall_dist.push_back(x_overall_length);
            overall_dist.push_back(y_overall_length);
            overall_dist.push_back(overall_length);
        }

        /**
         * @brief Fills out the trajectory
         * 
        */
        void fill()
        {
            double x, y, theta;
            int i;
            double counter;

            for (i = 0; i <= last; i++)
            {
                counter = static_cast<double>(i);
                x = start_pose.x() + counter * overall_dist[0] / last;
                y = start_pose.y() + counter * overall_dist[1] / last;
                // theta = start_pose.theta();
                theta = start_pose.theta() + counter * (end_pose.theta() - start_pose.theta()) / last;
                gtsam::Pose2 pose(x, y, theta);
                reference_trajectory.push_back(pose);
            }

                
            for (i = 0; i <= last; i++)
            {            
                reference_trajectory[i] = gtsam::Pose2(reference_trajectory[i].x(), reference_trajectory[i].y(), reference_trajectory[i].theta());
            }
        }

        /**
         * @brief Prints out the reference trajectory
        */
        void print()
        {
            for(int k = 0; k < reference_trajectory.size() ; ++k)
            {
                reference_trajectory[k].print();
            }
        }
    };
}