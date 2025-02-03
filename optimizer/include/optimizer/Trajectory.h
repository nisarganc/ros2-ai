#pragma once

#include "MotionModelArc.h"
#include "Utils.h"

namespace Trajectory
{
    class ref_trajv
    {
    private:

        gtsam::Pose2 start_pose;  // starting pose of the centroid
        gtsam::Pose2 end_pose; // ending pose of the centroid
        int last; // the number of iterations   
                                
        double overall_angle; // the angle formed by start and end positions
        std::vector<double> overall_dist; // distance between start and end positions in the x-direction, y-direction, and in length

        std::vector<gtsam::Pose2> reference_trajectory; // a vector storing the reference trajectory of the target

    public:
        ref_trajv(int nr_of_steps)
        { 
            gtsam::Pose2 empty_pose(0, 0, 0);
            start_pose = empty_pose;
            end_pose = empty_pose; 
            last = nr_of_steps;

            overall_angle = 0;  
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
            // print the overall distance
            std::cout << "Overall distance: " << overall_dist[0] << ", " << overall_dist[1] << ", " << overall_dist[2] << std::endl;
        }

        void set_overall_angle()
        {
            // ToDo: check the overall direction
            overall_angle = Utils::angle_between_points(start_pose, end_pose);

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

            // calculate angle difference between start_pose.theta() and end_pose.theta()
            double angle_diff = Utils::ensure_orientation_range2(end_pose.theta() - start_pose.theta());
            std::cout << "Angle difference: " << angle_diff << std::endl;

            for (i = 0; i <= last; i++)
            {
                counter = static_cast<double>(i);
                x = start_pose.x() + counter * overall_dist[0] / last;
                y = start_pose.y() + counter * overall_dist[1] / last;
                theta = start_pose.theta() + counter * angle_diff / last;
                gtsam::Pose2 pose(x, y, Utils::ensure_orientation_range2(theta));

                std::cout << "Reference Pose " << i << ": " << pose.x() << ", " << pose.y() << ", " << pose.theta() << std::endl;
                reference_trajectory.push_back(pose);
            }
        }
    };
}