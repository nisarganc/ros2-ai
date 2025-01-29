#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <string>
#include <fstream>
#include <vector>
#include <utility> 

const double M_PI_8 = M_PI_4 / 2.0;

namespace Utils
{

  /**
   * @brief Shifts any arbitrary angle to the range of [0, 2PI)
   * 
   * @param orientation Input angle
   * @return Shifted angle
   */
    double ensure_orientation_range(double orientation)
    {
      return fmod((fmod(orientation, 2 * M_PI) + 2 * M_PI), 2 * M_PI);
    }

  /**
   * @brief Shifts any arbitrary angle to the range of [-PI, PI)
   * 
   * @param orientation Input angle
   * @return Shifted Angle
   */
  double ensure_orientation_range2 (double orientation)
  {
    orientation = ensure_orientation_range(orientation);
    if ((orientation >= M_PI) && (orientation < 2 * M_PI))
    {
      orientation = orientation - 2 * M_PI;
    }
    return orientation;
  }

  double FastArcTan(double z) {
    if (z > 1.0) {
      z = 1.0 / z;
      return M_PI_2 - z * (M_PI_4 - (z - 1) * (M_PI_4 - z * (3.0 * M_PI_8)));
    } else if (z < -1.0) {
      z = 1.0 / z;
      return -M_PI_2 - z * (M_PI_4 - (z - 1) * (M_PI_4 - z * (3.0 * M_PI_8)));
    }
    return z * (M_PI_4 - (z - 1) * (M_PI_4 - z * (3.0 * M_PI_8)));
  }

  double FastAtan2(double y, double x) {
    if (x > 0) {
      return FastArcTan(y / x);
    } else if (y >= 0 > x) {
      return FastArcTan(y / x) + M_PI;
    } else if (y < 0 && x < 0) {
      return FastArcTan(y / x) - M_PI;
    } else if (y > 0 && x == 0) {
      return M_PI_2;
    } else if (y < 0 && x == 0) {
      return -M_PI_2;
    } else {
      return 0; // x = y = 0
    }
  }

  /**
   * @brief Creates a vector using two gtsam::Pose2, such that some calculations can be more easily done
   * 
   * @param vector_tail The gtsam::Pose2 used to form the vector tail
   * @param vector_head The gtsam::Pose2 used to form the vector tail
   * @return A vector formed by calculating vector_head - vector_tail
   */
  Eigen::Vector2f make_vector(gtsam::Pose2 vector_tail, gtsam::Pose2 vector_head)
  {
    Eigen::Vector2f vector_made((vector_head.x() - vector_tail.x()),(vector_head.y() - vector_tail.y()));
    return vector_made;
  }

  double distance_between_points(gtsam::Pose2 start_point, gtsam::Pose2 end_point)
  {
    Eigen::Vector2f vector_StartToEnd = make_vector(start_point, end_point);
    double distance = vector_StartToEnd.norm();

    return distance;
  }

  double angle_between_points(gtsam::Pose2 start_point, gtsam::Pose2 end_point)
  {
    Eigen::Vector2f vector_StartToEnd = make_vector(start_point, end_point);
    double angle = atan2(vector_StartToEnd(1), vector_StartToEnd(0));
    angle = ensure_orientation_range(angle);

    return angle;
  }

  /**
   * @brief Converts from a gtsam::Value to a vector of gtsam::Pose's (ONLY POSITION)
   * @param values: Values container with the values inside it 
   * @return returned_trajectory of std::vector<gtsam:Pose2> 
   */ 
  std::vector<gtsam::Pose2> valuesToPose_mod(gtsam::Values& values, int k, int nr_of_steps, int robot_id)
  {
    int max_states = nr_of_steps;
    std::vector<gtsam::Pose2> returned_trajectory;
    gtsam::Symbol key_pos;

    // Create the initial pose
    gtsam::Pose2 pose = (k < 0) ? gtsam::Pose2(0, 0, 0) : gtsam::Pose2();

    for (int i = 0; i <= max_states; ++i)
    {
        if (i >= k)
        {
            // Update the key_pos based on robot_id only when necessary
            switch (robot_id)
            {
                case 0:
                    key_pos = gtsam::Symbol('C', i);
                    break;
                case 1:
                    key_pos = gtsam::Symbol('X', i);
                    break;
                case 2:
                    key_pos = gtsam::Symbol('Y', i);
                    break;
            }

            // Update the pose directly from values
            pose = values.at<gtsam::Pose2>(key_pos);
        }

        returned_trajectory.push_back(pose);

        // print the trajectory
        std::cout << "GT Pose " << i << ": ";
        std::cout << "x = " << pose.x() << ", ";
        std::cout << "y = " << pose.y() << ", ";
        std::cout << "theta = " << pose.theta() << std::endl;

    }

    return returned_trajectory;
}

std::vector<gtsam::Pose2> valuesToVelocity_mod(gtsam::Values &values, int k, int nr_of_steps, int robot_id) {
    int max_states = nr_of_steps;
    std::vector<gtsam::Pose2> returned_velocity;
    gtsam::Symbol key_control;

    // Create the initial velocity
    gtsam::Pose2 velocity = (k < 0) ? gtsam::Pose2(0, 0, 0) : gtsam::Pose2();

    for (int i = 0; i < max_states; ++i) {
        // Update the key_control based on robot_id only when necessary
        if (i >= k && robot_id == 0) {
            key_control = gtsam::Symbol('U', i);
            // Update the velocity directly from values
            velocity = values.at<gtsam::Pose2>(key_control);
        }

        returned_velocity.push_back(velocity);
    }

    return returned_velocity;
}

}