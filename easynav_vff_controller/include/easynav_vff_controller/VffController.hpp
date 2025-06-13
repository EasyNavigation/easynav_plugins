// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Declaration of the VffController method.

#ifndef EASYNAV_CONTROLLER__VFFCONTROLLER_HPP_
#define EASYNAV_CONTROLLER__VFFCONTROLLER_HPP_

#include <expected>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "easynav_core/ControllerMethodBase.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace easynav
{

/**
 * @brief A structure to hold the VFF vectors.
 *
 * This structure contains three vectors: attractive, repulsive, and result.
 * Each vector is represented as a std::vector of doubles.
 */
struct VFFVectors
{
  std::vector<double> attractive;
  std::vector<double> repulsive;
  std::vector<double> result;
};

/**
 * @brief An enumeration to represent the color of the VFF vectors.
 *
 * This enumeration defines three colors: RED, GREEN, and BLUE.
 * These colors are used for visualization purposes in RViz.
 */
typedef enum {RED, GREEN, BLUE} VFFColor;

/**
 * @class VffController
 * @brief A default "Vff" implementation for the Control Method.
 *
 * This control method does nothing. It serves as an example, and will be used as a default plugin implementation
 * if the navigation system configuration does not specify one.
 */
class VffController : public easynav::ControllerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  VffController() = default;

  /**
   * @brief Default destructor.
   */
  ~VffController() = default;

  /**
   * @brief Initializes the control method plugin.
   *
   * This method is called once during the configuration phase of the controller node,
   * and can be optionally overridden by derived classes to perform custom setup logic.
   *
   * @return std::expected<void, std::string> Returns an expected object:
   *         - `void` if initialization was successful,
   *         - a `std::string` containing an error message if initialization failed.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Get the current control command.
   *
   * This method should return the last control command computed.
   * It should not run the control algorithm (see update method).
   *
   * @return A TwistStamped message with the current control command.
   */
  [[nodiscard]] virtual geometry_msgs::msg::TwistStamped get_cmd_vel() override;

  /**
   * @brief Updates the localization estimate based on the current navigation state.
   *
   * This method is intended to run the localization logic and update the odometry.
   * In this implementation, use the VFF algorithm to compute the control command.
   *
   * @param nav_state The current navigation state of the system.
   */
  virtual void update_rt(const NavState & nav_state) override;

private:
  /**
   * @brief Current robot velocity command.
   */
  geometry_msgs::msg::TwistStamped cmd_vel_;

  /**
   * @brief Current robot odometry.
   */
  geometry_msgs::msg::Point goal_;

  /**
   * @brief Normalizes an angle to the range [-π, π].
   *
   * @param angle The angle to normalize.
   * @return The normalized angle.
   */
  double normalize_angle(double angle);

  /**
   * @brief Computes the VFF vectors based on the angle error and the point cloud.
   *
   * @param angle_error The angle error between the robot and the goal.
   * @param pointcloud_ The point cloud data.
   */
  VFFVectors get_vff(
    double angle_error,
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud_, std::string frame_id);

  /**
   * @brief Publisher for the VFF vectors.
   *
   * This publisher is used to publish the VFF vectors for visualization.
   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  /**
   * @brief Creates a marker for visualization.
   *
   * This function creates a marker for visualization in RViz.
   *
   * @param vector The vector to visualize.
   * @param vff_color The color of the vector.
   *
   * @return A Marker message for visualization.
   */
  visualization_msgs::msg::Marker make_marker(
    const std::vector<double> & vector,
    VFFColor vff_color, std::string frame_id);

  /**
   * @brief Creates a marker array for visualization.
   *
   * This function creates a marker array for visualization in RViz.
   *
   * @param vff_vectors The VFF vectors to visualize.
   *
   * @return A MarkerArray message for visualization.
   */
  visualization_msgs::msg::MarkerArray get_debug_vff(
    const VFFVectors & vff_vectors,
    std::string frame_id);

  /**
   * @brief If the target is reached.
   */
  bool target_reached_ = false;

  /**
   * @brief Previous goal.
   */
  geometry_msgs::msg::Point previous_goal_;
  bool has_previous_goal_ = false;

  /**
   * @brief Distance to the obstacle.
   *
   * This parameter defines the distance which the obstacle is taken into account.
   * If the distance to the obstacle is less than this value, the robot will stop.
   */
  float distance_obstacle_detection_;

  /**
   * @brief Distance to the goal.
   *
   */
  float distance_to_goal_;

  /**
   * @brief Obstacle detection parameters.
   *
   * These parameters define the boundaries for obstacle detection in the PointCloud.
   */
  float obstacle_detection_x_min_;
  float obstacle_detection_x_max_;
  float obstacle_detection_y_min_;
  float obstacle_detection_y_max_;
  float obstacle_detection_z_min_;
  float obstacle_detection_z_max_;

  /**
   * @brief Maximum speed of the robot.
   *
   * This parameter defines the maximum linear speed of the robot.
   */
  double max_speed_;

  /**
   * @brief Maximum angular speed of the robot.
   *
   * This parameter defines the maximum angular speed of the robot.
   */
  double max_angular_speed_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__VFFCONTROLLER_HPP_
