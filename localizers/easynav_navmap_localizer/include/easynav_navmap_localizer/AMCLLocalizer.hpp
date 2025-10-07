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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Declaration of the AMCLLocalizer method.

#ifndef EASYNAV_NAVMAP_LOCALIZER__AMCLLOCALIZER_HPP_
#define EASYNAV_NAVMAP_LOCALIZER__AMCLLOCALIZER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>
#include <random>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "navmap_core/NavMap.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"

namespace easynav
{
namespace navmap
{

/// \brief Structure representing a single particle in the AMCL algorithm.
struct Particle
{
  tf2::Transform pose;      ///< Estimated pose of the particle.
  float hits;                 ///< Number of sensor matches (hits) for this particle.
  float possible_hits;        ///< Maximum number of possible hits.
  double weight;            ///< Normalized importance weight of the particle.

  ::navmap::NavCelId last_cid = std::numeric_limits<uint32_t>::max();
  std::size_t last_surface = 0;
};

/// \brief A localization method implementing a simplified AMCL (Adaptive Monte Carlo Localization) approach.
class AMCLLocalizer : public LocalizerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  AMCLLocalizer();

  /**
   * @brief Destructor.
   */
  ~AMCLLocalizer();

  /**
   * @brief Initializes the localization method.
   *
   * Sets up publishers, subscribers, and prepares the particle filter.
   *
   * @return std::expected<void, std::string> Success or error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Real-time update of the localization state.
   *
   * Used for time-critical update operations.
   *
   * @param nav_state The current navigation state (read/write).
   */
  void update_rt(NavState & nav_state) override;

  /**
   * @brief General update of the localization state.
   *
   * May include operations not suitable for real-time execution.
   *
   * @param nav_state The current navigation state (read/write).
   */
  void update(NavState & nav_state) override;

  /**
   * @brief Gets the current estimated pose as a transform.
   *
   * @return The transform from map to base footprint frame.
   */
  tf2::Transform getEstimatedPose() const;

  /**
   * @brief Gets the current estimated pose as an Odometry message.
   *
   * @return A nav_msgs::msg::Odometry message containing the estimated pose.
   */
  nav_msgs::msg::Odometry get_pose();

protected:
  /**
   * @brief Initializes the set of particles.
   */
  void initializeParticles();

  /**
   * @brief Publishes a TF transform between map and base footprint.
   *
   * @param map2bf The transform to be published.
   */
  void publishTF(const tf2::Transform & map2bf);

  /**
   * @brief Publishes the current set of particles.
   */
  void publishParticles();

  /**
   * @brief Publishes the estimated pose with covariance.
   *
   * @param est_pose The estimated transform to be published.
   */
  void publishEstimatedPose(const tf2::Transform & est_pose);

  /**
   * @brief Applies the motion model to update particle poses.
   *
   * @param nav_state The current navigation state.
   */
  void predict(NavState & nav_state);

  /**
   * @brief Applies the sensor model to update particle weights.
   *
   * @param nav_state The current navigation state.
   */
  void correct(NavState & nav_state);

  /**
   * @brief Re-initializes the particle cloud if necessary.
   */
  void reseed();

  /// TF broadcaster to publish map to base_footprint transform.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// Publisher for visualization of the particle cloud.
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

  /// Publisher for the estimated robot pose with covariance.
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate_pub_;

  /// Subscriber for odometry messages.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  /**
   * @brief Callback for receiving odometry updates.
   *
   * @param msg The incoming odometry message.
   */
  void odom_callback(nav_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Update odom from TFs instead of a odom topic
   *
   */
  void update_odom_from_tf();

  /// List of particles representing the belief distribution.
  std::vector<Particle> particles_;

  /// Random number generator used for sampling noise.
  std::default_random_engine rng_;

  /// Current estimated odometry-based pose.
  tf2::Transform pose_;

  /// Translational noise standard deviation.
  double noise_translation_ {0.01};

  /// Rotational noise standard deviation.
  double noise_rotation_ {0.01};

  /// Coupling noise between translation and rotation.
  double noise_translation_to_rotation_ {0.01};

  /// Minimum translation noise threshold.
  double min_noise_xy_ {0.05};

  /// Minimum yaw noise threshold.
  double min_noise_yaw_ {0.05};

  /// Whether to use TFs to compute odom
  bool compute_odom_from_tf_ {false};

  double inflation_stddev_   {1.5};     // meters
  double inflation_prob_min_ {0.01};    // [0..1] min prob kept in inflated map
  std::size_t correct_max_points_ {1500}; // hard cap of points per sensor cloud
  double weights_tau_ {0.7};            // <1 sharpens, >1 flattens
  // Fraction of particles (0,1] used as "top" set for stats and reseed
  double top_keep_fraction_{0.2};
  double downsampled_cloud_size_{0.05};

  /// Last odometry transform received.
  tf2::Transform odom_{tf2::Transform::getIdentity()};

  /// Previous odometry transform (used to compute deltas).
  tf2::Transform last_odom_{tf2::Transform::getIdentity()};

  /// Flag indicating if the odometry has been initialized.
  bool initialized_odom_ = false;

  /// Time interval (in seconds) after which the particles should be reseeded.
  double reseed_time_;

  /// Timestamp of the last reseed event.
  rclcpp::Time last_reseed_;

  /**
   * @brief Internal static map.
   */
  std::shared_ptr<Bonxai::ProbabilisticMap> bonxai_map_;
  ::navmap::NavMap navmap_;
  ::navmap::NavCelId last_cid_ {0};

  // PerceptionModel percepcion_model_;
};

}  // namespace navmap

}  // namespace easynav
#endif  // EASYNAV_NAVMAP_LOCALIZER__AMCLLOCALIZER_HPP_
