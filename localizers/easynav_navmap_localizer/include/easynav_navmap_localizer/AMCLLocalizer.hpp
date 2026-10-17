// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Declaration of the AMCLLocalizer method.

#ifndef EASYNAV_NAVMAP_LOCALIZER__AMCLLOCALIZER_HPP_
#define EASYNAV_NAVMAP_LOCALIZER__AMCLLOCALIZER_HPP_

#include <utility>
#include <vector>
#include <random>
#include <limits>
#include <atomic>
#include <memory>
#include <optional>

#include "Eigen/Geometry"
#include "bonxai/probabilistic_map.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
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
  double odom_scale = 1.0;    ///< Scale of the odometry's translation for it (< 1: slipping)
};

/// \brief A localization method implementing a simplified AMCL (Adaptive Monte Carlo Localization)
/// approach.
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
   * @throws std::runtime_error if initialization fails.
   */
  void on_initialize() override;

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

  /// Subscriber for the initial pose.
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

  /**
   * @brief Callback for receiving odometry updates.
   *
   * @param msg The incoming odometry message.
   */
  void odom_callback(nav_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Callback for receiving the initial pose.
   *
   * @param msg The incoming initial pose with covariance.
   */
  void init_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg);

  /// Spreads the particles around @p msg. With @p handover, their poses go to the RT thread
  /// (see pending_); without it they are written here, only valid while neither thread runs.
  void reinitialize_(const geometry_msgs::msg::PoseWithCovarianceStamped & msg, bool handover);

  /// @brief Starts from the last known pose after a reconfiguration
  /// ("initial_pose.use_last_known").
  void on_last_known_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) override;

  /// @brief Whether to start from the last known pose after a reconfiguration.
  bool use_last_known_pose_ {true};

  /**
   * @brief Update odom from TFs instead of a odom topic
   *
   */
  void update_odom_from_tf();

  /// List of particles representing the belief distribution.
  std::vector<Particle> particles_;

  /// Random number generator used for sampling noise (update() and the pose callbacks).
  std::mt19937 rng_;

  /// Random number generator of the RT thread (predict()).
  std::mt19937 rng_rt_;

  // The two threads share the particles without locks: the RT thread (predict()) is the only one
  // that writes their poses, and the other (correct(), reseed(), the pose callbacks) the only one
  // that writes their weights and hits. Poses computed outside the RT thread are handed over in a
  // single slot: pending_poses_ is filled while pending_ is false, then pending_ is set; predict()
  // applies them and clears it. The vector never changes size, so indices are always valid.
  std::vector<std::pair<std::size_t, tf2::Transform>> pending_poses_;
  std::vector<double> pending_scales_;  // their odometry scales (none: 1)
  std::atomic<bool> pending_ {false};

  // Odometry travelled and turned so far (written by the RT thread only), and where the last
  // reseed was (non-RT): reseed() needs the robot to have moved reseed_min_d or reseed_min_a
  std::atomic<double> travelled_ {0.0};
  std::atomic<double> turned_ {0.0};
  double travelled_at_reseed_ {0.0};
  double turned_at_reseed_ {0.0};
  double reseed_min_d_ {0.1};  // m
  double reseed_min_a_ {0.1};  // rad
  // Fraction of the reseeded particles given any odometry scale in [0, 1] (wheels slipping)
  double odom_slip_fraction_ {0.05};
  // Noise of the scale a reseeded particle takes from its reference
  double odom_scale_noise_ {0.02};

  /// Hands poses over to the RT thread. False if the slot is still busy.
  bool post_poses_(
    const std::vector<std::pair<std::size_t, tf2::Transform>> & poses,
    const std::vector<double> & scales = {});

  /// Applies the poses handed over (RT thread).
  void apply_pending_poses_();

  /// Particle indices by decreasing weight, sorted on a copy of the weights: the other thread may
  /// be writing them, and a sort whose keys change underneath runs out of bounds.
  std::vector<std::size_t> by_weight_(std::size_t n_first, bool sorted) const;

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
  std::size_t correct_max_points_ {1500};  // hard cap of points per sensor cloud
  double weights_tau_ {0.7};            // <1 sharpens, >1 flattens
  // Fraction of particles (0,1] used as "top" set for stats and reseed
  double top_keep_fraction_{0.2};
  double downsampled_cloud_size_{0.05};
  // Only points between these heights over the robot's footprint are scored (default: all)
  double min_point_height_ {-std::numeric_limits<double>::infinity()};
  double max_point_height_ {std::numeric_limits<double>::infinity()};
  // A point whose ray from the sensor crosses an occupied voxel scores nothing. Off by default:
  // with voxels a few decimetres wide, rays along a floor or a ceiling (a 3D lidar's, a camera
  // near the floor) look occluded, and the true pose scores worse than wrong ones
  bool ray_occlusion_ {false};

  // Recovery (augmented MCL): slow and fast averages of the scan likelihood; when the fast one
  // drops below the slow one, reseed() spreads part of the particles widely around the estimate.
  // Both written by the non-RT thread only.
  double recovery_alpha_slow_ {0.001};
  double recovery_alpha_fast_ {0.1};
  double recovery_spread_xy_ {1.0};   // m
  double recovery_spread_yaw_ {0.5};  // rad
  double w_slow_ {0.0};
  double w_fast_ {0.0};

  /// Feeds the averages with the likelihood of a correction.
  void update_recovery_(double mean_likelihood);

  /// Mean of the best fraction (top_keep_fraction, the ones reseed() keeps) of the particles'
  /// likelihoods: spread particles must not lower it. Reorders the vector.
  double best_mean_(std::vector<double> & likelihood) const;

  /// Fraction of the reseeded particles to spread widely, 0 when the likelihood holds.
  double recovery_probability_() const;

  /// Last odometry transform received.
  tf2::Transform odom_{tf2::Transform::getIdentity()};

  /// Previous odometry transform (used to compute deltas).
  tf2::Transform last_odom_{tf2::Transform::getIdentity()};
  /// IMU yaw at the last prediction: only its change is used.
  std::optional<double> last_imu_yaw_;

  /// Flag indicating if the odometry has been initialized.
  bool initialized_odom_ = false;

  /// Time interval (in seconds) after which the particles should be reseeded.
  double reseed_time_;

  /// Timestamp of the last reseed event.
  rclcpp::Time last_reseed_;

  /// Timestamp of the last input message (odometry or initial pose).
  rclcpp::Time last_input_time_;

  /**
   * @brief Internal static map.
   */
  std::shared_ptr<Bonxai::ProbabilisticMap> bonxai_map_;
  ::navmap::NavCelId last_cid_ {0};

  // PerceptionModel percepcion_model_;
};

}  // namespace navmap

}  // namespace easynav
#endif  // EASYNAV_NAVMAP_LOCALIZER__AMCLLOCALIZER_HPP_
