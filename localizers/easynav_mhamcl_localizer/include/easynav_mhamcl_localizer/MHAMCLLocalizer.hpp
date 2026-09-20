// Copyright 2026 Intelligent Robotics Lab
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
/// \brief Declaration of the MHAMCLLocalizer method.

#ifndef EASYNAV_MHAMCL_LOCALIZER__MHAMCLLOCALIZER_HPP_
#define EASYNAV_MHAMCL_LOCALIZER__MHAMCLLOCALIZER_HPP_

#include <future>
#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"

#include "easynav_mhamcl_localizer/MapMatcher.hpp"
#include "easynav_mhamcl_localizer/ParticlesDistribution.hpp"

namespace easynav
{
namespace mhamcl
{

/**
 * @brief Multi-Hypothesis AMCL localizer.
 *
 * Keeps several particle filters (hypotheses) about the pose of the robot at the same time.
 * Periodically, a cascade map matching looks in the whole map for poses that explain the last
 * perception and starts a new hypothesis on them. Hypotheses that stop explaining what the
 * robot sees are removed, and those that converge to the same pose are merged. The output pose
 * is the one of the hypothesis with the best quality. This allows to localize the robot without
 * knowing where it is, and to recover from wrong estimates and kidnapping.
 *
 * The odometry is read from NavState (see \c update_odom). Prediction runs in \c update_rt, and correction, reseed and hypotheses management in \c update.
 */
class MHAMCLLocalizer : public LocalizerMethodBase
{
public:
  /// \brief Default constructor.
  MHAMCLLocalizer();

  /// \brief Destructor.
  ~MHAMCLLocalizer();

  /**
   * @brief Declares the parameters, creates the initial hypothesis and the interfaces.
   *
   * @throws std::runtime_error if a parameter has an invalid value.
   */
  void on_initialize() override;

  /// \brief Prediction of all the hypotheses, and publication of the pose and map -> odom TF.
  void update_rt(NavState & nav_state) override;

  /// \brief Correction, reseed and hypotheses management.
  void update(NavState & nav_state) override;

  /// \brief Pose (map -> base footprint) of the selected hypothesis.
  tf2::Transform getEstimatedPose() const;

  /// \brief Pose of the selected hypothesis as an Odometry message.
  nav_msgs::msg::Odometry get_pose();

  /// \brief Number of hypotheses currently tracked.
  std::size_t get_num_hypotheses() const;

protected:
  using Hypothesis = std::shared_ptr<ParticlesDistribution>;

  /// \brief Parameters of the hypotheses management.
  struct HypothesesParams
  {
    bool multihypothesis {true};
    int max_hypotheses {5};
    double min_candidate_weight {0.5};
    double min_candidate_distance {1.0};
    double min_candidate_angle {M_PI_2};
    double low_q_hypo_threshold {0.25};
    double very_low_q_hypo_threshold {0.1};
    double hypo_merge_distance {0.3};
    double hypo_merge_angle {0.5};
    double good_hypo_threshold {0.6};
    double min_hypo_diff_winner {0.2};
    double new_hypothesis_std_xy {0.1};
    double new_hypothesis_std_yaw {0.1};
    int matcher_levels {4};
    double matcher_angle_step {M_PI / 8.0};
  };

  /// \brief Apply the odometry increment to every hypothesis.
  void predict(NavState & nav_state);

  /// \brief Score every hypothesis with the last perception.
  void correct(NavState & nav_state);

  /// \brief Reseed every hypothesis.
  void reseed();

  /**
   * @brief Create, remove, merge and select hypotheses.
   *
   * @param candidates Candidate poses coming from the map matching, best first.
   * @param map Base map.
   */
  void manage_hypotheses(
    const std::vector<TransformWeighted> & candidates, const easynav::Costmap2D & map);

  /// \brief Launch (or harvest) the asynchronous map matching.
  void run_matching(const easynav::Costmap2D & map);

  /// \brief Callback for the initial pose. Discards all the hypotheses and starts a new one.
  void init_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg);

  /**
   * @brief Update the odometry, from the odometry perception if there is one, or from TF.
   *
   * The odometry perception is a \c nav_msgs::msg::Odometry stored in NavState under
   * \c odom_key_ by the \c OdometryPerceptionHandler of easynav_sensors. If it has not been
   * received (there is no such sensor, or it has not published yet), odom -> base footprint is
   * read from the RTTFBuffer.
   *
   * @return true if an odometry is available.
   */
  bool update_odom(NavState & nav_state);

  /// \brief Read odom -> base footprint from the RTTFBuffer.
  bool update_odom_from_tf();

  /// \brief Pose of the selected hypothesis. Requires \c mutex_ to be held.
  tf2::Transform estimated_pose_locked() const;

  /// \brief Publish map -> odom. Requires \c mutex_ to be held.
  void publishTF(const tf2::Transform & map2odom);

  /// \brief Publish the pose with covariance. Requires \c mutex_ to be held.
  void publishEstimatedPose();

  /// \brief Pose of the selected hypothesis as an Odometry message. Requires \c mutex_.
  nav_msgs::msg::Odometry pose_msg_locked() const;

  /// \brief Publish the particles and, if someone listens, all the hypotheses. Requires \c mutex_.
  void publishParticles();

  /// \brief Distance and angular difference between two poses.
  static void get_distances(
    const tf2::Transform & a, const tf2::Transform & b, double & dist_xy, double & dist_yaw);

  /// \brief Whether the pose is in a free cell of the map.
  static bool is_free(const easynav::Costmap2D & map, const tf2::Transform & pose);

  /// Publishes map -> odom to /tf. The RTTFBuffer alone is only visible inside EasyNav.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hypotheses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

  /// Protects the hypotheses, shared by the real-time and the non real-time updates.
  mutable std::mutex mutex_;

  std::vector<Hypothesis> hypotheses_;
  Hypothesis current_;

  ParticlesParams particles_params_;
  HypothesesParams hypotheses_params_;

  std::mt19937 rng_;
  unsigned int seed_ {0};

  double min_noise_xy_ {0.05};
  double min_noise_yaw_ {0.05};
  std::size_t correct_max_points_ {500};
  /// Key of NavState where the odometry perception is stored.
  std::string odom_key_ {"odom"};

  tf2::Transform odom_{tf2::Transform::getIdentity()};
  tf2::Transform last_odom_{tf2::Transform::getIdentity()};
  bool initialized_odom_ {false};

  double reseed_time_ {3.0};
  double hypotheses_time_ {3.0};
  rclcpp::Time last_reseed_;
  rclcpp::Time last_hypotheses_;
  rclcpp::Time last_input_time_;

  /// Pyramid of the base map, rebuilt when the map changes.
  std::shared_ptr<const MapMatcher> matcher_;
  int64_t matcher_map_stamp_ns_ {-1};

  /// Points of the last perception in the robot frame, used by the map matching.
  std::vector<tf2::Vector3> last_points_;

  /// Map matching running in background, since it can take long in big maps.
  std::future<std::vector<TransformWeighted>> matching_;

  /// Odometry when the map matching was launched, to compensate the motion until its result.
  tf2::Transform matching_odom_{tf2::Transform::getIdentity()};
};

}  // namespace mhamcl
}  // namespace easynav

#endif  // EASYNAV_MHAMCL_LOCALIZER__MHAMCLLOCALIZER_HPP_
