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
/// \brief OrbitLocalizer: a few Gaussian hypotheses on SE(3), each corrected by Gauss-Newton
/// against the map's distance field, where the hypotheses are the poses the map's own
/// symmetries make indistinguishable (SymmetryGraph), not samples.

#ifndef EASYNAV_ORBIT_LOCALIZER__ORBITLOCALIZER_HPP_
#define EASYNAV_ORBIT_LOCALIZER__ORBITLOCALIZER_HPP_

#include <atomic>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "GeographicLib/LocalCartesian.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_orbit_localizer/DistanceField.hpp"
#include "easynav_orbit_localizer/Lie.hpp"
#include "easynav_orbit_localizer/Matcher.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "easynav_orbit_localizer/SymmetryGraph.hpp"

namespace easynav
{
namespace orbit
{

/// @brief One Gaussian hypothesis: the map -> odom transform it implies, and the covariance of
/// the body pose (right perturbation, body frame).
struct Hypothesis
{
  Pose X_mo {Pose::Identity()};
  Mat6 P {Mat6::Identity()};
  Pose X_ob_at {Pose::Identity()};   // the odometry at its last correction
  double weight {1.0};
  double score {0.0};
  double credit {0.0};                // updates it fitted as the estimate, spent when it does not
  int id {0};
  double best_since {-1.0};           // s (steady) since it is the best, -1 if it is not
  int bad_cycles {0};
};

class OrbitLocalizer : public LocalizerMethodBase
{
public:
  OrbitLocalizer() = default;
  ~OrbitLocalizer() override;

  void on_initialize() override;
  void update_rt(NavState & nav_state) override;
  void update(NavState & nav_state) override;

  /// @brief The hypotheses (a copy), best first.
  std::vector<Hypothesis> hypotheses() const;
  /// @brief The estimate: map -> body of the best hypothesis.
  Pose estimate() const;
  bool lost() const {return lost_;}
  bool graphReady() const {return graph_ready_;}

protected:
  void on_last_known_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) override;

  // Setup
  void readParameters();
  bool ensureMap(NavState & nav_state);
  void startGraphBuild();
  void resetAt(const Pose & X_mb, const Mat6 & P);

  // Inputs
  void initPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg);
  bool updateOdometry(NavState & nav_state);
  /// @brief The valid clouds in the footprint frame, and the stamp of the latest of them.
  std::vector<Eigen::Vector3d> collectPoints(
    NavState & nav_state, rclcpp::Time & stamp,
    std::vector<Eigen::Vector3d> * all_points = nullptr,
    std::vector<Eigen::Vector3d> * raw_points = nullptr);
  /// @brief The score of a pose: the scan's inlier fraction (forward) and how much of the
  /// map's view from there the scan explains (coverage), their geometric mean.
  /// @brief What of the map's view the sensor could see: heights and field of view.
  CoverageBand coverageBand() const;
  double fitScore(
    double forward, const Pose & X, const std::vector<Eigen::Vector3d> & scan) const;
  /// @brief The odometry at a time: from TF, or the nearest of the recent samples.
  Pose odomAt(const rclcpp::Time & stamp) const;
  void collectAbsolute(
    NavState & nav_state, const Pose & X_pred, const Mat6 & P,
    std::vector<PositionMeasurement> & positions, std::vector<DepthMeasurement> & depths,
    std::vector<AttitudeMeasurement> & attitudes);
  std::optional<Eigen::Matrix3d> imuAttitude(NavState & nav_state);
  std::optional<Pose> mounting(const std::string & frame);
  void sonarElevations(std::vector<Eigen::Vector3d> & points, const Pose & X) const;

  // The filter
  Mat6 processNoise(const Pose & U) const;
  void correctAll(
    NavState & nav_state, const std::vector<Eigen::Vector3d> & points, const Pose & X_ob);
  void placeOnTerrain(NavState & nav_state, Pose & X, const std::optional<Eigen::Matrix3d> & R_imu);
  void updateWeights(double travelled);
  void spawnTwins(const Pose & X_ob);
  void chooseWinner(bool allow_switch = true);
  void relocalize(
    NavState & nav_state, const std::vector<Eigen::Vector3d> & points,
    const std::vector<Eigen::Vector3d> & scan, const Pose & X_ob);
  bool covered(const Pose & X_mb) const;
  /// @brief Under it a score means the scans are not explained: the absolute floor, or a
  /// large drop from the usual score.
  double doubtScore() const;
  bool judged(const std::vector<Eigen::Vector3d> & points) const;

  // Outputs
  void publish(NavState & nav_state);
  /// @brief RViz markers (debug.markers): hypotheses with weight, score and credit, the scan
  /// where the estimate puts it coloured by fit, and a status line over the robot.
  void publishMarkers(const Pose & X_ob);
  /// @brief The symmetry graph once built: its places and the links between symmetric ones.
  void publishGraph();
  nav_msgs::msg::Odometry poseMessage(const Pose & X_mb, const Mat6 & P) const;

  // Parameters
  Mode mode_ {Mode::Planar};
  std::string map_type_ {"costmap"};
  std::string odometry_source_ {"tf"};
  std::string odom_key_ {"odom"};
  std::string dvl_key_ {"dvl"};
  std::string depth_key_;
  std::string imu_key_ {"imu"};
  std::string gps_key_ {"gps"};
  std::string navmap_key_ {"map.navmap"};
  double noise_translation_ {0.1}, noise_rotation_ {0.1}, noise_translation_to_rotation_ {0.05};
  double min_noise_xy_ {0.02}, min_noise_z_ {0.02}, min_noise_yaw_ {0.01},
  min_noise_roll_pitch_ {0.005};
  bool use_last_known_pose_ {true};
  bool global_init_ {false};
  bool use_points_ {true};
  int max_points_ {400};
  int min_evidence_points_ {20};  // fewer points judge nothing: no loss, no usual score
  double min_height_ {-std::numeric_limits<double>::infinity()};
  double max_height_ {std::numeric_limits<double>::infinity()};
  double max_range_ {30.0};
  double downsample_ {0.0};
  bool sonar_mode_ {false};
  double sonar_vertical_fov_ {0.35};
  MatchOptions match_options_;
  int max_hypotheses_ {8};
  double weight_sharpness_ {8.0}, min_weight_ {0.01};
  double score_deadband_ {0.05};       // a score gap below it does not move the weights
  double credit_max_ {100.0};          // updates (0.1 m each) of credit the estimate can hold
  double merge_distance_ {0.5}, merge_angle_ {0.3};
  double min_winner_time_ {1.0};
  double lost_score_ {0.3};
  int lost_cycles_ {5};
  int reloc_candidates_ {30}, reloc_yaw_seeds_ {3};
  double reloc_min_score_ {0.5};
  double reloc_min_travel_ {0.5};      // m moved before the map is searched again
  Pose reloc_X_ob_ {Pose::Identity()};
  bool have_reloc_ {false};
  double slip_sigmas_ {4.0};
  double fov_azimuth_ {2.0 * M_PI};         // rad, the sensor's field of view about the heading
  double fov_elevation_min_ {-M_PI_2};
  double fov_elevation_max_ {M_PI_2};
  double update_min_distance_ {0.1};   // m, motion before the weights see new evidence
  double update_min_angle_ {0.1};      // rad
  Pose evidence_X_ob_ {Pose::Identity()};
  bool have_evidence_pose_ {false};
  bool symmetry_enabled_ {true};
  GraphOptions graph_options_;
  double graph_range_ {12.0}, graph_z_min_ {0.2}, graph_z_max_ {3.0}, graph_ground_height_ {0.0};
  std::vector<double> graph_altitudes_;
  CloudViews3D::Rays graph_rays_;       // the simulated lidar of the 3D views
  std::vector<Eigen::Vector3d> scan_;   // this cycle's cloud, downsampled, for the coverage
  std::vector<Eigen::Vector3d> raw_scan_;   // every return of it, for the views
  std::string graph_cache_;
  std::string map_path_;             // the map's file, to name the cache after it
  uint64_t graph_fingerprint_ {0};   // of the map and the graph options
  double spawn_weight_ {0.1};
  bool gps_enabled_ {false};
  double latitude_origin_ {0.0}, longitude_origin_ {0.0}, altitude_origin_ {0.0};
  std::string projection_ {"utm"};
  double gps_sigma_scale_ {1.0}, gps_gate_ {16.0};
  bool gps_use_z_ {false};
  double gps_map_yaw_ {0.0};          // rad, the map's axes from east-north
  double terrain_max_step_ {1.0};      // m, a surface farther in z is another level
  bool imu_attitude_ {false}, imu_yaw_ {false};
  double imu_sigma_roll_pitch_ {0.02}, imu_sigma_yaw_ {0.05};
  double depth_sigma_ {0.1}, depth_gate_ {16.0};

  // Map
  std::unique_ptr<GridDistanceField2D> field2d_;
  std::unique_ptr<CloudDistanceField3D> field3d_;
  const DistanceField * field_ {nullptr};
  std::unique_ptr<ViewMaker> views_;
  std::unique_ptr<Matcher> matcher_;
  SymmetryGraph graph_;
  std::thread graph_thread_;
  std::atomic<bool> graph_ready_ {false};
  std::atomic<bool> graph_abort_ {false};
  std::atomic<bool> graph_building_ {false};

  // Odometry (written by the RT thread)
  mutable std::mutex odom_mutex_;
  Pose X_ob_ {Pose::Identity()};
  rclcpp::Time odom_stamp_ {0, 0, RCL_ROS_TIME};
  bool odom_ok_ {false};
  geometry_msgs::msg::Twist odom_twist_;
  std::deque<std::pair<rclcpp::Time, Pose>> odom_history_;   // the last seconds, for odomAt()
  rclcpp::Time dead_reckoning_stamp_ {0, 0, RCL_ROS_TIME};

  // Hypotheses (written by the non-RT thread; the RT thread reads the winner)
  mutable std::mutex hyp_mutex_;
  std::vector<Hypothesis> hyps_;
  int next_id_ {1};
  int winner_id_ {-1};
  Pose winner_X_mo_ {Pose::Identity()};
  Mat6 winner_P_ {Mat6::Identity()};
  double winner_score_ {0.0};
  double usual_score_ {-1.0};   // slow average of the winner's score: what "localized" looks like
  double ambiguity_ {0.0};
  std::atomic<bool> lost_ {false};
  int lost_count_ {0};
  std::optional<std::pair<Pose, Mat6>> pending_reset_;
  rclcpp::Time last_correction_ {0, 0, RCL_ROS_TIME};
  bool have_estimate_ {false};

  // Absolute measurements
  std::unique_ptr<GeographicLib::LocalCartesian> enu_;
  double utm_origin_x_ {0.0}, utm_origin_y_ {0.0};
  int utm_zone_ {0};
  std::vector<rclcpp::Time> gps_last_stamp_;
  int gps_rejected_ {0};
  int depth_rejected_ {0};
  rclcpp::Time depth_used_ {0, 0, RCL_ROS_TIME};
  std::unordered_map<std::string, Pose> mountings_;

  // ROS
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr hyps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  bool debug_markers_ {false};
  bool graph_published_ {false};
  double last_cycle_ms_ {0.0};
};

}  // namespace orbit
}  // namespace easynav

#endif  // EASYNAV_ORBIT_LOCALIZER__ORBITLOCALIZER_HPP_
