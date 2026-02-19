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
/// \brief Implementation of the AMCLLocalizer class.

#include <random>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_simple_common/SimpleMap.hpp"

#include "easynav_simple_localizer/AMCLLocalizer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"

namespace easynav
{

tf2::Vector3 computeMean(
  const std::vector<Particle> & particles,
  std::size_t start, std::size_t count)
{
  tf2::Vector3 weighted_sum(0.0, 0.0, 0.0);
  double total_weight = 0.0;

  if (start >= particles.size() || count == 0) {return weighted_sum;}

  std::size_t end = std::min(start + count, particles.size());

  for (std::size_t i = start; i < end; ++i) {
    const Particle & p = particles[i];
    const tf2::Vector3 & origin = p.pose.getOrigin();
    weighted_sum += origin * p.weight;
    total_weight += p.weight;
  }

  if (total_weight > 0.0) {
    return weighted_sum / total_weight;
  } else {
    return tf2::Vector3(0.0, 0.0, 0.0);
  }
}

tf2::Matrix3x3
computeCovariance(
  const std::vector<Particle> & particles,
  std::size_t start, std::size_t count,
  const tf2::Vector3 & mean)
{
  double total_weight = 0.0;
  double cov[3][3] = {{0.0}};

  if (start >= particles.size() || count == 0) {return tf2::Matrix3x3();}

  std::size_t end = std::min(start + count, particles.size());

  for (std::size_t i = start; i < end; ++i) {
    const Particle & p = particles[i];
    tf2::Vector3 delta = p.pose.getOrigin() - mean;

    double w = p.weight;
    total_weight += w;

    cov[0][0] += w * delta.x() * delta.x();
    cov[0][1] += w * delta.x() * delta.y();
    cov[0][2] += w * delta.x() * delta.z();
    cov[1][1] += w * delta.y() * delta.y();
    cov[1][2] += w * delta.y() * delta.z();
    cov[2][2] += w * delta.z() * delta.z();
  }

  if (total_weight > 0.0) {
    cov[0][0] /= total_weight;
    cov[0][1] /= total_weight;
    cov[0][2] /= total_weight;
    cov[1][1] /= total_weight;
    cov[1][2] /= total_weight;
    cov[2][2] /= total_weight;

    // La matriz es simétrica
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];
  }

  return tf2::Matrix3x3(
    cov[0][0], cov[0][1], cov[0][2],
    cov[1][0], cov[1][1], cov[1][2],
    cov[2][0], cov[2][1], cov[2][2]);
}

double
extractYaw(const tf2::Transform & transform)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

double
computeYawVariance(
  const std::vector<Particle> & particles,
  std::size_t start, std::size_t count)
{
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  double total_weight = 0.0;

  if (start >= particles.size() || count == 0) {
    return 0.0;
  }

  std::size_t end = std::min(start + count, particles.size());

  for (std::size_t i = start; i < end; ++i) {
    const Particle & p = particles[i];
    double yaw = extractYaw(p.pose);
    double w = p.weight;

    sum_cos += w * std::cos(yaw);
    sum_sin += w * std::sin(yaw);
    total_weight += w;
  }

  if (total_weight == 0.0) {
    return 0.0;
  }

  double mean_cos = sum_cos / total_weight;
  double mean_sin = sum_sin / total_weight;

  double R = std::sqrt(mean_cos * mean_cos + mean_sin * mean_sin);
  double variance = -2.0 * std::log(R);

  return variance;
}

using std::placeholders::_1;
using namespace std::chrono_literals;


AMCLLocalizer::AMCLLocalizer()
{
  NavState::register_printer<nav_msgs::msg::Odometry>(
    [](const nav_msgs::msg::Odometry & odom) {
      std::ostringstream ret;
      double x = odom.pose.pose.position.x;
      double y = odom.pose.pose.position.y;

      tf2::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      ret << "{" << rclcpp::Time(odom.header.stamp).seconds() << "} Odometry with pose: (x: " <<
        x << ", y: " << y << ", yaw: " << yaw << ")";
      return ret.str();
    });
}

AMCLLocalizer::~AMCLLocalizer()
{
}

void
AMCLLocalizer::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  int num_particles;
  double x_init, y_init, yaw_init, std_dev_xy, std_dev_yaw;

  node->declare_parameter<int>(plugin_name + ".num_particles", 100);
  node->declare_parameter<double>(plugin_name + ".initial_pose.x", 0.0);
  node->declare_parameter<double>(plugin_name + ".initial_pose.y", 0.0);
  node->declare_parameter<double>(plugin_name + ".initial_pose.yaw", 0.0);
  node->declare_parameter<double>(plugin_name + ".initial_pose.std_dev_xy", 0.5);
  node->declare_parameter<double>(plugin_name + ".initial_pose.std_dev_yaw", 0.5);
  node->declare_parameter<double>(plugin_name + ".reseed_freq", 1.0);
  node->declare_parameter<double>(plugin_name + ".noise_translation", 0.01);
  node->declare_parameter<double>(plugin_name + ".noise_rotation", 0.01);
  node->declare_parameter<double>(plugin_name + ".noise_translation_to_rotation", 0.01);
  node->declare_parameter<double>(plugin_name + ".min_noise_xy", 0.05);
  node->declare_parameter<double>(plugin_name + ".min_noise_yaw", 0.05);

  node->get_parameter<int>(plugin_name + ".num_particles", num_particles);
  node->get_parameter<double>(plugin_name + ".initial_pose.x", x_init);
  node->get_parameter<double>(plugin_name + ".initial_pose.y", y_init);
  node->get_parameter<double>(plugin_name + ".initial_pose.yaw", yaw_init);
  node->get_parameter<double>(plugin_name + ".initial_pose.std_dev_xy", std_dev_xy);
  node->get_parameter<double>(plugin_name + ".initial_pose.std_dev_yaw", std_dev_yaw);
  node->get_parameter<double>(plugin_name + ".noise_translation", noise_translation_);
  node->get_parameter<double>(plugin_name + ".noise_rotation", noise_rotation_);
  node->get_parameter<double>(plugin_name + ".noise_translation_to_rotation",
    noise_translation_to_rotation_);
  node->get_parameter<double>(plugin_name + ".min_noise_xy", min_noise_xy_);
  node->get_parameter<double>(plugin_name + ".min_noise_yaw", min_noise_yaw_);

  double reseed_freq;
  node->get_parameter<double>(plugin_name + ".reseed_freq", reseed_freq);
  reseed_time_ = 1.0 / reseed_freq;

  RCLCPP_INFO(node->get_logger(), "Initialized AMCL pose with %d particles", num_particles);
  RCLCPP_INFO(node->get_logger(), "at position (%lf, %lf, %lf) std_dev [%lf, %lf]",
    x_init, y_init, yaw_init, std_dev_xy, std_dev_yaw);

  std::normal_distribution<double> noise_x(x_init, std_dev_xy);
  std::normal_distribution<double> noise_y(y_init, std_dev_xy);
  std::normal_distribution<double> noise_yaw(yaw_init, std_dev_yaw);

  particles_.resize(num_particles);
  for (auto & p : particles_) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, noise_yaw(rng_));
    p.pose.setRotation(q);
    p.pose.setOrigin(tf2::Vector3(noise_x(rng_), noise_y(rng_), 0.0));

    p.hits = 0;
    p.possible_hits = 0;
    p.weight = 1.0 / num_particles;
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  auto node_typed = std::dynamic_pointer_cast<LocalizerNode>(get_node());
  auto rt_cbg = node_typed->get_real_time_cbg();
  rclcpp::SubscriptionOptions options;
  options.callback_group = rt_cbg;

  odom_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS().reliable(),
    std::bind(&AMCLLocalizer::odom_callback, this, _1), options);

  particles_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseArray>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/particles", 10);
  estimate_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/pose", 10);

  last_reseed_ = get_node()->now();

  get_node()->get_logger().set_level(rclcpp::Logger::Level::Debug);
}

void printTransform(const tf2::Transform & tf)
{
  const tf2::Vector3 & origin = tf.getOrigin();
  const tf2::Quaternion & rot = tf.getRotation();

  std::cerr << "Translation: ["
            << origin.x() << ", "
            << origin.y() << ", "
            << origin.z() << "]\t";

  std::cerr << "Rotation (quaternion): ["
            << rot.x() << ", "
            << rot.y() << ", "
            << rot.z() << ", "
            << rot.w() << "]\n";
}

void
AMCLLocalizer::update_rt(NavState & nav_state)
{
  predict(nav_state);

  nav_state.set("robot_pose", get_pose());
}

void
AMCLLocalizer::update(NavState & nav_state)
{
  correct(nav_state);

  if ((get_node()->now() - last_reseed_).seconds() > reseed_time_) {
    reseed();
    last_reseed_ = get_node()->now();
  }

  nav_state.set("robot_pose", get_pose());

  publishParticles();
}

void
AMCLLocalizer::odom_callback(nav_msgs::msg::Odometry::UniquePtr msg)
{
  tf2::fromMsg(msg->pose.pose, odom_);
  last_input_time_ = msg->header.stamp;

  if (!initialized_odom_) {
    last_odom_ = odom_;
    initialized_odom_ = true;
  }
}

void
AMCLLocalizer::predict([[maybe_unused]] NavState & nav_state)
{
  if (!initialized_odom_) {
    return;
  }

  tf2::Transform delta = last_odom_.inverseTimes(odom_);

  tf2::Vector3 t = delta.getOrigin();
  double dx = t.x(), dy = t.y(), dz = t.z();
  double trans_len = std::sqrt(dx * dx + dy * dy + dz * dz);

  double roll, pitch, yaw;
  tf2::Matrix3x3(delta.getRotation()).getRPY(roll, pitch, yaw);
  double rot_len = std::abs(yaw);

  std::random_device rd;
  std::mt19937 gen(rd());

  for (auto & p : particles_) {
    std::normal_distribution<double> noise_dx(0.0, std::abs(dx) * noise_translation_);
    std::normal_distribution<double> noise_dy(0.0, std::abs(dy) * noise_translation_);
    std::normal_distribution<double> noise_dz(0.0, std::abs(dz) * noise_translation_);

    std::normal_distribution<double> noise_yaw(
      0.0,
      rot_len * noise_rotation_ + trans_len * noise_translation_to_rotation_);

    tf2::Vector3 noisy_translation(
      dx + noise_dx(gen),
      dy + noise_dy(gen),
      dz + noise_dz(gen));

    double noisy_yaw = yaw + noise_yaw(gen);
    tf2::Quaternion noisy_q;
    noisy_q.setRPY(0.0, 0.0, noisy_yaw);

    tf2::Transform noisy_delta(noisy_q, noisy_translation);

    p.pose = p.pose * noisy_delta;
  }

  last_odom_ = odom_;

  tf2::Transform map2bf = getEstimatedPose();
  tf2::Transform map2odom = map2bf * odom_.inverse();

  publishTF(map2odom);
  publishEstimatedPose(map2bf);
}

void AMCLLocalizer::correct(NavState & nav_state)
{
  if (!nav_state.has("points")) {
    RCLCPP_WARN(get_node()->get_logger(), "There is yet no points perceptions");
    return;
  }

  const auto & perceptions = nav_state.get<PointPerceptions>("points");

  if (!nav_state.has("map.static")) {
    RCLCPP_WARN(get_node()->get_logger(), "There is yet no a map.static map");
    return;
  }

  const auto & map_static = nav_state.get<SimpleMap>("map.static");

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  const auto & filtered = PointPerceptionsOpsView(perceptions)
    .downsample(map_static.resolution())
    .fuse(tf_info.robot_frame)
    .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .collapse({NAN, NAN, 0.1})
    .downsample(map_static.resolution())
    .as_points();

  if (filtered.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No points to correct");
    return;
  }

  for (auto & particle : particles_) {
    int hits = 0;
    int possible_hits = 0;

    for (const auto & pt : filtered) {
      tf2::Vector3 pt_world = particle.pose * tf2::Vector3(pt.x, pt.y, pt.z);

      if (!map_static.check_bounds_metric(pt_world.x(), pt_world.y())) {continue;}

      try {
        auto [cell_x, cell_y] = map_static.metric_to_cell(pt_world.x(), pt_world.y());
        ++possible_hits;
        if (map_static.at(cell_x, cell_y)) {
          ++hits;
        }
      } catch (const std::out_of_range & e) {
        continue;
      }
    }

    particle.hits += hits;
    particle.possible_hits += possible_hits;
  }

  for (auto & particle : particles_) {
    if (particle.possible_hits > 0) {
      particle.weight = static_cast<double>(particle.hits) /
        static_cast<double>(particle.possible_hits);
    } else {
      particle.weight = 0;
    }
  }

  double total_weight = 0.0;
  for (const auto & p : particles_) {
    total_weight += p.weight;
  }

  if (total_weight > 0.0) {
    for (auto & p : particles_) {
      p.weight /= total_weight;
    }
  }
}

void
AMCLLocalizer::reseed()
{
  if (particles_.empty()) {return;}
  if (particles_[0].possible_hits == 0) {return;}

  const std::size_t N = particles_.size();
  const std::size_t N_top = N / 2;

  std::sort(particles_.begin(), particles_.end(),
    [](const Particle & a, const Particle & b) {
      return a.weight > b.weight;
    });

  tf2::Vector3 mean = computeMean(particles_, 0, N_top);
  tf2::Matrix3x3 cov = computeCovariance(particles_, 0, N_top, mean);

  double a = cov[0][0];
  double b = cov[0][1];
  double c = cov[1][1];

  double l00 = std::sqrt(a);
  double l10 = b / l00;
  double l11 = std::sqrt(c - l10 * l10);

  double yaw_variance = computeYawVariance(particles_, 0, N_top);
  double yaw_stddev = std::sqrt(yaw_variance);
  std::normal_distribution<double> yaw_noise(0.0, std::max(yaw_stddev, min_noise_yaw_));
  std::normal_distribution<double> index_dist(0.0, 0.05 * static_cast<double>(N));
  std::normal_distribution<double> standard_normal(0.0, 1.0);

  for (std::size_t i = N_top; i < N; ++i) {
    int selected_idx;
    do {
      selected_idx = static_cast<int>(std::round(index_dist(rng_)));
    } while (selected_idx < 0 || static_cast<std::size_t>(selected_idx) >= N_top);

    const auto & ref_particle = particles_[selected_idx];

    tf2::Vector3 origin = ref_particle.pose.getOrigin();

    double z0 = standard_normal(rng_);
    double z1 = standard_normal(rng_);
    double dx = l00 * z0;
    double dy = l10 * z0 + l11 * z1;

    std::normal_distribution<double> xy_noise(0.0, std::max(sqrt(dx * dx + dy * dy),
      min_noise_xy_));

    tf2::Vector3 new_origin(origin.x() + xy_noise(rng_), origin.y() + xy_noise(rng_), 0.0);

    double roll, pitch, yaw;
    tf2::Matrix3x3(ref_particle.pose.getRotation()).getRPY(roll, pitch, yaw);
    double noisy_yaw = yaw + yaw_noise(rng_);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, noisy_yaw);

    particles_[i].pose.setOrigin(new_origin);
    particles_[i].pose.setRotation(q);
    particles_[i].weight = ref_particle.weight;
  }

  for (auto & particle : particles_) {
    particle.hits = 0;
    particle.possible_hits = 0;
  }

  double total_weight = 0.0;
  for (const auto & p : particles_) {
    total_weight += p.weight;
  }
  for (auto & p : particles_) {
    p.weight /= total_weight;
  }
}

void
AMCLLocalizer::publishTF(const tf2::Transform & map2bf)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = last_input_time_;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  tf_msg.header.frame_id = tf_info.map_frame;
  tf_msg.child_frame_id = tf_info.odom_frame;
  tf_msg.transform = tf2::toMsg(map2bf);

  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  tf_broadcaster_->sendTransform(tf_msg);
}

void
AMCLLocalizer::publishParticles()
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::PoseArray array_msg;
  array_msg.header.stamp = last_input_time_;
  array_msg.header.frame_id = tf_info.map_frame;

  array_msg.poses.reserve(particles_.size());
  for (const auto & p : particles_) {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = p.pose.getOrigin().x();
    pose_msg.position.y = p.pose.getOrigin().y();
    pose_msg.position.z = p.pose.getOrigin().z();
    pose_msg.orientation.x = p.pose.getRotation().x();
    pose_msg.orientation.y = p.pose.getRotation().y();
    pose_msg.orientation.z = p.pose.getRotation().z();
    pose_msg.orientation.w = p.pose.getRotation().w();
    array_msg.poses.push_back(pose_msg);
  }

  particles_pub_->publish(array_msg);
}

tf2::Transform
AMCLLocalizer::getEstimatedPose() const
{
  if (particles_.empty()) {
    return tf2::Transform::getIdentity();
  }

  const std::size_t N = particles_.size();
  const std::size_t N_top = N / 2;

  std::vector<Particle> sorted_particles = particles_;
  std::sort(sorted_particles.begin(), sorted_particles.end(),
    [](const Particle & a, const Particle & b) {
      return a.weight > b.weight;
    });

  tf2::Vector3 mean = computeMean(sorted_particles, 0, N_top);

  double roll, pitch, yaw;
  tf2::Matrix3x3(sorted_particles[0].pose.getRotation()).getRPY(roll, pitch, yaw);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  tf2::Transform est;
  est.setOrigin(mean);
  est.setRotation(q);
  return est;
}

void
AMCLLocalizer::publishEstimatedPose(const tf2::Transform & est_pose)
{
  if (particles_.empty()) {return;}

  const std::size_t N = particles_.size();
  const std::size_t N_top = N / 2;

  tf2::Vector3 mean = est_pose.getOrigin();
  tf2::Matrix3x3 cov = computeCovariance(particles_, 0, N_top, mean);
  double yaw_variance = computeYawVariance(particles_, 0, N_top);

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = last_input_time_;
  msg.header.frame_id = tf_info.map_frame;

  msg.pose.pose.position.x = mean.x();
  msg.pose.pose.position.y = mean.y();
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = tf2::toMsg(est_pose.getRotation());

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      msg.pose.covariance[6 * r + c] = cov[r][c];
    }
  }
  msg.pose.covariance[6 * 5 + 5] = yaw_variance;

  estimate_pub_->publish(msg);
}

nav_msgs::msg::Odometry
AMCLLocalizer::get_pose()
{
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = last_input_time_;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  odom_msg.header.frame_id = tf_info.map_frame;
  odom_msg.child_frame_id = tf_info.robot_frame;

  tf2::Transform est_pose = getEstimatedPose();

  odom_msg.pose.pose.position.x = est_pose.getOrigin().x();
  odom_msg.pose.pose.position.y = est_pose.getOrigin().y();
  odom_msg.pose.pose.position.z = est_pose.getOrigin().z();
  odom_msg.pose.pose.orientation = tf2::toMsg(est_pose.getRotation());

  if (!particles_.empty()) {
    const std::size_t N = particles_.size();
    const std::size_t N_top = N / 2;

    std::vector<Particle> sorted_particles = particles_;
    std::sort(sorted_particles.begin(), sorted_particles.end(),
      [](const Particle & a, const Particle & b) {
        return a.weight > b.weight;
      });

    tf2::Vector3 mean = computeMean(sorted_particles, 0, N_top);
    tf2::Matrix3x3 cov = computeCovariance(sorted_particles, 0, N_top, mean);
    double yaw_var = computeYawVariance(sorted_particles, 0, N_top);

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        odom_msg.pose.covariance[6 * r + c] = cov[r][c];
      }
    }

    odom_msg.pose.covariance[6 * 5 + 5] = yaw_var;
  }

  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  return odom_msg;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::AMCLLocalizer, easynav::LocalizerMethodBase)
