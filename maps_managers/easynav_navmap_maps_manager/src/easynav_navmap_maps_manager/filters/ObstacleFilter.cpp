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


#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "navmap_core/Combine.hpp"
#include "navmap_core/Iterators.hpp"
#include "navmap_core/NavMap.hpp"
#include "navmap_core/Occupancy.hpp"
#include "navmap_ros/conversions.hpp"

#include "easynav_navmap_maps_manager/filters/ObstacleFilter.hpp"


namespace easynav
{
namespace navmap
{

ObstacleFilter::ObstacleFilter()
{
}

void
ObstacleFilter::on_initialize()
{
  auto node = get_node();
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".max_range", max_range_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".min_height", min_height_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".max_height", max_height_);
  easynav::declare_parameter_if_absent(
    *node, plugin_name_ + ".downsample_resolution", downsample_resolution_);
  node->get_parameter(plugin_name_ + ".max_range", max_range_);
  node->get_parameter(plugin_name_ + ".min_height", min_height_);
  node->get_parameter(plugin_name_ + ".max_height", max_height_);
  node->get_parameter(plugin_name_ + ".downsample_resolution", downsample_resolution_);

  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".memory.enabled", memory_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".memory.clearing", clearing_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".memory.decay_time", decay_time_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".memory.p_hit", p_hit_);
  easynav::declare_parameter_if_absent(*node, plugin_name_ + ".memory.p_miss", p_miss_);
  node->get_parameter(plugin_name_ + ".memory.enabled", memory_);
  node->get_parameter(plugin_name_ + ".memory.clearing", clearing_);
  node->get_parameter(plugin_name_ + ".memory.decay_time", decay_time_);
  node->get_parameter(plugin_name_ + ".memory.p_hit", p_hit_);
  node->get_parameter(plugin_name_ + ".memory.p_miss", p_miss_);
  if (!(p_hit_ > 0.5 && p_hit_ < 1.0) || !(p_miss_ > 0.0 && p_miss_ < 0.5)) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] memory.p_hit must be in (0.5, 1) and memory.p_miss in (0, 0.5): "
      "using 0.7 and 0.4", plugin_name_.c_str());
    p_hit_ = 0.7;
    p_miss_ = 0.4;
  }
  if (!(decay_time_ >= 0.0)) {decay_time_ = 0.0;}
  if (memory_) {
    RCLCPP_INFO(
      node->get_logger(), "[%s] obstacle memory: clearing %s, decay %s", plugin_name_.c_str(),
      clearing_ ? "on" : "off",
      decay_time_ > 0.0 ? (std::to_string(decay_time_) + " s").c_str() : "off");
  }

  easynav::declare_parameter_if_absent(
    *node, plugin_name_ + ".min_height_per_meter", min_height_per_meter_);
  node->get_parameter(plugin_name_ + ".min_height_per_meter", min_height_per_meter_);
  if (!std::isfinite(min_height_per_meter_) || min_height_per_meter_ < 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] min_height_per_meter = %f must be >= 0: using 0",
      plugin_name_.c_str(), min_height_per_meter_);
    min_height_per_meter_ = 0.0;
  }
}

void ObstacleFilter::update(::easynav::NavState & nav_state)
{
  if (!nav_state.has("map.navmap")) {return;}

  const auto & perceptions = nav_state.get_no_group<PointPerception>();
  if (perceptions.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "There are no points perceptions");
    return;
  }

  // In place: the map in NavState is updated, not copied
  const auto map = nav_state.get_ptr<::navmap::NavMap>("map.navmap");
  auto & nm = *map;
  const std::size_t N = nm.navcels.size();
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  // The NavMap's own frame (a local NavMap is in the robot frame), the map frame by default
  const std::string map_frame = !nm.surfaces.empty() && !nm.surfaces[0].frame_id.empty() ?
    nm.surfaces[0].frame_id : tf_info.map_frame;

  // The layer starts from the static map, if the NavMap has one (built from an occupancy grid):
  // what the sensors do not see right now (behind something, far, too low) is still an obstacle.
  // It is rebuilt only for a new map (or static layer); then, each cycle, only the cells seen as
  // obstacles now or in the last cycle are written.
  const auto occ = nm.layers.get("occupancy");
  const std::uint64_t occ_hash = occ && occ->size() == N ? occ->content_hash() : 0;
  auto layer = nm.layers.get(get_layer_name());
  if (!layer || layer != last_layer_.lock() || layer->size() != N || occ_hash != last_occ_hash_) {
    nm.layers.remove(get_layer_name());
    base_.assign(N, navmap_ros::FREE_SPACE);
    if (occ_hash != 0) {
      for (std::size_t c = 0; c < N; ++c) {
        const auto cid = static_cast<::navmap::NavCelId>(c);
        base_[c] = nm.layer_get<uint8_t>("occupancy", cid, navmap_ros::FREE_SPACE);
      }
    }
    nm.add_layer<uint8_t>(
      get_layer_name(), "obstacles: static map plus what the sensors see", "",
      navmap_ros::FREE_SPACE)->set_data(base_);
    nm.set_layer_unknown(get_layer_name(), navmap_ros::NO_INFORMATION);
    // The memory belongs to the map it was built on
    nm.layers.remove("obstacles_logodds");
    nm.layers.remove("obstacles_sensor");
    active_.clear();
    in_active_.assign(N, 0);
    stamp_.assign(N, 0);
    flag_.assign(N, 0);
    last_time_ = -1.0;
    layer = nm.layers.get(get_layer_name());
    last_layer_ = layer;
    last_occ_hash_ = occ_hash;
    marked_.clear();
    seen_.assign(N, 0);
  }
  auto & values = std::dynamic_pointer_cast<::navmap::LayerView<uint8_t>>(layer)->data();
  std::vector<::navmap::NavCelId> marked;

  // Only the points around the robot and below its height (robot frame), downsampled first
  // (indices only) so that fewer points are transformed.
  const auto & points = PointPerceptionsOpsView(perceptions)
    .downsample(downsample_resolution_)
    .fuse(tf_info.robot_frame)
    .filter({-max_range_, -max_range_, NAN}, {max_range_, max_range_, max_height_}, false)
    .fuse(map_frame)
    .as_points();

  // The robot's position, for min_height_per_meter and for the clearing rays
  Eigen::Vector3f robot(0.0f, 0.0f, 0.0f);
  bool robot_known = false;
  if (min_height_per_meter_ > 0.0 || (memory_ && clearing_)) {
    try {
      const auto tf = RTTFBuffer::getInstance()->lookupTransform(
        map_frame, tf_info.robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
      robot = {static_cast<float>(tf.transform.translation.x),
        static_cast<float>(tf.transform.translation.y),
        static_cast<float>(tf.transform.translation.z)};
      robot_known = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "[%s] No robot pose (%s): min_height_per_meter and clearing not applied",
        plugin_name_.c_str(), ex.what());
    }
  }
  const Eigen::Vector2f robot_xy = robot.head<2>();
  std::vector<std::pair<::navmap::NavCelId, Eigen::Vector3f>> ends;  // memory: every point

  std::optional<size_t> last_surface;
  std::optional<::navmap::NavCelId> last_cid;

  // Each point is checked where it is: the NavCel under it, and its height above that NavCel.
  for (const auto & p : points.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const Eigen::Vector3f query(p.x, p.y, p.z);

    size_t surface_idx = 0;
    ::navmap::NavCelId cid;
    Eigen::Vector3f bary, hit;

    ::navmap::NavMap::LocateOpts opts;
    opts.use_downward_ray = true;
    opts.height_eps = 0.50f;
    if (last_surface) {opts.hint_surface = *last_surface;}
    if (last_cid) {opts.hint_cid = *last_cid;}

    bool ok = nm.locate_navcel(query, surface_idx, cid, bary, &hit, opts);

    if (!ok) {
      ::navmap::NavMap::LocateOpts nohint;
      nohint.use_downward_ray = true;
      nohint.height_eps = 0.50f;
      ok = nm.locate_navcel(query, surface_idx, cid, bary, &hit, nohint);
      if (!ok) {
        ok = nm.locate_navcel(query, surface_idx, cid, bary, &hit);
      }
    }
    if (!ok) {continue;}

    // Height above the NavCel's plane (along its upward normal): on a ramp, the ramp is not one
    auto height_over = [&nm, &query](::navmap::NavCelId c) {
        const auto & cel = nm.navcels[c];
        Eigen::Vector3f normal = cel.normal;
        if (normal.z() < 0.0f) {normal = -normal;}
        return normal.dot(query - nm.positions.at(cel.v[0]));
      };
    float height = height_over(cid);
    // The hint takes any NavCel within height_eps: off it, the closest surface in z is the one
    // under the point (a ramp, not the floor 0.4 m below it)
    if (std::fabs(height) > min_height_) {
      ::navmap::NavCelId nearest;
      if (nm.locate_navcel(query, surface_idx, nearest, bary, &hit) && nearest != cid) {
        const float h = height_over(nearest);
        if (std::fabs(h) < std::fabs(height)) {
          cid = nearest;
          height = h;
        }
      }
    }
    last_surface = surface_idx;
    last_cid = cid;
    // Farther, a small attitude error lifts the ground more: a higher threshold
    const double distance = robot_known ? (query.head<2>() - robot_xy).norm() : 0.0;
    if (memory_) {ends.emplace_back(cid, query);}
    if (!(height > min_height_ + min_height_per_meter_ * distance)) {continue;}

    if (seen_[cid] != 2) {
      seen_[cid] = 2;
      marked.push_back(cid);
    }
  }

  if (memory_) {
    for (const auto cid : marked) {
      seen_[cid] = 0;
    }
    update_memory(nm, robot, robot_known, marked, ends);
    return;
  }

  // Obstacles now; cells that were obstacles in the last cycle and are not now go back to the
  // static map. Only cells whose value changes are written (and recorded as changed).
  for (const auto cid : marked) {
    if (values[cid] != navmap_ros::LETHAL_OBSTACLE) {
      nm.layer_set<uint8_t>(get_layer_name(), cid, navmap_ros::LETHAL_OBSTACLE);
    }
  }
  for (const auto cid : marked_) {
    if (seen_[cid] == 2) {continue;}
    seen_[cid] = 0;
    if (values[cid] != base_[cid]) {nm.layer_set<uint8_t>(get_layer_name(), cid, base_[cid]);}
  }
  for (const auto cid : marked) {
    seen_[cid] = 1;
  }
  marked_.swap(marked);
}

void ObstacleFilter::update_memory(
  ::navmap::NavMap & nm, const Eigen::Vector3f & robot, bool robot_known,
  const std::vector<::navmap::NavCelId> & hits,
  const std::vector<std::pair<::navmap::NavCelId, Eigen::Vector3f>> & ends)
{
  using ::navmap::NavCelId;
  using LayerU8 = ::navmap::LayerView<uint8_t>;
  const std::size_t N = nm.navcels.size();
  ::navmap::OccupancyParams op;
  op.p_hit = static_cast<float>(p_hit_);
  op.p_miss = static_cast<float>(p_miss_);
  ::navmap::OccupancyLayer occ(nm, "obstacles_logodds", op);  // created (unknown) if absent
  if (!nm.has_layer("obstacles_sensor")) {
    nm.add_layer<uint8_t>(
      "obstacles_sensor", "what the sensors remember: obstacle, free or unknown", "",
      navmap_ros::NO_INFORMATION);
    nm.set_layer_unknown("obstacles_sensor", navmap_ros::NO_INFORMATION);
  }
  auto sensor = std::dynamic_pointer_cast<LayerU8>(nm.layers.get("obstacles_sensor"));
  auto & logodds = occ.view()->data();
  if (++cycle_ == 0) {
    std::fill(stamp_.begin(), stamp_.end(), 0);
    cycle_ = 1;
  }
  std::vector<NavCelId> touched;
  auto note = [&](NavCelId c) {
      if (stamp_[c] != cycle_) {
        stamp_[c] = cycle_;
        touched.push_back(c);
      }
    };

  // Evidence of this cycle, once per cell: occupied where a point rises above the surface, free
  // where a ray from the robot crosses the surface or ends on it. A hit beats a miss.
  for (const auto c : hits) {
    flag_[c] = 1;
    note(c);
  }
  if (clearing_ && robot_known && !ends.empty()) {
    std::size_t s;
    NavCelId rc;
    Eigen::Vector3f bary, hit;
    ::navmap::NavMap::LocateOpts opts;
    opts.use_downward_ray = true;
    opts.height_eps = 0.5f;
    if (nm.locate_navcel(robot + Eigen::Vector3f(0, 0, 0.1f), s, rc, bary, &hit, opts)) {
      for (const auto & [end, p] : ends) {
        ::navmap::for_each_on_segment(
          nm, rc, robot, p, [&](NavCelId x, float, float) {
            if (flag_[x] == 1) {return x != end;}  // stop at the obstacle it hits
            note(x);
            return true;
          });
      }
    }
  }
  for (const auto c : touched) {
    if (flag_[c] == 1) {occ.hit(c);} else {occ.miss(c);}
    flag_[c] = 0;
  }

  // Cells not observed fade back to unknown
  const double now = get_node()->now().seconds();
  if (decay_time_ > 0.0 && last_time_ >= 0.0 && now > last_time_) {
    const double full = ::navmap::OccupancyLayer::to_fixed(
      ::navmap::OccupancyLayer::logodds_of(op.p_max));
    decay_acc_ += full * (now - last_time_) / decay_time_;
    const int step = static_cast<int>(decay_acc_);
    decay_acc_ -= step;
    if (step > 0) {
      for (const auto c : active_) {
        if (stamp_[c] == cycle_) {continue;}
        const int v = logodds[c];
        const int nv = v > 0 ? std::max(0, v - step) : std::min(0, v + step);
        if (nv != v) {
          occ.add(c, static_cast<int16_t>(nv - v));
          note(c);
        }
      }
    }
  }
  last_time_ = now;
  std::size_t k = 0;
  for (const auto c : active_) {
    if (logodds[c] != 0) {active_[k++] = c;} else {in_active_[c] = 0;}
  }
  active_.resize(k);
  for (const auto c : touched) {
    if (logodds[c] != 0 && !in_active_[c]) {
      in_active_[c] = 1;
      active_.push_back(c);
    }
  }

  // What the sensors say, and the layer recomposed where it changed: the static map, and on top
  // the sensors (max; they fill in unknown static cells, and never erase a static obstacle)
  std::vector<NavCelId> changed;
  for (const auto c : touched) {
    const auto st = occ.state(c);
    const uint8_t v = st == ::navmap::OccupancyState::OCCUPIED ? navmap_ros::LETHAL_OBSTACLE :
      st == ::navmap::OccupancyState::FREE ? navmap_ros::FREE_SPACE : navmap_ros::NO_INFORMATION;
    if ((*sensor)[c] != v) {
      (*sensor)[c] = v;
      sensor->mark_changed(c);
      changed.push_back(c);
    }
  }
  if (changed.empty()) {return;}
  auto layer = std::dynamic_pointer_cast<LayerU8>(nm.layers.get(get_layer_name()));
  std::vector<uint8_t> before(changed.size());
  for (std::size_t i = 0; i < changed.size(); ++i) {
    before[i] = (*layer)[changed[i]];
    (*layer)[changed[i]] = base_[changed[i]];
  }
  ::navmap::CombineOptions co;
  co.cells = changed;
  ::navmap::combine_layer(nm, get_layer_name(), "obstacles_sensor", ::navmap::CombineOp::MAX, co);
  for (std::size_t i = 0; i < changed.size(); ++i) {
    if ((*layer)[changed[i]] != before[i]) {layer->mark_changed(changed[i]);}
  }
  (void)N;
}

}  // namespace navmap
}  // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::ObstacleFilter, easynav::navmap::NavMapFilter)
