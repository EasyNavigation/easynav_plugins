// Copyright 2026 José Miguel Guerrero
//
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

#include "easynav_bigmap_maps_manager/pipeline_check.hpp"

#include <algorithm>
#include <stdexcept>

namespace bigmap_maps_manager
{

ObstacleDetector parse_detector(const std::string & value)
{
  if (value == "bigmap_dynamic" || value == "dynamic") {return ObstacleDetector::BigmapDynamic;}
  if (value == "navmap_filter" || value == "filter") {return ObstacleDetector::NavmapFilter;}
  throw std::invalid_argument(
          "obstacles.detector: bigmap_dynamic or navmap_filter, not '" + value + "'");
}

const char * to_string(ObstacleDetector detector)
{
  return detector == ObstacleDetector::BigmapDynamic ? "bigmap_dynamic" : "navmap_filter";
}

PipelineReport check_pipeline(const PipelineSettings & s)
{
  PipelineReport r;
  const auto lists = [&s](const char * layer) {
      return std::any_of(
        s.filters.begin(), s.filters.end(),
        [layer](const LoadedFilter & f) {return f.layer == layer;});
    };
  const bool obstacle_filter = lists("obstacles");
  const bool inflation = lists("inflated_obstacles");
  const bool bigmap_detects = s.detector == ObstacleDetector::BigmapDynamic;

  if (!bigmap_detects && !obstacle_filter) {
    r.errors.push_back(
      "obstacles.detector: navmap_filter, but no obstacle filter is in 'filters' (an entry whose "
      "plugin is ...ObstaclesFilter, named other than 'obstacles'): the sensors would never "
      "detect anything. List it, or use obstacles.detector: bigmap_dynamic");
  }
  {
    // the inflation spreads what is in `obstacles` when it runs: listed before the obstacle
    // filter it always works on the previous cycle's obstacles
    std::size_t first_inflation = s.filters.size(), last_obstacle = 0;
    for (std::size_t k = 0; k < s.filters.size(); ++k) {
      if (s.filters[k].layer == "inflated_obstacles") {
        first_inflation = std::min(first_inflation, k);
      }
      if (s.filters[k].layer == "obstacles") {last_obstacle = k;}
    }
    if (obstacle_filter && inflation && first_inflation < last_obstacle) {
      r.warnings.push_back(
        "'filters' lists the inflation before the obstacle filter: it inflates the obstacles of "
        "the previous cycle. List the obstacle filter first");
    }
  }
  if (bigmap_detects && obstacle_filter) {
    r.warnings.push_back(
      "obstacles.detector: bigmap_dynamic, and an obstacle filter is listed in 'filters' too: "
      "both write the 'obstacles' layer and the filter's marks are never removed from the map. "
      "Set obstacles.detector: navmap_filter or take the filter out of 'filters'");
  }

  r.sensors_processed = bigmap_detects && (s.dynamic_map_enabled || s.memory_enabled);
  r.memory_effective = bigmap_detects && s.memory_enabled;
  if (!bigmap_detects && s.memory_enabled) {
    r.warnings.push_back(
      "obstacles.memory.enabled needs obstacles.detector: bigmap_dynamic (the filter's obstacles "
      "carry no detection time): the memory is off");
  }
  if (bigmap_detects && !r.sensors_processed) {
    r.notes.push_back(
      "no sensor obstacles: dynamic_map.enabled and obstacles.memory.enabled are both false; "
      "only terrain steeper than terrain.max_slope_deg blocks");
  }
  if (r.sensors_processed && !s.removal_enabled) {
    r.warnings.push_back(
      "obstacles.removal.enabled is false: what the sensors detect reaches the dynamic map but "
      "NOT the 'obstacles' layer, so the planner does not see it");
  }
  r.planner_layer = inflation ? "inflated_obstacles" : "obstacles";
  if (!inflation) {
    r.notes.push_back(
      "no inflation filter in 'filters': A* reads 'obstacles' and keeps no distance to obstacles");
  }

  std::string loaded;
  for (const auto & f : s.filters) {
    loaded += (loaded.empty() ? "" : ", ") + f.instance;
  }
  r.notes.push_back(
    std::string("obstacles: detector ") + to_string(s.detector) + ", sensors " +
    (r.sensors_processed || !bigmap_detects ? "analysed" : "not analysed") + ", memory " +
    (r.memory_effective ? "on" : "off") + ", removal " + (s.removal_enabled ? "on" : "off") +
    "; filters [" + loaded + "]; A* reads '" + r.planner_layer + "'");
  return r;
}

}  // namespace bigmap_maps_manager
