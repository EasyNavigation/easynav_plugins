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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__PIPELINE_CHECK_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__PIPELINE_CHECK_HPP_

#include <string>
#include <vector>

namespace bigmap_maps_manager
{

/// Who puts what the sensors see into the `obstacles` layer (parameter obstacles.detector).
enum class ObstacleDetector
{
  /// bigmap's own DynamicLayer (heights relative to the robot, independent of the localization's z)
  BigmapDynamic,
  /// the ObstacleFilter NavMapFilter plugin of easynav_navmap_maps_manager (it writes `obstacles`)
  NavmapFilter,
};

/// "bigmap_dynamic" | "navmap_filter", and the names this option had before ("dynamic", "filter").
/// Throws std::invalid_argument for anything else.
ObstacleDetector parse_detector(const std::string & value);
const char * to_string(ObstacleDetector detector);

/// A NavMapFilter loaded through the `filters` list: its instance name (the key of its parameters)
/// and the layer it writes ("obstacles": an obstacle filter, "inflated_obstacles": the inflation).
struct LoadedFilter
{
  std::string instance;
  std::string layer;
};

/// What decides how obstacles reach the planner (see check_pipeline).
struct PipelineSettings
{
  ObstacleDetector detector{ObstacleDetector::BigmapDynamic};
  bool dynamic_map_enabled{false};   ///< dynamic_map.enabled
  bool memory_enabled{true};         ///< obstacles.memory.enabled (at start)
  bool removal_enabled{true};        ///< obstacles.removal.enabled
  std::vector<LoadedFilter> filters;  ///< in the order of `filters`
};

/// The consequences of a PipelineSettings, for the log, and the combinations that cannot work.
struct PipelineReport
{
  std::vector<std::string> errors;     ///< the node must not start
  std::vector<std::string> warnings;   ///< works, but probably not as intended
  std::vector<std::string> notes;      ///< what is on and what is off, for the log
  bool sensors_processed{false};       ///< bigmap analyses the sensor clouds
  bool memory_effective{false};        ///< obstacles.memory does anything
  std::string planner_layer;           ///< the cost layer A* reads
};

/// The dependencies between the options, in one place:
///  - detector navmap_filter needs an obstacle filter in `filters` (else nothing is ever detected);
///  - detector bigmap_dynamic with an obstacle filter listed too: two detectors, one layer;
///  - bigmap's detector works only with obstacles.removal.enabled (it is what puts the detections
///    in `obstacles`; without it they only reach the dynamic map);
///  - bigmap analyses the sensors if dynamic_map.enabled or obstacles.memory.enabled;
///  - obstacles.memory needs the bigmap detector;
///  - the planner reads `inflated_obstacles` if the inflation filter is listed, else `obstacles`;
///  - the obstacle filter must come before the inflation in `filters` (they run in that order).
PipelineReport check_pipeline(const PipelineSettings & s);

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__PIPELINE_CHECK_HPP_
