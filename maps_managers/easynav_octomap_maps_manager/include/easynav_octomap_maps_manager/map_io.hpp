// Copyright (c) 2018 Intel Corporation
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

/* OccupancyGrid map input-output library */

#ifndef EASYNAV_OCTOMAP_MAPS_MANAGER__MAP_IO_HPP_
#define EASYNAV_OCTOMAP_MAPS_MANAGER__MAP_IO_HPP_

#include <string>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

/* Map input part */

namespace easynav
{
namespace octomap
{

/**
 * @enum nav2_map_server::MapMode
 * @brief Describes the relation between image pixel values and map occupancy
 * status (0-100; -1). Lightness refers to the mean of a given pixel's RGB
 * channels on a scale from 0 to 255.
 */
enum class MapMode
{
  /**
   * Together with associated threshold values (occupied and free):
   *   lightness >= occupied threshold - Occupied (100)
   *             ... (anything in between) - Unknown (-1)
   *    lightness <= free threshold - Free (0)
   */
  Trinary,
  /**
   * Together with associated threshold values (occupied and free):
   *   alpha < 1.0 - Unknown (-1)
   *   lightness >= occ_th - Occupied (100)
   *             ... (linearly interpolate to)
   *   lightness <= free_th - Free (0)
   */
  Scale,
  /**
   * Lightness = 0 - Free (0)
   *          ... (linearly interpolate to)
   * Lightness = 100 - Occupied (100)
   * Lightness >= 101 - Unknown
   */
  Raw,
};

/**
 * @brief Convert a MapMode enum to the name of the map mode
 * @param map_mode Mode for the map
 * @return String identifier of the given map mode
 * @throw std::invalid_argument if the given value is not a defined map mode
 */
const char * map_mode_to_string(MapMode map_mode);

/**
 * @brief Convert the name of a map mode to a MapMode enum
 * @param map_mode_name Name of the map mode
 * @throw std::invalid_argument if the name does not name a map mode
 * @return map mode corresponding to the string
 */
MapMode map_mode_from_string(std::string map_mode_name);


/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}


struct LoadParameters
{
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

typedef enum
{
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
} LOAD_MAP_STATUS;

static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

/**
 * @brief Load and parse the given YAML file
 * @param yaml_filename Name of the map file passed though parameter
 * @return Map loading parameters obtained from YAML file
 * @throw YAML::Exception
 */
LoadParameters loadMapYaml(const std::string & yaml_filename);

/**
 * @brief Load the image from map file and generate an OccupancyGrid
 * @param load_parameters Parameters of loading map
 * @param map Output loaded map
 * @throw std::exception
 */
void loadMapFromFile(
  const LoadParameters & load_parameters,
  nav_msgs::msg::OccupancyGrid & map);

/**
 * @brief Load the map YAML, image from map file and
 * generate an OccupancyGrid
 * @param yaml_file Name of input YAML file
 * @param map Output loaded map
 * @return status of map loaded
 */
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav_msgs::msg::OccupancyGrid & map);


/* Map output part */

struct SaveParameters
{
  std::string map_file_name{""};
  std::string image_format{""};
  double free_thresh{0.0};
  double occupied_thresh{0.0};
  MapMode mode{MapMode::Trinary};
};

/**
 * @brief Write OccupancyGrid map to file
 * @param map OccupancyGrid map data
 * @param save_parameters Map saving parameters.
 * @return true or false
 */
bool saveMapToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters);

/**
 * @brief Expand ~/ to home user dir.
 * @param yaml_filename Name of input YAML file.
 * @param home_dir Expanded `~/`home dir or empty string if HOME not set
 *
 * @return Expanded string or input string if `~/` not expanded
 */
std::string expand_user_home_dir_if_needed(
  std::string yaml_filename,
  std::string home_dir);

}  // namespace octomap
}  // namespace easynav
#endif  // EASYNAV_OCTOMAP_MAPS_MANAGER__MAP_IO_HPP_
