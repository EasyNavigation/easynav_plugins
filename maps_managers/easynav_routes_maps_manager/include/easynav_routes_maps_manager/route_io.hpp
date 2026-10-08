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
/// \brief Routes YAML input, and RoutesMap <-> RoutesMap.msg conversion.
///
/// Free functions, not RoutesMapsManager members, so that any other
/// node (e.g. a fleet-wide navigation manager publishing on
/// /global_routes) can load/convert routes the exact same way
/// RoutesMapsManager itself does, without depending on
/// RoutesMapsManager's own heavier machinery (pluginlib,
/// interactive_markers, ...).

#ifndef EASYNAV_ROUTES_MAPS_MANAGER__ROUTE_IO_HPP_
#define EASYNAV_ROUTES_MAPS_MANAGER__ROUTE_IO_HPP_

#include <string>

#include "easynav_routes_maps_manager/msg/routes_map.hpp"
#include "easynav_routes_maps_manager/routes_map.hpp"

namespace easynav
{

/**
 * @brief Load a RoutesMap from a routes YAML file.
 *
 * Same format used by RoutesMapsManager's own `map_path_file`
 * parameter: a top-level `routes: [name, ...]` list plus one
 * `start`/`end` pose-pair entry per name. Falls back to a single
 * default segment ((0,0,0) -> (1,0,0), id "route0") when the file is
 * empty, missing, invalid, or has no "routes" key.
 * @param yaml_file Path to the routes YAML file (empty means "use the
 *   default segment").
 * @return The parsed (or default) RoutesMap.
 */
RoutesMap load_routes_from_yaml(const std::string & yaml_file);

/**
 * @brief Convert an in-memory RoutesMap to its wire message form.
 * @param routes Routes to convert.
 * @return The equivalent RoutesMap message (edit_mode is not carried
 *   over -- it is UI-editor-only state).
 */
easynav_routes_maps_manager::msg::RoutesMap to_msg(const RoutesMap & routes);

/**
 * @brief Convert a wire RoutesMap message back to the in-memory form.
 * @param msg Message to convert.
 * @return The equivalent RoutesMap.
 */
RoutesMap from_msg(const easynav_routes_maps_manager::msg::RoutesMap & msg);

/**
 * @brief Persist a RoutesMap to a routes YAML file, in the exact format
 * load_routes_from_yaml() reads back: a top-level `routes: [name, ...]`
 * list plus one `start`/`end` pose-pair entry per name.
 *
 * A free function (rather than a RoutesMapsManager member) for the same
 * reason load_routes_from_yaml()/to_msg()/from_msg() are: so any other
 * node wanting to persist edited routes to the same file format --
 * RoutesMapsManager's own `save_routes` service included -- can do so
 * without depending on RoutesMapsManager itself.
 *
 * @param yaml_file Path to write to.
 * @param routes Routes to persist. Segment ids are used as the YAML
 *   keys; segments with an empty id are named "route<index>".
 * @param error_message Set to a human-readable reason on failure;
 *   untouched on success.
 * @return true on success, false if the file could not be opened for
 *   writing (see error_message).
 */
bool save_routes_to_yaml(
  const std::string & yaml_file, const RoutesMap & routes, std::string & error_message);

}  // namespace easynav

#endif  // EASYNAV_ROUTES_MAPS_MANAGER__ROUTE_IO_HPP_
