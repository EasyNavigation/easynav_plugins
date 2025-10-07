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
/// \brief Declaration of the SimpleMap type.

#ifndef EASYNAV_PLANNER__SIMPLEMAP_HPP_
#define EASYNAV_PLANNER__SIMPLEMAP_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>

#include "nav_msgs/msg/occupancy_grid.hpp"


namespace easynav
{

/**
 * @class SimpleMap
 * @brief Simple 2D uint8_t grid using basic C++ types, with full metric conversion support.
 *
 * Supports arbitrary metric origins, allowing negative coordinates.
 */
class SimpleMap
{
public:
  /**
   * @brief Default constructor.
   *
   * Creates an empty (0x0) SimpleMap with 1.0 meter resolution and origin (0.0, 0.0).
   */
  SimpleMap();

  /**
   * @brief Initialize the map to new dimensions, resolution, and origin.
   *
   * @param width Number of columns.
   * @param height Number of rows.
   * @param resolution Size of each cell in meters.
   * @param origin_x Metric X coordinate corresponding to cell (0,0).
   * @param origin_y Metric Y coordinate corresponding to cell (0,0).
   * @param initial_value Value to initialize all cells.
   */
  void initialize(
    int width, int height, double resolution, double origin_x,
    double origin_y, bool initial_value = false);

  /**
   * @brief Returns the width (number of columns) of the map.
   */
  size_t width() const {return width_;}

  /**
   * @brief Returns the height (number of rows) of the map.
   */
  size_t height() const {return height_;}

  /**
   * @brief Returns the resolution (cell size in meters).
   */
  double resolution() const {return resolution_;}

  /**
   * @brief Returns the metric origin x coordinate.
   */
  double origin_x() const {return origin_x_;}

  /**
   * @brief Returns the metric origin y coordinate.
   */
  double origin_y() const {return origin_y_;}

  /**
   * @brief Access a cell (const) at (x, y).
   * @throw std::out_of_range if (x,y) is out of bounds.
   */
  uint8_t at(int x, int y) const;

  /**
   * @brief Access a cell (non-const) at (x, y).
   * @throw std::out_of_range if (x,y) is out of bounds.
   */
  uint8_t & at(int x, int y);

  /**
   * @brief Set all cells to a given value.
   *
   * @param value Value to fill the map with.
   */
  void fill(uint8_t value);

  /**
   * @brief Performs a deep copy from another SimpleMap.
   *
   * Copies width, height, resolution, origin, and all grid data.
   *
   * @param other The SimpleMap to copy from.
   */
  void deep_copy(const SimpleMap & other);

  /**
   * @brief Checks if given metric coordinates are within the map bounds.
   *
   * @param mx Metric x coordinate.
   * @param my Metric y coordinate.
   * @return true if inside the map bounds, false otherwise.
   */
  bool check_bounds_metric(double mx, double my) const;

  /**
   * @brief Converts a cell index (x, y) to real-world metric coordinates (meters).
   *
   * @param x Cell column index.
   * @param y Cell row index.
   * @return Pair (meters_x, meters_y).
   */
  std::pair<double, double> cell_to_metric(int x, int y) const;

  /**
   * @brief Converts real-world metric coordinates (meters) to a cell index (x, y).
   *
   * @param mx Metric x coordinate.
   * @param my Metric y coordinate.
   * @return Pair (x, y) cell indices.
   * @throw std::out_of_range if resulting indices are out of map bounds.
   */
  std::pair<int, int> metric_to_cell(double mx, double my) const;

  /**
   * @brief Updates a nav_msgs::msg::OccupancyGrid message from the SimpleMap contents.
   *
   * Cells are mapped as follows:
   * - true  -> 100 (occupied)
   * - false -> 0 (free)
   *
   * If the grid dimensions do not match, the message is rdata_esized automatically.
   *
   * @param grid_msg The occupancy grid message to fill or update.
   */
  void to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg) const;

  /**
   * @brief Load map data and metadata from a nav_msgs::msg::OccupancyGrid message.
   *
   * This function resizes the internal grid to match the occupancy grid dimensions,
   * sets the resolution and origin, and copies the data.
   *
   * @param grid_msg The occupancy grid message to load from.
   */
  void from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid_msg);

  /**
  * @brief Saves the map to a file, including metadata and cell data.
  * @param path Path to the output file.
  * @return true if the file was written successfully, false otherwise.
  */
  bool save_to_file(const std::string & path) const;

  /**
   * @brief Loads the map from a file, reading metadata and cell data.
   * @param path Path to the input file.
   * @return true if the file was read successfully and is valid, false otherwise.
   */
  bool load_from_file(const std::string & path);

  /**
   * @brief Prints metadata and optionally all map cell values with coordinates.
   *
   * @param view_data If true, prints cell-by-cell content with coordinates. Default is false.
   */
  void print(bool view_data = false) const;

  /**
   * @brief Creates a downsampled version of the map by an integer factor.
   *
   * @param factor Integer factor (>1) to reduce resolution.
   * @return A shared pointer to the new downsampled SimpleMap.
   */
  std::shared_ptr<SimpleMap> downsample_factor(int factor) const;

  /**
   * @brief Creates a downsampled version of the map to match a target resolution.
   *
   * @param new_resolution New desired resolution (must be a multiple of current).
   * @return A shared pointer to the new downsampled SimpleMap.
   */
  std::shared_ptr<SimpleMap> downsample(double new_resolution) const;

private:
  size_t width_;
  size_t height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<uint8_t> data_;

  int index(int x, int y) const;
  void check_bounds(size_t x, size_t y) const;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__SIMPLEMAP_HPP_
