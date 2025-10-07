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


#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "easynav_simple_common/SimpleMap.hpp"

namespace easynav
{

SimpleMap::SimpleMap()
: width_(0), height_(0), resolution_(1.0), origin_x_(0.0), origin_y_(0.0), data_()
{}

void
SimpleMap::initialize(
  int width, int height, double resolution,
  double origin_x, double origin_y, bool initial_value)
{
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  data_.assign(width * height, initial_value);
}

uint8_t
SimpleMap::at(int x, int y) const
{
  check_bounds(x, y);
  return data_[index(x, y)];
}


uint8_t &
SimpleMap::at(int x, int y)
{
  check_bounds(x, y);
  return data_.at(index(x, y));
}

void
SimpleMap::fill(uint8_t value)
{
  std::fill(data_.begin(), data_.end(), value);
}

void
SimpleMap::deep_copy(const SimpleMap & other)
{
  width_ = other.width_;
  height_ = other.height_;
  resolution_ = other.resolution_;
  origin_x_ = other.origin_x_;
  origin_y_ = other.origin_y_;
  data_ = other.data_;
}

bool
SimpleMap::check_bounds_metric(double mx, double my) const
{
  double relative_x = mx - origin_x_;
  double relative_y = my - origin_y_;

  if (relative_x < 0.0 || relative_y < 0.0) {
    return false;
  }

  size_t x = static_cast<size_t>(relative_x / resolution_);
  size_t y = static_cast<size_t>(relative_y / resolution_);

  return x < width_ && y < height_;
}

std::pair<double, double>
SimpleMap::cell_to_metric(int x, int y) const
{
  double mx = origin_x_ + (static_cast<double>(x) + 0.5) * resolution_;
  double my = origin_y_ + (static_cast<double>(y) + 0.5) * resolution_;
  return {mx, my};
}

std::pair<int, int>
SimpleMap::metric_to_cell(double mx, double my) const
{
  double relative_x = mx - origin_x_;
  double relative_y = my - origin_y_;

  int x = static_cast<int>(relative_x / resolution_);
  int y = static_cast<int>(relative_y / resolution_);

  return {x, y};
}

void
SimpleMap::to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg) const
{
  grid_msg.info.width = static_cast<uint32_t>(width_);
  grid_msg.info.height = static_cast<uint32_t>(height_);
  grid_msg.info.resolution = static_cast<float>(resolution_);


  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.x = 0.0;
  grid_msg.info.origin.orientation.y = 0.0;
  grid_msg.info.origin.orientation.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  if (grid_msg.data.size() != width_ * height_) {
    grid_msg.data.resize(width_ * height_);
  }

  for (size_t idx = 0; idx < data_.size(); ++idx) {
    grid_msg.data[idx] = data_[idx] ? 100 : 0;
  }
}

void
SimpleMap::from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid_msg)
{
  initialize(
    grid_msg.info.width,
    grid_msg.info.height,
    grid_msg.info.resolution,
    grid_msg.info.origin.position.x,
    grid_msg.info.origin.position.y,
    false);  // valor inicial: libre (false)

  for (size_t y = 0; y < height_; ++y) {
    for (size_t x = 0; x < width_; ++x) {
      size_t idx = y * width_ + x;
      int8_t val = grid_msg.data[idx];
      data_[idx] = (val >= 50);
    }
  }
}

bool
SimpleMap::save_to_file(const std::string & path) const
{
  std::ofstream out(path);
  if (!out.is_open()) {
    return false;
  }

  out << width_ << " " << height_ << " "
      << resolution_ << " "
      << origin_x_ << " " << origin_y_ << "\n";

  for (size_t i = 0; i < data_.size(); ++i) {
    out << (data_[i] ? "1" : "0");
    if (i + 1 < data_.size()) {
      out << " ";
    }
  }

  out << "\n";
  return true;
}


bool
SimpleMap::load_from_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }

  std::string line;

  if (!std::getline(in, line)) {return false;}
  std::istringstream meta_stream(line);
  size_t w, h;
  double res, ox, oy;
  if (!(meta_stream >> w >> h >> res >> ox >> oy)) {
    return false;
  }

  if (!std::getline(in, line)) {return false;}
  std::istringstream data_stream(line);

  std::vector<uint8_t> new_data;
  int val;
  while (data_stream >> val) {
    new_data.push_back(val == 1);
  }

  if (new_data.size() != w * h) {
    return false;
  }

  width_ = w;
  height_ = h;
  resolution_ = res;
  origin_x_ = ox;
  origin_y_ = oy;
  data_ = std::move(new_data);

  return true;
}

int
SimpleMap::index(int x, int y) const
{
  return y * width_ + x;  // Row-major order
}

void
SimpleMap::check_bounds(size_t x, size_t y) const
{
  if (x >= width_ || y >= height_) {
    throw std::out_of_range("SimpleMap: index out of bounds");
  }
}

/**
 * @brief Prints metadata and optionally all map cell values with coordinates.
 *
 * @param view_data If true, prints cell-by-cell content with coordinates. Default is false.
 */
void
SimpleMap::print(bool view_data) const
{
  std::cerr << "SimpleMap Metadata:\n";
  std::cerr << "  Size: " << width_ << " x " << height_ << "\n";
  std::cerr << "  Resolution: " << resolution_ << " m/cell\n";
  std::cerr << "  Origin: (" << origin_x_ << ", " << origin_y_ << ")\n";

  if (!view_data) {return;}

  std::cerr << "SimpleMap Data:\n";
  for (size_t y = 0; y < height_; ++y) {
    for (size_t x = 0; x < width_; ++x) {
      double mx = origin_x_ + (x + 0.5) * resolution_;
      double my = origin_y_ + (y + 0.5) * resolution_;
      std::cerr << "[" << x << ", " << y << "][" << mx << ", " << my << "] "
                << static_cast<int>(data_[index(x, y)]) << "\n";
    }
  }
}

std::shared_ptr<SimpleMap>
SimpleMap::downsample_factor(int factor) const
{
  if (factor <= 1) {
    throw std::invalid_argument("Downsampling factor must be > 1");
  }

  int new_width = width_ / factor;
  int new_height = height_ / factor;
  double new_resolution = resolution_ * factor;

  auto new_map = std::make_shared<SimpleMap>();
  new_map->initialize(new_width, new_height, new_resolution, origin_x_, origin_y_, false);

  for (int y = 0; y < new_height; ++y) {
    for (int x = 0; x < new_width; ++x) {
      int occupied_count = 0;
      for (int dy = 0; dy < factor; ++dy) {
        for (int dx = 0; dx < factor; ++dx) {
          int src_x = x * factor + dx;
          int src_y = y * factor + dy;
          if (at(src_x, src_y)) {
            occupied_count++;
          }
        }
      }

      new_map->at(x, y) = (occupied_count > (factor * factor / 2));
    }
  }

  return new_map;
}

std::shared_ptr<SimpleMap>
SimpleMap::downsample(double new_resolution) const
{
  if (new_resolution <= resolution_) {
    throw std::invalid_argument("new_resolution must be greater than current resolution");
  }

  int factor = static_cast<int>(std::floor(new_resolution / resolution_));

  if (factor < 1) {
    factor = 1;
  }

  return downsample_factor(factor);
}

}  // namespace easynav
