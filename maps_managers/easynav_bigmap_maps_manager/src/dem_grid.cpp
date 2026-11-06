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

#include "easynav_bigmap_maps_manager/dem_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace bigmap_maps_manager
{

namespace
{
constexpr char kMagic[8] = {'L', 'M', 'D', 'E', 'M', '0', '0', '6'};
// the optional block after the colours: which cells have ground (DemGrid::cell_valid)
constexpr char kCellsTag[8] = {'C', 'E', 'L', 'L', 'S', '\0', '\0', '\0'};
}  // namespace

double DemGrid::height_at(double x, double y) const
{
  // a NaN survives std::clamp and its cast to int is undefined (INT_MIN on x86: a read far
  // outside z): no height there
  if (!std::isfinite(x) || !std::isfinite(y)) {return std::numeric_limits<double>::quiet_NaN();}
  const double c = (side - 1) / 2.0;
  const double u = std::clamp(x / resolution + c, 0.0, side - 1.0);
  const double v = std::clamp(y / resolution + c, 0.0, side - 1.0);
  const int i = std::min(static_cast<int>(u), side - 2),
    j = std::min(static_cast<int>(v), side - 2);
  const double tu = u - i, tv = v - j;
  return (1 - tv) * ((1 - tu) * at(i, j) + tu * at(i + 1, j)) +
         tv * ((1 - tu) * at(i, j + 1) + tu * at(i + 1, j + 1));
}

DemGrid DemGrid::subsample(int stride) const
{
  if (stride < 1 || (side - 1) % stride != 0) {
    throw std::invalid_argument("stride must divide side - 1");
  }
  DemGrid g = *this;
  g.side = (side - 1) / stride + 1;
  g.resolution = resolution * stride;
  g.rgb.clear();
  g.imagery_zoom = 0;
  g.z.resize(static_cast<std::size_t>(g.side) * g.side);
  for (int j = 0; j < g.side; ++j) {
    for (int i = 0; i < g.side; ++i) {
      g.z[static_cast<std::size_t>(j) * g.side + i] = at(i * stride, j * stride);
    }
  }
  if (!cell_valid.empty()) {
    const int nc = side - 1, gc = g.side - 1;
    g.cell_valid.assign(static_cast<std::size_t>(gc) * gc, 1);
    for (int j = 0; j < nc; ++j) {
      for (int i = 0; i < nc; ++i) {
        if (!cell_valid[static_cast<std::size_t>(j) * nc + i]) {
          g.cell_valid[static_cast<std::size_t>(j / stride) * gc + i / stride] = 0;
        }
      }
    }
  }
  return g;
}

bool DemGrid::save(const std::string & path) const
{
  std::error_code ec;
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {std::filesystem::create_directories(parent, ec);}
  const std::string tmp = path + ".tmp";
  std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
  if (!f) {return false;}
  const std::int32_t s = side;
  const std::int32_t syn = source;
  const double hdr[7] = {resolution, radius, origin_lat, origin_lon, reference_altitude,
    requested_reference, data_resolution};
  f.write(kMagic, sizeof(kMagic));
  f.write(reinterpret_cast<const char *>(&s), sizeof(s));
  f.write(reinterpret_cast<const char *>(&syn), sizeof(syn));
  f.write(reinterpret_cast<const char *>(hdr), sizeof(hdr));
  f.write(reinterpret_cast<const char *>(z.data()),
      static_cast<std::streamsize>(z.size() * sizeof(float)));
  const std::int32_t zoom = rgb.empty() ? 0 : imagery_zoom;
  f.write(reinterpret_cast<const char *>(&zoom), sizeof(zoom));
  if (zoom != 0) {
    f.write(reinterpret_cast<const char *>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
  }
  if (!cell_valid.empty()) {
    if (cell_valid.size() != static_cast<std::size_t>(side - 1) * (side - 1)) {return false;}
    f.write(kCellsTag, sizeof(kCellsTag));
    f.write(reinterpret_cast<const char *>(cell_valid.data()),
      static_cast<std::streamsize>(cell_valid.size()));
  }
  f.close();
  if (!f) {return false;}
  std::filesystem::rename(tmp, path, ec);  // atomic: readers never see a half-written file
  return !ec;
}

std::optional<DemGrid> DemGrid::load(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {return std::nullopt;}
  char magic[8];
  std::int32_t s = 0, syn = 0;
  double hdr[7];
  f.read(magic, sizeof(magic));
  f.read(reinterpret_cast<char *>(&s), sizeof(s));
  f.read(reinterpret_cast<char *>(&syn), sizeof(syn));
  f.read(reinterpret_cast<char *>(hdr), sizeof(hdr));
  // side 1 is no grid (height_at reads node side - 2); the resolution and the header values are
  // used as divisors and as the origin of the whole simulation
  if (!f || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0 || s < 3 || s % 2 == 0 || s > 20000 ||
    !(hdr[0] > 0.0) ||
    !std::isfinite(hdr[0]) || !std::isfinite(hdr[1]) || !std::isfinite(hdr[2]) ||
    !std::isfinite(hdr[3]) || !std::isfinite(hdr[4]))
  {
    return std::nullopt;
  }
  // a header must not make us allocate what the file does not hold: heights + zoom at least
  const auto nodes = static_cast<std::uintmax_t>(s) * static_cast<std::uintmax_t>(s);
  std::error_code size_ec;
  const auto file_size = std::filesystem::file_size(path, size_ec);
  const std::uintmax_t header_bytes = sizeof(magic) + 2 * sizeof(std::int32_t) + sizeof(hdr);
  if (size_ec || file_size < header_bytes + nodes * sizeof(float) + sizeof(std::int32_t)) {
    return std::nullopt;
  }
  DemGrid g;
  g.side = s;
  g.source = syn;
  g.resolution = hdr[0];
  g.radius = hdr[1];
  g.origin_lat = hdr[2];
  g.origin_lon = hdr[3];
  g.reference_altitude = hdr[4];
  g.requested_reference = hdr[5];
  g.data_resolution = hdr[6];
  g.z.resize(static_cast<std::size_t>(s) * s);
  f.read(reinterpret_cast<char *>(g.z.data()),
      static_cast<std::streamsize>(g.z.size() * sizeof(float)));
  if (!f) {return std::nullopt;}
  std::int32_t zoom = 0;
  f.read(reinterpret_cast<char *>(&zoom), sizeof(zoom));
  if (!f) {return std::nullopt;}
  if (zoom < 0 || zoom > 30) {return std::nullopt;}
  if (std::any_of(g.z.begin(), g.z.end(), [](float v) {return !std::isfinite(v);})) {
    return std::nullopt;
  }
  if (zoom != 0) {
    if (file_size < header_bytes + nodes * (sizeof(float) + 3) + sizeof(std::int32_t)) {
      return std::nullopt;
    }
    g.rgb.resize(g.z.size() * 3);
    f.read(reinterpret_cast<char *>(g.rgb.data()), static_cast<std::streamsize>(g.rgb.size()));
    if (!f) {return std::nullopt;}
    g.imagery_zoom = zoom;
  }
  // the optional cells block: absent in older files (the file ends here); present, it is whole
  char tag[8];
  f.read(tag, sizeof(tag));
  if (f.gcount() == 0) {return g;}
  if (f.gcount() != sizeof(tag) || std::memcmp(tag, kCellsTag, sizeof(kCellsTag)) != 0) {
    return std::nullopt;
  }
  g.cell_valid.resize(static_cast<std::size_t>(s - 1) * (s - 1));
  f.read(reinterpret_cast<char *>(g.cell_valid.data()),
    static_cast<std::streamsize>(g.cell_valid.size()));
  if (!f) {return std::nullopt;}
  for (auto & c : g.cell_valid) {
    c = c != 0;
  }
  return g;
}

}  // namespace bigmap_maps_manager
