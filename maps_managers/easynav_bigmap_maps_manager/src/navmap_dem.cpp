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

#include "easynav_bigmap_maps_manager/navmap_dem.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "navmap_core/Costs.hpp"

namespace bigmap_maps_manager
{

void SourceMesh::validate() const
{
  const std::size_t nv = x.size(), nt = v0.size();
  if (y.size() != nv || z.size() != nv || v1.size() != nt || v2.size() != nt) {
    throw std::invalid_argument("the NavMap's arrays differ in size");
  }
  for (std::size_t v = 0; v < nv; ++v) {
    // a NaN would reach every mesh, or an index cast (undefined): refused, by name
    if (!std::isfinite(x[v]) || !std::isfinite(y[v]) || !std::isfinite(z[v])) {
      throw std::invalid_argument(
              "the NavMap has a vertex with a non-finite position (vertex " + std::to_string(v) +
              ")");
    }
  }
  for (std::size_t t = 0; t < nt; ++t) {
    if (v0[t] >= nv || v1[t] >= nv || v2[t] >= nv) {
      throw std::invalid_argument(
              "the NavMap has a NavCel with a vertex index out of range (NavCel " +
              std::to_string(t) + ")");
    }
  }
  if ((!traversed.empty() && traversed.size() != nt) ||
    (!obstacle_hits.empty() && obstacle_hits.size() != nt))
  {
    throw std::invalid_argument("the NavMap has a layer of another size than its NavCels");
  }
}

SourceMesh SourceMesh::from_navmap(const ::navmap::NavMap & map)
{
  SourceMesh m;
  m.x = map.positions.x;
  m.y = map.positions.y;
  m.z = map.positions.z;
  const std::size_t nt = map.navcels.size();
  m.v0.resize(nt);
  m.v1.resize(nt);
  m.v2.resize(nt);
  for (std::size_t t = 0; t < nt; ++t) {
    m.v0[t] = map.navcels[t].v[0];
    m.v1[t] = map.navcels[t].v[1];
    m.v2[t] = map.navcels[t].v[2];
  }
  if (map.colors && map.colors->r.size() == m.x.size() && map.colors->g.size() == m.x.size() &&
    map.colors->b.size() == m.x.size())
  {
    m.r = map.colors->r;
    m.g = map.colors->g;
    m.b = map.colors->b;
  }
  for (const char * name : {"traversed", "obstacle_hits"}) {
    if (!map.layers.get(name)) {continue;}
    auto & out = std::string(name) == "traversed" ? m.traversed : m.obstacle_hits;
    out.resize(nt);
    for (std::size_t t = 0; t < nt; ++t) {
      out[t] = map.layer_get<std::uint8_t>(name, static_cast<::navmap::NavCelId>(t), 0);
    }
  }
  m.has_traversed_layer = map.layers.get("traversed") != nullptr;
  return m;
}

NavMapDem dem_from_navmap(const ::navmap::NavMap & map, double resolution, int max_side)
{
  if (map.navcels.empty() || map.positions.x.empty()) {
    throw std::invalid_argument("the NavMap has no NavCels");
  }
  return dem_from_navmap(SourceMesh::from_navmap(map), resolution, max_side);
}

NavMapDem dem_from_navmap(const SourceMesh & map, double resolution, int max_side)
{
  if (map.triangle_count() == 0 || map.x.empty()) {
    throw std::invalid_argument("the NavMap has no NavCels");
  }
  if (resolution <= 0.0) {throw std::invalid_argument("resolution must be > 0");}

  const auto & px = map.x;
  const auto & py = map.y;
  const auto & pz = map.z;
  const std::size_t nt = map.triangle_count();
  auto vtx = [&map](std::size_t t, int k) {
      return k == 0 ? map.v0[t] : (k == 1 ? map.v1[t] : map.v2[t]);
    };
  double half = 0.0;
  for (std::size_t t = 0; t < nt; ++t) {
    for (int k = 0; k < 3; ++k) {
      const auto v = vtx(t, k);
      if (v >= px.size()) {continue;}
      // (a non-finite vertex is skipped below with its NavCel: it must not size the grid)
      if (!std::isfinite(px[v]) || !std::isfinite(py[v])) {continue;}
      half = std::max({half, std::abs(static_cast<double>(px[v])),
            std::abs(static_cast<double>(py[v]))});
    }
  }
  // the grid size is checked BEFORE any cast to int: an absurd extent (a corrupt file) would
  // overflow the cast (undefined) and then every index below
  const double reach_d = std::ceil(half / resolution) + 1.0;
  const std::int64_t side_l = 2 * static_cast<std::int64_t>(std::min(reach_d, 1e9)) + 1;
  if (side_l > max_side) {
    throw std::invalid_argument(
            "the NavMap spans too much for a " + std::to_string(resolution) + " m grid (" +
            std::to_string(side_l) + " nodes per side, at most " + std::to_string(max_side) +
            "): raise lod_settings.min_triangle_size / initial_navmap_resolution");
  }
  const int side = static_cast<int>(side_l);
  const int reach = (side - 1) / 2;

  NavMapDem out;
  auto & dem = out.dem;
  dem.side = side;
  dem.resolution = resolution;
  dem.data_resolution = resolution;
  dem.radius = reach * resolution;
  dem.source = DemGrid::kMesh;
  const std::size_t n = static_cast<std::size_t>(side) * side;
  const int nc = side - 1;
  constexpr float kNone = -std::numeric_limits<float>::infinity();
  dem.z.assign(n, kNone);
  out.valid.assign(n, 0);
  out.cell_valid.assign(static_cast<std::size_t>(nc) * nc, 0);
  const bool coloured = map.coloured();
  if (coloured) {dem.rgb.assign(n * 3, 128);}

  const bool has_traversed = map.has_traversed_layer || !map.traversed.empty();
  out.has_traversed = has_traversed;

  for (std::size_t t = 0; t < nt; ++t) {
    const auto a = vtx(t, 0), b = vtx(t, 1), c = vtx(t, 2);
    if (a >= px.size() || b >= px.size() || c >= px.size()) {continue;}
    const double x0 = px[a], y0 = py[a], x1 = px[b], y1 = py[b];
    const double x2 = px[c], y2 = py[c];
    if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1) ||
      !std::isfinite(x2) || !std::isfinite(y2))
    {
      continue;   // (NaN passes every comparison below, and its cast to int is undefined)
    }
    const double det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
    if (std::abs(det) < 1e-12) {continue;}
    const bool trav = !map.traversed.empty() && map.traversed[t] > 0;
    const double xmin = std::min({x0, x1, x2}), xmax = std::max({x0, x1, x2});
    const double ymin = std::min({y0, y1, y2}), ymax = std::max({y0, y1, y2});
    const int i0 = std::max(0, static_cast<int>(std::floor(xmin / resolution)) + reach);
    const int i1 = std::min(side - 1, static_cast<int>(std::ceil(xmax / resolution)) + reach);
    const int j0 = std::max(0, static_cast<int>(std::floor(ymin / resolution)) + reach);
    const int j1 = std::min(side - 1, static_cast<int>(std::ceil(ymax / resolution)) + reach);
    auto bary = [&](double x, double y, double & l0, double & l1, double & l2) {
        l0 = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / det;
        l1 = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / det;
        l2 = 1.0 - l0 - l1;
        constexpr double kEps = -1e-9;
        return l0 >= kEps && l1 >= kEps && l2 >= kEps;
      };
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        double l0, l1, l2;
        // the cell whose lower corner is this node: covered if its centre is in the NavCel
        if (i < nc && j < nc &&
          bary((i - reach + 0.5) * resolution, (j - reach + 0.5) * resolution, l0, l1, l2))
        {
          out.cell_valid[static_cast<std::size_t>(j) * nc + i] = 1;
        }
        const double x = (i - reach) * resolution, y = (j - reach) * resolution;
        if (!bary(x, y, l0, l1, l2)) {continue;}
        const std::size_t k = static_cast<std::size_t>(j) * side + i;
        const float z = static_cast<float>(l0 * pz[a] + l1 * pz[b] + l2 * pz[c]);
        if (!std::isfinite(z)) {continue;}   // a NaN height is no data, not a node at -inf
        if (z > dem.z[k]) {
          dem.z[k] = z;
          if (coloured) {
            dem.rgb[3 * k] = static_cast<std::uint8_t>(
              l0 * map.r[a] + l1 * map.r[b] + l2 * map.r[c]);
            dem.rgb[3 * k + 1] = static_cast<std::uint8_t>(
              l0 * map.g[a] + l1 * map.g[b] + l2 * map.g[c]);
            dem.rgb[3 * k + 2] = static_cast<std::uint8_t>(
              l0 * map.b[a] + l1 * map.b[b] + l2 * map.b[c]);
          }
        }
        if (!out.valid[k]) {
          out.valid[k] = 1;
          ++out.covered_nodes;
        }
        if (trav) {out.traversed_nodes.emplace_back(i, j);}
      }
    }
  }
  if (out.covered_nodes == 0) {
    throw std::invalid_argument("no NavCel of the map contains a node of the grid");
  }

  // Close the sampling gaps (NavCels smaller than the grid can miss a node): two passes of "a
  // node with at least 5 of its 8 neighbours covered takes their mean". 5, not fewer: a node just
  // outside a straight border has 3, so the borders do not grow; a missing node inside the surface
  // has 8 and a missing line of nodes 6, so they are filled. A gap of 2 nodes or more stays.
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<std::uint8_t> newly;
    std::vector<std::size_t> idx;
    std::vector<float> zs;
    std::vector<std::array<int, 3>> cols;
    for (int j = 1; j < side - 1; ++j) {
      for (int i = 1; i < side - 1; ++i) {
        const std::size_t k = static_cast<std::size_t>(j) * side + i;
        if (out.valid[k]) {continue;}
        double sz = 0;
        int cnt = 0;
        std::array<int, 3> sc{0, 0, 0};
        for (int dj = -1; dj <= 1; ++dj) {
          for (int di = -1; di <= 1; ++di) {
            const std::size_t q = static_cast<std::size_t>(j + dj) * side + (i + di);
            if (!out.valid[q]) {continue;}
            sz += dem.z[q];
            ++cnt;
            if (coloured) {
              for (int c = 0; c < 3; ++c) {
                sc[c] += dem.rgb[3 * q + c];
              }
            }
          }
        }
        if (cnt >= 5) {
          idx.push_back(k);
          zs.push_back(static_cast<float>(sz / cnt));
          cols.push_back({sc[0] / cnt, sc[1] / cnt, sc[2] / cnt});
        }
      }
    }
    for (std::size_t m = 0; m < idx.size(); ++m) {
      dem.z[idx[m]] = zs[m];
      out.valid[idx[m]] = 1;
      ++out.covered_nodes;
      if (coloured) {
        for (int c = 0; c < 3; ++c) {
          dem.rgb[3 * idx[m] + c] = cols[m][c];
        }
      }
    }
  }

  // No-data nodes take the height of the nearest covered one (rings of dilation), so the error
  // map and the interpolation never see an infinity; the mesher drops what is not valid
  // (the frontier is the set of undone nodes next to a done one: each ring costs its own size, not
  // the whole grid's)
  std::vector<std::uint8_t> done = out.valid;
  std::vector<std::uint8_t> queued(n, 0);
  std::vector<std::size_t> frontier;
  auto push_neighbours = [&](int i, int j) {
      for (int dj = -1; dj <= 1; ++dj) {
        for (int di = -1; di <= 1; ++di) {
          const int a = i + di, b = j + dj;
          if (a < 0 || b < 0 || a >= side || b >= side) {continue;}
          const std::size_t q = static_cast<std::size_t>(b) * side + a;
          if (!done[q] && !queued[q]) {
            queued[q] = 1;
            frontier.push_back(q);
          }
        }
      }
    };
  for (int j = 0; j < side; ++j) {
    for (int i = 0; i < side; ++i) {
      if (done[static_cast<std::size_t>(j) * side + i]) {push_neighbours(i, j);}
    }
  }
  while (!frontier.empty()) {
    std::vector<float> zs(frontier.size());
    for (std::size_t m = 0; m < frontier.size(); ++m) {
      const int i = static_cast<int>(frontier[m] % side), j = static_cast<int>(frontier[m] / side);
      double sz = 0;
      int cnt = 0;
      for (int dj = -1; dj <= 1; ++dj) {
        for (int di = -1; di <= 1; ++di) {
          const int a = i + di, b = j + dj;
          if (a < 0 || b < 0 || a >= side || b >= side) {continue;}
          const std::size_t q = static_cast<std::size_t>(b) * side + a;
          if (done[q]) {sz += dem.z[q]; ++cnt;}
        }
      }
      zs[m] = static_cast<float>(sz / cnt);   // the frontier has at least one done neighbour
    }
    const std::vector<std::size_t> ring = std::move(frontier);
    frontier.clear();
    for (std::size_t m = 0; m < ring.size(); ++m) {
      dem.z[ring[m]] = zs[m];
      done[ring[m]] = 1;
    }
    for (const auto k : ring) {
      push_neighbours(static_cast<int>(k % side), static_cast<int>(k / side));
    }
  }
  return out;
}

std::string origin_file_for(const std::string & navmap_path)
{
  const auto dot = navmap_path.find_last_of('.');
  const auto slash = navmap_path.find_last_of('/');
  const bool has_ext = dot != std::string::npos && (slash == std::string::npos || dot > slash);
  return (has_ext ? navmap_path.substr(0, dot) : navmap_path) + ".yaml";
}

bool save_origin(const std::string & navmap_path, const MapOrigin & origin, double resolution)
{
  std::ofstream f(origin_file_for(navmap_path), std::ios::trunc);
  if (!f) {return false;}
  f.precision(10);
  f << "# Position of the map's (0, 0, 0). Without this file a map is loaded as a plain NavMap\n"
    << "# without geolocation.\n"
    << "georeferenced: " << (origin.georeferenced ? "true" : "false") << "\n"
    << "origin_lat: " << origin.lat << "\n"
    << "origin_lon: " << origin.lon << "\n"
    << "reference_altitude: " << origin.altitude << "\n"
    << "min_resolution: " << resolution << "\n";
  return static_cast<bool>(f);
}

MapOrigin load_origin(const std::string & navmap_path)
{
  MapOrigin o;
  std::ifstream f(origin_file_for(navmap_path));
  if (!f) {return o;}
  bool has_lat = false, has_lon = false;
  std::string line;
  while (std::getline(f, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {line.erase(hash);}
    const auto colon = line.find(':');
    if (colon == std::string::npos) {continue;}
    auto trim = [](std::string s) {
        const auto b = s.find_first_not_of(" \t\r\n");
        const auto e = s.find_last_not_of(" \t\r\n");
        return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
      };
    const std::string key = trim(line.substr(0, colon)), val = trim(line.substr(colon + 1));
    try {
      if (key == "georeferenced") {
        o.georeferenced = val == "true";
      } else if (key == "origin_lat") {
        o.lat = std::stod(val);
        has_lat = true;
      } else if (key == "origin_lon") {
        o.lon = std::stod(val);
        has_lon = true;
      } else if (key == "reference_altitude") {
        o.altitude = std::stod(val);
      }
    } catch (const std::exception &) {
      return MapOrigin{};
    }
  }
  // (a "nan" or "inf" reads as a number: not a place on Earth, so not georeferenced)
  o.georeferenced = o.georeferenced && has_lat && has_lon && std::isfinite(o.lat) &&
    std::isfinite(o.lon) && std::isfinite(o.altitude);
  return o;
}

}  // namespace bigmap_maps_manager
