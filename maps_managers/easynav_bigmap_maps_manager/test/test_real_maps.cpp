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

// Runs the surface LOD on a real NavMap file: BIGMAP_TEST_NAVMAP=<file> (skipped without it)

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <string>
#include <system_error>

#include "easynav_bigmap_maps_manager/lod_mesher.hpp"
#include "easynav_bigmap_maps_manager/narrow_passages.hpp"
#include "easynav_bigmap_maps_manager/navmap_dem.hpp"
#include "easynav_bigmap_maps_manager/surface_lod.hpp"
#include "navmap_core/DistanceField.hpp"
#include "navmap_core/Costs.hpp"
#include "navmap_ros/navmap_io.hpp"

namespace
{
// Triangles reachable through shared edges from the one nearest to (0, 0, 0) over cells whose slope
// is at most `max_deg`; returns the heights (rounded to metres) of what is reached
template<typename PosX, typename PosY, typename PosZ>
std::map<int, std::size_t> flood(
  const PosX & X, const PosY & Y, const PosZ & Z,
  const std::vector<std::array<std::uint32_t, 3>> & tris,
  double max_deg, double sx, double sy, double sz, double keep_off = 0.0)
{
  std::map<std::array<std::int64_t, 3>, std::uint32_t> weld;
  std::vector<std::uint32_t> id(X.size());
  for (std::size_t v = 0; v < id.size(); ++v) {
    const std::array<std::int64_t, 3> k{std::lround(X[v] * 1000.0), std::lround(Y[v] * 1000.0),
      std::lround(Z[v] * 1000.0)};
    id[v] = weld.emplace(k, static_cast<std::uint32_t>(weld.size())).first->second;
  }
  const std::size_t nt = tris.size();
  std::vector<char> ok(nt, 1);
  for (std::size_t t = 0; t < nt; ++t) {
    const auto a = tris[t][0], b = tris[t][1], d = tris[t][2];
    const double ux = X[b] - X[a], uy = Y[b] - Y[a], uz = Z[b] - Z[a];
    const double vx = X[d] - X[a], vy = Y[d] - Y[a], vz = Z[d] - Z[a];
    const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    ok[t] = len > 1e-12 && std::acos(std::min(1.0, std::abs(nz) / len)) * 180.0 / M_PI <= max_deg;
  }
  std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edges;
  for (std::size_t t = 0; t < nt; ++t) {
    for (int k = 0; k < 3; ++k) {
      auto a = id[tris[t][k]], b = id[tris[t][(k + 1) % 3]];
      if (a > b) {std::swap(a, b);}
      edges[{a, b}].push_back(static_cast<std::uint32_t>(t));
    }
  }
  if (keep_off > 0.0) {
    // triangles whose centre is closer than keep_off to a border vertex (an edge of one triangle)
    std::vector<std::uint32_t> border;
    for (const auto & e : edges) {
      if (e.second.size() == 1) {
        const auto t = e.second[0];
        for (int k = 0; k < 3; ++k) {
          const auto a = id[tris[t][k]], b = id[tris[t][(k + 1) % 3]];
          if (std::make_pair(std::min(a, b), std::max(a, b)) == e.first) {
            border.push_back(tris[t][k]);
            border.push_back(tris[t][(k + 1) % 3]);
          }
        }
      }
    }
    std::multimap<std::pair<std::int64_t, std::int64_t>, std::uint32_t> grid;
    for (const auto v : border) {
      grid.emplace(std::make_pair(std::lround(X[v] / 1.0), std::lround(Y[v] / 1.0)), v);
    }
    for (std::size_t t = 0; t < nt; ++t) {
      const double cx = (X[tris[t][0]] + X[tris[t][1]] + X[tris[t][2]]) / 3.0;
      const double cy = (Y[tris[t][0]] + Y[tris[t][1]] + Y[tris[t][2]]) / 3.0;
      const double cz = (Z[tris[t][0]] + Z[tris[t][1]] + Z[tris[t][2]]) / 3.0;
      const std::int64_t gx = std::lround(cx), gy = std::lround(cy);
      for (std::int64_t j = gy - 2; j <= gy + 2 && ok[t]; ++j) {
        for (std::int64_t i = gx - 2; i <= gx + 2 && ok[t]; ++i) {
          const auto range = grid.equal_range({i, j});
          for (auto it = range.first; it != range.second; ++it) {
            const auto v = it->second;
            if (std::abs(Z[v] - cz) < 0.8 && std::hypot(X[v] - cx, Y[v] - cy) < keep_off) {
              ok[t] = 0;
              break;
            }
          }
        }
      }
    }
  }
  std::size_t start = 0;
  double best = 1e18;
  for (std::size_t t = 0; t < nt; ++t) {
    const auto v = tris[t][0];
    const double d = std::hypot(X[v] - sx, Y[v] - sy) + 3.0 * std::abs(Z[v] - sz);
    if (ok[t] && d < best) {best = d; start = t;}
  }
  std::vector<char> seen(nt, 0);
  std::vector<std::size_t> st{start};
  seen[start] = 1;
  std::map<int, std::size_t> heights;
  while (!st.empty()) {
    const auto t = st.back();
    st.pop_back();
    ++heights[static_cast<int>(std::lround(Z[tris[t][0]]))];
    for (int k = 0; k < 3; ++k) {
      auto a = id[tris[t][k]], b = id[tris[t][(k + 1) % 3]];
      if (a > b) {std::swap(a, b);}
      for (const auto n : edges[{a, b}]) {
        if (!seen[n] && ok[n]) {seen[n] = 1; st.push_back(n);}
      }
    }
  }
  return heights;
}

void print_heights(const char * what, const std::map<int, std::size_t> & h)
{
  std::printf("[flood] %s reaches z:", what);
  for (const auto & kv : h) {
    std::printf(" %d(%zu)", kv.first, kv.second);
  }
  std::printf("\n");
}
}  // namespace

namespace
{
// What the planner can drive: cells farther than `inscribed` (the robot's radius) from a border
// (the way the InflationFilter measures it), reached through shared edges from the one nearest to
// (0, 0, 0), over cells no steeper than `max_deg`
std::map<int, std::size_t> drivable(
  ::navmap::NavMap & nm, double inscribed, double max_deg,
  const std::vector<float> * slope = nullptr)
{
  nm.surfaces.resize(1);
  nm.surfaces[0].navcels.resize(nm.navcels.size());
  for (std::size_t t = 0; t < nm.navcels.size(); ++t) {
    nm.surfaces[0].navcels[t] = static_cast<::navmap::NavCelId>(t);
  }
  nm.rebuild_geometry_accels();
  ::navmap::DistanceField df(nm, true, 3.0f);
  df.compute({});
  const std::size_t n = nm.navcels.size();
  std::vector<char> ok(n, 0);
  for (std::size_t t = 0; t < n; ++t) {
    double deg = 0.0;
    if (slope) {
      deg = (*slope)[t];
    } else {
      deg = std::acos(std::min(1.0, std::abs(static_cast<double>(nm.navcels[t].normal.z())))) *
        180.0 / M_PI;
    }
    ok[t] = df[static_cast<::navmap::NavCelId>(t)] > inscribed && deg <= max_deg;
  }
  std::size_t start = n;
  double best = 1e18;
  for (std::size_t t = 0; t < n; ++t) {
    const auto v = nm.navcels[t].v[0];
    const double d = std::hypot(nm.positions.x[v], nm.positions.y[v]) +
      3.0 * std::abs(nm.positions.z[v]);
    if (ok[t] && d < best) {best = d; start = t;}
  }
  std::map<int, std::size_t> heights;
  if (start == n) {return heights;}
  std::vector<char> seen(n, 0);
  std::vector<std::size_t> st{start};
  seen[start] = 1;
  while (!st.empty()) {
    const auto t = st.back();
    st.pop_back();
    ++heights[static_cast<int>(std::lround(nm.positions.z[nm.navcels[t].v[0]]))];
    for (const auto nb : nm.navcel_neighbors(static_cast<::navmap::NavCelId>(t))) {
      if (!seen[nb] && ok[nb]) {seen[nb] = 1; st.push_back(nb);}
    }
  }
  return heights;
}
}  // namespace

TEST(RealMap, SurfaceLod)
{
  const char * path = std::getenv("BIGMAP_TEST_NAVMAP");
  if (!path) {GTEST_SKIP() << "BIGMAP_TEST_NAVMAP not set";}
  ::navmap::NavMap nm;
  std::error_code ec;
  ASSERT_TRUE(navmap_ros::io::load_from_file(path, nm, &ec)) << ec.message();
  std::size_t up = 0, down = 0;
  float zmin = 1e9f, zmax = -1e9f, xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
  for (const auto & c : nm.navcels) {
    const auto a = c.v[0], b = c.v[1], d = c.v[2];
    const double nz = (nm.positions.x[b] - nm.positions.x[a]) *
      (nm.positions.y[d] - nm.positions.y[a]) -
      (nm.positions.y[b] - nm.positions.y[a]) * (nm.positions.x[d] - nm.positions.x[a]);
    (nz > 0 ? up : down)++;
  }
  for (std::size_t v = 0; v < nm.positions.x.size(); ++v) {
    zmin = std::min(zmin, nm.positions.z[v]); zmax = std::max(zmax, nm.positions.z[v]);
    xmin = std::min(xmin, nm.positions.x[v]); xmax = std::max(xmax, nm.positions.x[v]);
    ymin = std::min(ymin, nm.positions.y[v]); ymax = std::max(ymax, nm.positions.y[v]);
  }
  std::printf(
    "[map] %zu vertices, %zu NavCels (%zu facing up, %zu down), x %.1f..%.1f y %.1f..%.1f "
    "z %.2f..%.2f, colours %s, layers:", nm.positions.x.size(), nm.navcels.size(), up, down,
    xmin, xmax, ymin, ymax, zmin, zmax, nm.colors ? "yes" : "no");
  for (const auto & l : nm.list_layers()) {
    std::printf(" %s", l.c_str());
  }
  std::printf("\n");

  std::size_t slope_bins[7] = {0};  // <5, <10, <15, <20, <25, <30, >=30 degrees
  for (const auto & c : nm.navcels) {
    const auto a = c.v[0], b = c.v[1], d = c.v[2];
    const auto & P = nm.positions;
    const double ux = P.x[b] - P.x[a], uy = P.y[b] - P.y[a], uz = P.z[b] - P.z[a];
    const double vx = P.x[d] - P.x[a], vy = P.y[d] - P.y[a], vz = P.z[d] - P.z[a];
    const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    const double deg = std::acos(std::min(1.0, std::abs(nz) / len)) * 180.0 / M_PI;
    ++slope_bins[std::min<std::size_t>(6, static_cast<std::size_t>(deg / 5.0))];
  }
  std::printf("[slope] NavCels by slope <5/<10/<15/<20/<25/<30/>=30 deg:");
  for (const auto n : slope_bins) {
    std::printf(" %zu", n);
  }
  std::printf("\n");

  if (std::getenv("BIGMAP_DUMP_LEVELS")) {
    // heights of the surfaces over a 3 m grid (rows: y from the top)
    for (float y = ymax; y >= ymin; y -= 3.0f) {
      for (float x = xmin; x <= xmax; x += 3.0f) {
        std::set<int> zs;
        for (std::size_t v = 0; v < nm.positions.x.size(); ++v) {
          if (std::abs(nm.positions.x[v] - x) < 1.5f && std::abs(nm.positions.y[v] - y) < 1.5f) {
            zs.insert(static_cast<int>(std::lround(nm.positions.z[v])));
          }
        }
        std::printf("%-8s", [&zs]() {
            std::string t;
            for (const auto z : zs) {
              t += std::to_string(z) + ",";
            }
            return t.empty() ? std::string(".") : t;
          }().c_str());
      }
      std::printf("   y=%.0f\n", y);
    }
  }

  {
    // connectivity of the source mesh itself (shared edges, vertices welded to a millimetre)
    std::map<std::array<std::int64_t, 3>, std::uint32_t> weld;
    std::vector<std::uint32_t> id(nm.positions.x.size());
    for (std::size_t v = 0; v < id.size(); ++v) {
      const std::array<std::int64_t, 3> k{std::lround(nm.positions.x[v] * 1000.0),
        std::lround(nm.positions.y[v] * 1000.0), std::lround(nm.positions.z[v] * 1000.0)};
      id[v] = weld.emplace(k, static_cast<std::uint32_t>(weld.size())).first->second;
    }
    const std::size_t nt = nm.navcels.size();
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edges;
    for (std::size_t t = 0; t < nt; ++t) {
      for (int k = 0; k < 3; ++k) {
        auto a = id[nm.navcels[t].v[k]], b = id[nm.navcels[t].v[(k + 1) % 3]];
        if (a > b) {std::swap(a, b);}
        edges[{a, b}].push_back(static_cast<std::uint32_t>(t));
      }
    }
    std::vector<int> comp(nt, -1);
    std::vector<std::size_t> csize;
    std::vector<std::set<int>> cz;
    for (std::size_t t0 = 0; t0 < nt; ++t0) {
      if (comp[t0] >= 0) {continue;}
      const int cid = static_cast<int>(csize.size());
      csize.push_back(0);
      cz.emplace_back();
      std::vector<std::size_t> st{t0};
      comp[t0] = cid;
      while (!st.empty()) {
        const auto t = st.back();
        st.pop_back();
        ++csize[cid];
        cz[cid].insert(static_cast<int>(std::lround(nm.positions.z[nm.navcels[t].v[0]])));
        for (int k = 0; k < 3; ++k) {
          auto a = id[nm.navcels[t].v[k]], b = id[nm.navcels[t].v[(k + 1) % 3]];
          if (a > b) {std::swap(a, b);}
          for (const auto n : edges[{a, b}]) {
            if (comp[n] < 0) {comp[n] = cid; st.push_back(n);}
          }
        }
      }
    }
    std::printf("[source topology] %zu components; the 6 largest:", csize.size());
    std::vector<std::size_t> order(csize.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
      order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](auto a, auto b) {return csize[a] > csize[b];});
    for (std::size_t i = 0; i < std::min<std::size_t>(6, order.size()); ++i) {
      std::printf(" %zu(z:", csize[order[i]]);
      for (const auto z : cz[order[i]]) {
        std::printf("%d,", z);
      }
      std::printf(")");
    }
    std::printf("\n");
    // the component under the origin at floor 0
    int best = -1;
    double bd = 1e9;
    for (std::size_t t = 0; t < nt; ++t) {
      const auto v = nm.navcels[t].v[0];
      const double d = std::hypot(nm.positions.x[v], nm.positions.y[v]) +
        3.0 * std::abs(nm.positions.z[v]);
      if (d < bd) {bd = d; best = comp[t];}
    }
    std::printf(
      "[source topology] the component at (0, 0, 0): %zu NavCels, floors z:", csize[best]);
    for (const auto z : cz[best]) {
      std::printf(" %d", z);
    }
    std::printf("\n");
  }

  {
    std::vector<std::array<std::uint32_t, 3>> tris;
    for (const auto & c : nm.navcels) {
      tris.push_back({c.v[0], c.v[1], c.v[2]});
    }
    for (const double off : {0.0, 0.5, 0.7}) {
      char name[96];
      std::snprintf(name, sizeof(name), "source, slope <= 30 deg, %.1f m off the borders", off);
      print_heights(
        name, flood(nm.positions.x, nm.positions.y, nm.positions.z, tris, 30.0, 0, 0, 0, off));
    }
  }

  {
    auto copy = nm;
    print_heights("source, drivable by a 0.7 m robot", drivable(copy, 0.7, 30.0));
  }

  using clock = std::chrono::steady_clock;
  auto t0 = clock::now();
  bigmap_maps_manager::SurfaceLodMesher::Config cfg;
  if (const char * ms = std::getenv("BIGMAP_MIN_SIZE")) {cfg.min_size = std::atof(ms);}
  if (const char * kn = std::getenv("BIGMAP_NARROW")) {cfg.keep_narrow_width = std::atof(kn);}
  bigmap_maps_manager::SurfaceLodMesher mesher(nm, cfg);
  std::printf(
    "[build] %.1f ms: %zu welded vertices, %zu triangles, %d levels\n",
    std::chrono::duration<double, std::milli>(clock::now() - t0).count(), mesher.vertex_count(),
    mesher.source_triangle_count(), mesher.level_count());
  bigmap_maps_manager::LodParams p;
  p.high_res_radius = 10; p.low_res_distance = 30; p.min_size = cfg.min_size; p.max_size = 32;
  if (const char * hr = std::getenv("BIGMAP_HIGH_RES")) {p.high_res_radius = std::atof(hr);}
  p.known_margin = 1; p.known_falloff = 5;
  const float xs[] = {0.5f * (xmin + xmax), xmin, xmax};
  for (const float x : xs) {
    t0 = clock::now();
    const auto m = mesher.extract(x, 0.5f * (ymin + ymax), zmin, p);
    std::printf(
      "[extract] robot x=%.1f: %zu triangles, %zu vertices, %.1f ms\n", x, m.triangle_count(),
      m.x.size(), std::chrono::duration<double, std::milli>(clock::now() - t0).count());
    {
      // what BigMapMapsManager::build_navmap does with the mesh
      t0 = clock::now();
      ::navmap::NavMap out;
      out.positions.x = m.x; out.positions.y = m.y; out.positions.z = m.z;
      out.navcels.resize(m.triangle_count());
      out.surfaces.resize(1);
      out.surfaces[0].navcels.resize(m.triangle_count());
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        for (int k = 0; k < 3; ++k) {
          out.navcels[t].v[k] = m.tri[3 * t + k];
        }
        out.surfaces[0].navcels[t] = static_cast<::navmap::NavCelId>(t);
      }
      const auto t1 = clock::now();
      out.rebuild_geometry_accels();
      {
        const auto t2 = clock::now();
        out.add_layer<std::uint8_t>("occupancy", "", "", ::navmap::FREE_SPACE);
        out.add_layer<std::uint8_t>("obstacles", "", "", ::navmap::FREE_SPACE);
        out.add_layer<float>("elevation", "", "m");
        out.add_layer<float>("lod_size", "", "m");
        out.add_layer<std::uint8_t>("traversed", "", "", 0);
        for (std::size_t t = 0; t < m.triangle_count(); ++t) {
          const auto id = static_cast<::navmap::NavCelId>(t);
          out.layer_set<float>("elevation", id, 1.0f);
          out.layer_set<float>("lod_size", id, m.leg[t]);
          if (t % 7 == 0) {out.layer_set<std::uint8_t>("traversed", id, 255);}
        }
        std::printf(
          "          layers (add + set per NavCel): %.1f ms\n",
          std::chrono::duration<double, std::milli>(clock::now() - t2).count());
      }
      std::printf(
        "          rebuild_geometry_accels: %.1f ms\n",
        std::chrono::duration<double, std::milli>(clock::now() - t1).count());
      (void)t0;
    }
    {
      // topology: components through shared edges, and edges shared by more than two triangles
      const std::size_t nt = m.triangle_count();
      std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edges;
      for (std::size_t t = 0; t < nt; ++t) {
        for (int k = 0; k < 3; ++k) {
          auto a = m.tri[3 * t + k], b = m.tri[3 * t + (k + 1) % 3];
          if (a > b) {std::swap(a, b);}
          edges[{a, b}].push_back(static_cast<std::uint32_t>(t));
        }
      }
      std::size_t nonmanifold = 0, border = 0;
      for (const auto & e : edges) {
        nonmanifold += e.second.size() > 2;
        border += e.second.size() == 1;
      }
      std::vector<int> comp(nt, -1);
      std::vector<std::size_t> sizes_c;
      for (std::size_t t0 = 0; t0 < nt; ++t0) {
        if (comp[t0] >= 0) {continue;}
        const int id = static_cast<int>(sizes_c.size());
        sizes_c.push_back(0);
        std::vector<std::size_t> st{t0};
        comp[t0] = id;
        while (!st.empty()) {
          const auto t = st.back();
          st.pop_back();
          ++sizes_c[id];
          for (int k = 0; k < 3; ++k) {
            auto a = m.tri[3 * t + k], b = m.tri[3 * t + (k + 1) % 3];
            if (a > b) {std::swap(a, b);}
            for (const auto n : edges[{a, b}]) {
              if (comp[n] < 0) {comp[n] = id; st.push_back(n);}
            }
          }
        }
      }
      std::sort(sizes_c.rbegin(), sizes_c.rend());
      std::printf(
        "          topology: %zu components (largest %zu, 2nd %zu), %zu non-manifold edges, "
        "%zu border edges\n", sizes_c.size(), sizes_c[0], sizes_c.size() > 1 ? sizes_c[1] : 0,
        nonmanifold, border);
    }
    {
      std::vector<std::array<std::uint32_t, 3>> tris;
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        tris.push_back({m.tri[3 * t], m.tri[3 * t + 1], m.tri[3 * t + 2]});
      }
      for (const double off : {0.0, 0.5, 0.7}) {
        char name[96];
        std::snprintf(
          name, sizeof(name), "LOD mesh (robot x=%.1f), slope <= 30, %.1f m off borders", x, off);
        print_heights(name, flood(m.x, m.y, m.z, tris, 90.0, 0, 0, 0, off));
      }
    }
    {
      std::size_t bins[8] = {0}, flipped = 0;  // slope <10/20/30/40/50/60/>=60 degrees
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], d = m.tri[3 * t + 2];
        const double ux = m.x[b] - m.x[a], uy = m.y[b] - m.y[a], uz = m.z[b] - m.z[a];
        const double vx = m.x[d] - m.x[a], vy = m.y[d] - m.y[a], vz = m.z[d] - m.z[a];
        const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-12) {++bins[7]; continue;}
        flipped += nz < 0;
        ++bins[std::min<std::size_t>(
            6, static_cast<std::size_t>(
              std::acos(std::min(1.0, std::abs(nz) / len)) * 18.0 / M_PI))];
      }
      std::printf("          slope <10/20/30/40/50/60/>=60/degenerate:");
      for (int k = 0; k < 8; ++k) {
        std::printf(" %zu", bins[k]);
      }
      std::printf("; facing down: %zu\n", flipped);
    }
    {
      ::navmap::NavMap out;
      out.positions.x = m.x; out.positions.y = m.y; out.positions.z = m.z;
      out.navcels.resize(m.triangle_count());
      for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        for (int k = 0; k < 3; ++k) {
          out.navcels[t].v[k] = m.tri[3 * t + k];
        }
      }
      for (const double ins : {0.0, 0.3, 0.5, 0.7}) {
        char name[96];
        std::snprintf(name, sizeof(name), "LOD mesh, %.1f m from the borders", ins);
        print_heights(name, drivable(out, ins, 90.0, &m.slope));
      }
      std::printf("          non-manifold edges (open for the planner): %zu\n",
        out.nonmanifold_edges());
    }
    std::size_t sizes[8] = {0};
    for (const auto l : m.leg) {
      std::size_t k = 0;
      for (float s = 1.0f; s < l - 0.01f && k < 7; s *= 2.0f) {
        ++k;
      }
      ++sizes[k];
    }
    std::printf("          leg 1/2/4/8/16/32/64:");
    for (int k = 0; k < 7; ++k) {
      std::printf(" %zu", sizes[k]);
    }
    std::printf("\n");
  }
}

// ---------------------------------------------------------------------------------------------
// BIGMAP_TEST_NAVMAP=<file> BIGMAP_COMPARE=1: the two engines on the same high-resolution map,
// against
// the map itself: height error and obstacles (slope > 25 deg) by distance from the robot.
//   grid      : the map rasterised to a 1 m height grid, RTIN (what a DEM source or a plain
//               initial_navmap gets); a NavCel is an obstacle if its own plane is steeper
//   hierarchy : SurfaceLodMesher on the NavMap's own triangles; an obstacle if the steepest source
//               NavCel it stands for is steeper
namespace
{
struct Raster
{
  int n; double res;
  std::vector<float> z; std::vector<std::int8_t> lethal;  // lethal: -1 uncovered
};

Raster rasterise(
  const bigmap_maps_manager::LodMesh & m, const std::vector<char> & leth, int n,
  double res)
{
  Raster r{n, res, std::vector<float>(static_cast<std::size_t>(n) * n, -1e9f),
    std::vector<std::int8_t>(static_cast<std::size_t>(n) * n, -1)};
  const double c = (n - 1) / 2.0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], d = m.tri[3 * t + 2];
    const double x0 = m.x[a], y0 = m.y[a], x1 = m.x[b], y1 = m.y[b], x2 = m.x[d], y2 = m.y[d];
    const double det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
    if (std::abs(det) < 1e-9) {continue;}
    const int i0 = std::max(0,
        static_cast<int>(std::ceil(std::min({x0, x1, x2}) / res + c - 1e-9)));
    const int i1 = std::min(n - 1,
        static_cast<int>(std::floor(std::max({x0, x1, x2}) / res + c + 1e-9)));
    const int j0 = std::max(0,
        static_cast<int>(std::ceil(std::min({y0, y1, y2}) / res + c - 1e-9)));
    const int j1 = std::min(n - 1,
        static_cast<int>(std::floor(std::max({y0, y1, y2}) / res + c + 1e-9)));
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        const double px = (i - c) * res, py = (j - c) * res;
        const double l0 = ((y1 - y2) * (px - x2) + (x2 - x1) * (py - y2)) / det;
        const double l1 = ((y2 - y0) * (px - x2) + (x0 - x2) * (py - y2)) / det;
        const double l2 = 1.0 - l0 - l1;
        if (l0 < -1e-9 || l1 < -1e-9 || l2 < -1e-9) {continue;}
        const float z = static_cast<float>(l0 * m.z[a] + l1 * m.z[b] + l2 * m.z[d]);
        const std::size_t k = static_cast<std::size_t>(j) * n + i;
        if (z > r.z[k]) {r.z[k] = z; r.lethal[k] = leth[t] ? 1 : 0;}
      }
    }
  }
  return r;
}
}  // namespace

TEST(RealMap, EngineComparison)
{
  const char * path = std::getenv("BIGMAP_TEST_NAVMAP");
  if (!path || !std::getenv("BIGMAP_COMPARE")) {
    GTEST_SKIP() << "BIGMAP_TEST_NAVMAP + BIGMAP_COMPARE";
  }
  ::navmap::NavMap nm;
  std::error_code ec;
  ASSERT_TRUE(navmap_ros::io::load_from_file(path, nm, &ec)) << ec.message();
  const double max_slope = 25.0, hi_res = 0.5;
  // the map itself, as a 0.5 m grid: the reference
  const auto src = bigmap_maps_manager::dem_from_navmap(nm, hi_res);
  const int n = src.dem.side;
  const double c = (n - 1) / 2.0;
  std::vector<std::int8_t> src_leth(static_cast<std::size_t>(n) * n, 0);
  for (int j = 0; j + 1 < n; ++j) {
    for (int i = 0; i + 1 < n; ++i) {
      const double zx = (src.dem.at(i + 1, j) - src.dem.at(i, j)) / hi_res;
      const double zy = (src.dem.at(i, j + 1) - src.dem.at(i, j)) / hi_res;
      src_leth[static_cast<std::size_t>(j) * n + i] =
        std::atan(std::hypot(zx, zy)) * 180.0 / M_PI > max_slope;
    }
  }
  bigmap_maps_manager::LodParams p;
  p.high_res_radius = 20; p.low_res_distance = 80; p.min_size = 1.0; p.max_size = 32;
  p.base_error = 0.02; p.error_growth = 0.05; p.known_margin = 1; p.known_falloff = 5;
  const double rx = 0.0, ry = 0.0;
  const double rz = src.dem.height_at(rx, ry);

  using clock = std::chrono::steady_clock;
  struct Result {const char * name; Raster r; std::size_t tris; double build_ms, extract_ms;};
  std::vector<Result> results;
  {  // grid engine: what the manager does with a plain initial_navmap (1 m grid, RTIN)
    auto t0 = clock::now();
    auto nd = bigmap_maps_manager::dem_from_navmap(nm, p.min_size);
    double narrow = 0.0;
    if (const char * kn = std::getenv("BIGMAP_NARROW")) {narrow = std::atof(kn);}
    const auto pinned = bigmap_maps_manager::narrow_passage_nodes(nd.dem, nd.valid, max_slope,
      narrow);
    std::printf("grid engine: %zu of %zu nodes narrow (width %.1f)\n",
      static_cast<std::size_t>(std::count(pinned.begin(), pinned.end(), 1)), nd.dem.z.size(),
      narrow);
    bigmap_maps_manager::LodMesher mesher(nd.dem, nd.valid, pinned);
    const double build = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    t0 = clock::now();
    const auto m = mesher.extract(rx, ry, p, nullptr);
    const double ex = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    const float min_nz = static_cast<float>(std::cos(max_slope * M_PI / 180.0));
    std::vector<char> leth(m.triangle_count());
    for (std::size_t t = 0; t < leth.size(); ++t) {
      const auto a = m.tri[3 * t], b = m.tri[3 * t + 1], d = m.tri[3 * t + 2];
      const double ux = m.x[b] - m.x[a], uy = m.y[b] - m.y[a], uz = m.z[b] - m.z[a];
      const double vx = m.x[d] - m.x[a], vy = m.y[d] - m.y[a], vz = m.z[d] - m.z[a];
      const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      leth[t] = nz / std::sqrt(nx * nx + ny * ny + nz * nz) < min_nz;
    }
    results.push_back({"grid (RTIN, 1 m DEM)", rasterise(m, leth, n, hi_res), m.triangle_count(),
        build, ex});
  }
  {
    auto t0 = clock::now();
    bigmap_maps_manager::SurfaceLodMesher::Config cfg;
    cfg.min_size = 1.0; cfg.max_size = 32.0;
    bigmap_maps_manager::SurfaceLodMesher mesher(nm, cfg);
    const double build = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    t0 = clock::now();
    const auto m = mesher.extract(rx, ry, rz, p);
    const double ex = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    std::vector<char> leth(m.triangle_count());
    for (std::size_t t = 0; t < leth.size(); ++t) {
      leth[t] = m.slope[t] > max_slope;
    }
    results.push_back({"hierarchy (NavMap triangles)", rasterise(m, leth, n, hi_res),
        m.triangle_count(), build, ex});
  }
  const double bands[] = {0, 20, 50, 100, 1e9};
  for (const auto & res : results) {
    std::printf("[compare] %s: %zu triangles, build %.0f ms, extract %.1f ms\n", res.name, res.tris,
      res.build_ms, res.extract_ms);
    for (int bnd = 0; bnd < 4; ++bnd) {
      std::vector<float> err;
      std::size_t cells = 0, holes = 0, s_l = 0, both = 0, l_only = 0, l_total = 0, free_cells = 0;
      for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
          const double d = std::hypot((i - c) * hi_res - rx, (j - c) * hi_res - ry);
          if (d < bands[bnd] || d >= bands[bnd + 1]) {continue;}
          const std::size_t k = static_cast<std::size_t>(j) * n + i;
          if (!src.valid[k]) {continue;}
          ++cells;
          if (res.r.lethal[k] < 0) {++holes; continue;}
          err.push_back(std::abs(res.r.z[k] - src.dem.z[k]));
          const bool sl = src_leth[k], ll = res.r.lethal[k] == 1;
          s_l += sl; l_total += ll;
          if (sl && ll) {++both;}
          if (!sl) {++free_cells; if (ll) {++l_only;}}
        }
      }
      std::sort(err.begin(), err.end());
      auto pc = [&](double q) {
          return err.empty() ? 0.0f : err[static_cast<std::size_t>(q * (err.size() - 1))];
        };
      std::printf(
        "          %3.0f-%3.0f m: %7zu cells, holes %.2f%%, |dz| median %.2f p95 %.2f max %.2f m; "
        "obstacles: source %.1f%% of cells, kept %.1f%%, false %.1f%% of the free ones\n",
        bands[bnd], std::min(bands[bnd + 1], 400.0), cells,
        100.0 * holes / std::max<std::size_t>(cells, 1),
        pc(0.5), pc(0.95), err.empty() ? 0.0f : err.back(),
        100.0 * s_l / std::max<std::size_t>(cells - holes, 1),
        100.0 * both / std::max<std::size_t>(s_l, 1),
        100.0 * l_only / std::max<std::size_t>(free_cells, 1));
    }
  }
}
