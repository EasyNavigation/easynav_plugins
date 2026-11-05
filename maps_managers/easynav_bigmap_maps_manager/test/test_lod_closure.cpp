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

// The argument for a mesh without T-junctions (paper, Section 3.2), checked exhaustively on the
// whole implicit RTIN tree of random terrains, with random robots, driven-over ground, observed
// borders, holes and pinned (narrow passage) nodes:
//   (a)+(b) the split decision is closed upwards: split(child) => split(parent), for every pair
//           (this needs the nested error and kReach >= (1 + sqrt 2) / 2);
//   the stored error is nested: e(child) <= e(parent), pinned(child) => pinned(parent);
//   (c) the meshes extract() returns have no T-junctions.
// And what the error is NOT: the vertical deviation of a coarse triangle can exceed the stored
// error, so the tolerance bounds the hierarchical surplus, not the deviation.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "easynav_bigmap_maps_manager/known_map.hpp"
#include "easynav_bigmap_maps_manager/lod_mesher.hpp"
#include "easynav_bigmap_maps_manager/lod_policy.hpp"

using namespace bigmap_maps_manager;  // NOLINT(build/namespaces)

namespace
{

DemGrid random_dem(int side, double res, std::mt19937 & rng, int kind)
{
  DemGrid g;
  g.side = side;
  g.resolution = res;
  g.radius = (side - 1) / 2 * res;
  g.z.resize(static_cast<std::size_t>(side) * side);
  std::normal_distribution<double> n(0.0, 1.0);
  const double a1 = n(rng), a2 = n(rng);
  const double p1 = 5 + 20 * std::abs(n(rng)), p2 = 2 + 5 * std::abs(n(rng));
  for (int j = 0; j < side; ++j) {
    for (int i = 0; i < side; ++i) {
      const double x = g.x(i), y = g.y(j);
      double z = 0.0;
      if (kind == 0) {          // smooth relief and noise
        z = 3.0 * a1 * std::sin(x / p1) * std::cos(y / p1) +
          0.5 * a2 * std::sin(x / p2 + y / (p2 + 1)) + 0.05 * n(rng);
      } else if (kind == 1) {   // steps (raised blocks)
        z = ((static_cast<int>(std::floor(x / 7.0)) + static_cast<int>(std::floor(y / 5.0))) % 3 ==
          0) ? 1.5 : 0.0;
      } else {                  // white noise
        z = 0.3 * n(rng);
      }
      g.z[static_cast<std::size_t>(j) * side + i] = static_cast<float>(z);
    }
  }
  return g;
}

// The triangles of the implicit RTIN tree, with the vertices Extractor uses (c: right angle)
struct Tri
{
  int ax, ay, bx, by, cx, cy;
  int parent;
};

void enumerate(std::vector<Tri> & out, int ax, int ay, int bx, int by, int cx, int cy, int parent)
{
  const int me = static_cast<int>(out.size());
  out.push_back({ax, ay, bx, by, cx, cy, parent});
  if (std::abs(ax - cx) + std::abs(ay - cy) <= 1) {return;}
  const int mx = (ax + bx) >> 1, my = (ay + by) >> 1;
  enumerate(out, cx, cy, ax, ay, mx, my, me);
  enumerate(out, bx, by, cx, cy, mx, my, me);
}

bool splittable(const Tri & t) {return std::abs(t.ax - t.cx) + std::abs(t.ay - t.cy) > 1;}

// The decision of Extractor for one triangle (the same effective distances)
struct Decision
{
  const LodMesher & m;
  const LodParams & p;
  LodParams po;
  double rx, ry;
  const KnownMap * known;
  const KnownMap * observed;

  bool operator()(const Tri & t) const
  {
    if (!splittable(t)) {return false;}
    const DemGrid & dem = m.dem();
    const int mx = (t.ax + t.bx) >> 1, my = (t.ay + t.by) >> 1;
    const double leg = std::hypot(t.ax - t.cx, t.ay - t.cy) * dem.resolution;
    const double hyp = leg * M_SQRT2;
    const int center = (dem.side - 1) / 2;
    const double dist = std::hypot((mx - center) * dem.resolution - rx,
        (my - center) * dem.resolution - ry);
    double d = effective_distance(dist, hyp);
    if (known && !known->empty()) {
      d = std::min(d, known_effective_distance(p, known->distance(mx, my), hyp));
    }
    if (observed && !observed->empty()) {
      d = std::min(d, known_effective_distance(po, observed->distance(mx, my), hyp));
    }
    const auto & em = m.error_map();
    return em.pinned(mx, my) || should_split_at(p, em.error(mx, my), leg, d);
  }
};

// Edges used by one triangle only with a mesh vertex strictly inside: a T-junction
std::size_t t_junctions(const LodMesh & m, double res)
{
  using Edge = std::pair<uint32_t, uint32_t>;
  std::map<Edge, int> use;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    for (int k = 0; k < 3; ++k) {
      const uint32_t a = m.tri[3 * t + k], b = m.tri[3 * t + (k + 1) % 3];
      ++use[{std::min(a, b), std::max(a, b)}];
    }
  }
  std::set<std::pair<long, long>> nodes;   // NOLINT(runtime/int)
  auto node = [&](uint32_t v) {
      return std::make_pair(std::lround(m.x[v] / res), std::lround(m.y[v] / res));
    };
  for (uint32_t v = 0; v < m.x.size(); ++v) {
    nodes.insert(node(v));
  }
  std::size_t bad = 0;
  for (const auto & [e, n] : use) {
    if (n != 1) {continue;}
    const auto a = node(e.first), b = node(e.second);
    const long dx = b.first - a.first, dy = b.second - a.second;   // NOLINT(runtime/int)
    const long g = std::gcd(std::labs(dx), std::labs(dy));          // NOLINT(runtime/int)
    for (long k = 1; k < g; ++k) {                                  // NOLINT(runtime/int)
      if (nodes.count({a.first + dx / g * k, a.second + dy / g * k})) {++bad;}
    }
  }
  return bad;
}

// The warehouse configuration (Section 4.2), scaled to the resolution of the test DEM
LodParams warehouse_params(double res)
{
  LodParams p;
  p.min_size = res;
  p.max_size = 64 * res;
  p.high_res_radius = 30 * res;
  p.low_res_distance = 120 * res;
  p.base_error = 0.02;
  p.error_growth = 0.05;
  return p;
}

struct Totals
{
  long nest_pairs = 0, nest_bad = 0, pinned_bad = 0;            // NOLINT(runtime/int)
  long decision_pairs = 0, decision_bad = 0;                    // NOLINT(runtime/int)
  long meshes = 0, meshes_with_t = 0;                           // NOLINT(runtime/int)
  double worst_deviation_ratio = 0.0;   // vertical deviation / stored error (error > 5 cm)
};

// 36 terrains (6 sizes, 2 resolutions, 3 kinds, with and without holes, hole_error and pinned
// nodes), 12 robots and memories each
const Totals & exhaustive()
{
  static Totals r;
  static bool done = false;
  if (done) {return r;}
  done = true;
  std::mt19937 rng(20261026);
  for (int trial = 0; trial < 36; ++trial) {
    const int sides[] = {33, 65, 129, 50, 100, 97};
    const int side = sides[trial % 6];
    const double res = (trial % 2) ? 0.5 : 1.0;
    DemGrid dem = random_dem(side, res, rng, trial % 3);
    std::vector<std::uint8_t> cell_valid, pinned;
    std::bernoulli_distribution hole(0.04), pin(0.003);
    const bool with_holes = trial % 4 >= 2;
    if (with_holes) {
      cell_valid.assign(static_cast<std::size_t>(side - 1) * (side - 1), 1);
      for (auto & c : cell_valid) {
        c = hole(rng) ? 0 : 1;
      }
    }
    if (trial % 5 == 0) {
      pinned.assign(static_cast<std::size_t>(side) * side, 0);
      for (auto & q : pinned) {
        q = pin(rng) ? 1 : 0;
      }
    }
    const double hole_error = with_holes && (trial % 3 == 0) ? 0.5 : 0.0;
    LodMesher mesher(dem, {}, pinned, cell_valid, hole_error);
    const auto & em = mesher.error_map();
    const int ts = em.tile_size(), last = side - 1;
    std::vector<Tri> T;
    enumerate(T, 0, 0, ts, ts, ts, 0, -1);
    enumerate(T, ts, ts, 0, 0, 0, ts, -1);
    for (const auto & t : T) {   // the stored error and the pinned flag are nested
      if (t.parent < 0 || !splittable(t)) {continue;}
      const Tri & P = T[t.parent];
      const int mx = (t.ax + t.bx) >> 1, my = (t.ay + t.by) >> 1;
      const int px = (P.ax + P.bx) >> 1, py = (P.ay + P.by) >> 1;
      ++r.nest_pairs;
      r.nest_bad += em.error(mx, my) > em.error(px, py);
      r.pinned_bad += em.pinned(mx, my) && !em.pinned(px, py);
    }
    for (const auto & t : T) {   // vertical deviation of the triangle against its stored error
      if (!splittable(t)) {continue;}
      if (std::max({t.ax, t.bx, t.cx}) > last || std::max({t.ay, t.by, t.cy}) > last) {continue;}
      const int mx = (t.ax + t.bx) >> 1, my = (t.ay + t.by) >> 1;
      const double e = em.error(mx, my);
      if (e <= 0.05) {continue;}
      const double x0 = t.ax, y0 = t.ay, x1 = t.bx, y1 = t.by, x2 = t.cx, y2 = t.cy;
      const double z0 = em.height(t.ax, t.ay), z1 = em.height(t.bx, t.by),
        z2 = em.height(t.cx, t.cy);
      const double det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
      double dev = 0.0;
      for (int j = std::min({t.ay, t.by, t.cy}); j <= std::max({t.ay, t.by, t.cy}); ++j) {
        for (int i = std::min({t.ax, t.bx, t.cx}); i <= std::max({t.ax, t.bx, t.cx}); ++i) {
          const double l0 = ((y1 - y2) * (i - x2) + (x2 - x1) * (j - y2)) / det;
          const double l1 = ((y2 - y0) * (i - x2) + (x0 - x2) * (j - y2)) / det;
          const double l2 = 1.0 - l0 - l1;
          if (l0 < -1e-9 || l1 < -1e-9 || l2 < -1e-9) {continue;}
          dev = std::max(dev, std::abs(l0 * z0 + l1 * z1 + l2 * z2 - em.height(i, j)));
        }
      }
      r.worst_deviation_ratio = std::max(r.worst_deviation_ratio, dev / e);
    }
    const double half = (side - 1) / 2 * res;
    std::uniform_real_distribution<double> U(-1.6 * half, 1.6 * half);
    LodParams p = warehouse_params(res);
    for (int q = 0; q < 12; ++q) {
      p.known_margin = (q % 3) * 2.0 * res;
      p.known_falloff = (q % 4 == 0) ? 1.0 : (q % 4 == 1 ? 3.0 : 40.0);
      p.observed_margin = (q % 2) ? -1.0 : 1.0;
      KnownMap known(dem.side, dem.resolution), observed(dem.side, dem.resolution);
      const bool use_known = q % 3 != 0, use_observed = q % 4 >= 2;
      if (use_known) {
        const double x0 = U(rng), y0 = U(rng), x1 = U(rng), y1 = U(rng);
        for (double s = 0; s <= 1.0; s += 0.01) {
          known.mark(x0 + s * (x1 - x0), y0 + s * (y1 - y0), 1.0 * res);
        }
        known.update();
      }
      if (use_observed) {
        std::uniform_int_distribution<int> I(0, side - 1);
        for (int k = 0; k < 40; ++k) {
          observed.mark_node(I(rng), I(rng));
        }
        observed.update();
      }
      const double rx = U(rng), ry = U(rng);
      const Decision decide{mesher, p, observed_params(p), rx, ry,
        use_known ? &known : nullptr, use_observed ? &observed : nullptr};
      std::vector<char> D(T.size());
      for (std::size_t i = 0; i < T.size(); ++i) {
        D[i] = decide(T[i]);
      }
      for (std::size_t i = 0; i < T.size(); ++i) {
        if (T[i].parent < 0) {continue;}
        ++r.decision_pairs;
        r.decision_bad += D[i] && !D[T[i].parent];
      }
      const auto m = mesher.extract(rx, ry, p, use_known ? &known : nullptr,
          use_observed ? &observed : nullptr);
      ++r.meshes;
      r.meshes_with_t += t_junctions(m, res) > 0;
    }
  }
  std::cout << "[closure] " << r.nest_pairs << " child-parent errors, " << r.decision_pairs <<
    " child-parent decisions, " << r.meshes <<
    " meshes; worst vertical deviation / stored error " <<
    r.worst_deviation_ratio << std::endl;
  return r;
}

}  // namespace

TEST(LodClosure, TheReachConstantNestsTheDiscsOfParentAndChild)
{
  // (a): split vertices h / (2 sqrt 2) apart, child hypotenuse h / sqrt 2. The worst child has its
  // vertex that much closer to the robot; d_eff(parent) <= d_eff(child) iff
  // k h >= h / (2 sqrt 2) + k h / sqrt 2, i.e. k >= (1 + sqrt 2) / 2
  EXPECT_GE(kReach, (1.0 + std::sqrt(2.0)) / 2.0);
  for (double h : {1.0, 8.0, 45.0}) {
    for (double d : {0.0, 3.0, 37.0, 500.0}) {
      const double child = std::max(0.0, d - h / (2.0 * std::sqrt(2.0)));
      EXPECT_LE(effective_distance(d, h), effective_distance(child, h / std::sqrt(2.0)) + 1e-12);
    }
  }
}

TEST(LodClosure, TheStoredErrorAndThePinnedFlagAreNested)
{
  const auto & r = exhaustive();
  EXPECT_GT(r.nest_pairs, 100000);
  EXPECT_EQ(r.nest_bad, 0);
  EXPECT_EQ(r.pinned_bad, 0);
}

TEST(LodClosure, SplittingACellAlwaysSplitsItsParent)
{
  const auto & r = exhaustive();
  EXPECT_GT(r.decision_pairs, 1000000);
  EXPECT_EQ(r.decision_bad, 0);
}

TEST(LodClosure, ExtractedMeshesHaveNoTJunctions)
{
  const auto & r = exhaustive();
  EXPECT_EQ(r.meshes, 432);
  EXPECT_EQ(r.meshes_with_t, 0);
}

// The tolerance bounds the hierarchical surplus, not the vertical deviation: on random terrain a
// coarse triangle deviates more than its stored error (the paper reports the worst ratio), and on
// a surface built with the same surplus E at every level the root triangle deviates several E.
TEST(LodClosure, TheStoredErrorIsTheSurplusNotTheDeviation)
{
  EXPECT_GT(exhaustive().worst_deviation_ratio, 1.0);
  const int side = 129;
  const double E = 0.25;
  DemGrid g;
  g.side = side;
  g.resolution = 1.0;
  g.radius = (side - 1) / 2;
  g.z.assign(static_cast<std::size_t>(side) * side, 0.0f);
  std::vector<char> set(g.z.size(), 0);
  auto at = [&](int x, int y) -> float & {return g.z[static_cast<std::size_t>(y) * side + x];};
  std::vector<Tri> T;
  enumerate(T, 0, 0, side - 1, side - 1, side - 1, 0, -1);
  enumerate(T, side - 1, side - 1, 0, 0, 0, side - 1, -1);
  std::stable_sort(T.begin(), T.end(), [](const Tri & a, const Tri & b) {
      return std::abs(a.ax - a.cx) + std::abs(a.ay - a.cy) >
             std::abs(b.ax - b.cx) + std::abs(b.ay - b.cy);
    });
  for (const auto & t : T) {   // coarse to fine: every midpoint E above its hypotenuse's mean
    if (!splittable(t)) {continue;}
    const int mx = (t.ax + t.bx) >> 1, my = (t.ay + t.by) >> 1;
    const std::size_t k = static_cast<std::size_t>(my) * side + mx;
    if (set[k]) {continue;}
    at(mx, my) = static_cast<float>((at(t.ax, t.ay) + at(t.bx, t.by)) / 2.0 + E);
    set[k] = 1;
  }
  LodMesher m(g);
  double stored = 0.0, deviation = 0.0;
  for (int j = 0; j < side; ++j) {
    for (int i = 0; i < side; ++i) {
      stored = std::max(stored, static_cast<double>(m.error_map().error(i, j)));
      if (i >= j) {deviation = std::max(deviation, static_cast<double>(at(i, j)));}
    }
  }
  EXPECT_NEAR(stored, E, 0.011);       // the stored error is E (rounded up to 1 cm)
  EXPECT_GT(deviation, 4.0 * E);       // the root triangle (z = 0) deviates far more
}
