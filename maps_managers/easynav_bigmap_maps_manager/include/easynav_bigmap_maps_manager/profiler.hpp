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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__PROFILER_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__PROFILER_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace bigmap_maps_manager
{

/// Accumulates the time of named sections and reports them every `period` seconds (wall clock).
class Profiler
{
public:
  using Clock = std::chrono::steady_clock;

  explicit Profiler(double period_s = 5.0)
  : period_(period_s), window_start_(Clock::now()) {}

  void add(const std::string & section, double ms)
  {
    auto & s = stats_[section];
    s.sum += ms;
    s.max = std::max(s.max, ms);
    ++s.calls;
  }

  double period() const {return period_;}

  /// True once `period` seconds have passed since the last report.
  bool due() const
  {
    return std::chrono::duration<double>(Clock::now() - window_start_).count() >= period_;
  }

  /// A table of the window (mean and worst ms per call, calls, and ms spent per second of wall
  /// time) sorted by the time each section takes per second; then starts a new window.
  std::string report_and_reset(const std::string & title)
  {
    const double window = std::chrono::duration<double>(Clock::now() - window_start_).count();
    struct Row {std::string name; double mean, max, per_s; std::int64_t calls;};
    std::vector<Row> rows;
    for (const auto & [name, s] : stats_) {
      rows.push_back({name, s.sum / std::max(1L, s.calls), s.max, s.sum / std::max(1e-9, window),
          s.calls});
    }
    std::sort(rows.begin(), rows.end(), [](const Row & a, const Row & b) {
        return a.per_s > b.per_s;
      });
    std::string out = title + " (window " + fmt("%.1f", window) + " s)\n";
    out += "    section                                  calls   mean ms    max ms   ms per s\n";
    for (const auto & r : rows) {
      char line[160];
      std::snprintf(
        line, sizeof(line), "    %-40s %5ld %9.2f %9.2f %10.1f\n", r.name.c_str(), r.calls, r.mean,
        r.max, r.per_s);
      out += line;
    }
    stats_.clear();
    window_start_ = Clock::now();
    return out;
  }

private:
  struct Stat
  {
    double sum{0.0}, max{0.0};
    std::int64_t calls{0};
  };
  static std::string fmt(const char * f, double v)
  {
    char b[32];
    std::snprintf(b, sizeof(b), f, v);
    return b;
  }
  double period_;
  Clock::time_point window_start_;
  std::map<std::string, Stat> stats_;
};

/// Times its own lifetime into a Profiler; does nothing for a null profiler.
class ProfScope
{
public:
  ProfScope(Profiler * p, const char * name)
  : p_(p), name_(name), t0_(Profiler::Clock::now()) {}
  ~ProfScope()
  {
    if (p_) {
      p_->add(name_,
          std::chrono::duration<double, std::milli>(Profiler::Clock::now() - t0_).count());
    }
  }
  ProfScope(const ProfScope &) = delete;
  ProfScope & operator=(const ProfScope &) = delete;

private:
  Profiler * p_;
  const char * name_;
  Profiler::Clock::time_point t0_;
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__PROFILER_HPP_
