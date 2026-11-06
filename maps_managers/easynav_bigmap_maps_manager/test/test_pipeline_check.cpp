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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "easynav_bigmap_maps_manager/pipeline_check.hpp"

using bigmap_maps_manager::check_pipeline;
using bigmap_maps_manager::LoadedFilter;
using bigmap_maps_manager::ObstacleDetector;
using bigmap_maps_manager::parse_detector;
using bigmap_maps_manager::PipelineSettings;

namespace
{
const LoadedFilter kObstacleFilter{"obstacle_filter", "obstacles"};
const LoadedFilter kInflation{"inflation", "inflated_obstacles"};

bool mentions(const std::vector<std::string> & v, const std::string & text)
{
  for (const auto & s : v) {
    if (s.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}
}  // namespace

TEST(PipelineCheck, DetectorNamesAndTheOldOnes)
{
  EXPECT_EQ(parse_detector("bigmap_dynamic"), ObstacleDetector::BigmapDynamic);
  EXPECT_EQ(parse_detector("dynamic"), ObstacleDetector::BigmapDynamic);
  EXPECT_EQ(parse_detector("navmap_filter"), ObstacleDetector::NavmapFilter);
  EXPECT_EQ(parse_detector("filter"), ObstacleDetector::NavmapFilter);
  EXPECT_THROW(parse_detector("both"), std::invalid_argument);
}

TEST(PipelineCheck, TheDefaultSetupIsClean)
{
  PipelineSettings s;
  s.filters = {kInflation};
  const auto r = check_pipeline(s);
  EXPECT_TRUE(r.errors.empty());
  EXPECT_TRUE(r.warnings.empty());
  EXPECT_TRUE(r.sensors_processed);
  EXPECT_TRUE(r.memory_effective);
  EXPECT_EQ(r.planner_layer, "inflated_obstacles");
}

TEST(PipelineCheck, InflationIsIndependentOfTheDetector)
{
  for (const auto detector : {ObstacleDetector::BigmapDynamic, ObstacleDetector::NavmapFilter}) {
    PipelineSettings s;
    s.detector = detector;
    s.memory_enabled = false;
    if (detector == ObstacleDetector::NavmapFilter) {s.filters = {kObstacleFilter};}
    EXPECT_EQ(check_pipeline(s).planner_layer, "obstacles");   // not listed: not inflated
    s.filters.push_back(kInflation);
    const auto r = check_pipeline(s);
    EXPECT_EQ(r.planner_layer, "inflated_obstacles");
    EXPECT_TRUE(r.errors.empty());
  }
}

TEST(PipelineCheck, TheFilterDetectorNeedsAnObstacleFilter)
{
  PipelineSettings s;
  s.detector = ObstacleDetector::NavmapFilter;
  s.memory_enabled = false;
  s.filters = {kInflation};
  EXPECT_TRUE(mentions(check_pipeline(s).errors, "no obstacle filter"));
  s.filters = {kObstacleFilter, kInflation};
  const auto r = check_pipeline(s);
  EXPECT_TRUE(r.errors.empty());
  EXPECT_TRUE(r.warnings.empty());
  EXPECT_FALSE(r.sensors_processed);   // the filter analyses them, not bigmap
}

TEST(PipelineCheck, TwoDetectorsAtOnceAreAWarning)
{
  PipelineSettings s;
  s.filters = {kObstacleFilter, kInflation};
  const auto r = check_pipeline(s);
  EXPECT_TRUE(r.errors.empty());
  EXPECT_TRUE(mentions(r.warnings, "both write"));
}

TEST(PipelineCheck, MemoryNeedsTheBigmapDetector)
{
  PipelineSettings s;
  s.detector = ObstacleDetector::NavmapFilter;
  s.memory_enabled = true;
  s.filters = {kObstacleFilter, kInflation};
  const auto r = check_pipeline(s);
  EXPECT_FALSE(r.memory_effective);
  EXPECT_TRUE(mentions(r.warnings, "memory is off"));
}

TEST(PipelineCheck, SensorsAreAnalysedForTheDynamicMapOrForTheMemory)
{
  PipelineSettings s;
  s.filters = {kInflation};
  s.dynamic_map_enabled = false;
  s.memory_enabled = false;
  auto r = check_pipeline(s);
  EXPECT_FALSE(r.sensors_processed);
  EXPECT_TRUE(mentions(r.notes, "no sensor obstacles"));
  s.dynamic_map_enabled = true;
  EXPECT_TRUE(check_pipeline(s).sensors_processed);
  s.dynamic_map_enabled = false;
  s.memory_enabled = true;
  EXPECT_TRUE(check_pipeline(s).sensors_processed);
}

TEST(PipelineCheck, DetectionsNeedRemovalToReachTheObstaclesLayer)
{
  PipelineSettings s;
  s.filters = {kInflation};
  s.removal_enabled = false;
  EXPECT_TRUE(mentions(check_pipeline(s).warnings, "planner does not see it"));
  s.memory_enabled = false;   // nothing analysed: nothing to lose
  EXPECT_TRUE(check_pipeline(s).warnings.empty());
}

TEST(PipelineCheck, TheInflationListedBeforeTheObstacleFilterIsAWarning)
{
  PipelineSettings s;
  s.detector = ObstacleDetector::NavmapFilter;
  s.memory_enabled = false;
  s.filters = {kInflation, kObstacleFilter};
  EXPECT_TRUE(mentions(check_pipeline(s).warnings, "previous cycle"));
  s.filters = {kObstacleFilter, kInflation};
  EXPECT_TRUE(check_pipeline(s).warnings.empty());
}
