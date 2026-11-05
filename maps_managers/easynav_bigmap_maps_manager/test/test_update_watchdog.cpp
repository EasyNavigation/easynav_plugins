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

#include <atomic>
#include <thread>

#include "easynav_bigmap_maps_manager/update_watchdog.hpp"

using bigmap_maps_manager::UpdateWatchdog;
using Event = UpdateWatchdog::Event;

TEST(UpdateWatchdog, NothingIsToldBeforeItStartsOrWhileUpdateRuns)
{
  UpdateWatchdog w(10.0, 30.0);
  EXPECT_EQ(w.check(1000.0), Event::None);     // not started: the plugin is not initialized yet
  w.start(0.0);
  for (double t = 1.0; t < 100.0; t += 1.0) {
    w.ran(t);
    EXPECT_EQ(w.check(t + 0.5), Event::None) << t;
  }
}

TEST(UpdateWatchdog, AClockThatNeverMovesIsAStallFromTheStart)
{
  // the case seen end to end: update() never runs at all
  UpdateWatchdog w(10.0, 30.0);
  w.start(100.0);
  EXPECT_EQ(w.check(105.0), Event::None);      // within the grace
  EXPECT_EQ(w.check(110.5), Event::Stalled);
  EXPECT_DOUBLE_EQ(w.stalled_s(), 10.5);
  EXPECT_EQ(w.check(115.0), Event::None);      // told once...
  EXPECT_EQ(w.check(140.5), Event::Stalled);   // ...and again every repeat_s while it goes on
  EXPECT_DOUBLE_EQ(w.stalled_s(), 40.5);
}

TEST(UpdateWatchdog, AStallThatEndsIsToldOnceAndThenItWatchesAgain)
{
  UpdateWatchdog w(10.0, 30.0);
  w.start(0.0);
  w.ran(5.0);
  EXPECT_EQ(w.check(20.0), Event::Stalled);
  w.ran(21.0);
  EXPECT_EQ(w.check(21.5), Event::Resumed);
  EXPECT_EQ(w.check(22.0), Event::None);
  // a second stall is a new one (told after the grace, not after repeat_s)
  EXPECT_EQ(w.check(31.5), Event::Stalled);
}

TEST(UpdateWatchdog, RunsFromAnotherThreadAreSeen)
{
  // update() and the wall timer run on different threads: the checker sees the latest run
  UpdateWatchdog w(0.5, 30.0);
  w.start(0.0);
  std::atomic<double> last{0.0};
  std::thread updater([&] {
      for (int i = 1; i <= 100000; ++i) {
        w.ran(i * 0.001);
        last.store(i * 0.001);
      }
    });
  while (last.load() < 50.0) {
    EXPECT_NE(w.check(last.load() + 0.1), Event::Stalled);
  }
  updater.join();
  EXPECT_EQ(w.check(100.1), Event::None);
  EXPECT_EQ(w.check(101.0), Event::Stalled);
}
