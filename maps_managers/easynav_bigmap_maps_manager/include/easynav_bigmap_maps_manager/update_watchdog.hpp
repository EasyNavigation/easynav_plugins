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

#ifndef EASYNAV_BIGMAP_MAPS_MANAGER__UPDATE_WATCHDOG_HPP_
#define EASYNAV_BIGMAP_MAPS_MANAGER__UPDATE_WATCHDOG_HPP_

#include <atomic>

namespace bigmap_maps_manager
{

/// Tells, on the wall clock, when update() has stopped being called.
///
/// EasyNav runs a maps manager when its ROS clock says it is due. If that clock never moves (a
/// node with use_sim_time and no /clock reaching it) update() is never called, and every log
/// throttled on the ROS clock is silent too: nothing says why there is no map. ran() is called by
/// update() and check() by a wall timer, from another thread: the time of the last run is atomic,
/// the rest is only touched by check().
class UpdateWatchdog
{
public:
  enum class Event {None, Stalled, Resumed};

  /// `grace_s`: how long update() may not run before it is a stall (also from start());
  /// `repeat_s`: how often a stall that goes on is told again.
  explicit UpdateWatchdog(double grace_s = 10.0, double repeat_s = 30.0)
  : grace_(grace_s), repeat_(repeat_s) {}

  /// From now on update() is expected (seconds of a monotonic clock)
  void start(double now_s)
  {
    last_ran_.store(now_s);
    started_ = true;
    stalled_ = false;
  }

  /// update() ran
  void ran(double now_s) {last_ran_.store(now_s);}

  /// What to tell now; stalled_s() is how long update() has not run.
  Event check(double now_s)
  {
    if (!started_) {return Event::None;}
    const double last = last_ran_.load();
    stalled_s_ = now_s - last;
    if (stalled_) {
      if (last > stalled_since_) {
        stalled_ = false;
        return Event::Resumed;
      }
      if (now_s - last_told_ >= repeat_) {
        last_told_ = now_s;
        return Event::Stalled;
      }
      return Event::None;
    }
    if (stalled_s_ > grace_) {
      stalled_ = true;
      stalled_since_ = last;
      last_told_ = now_s;
      return Event::Stalled;
    }
    return Event::None;
  }

  double stalled_s() const {return stalled_s_;}

private:
  double grace_, repeat_;
  std::atomic<double> last_ran_{0.0};
  bool started_{false};
  bool stalled_{false};
  double stalled_since_{0.0};   // the last run before the stall
  double last_told_{0.0};
  double stalled_s_{0.0};
};

}  // namespace bigmap_maps_manager

#endif  // EASYNAV_BIGMAP_MAPS_MANAGER__UPDATE_WATCHDOG_HPP_
