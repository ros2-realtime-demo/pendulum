// Copyright 2019 Carlos San Vicente
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

#ifndef PENDULUM_TOOLS__JITTER_TRACKER_HPP_
#define PENDULUM_TOOLS__JITTER_TRACKER_HPP_

#include <cmath>
#include <limits>
#include <chrono>

#include "pendulum_tools/statistics_tracker.hpp"

namespace pendulum
{

class JitterTracker : public StatisticsTracker
{
public:
  explicit JitterTracker(std::chrono::nanoseconds period)
  : period_(period.count())
  {}
  double period() {return period_;}
  void update();

private:
  std::chrono::system_clock::time_point previous_;
  double period_;
  bool first_sample_ = true;
};

}  // namespace pendulum
#endif  // PENDULUM_TOOLS__JITTER_TRACKER_HPP_
