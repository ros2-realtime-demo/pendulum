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

#include "pendulum_tools/jitter_tracker.hpp"

namespace pendulum
{
void JitterTracker::update()
{
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  if (first_sample_) {
    first_sample_ = false;
  } else {
    std::chrono::duration<double, std::micro> interval_micro = now - previous_;
    double diff_from_desired_period = interval_micro.count() - period();
    add_sample(diff_from_desired_period);
  }
  previous_ = now;
}
}  // namespace pendulum
