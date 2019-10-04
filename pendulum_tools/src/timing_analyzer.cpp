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

#include "pendulum_tools/timing_analyzer.hpp"

namespace pendulum
{
using namespace std::chrono_literals;

void TimingAnalyzer::update()
{
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  if (count_ > 0) {
    // std::chrono::duration<double> interval = now - previous_;
    std::chrono::duration<double, std::nano> interval_ns = now - previous_;
    double new_value =
      interval_ns.count() - period_;
    auto mean_diff = (new_value - mean_) / count_;
    auto new_mean = mean_ + mean_diff;
    auto d_squared_increment = (new_value - new_mean) * (new_value - mean_);
    auto new_d_squared = d_squared_ + d_squared_increment;
    mean_ = new_mean;
    d_squared_ = new_d_squared;
    if (new_value > max_) {max_ = new_value;}
    if (new_value <= min_) {min_ = new_value;}
  }
  count_++;
  previous_ = now;
}

}  // namespace pendulum
