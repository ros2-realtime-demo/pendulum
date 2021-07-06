// Copyright 2021 Carlos San Vicente
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

#ifndef PENDULUM_UTILS__PERIODIC_TASK_HPP_
#define PENDULUM_UTILS__PERIODIC_TASK_HPP_

#include <chrono>
#include <thread>

namespace utils
{

struct PeriodInfo
{
  std::chrono::time_point<std::chrono::steady_clock> wake_up_time;
  std::chrono::microseconds period_us;
};

void periodic_task_init(PeriodInfo & period_info, std::chrono::microseconds period)
{
  period_info.period_us = period;
  period_info.wake_up_time = std::chrono::steady_clock::now();
}

void increment_period(PeriodInfo & period_info)
{
  period_info.wake_up_time += period_info.period_us;
}

uint64_t increment_period_until_no_overruns(PeriodInfo & period_info)
{
  uint64_t overruns = 0;
  auto now = std::chrono::steady_clock::now();
  increment_period(period_info);
  while (period_info.wake_up_time < now) {
    increment_period(period_info);
    ++overruns;
  }
  return overruns;
}

void wait_rest_of_period(PeriodInfo & period_info)
{
  std::this_thread::sleep_until(period_info.wake_up_time);
}

}  // namespace utils

#endif  // PENDULUM_UTILS__PERIODIC_TASK_HPP_
