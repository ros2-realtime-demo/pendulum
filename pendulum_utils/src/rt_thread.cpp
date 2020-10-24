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

#include "pendulum_utils/rt_thread.hpp"

namespace pendulum
{
namespace utils
{
int set_thread_priority(pid_t pid, size_t sched_priority, int policy)
{
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = sched_priority;
  return sched_setscheduler(pid, policy, &param);
}

int set_this_thread_priority(size_t sched_priority, int policy)
{
  return set_thread_priority(getpid(), sched_priority, policy);
}

int set_thread_cpu_affinity(pid_t pid, uint32_t cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0U;
  CPU_ZERO(&set);
  while (cpu_bit_mask > 0U) {
    if ((cpu_bit_mask & 0x1U) > 0) {
      CPU_SET(cpu_cnt, &set);
    }
    cpu_bit_mask = (cpu_bit_mask >> 1U);
    cpu_cnt++;
  }
  return sched_setaffinity(pid, sizeof(set), &set);
}

int set_this_thread_cpu_affinity(uint32_t cpu_bit_mask)
{
  return set_thread_cpu_affinity(getpid(), cpu_bit_mask);
}
}  // namespace utils
}  // namespace pendulum
