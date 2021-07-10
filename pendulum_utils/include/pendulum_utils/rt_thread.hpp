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

#ifndef PENDULUM_UTILS__RT_THREAD_HPP_
#define PENDULUM_UTILS__RT_THREAD_HPP_

#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>

#include <stdlib.h>
#include <limits.h>
#include <malloc.h>
#include <sys/resource.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>

#include <iostream>

namespace pendulum
{
namespace utils
{
int set_thread_priority(pid_t pid, size_t sched_priority, int policy);
int set_this_thread_priority(size_t sched_priority, int policy);
int set_thread_cpu_affinity(pid_t pid, uint32_t cpu_bit_mask);
int set_this_thread_cpu_affinity(uint32_t cpu_bit_mask);
}  // namespace utils
}  // namespace pendulum
#endif  // PENDULUM_UTILS__RT_THREAD_HPP_
