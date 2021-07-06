// Copyright 2020 Carlos San Vicente
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

#ifndef PENDULUM_UTILS__PROCESS_SETTINGS_HPP_
#define PENDULUM_UTILS__PROCESS_SETTINGS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

#include "rcutils/cmdline_parser.h"

#include "pendulum_utils/memory_lock.hpp"
#include "pendulum_utils/rt_thread.hpp"


namespace utils
{
struct ProcessSettings
{
  ProcessSettings(
    bool lock_memory,
    int process_priority,
    uint32_t cpu_affinity,
    size_t lock_memory_size_mb,
    bool configure_child_threads)
  : lock_memory(lock_memory),
    process_priority(process_priority),
    cpu_affinity(cpu_affinity),
    lock_memory_size_mb(lock_memory_size_mb),
    configure_child_threads(configure_child_threads)
  {}

  /// lock and prefault memory
  bool lock_memory = false;
  /// process priority value to set
  int process_priority = 0;
  /// process cpu affinity value to set
  uint32_t cpu_affinity = 0;
  /// Memory size to lock in Megabytes
  size_t lock_memory_size_mb = 0;
  /// configure process child threads (typically DDS threads)
  bool configure_child_threads = false;
};


void lock_process_memory(size_t lock_memory_size_mb)
{
  int res = 0;
  if (lock_memory_size_mb > 0) {
    res = lock_and_prefault_dynamic(lock_memory_size_mb * 1024 * 1024);
  } else {
    res = lock_and_prefault_dynamic();
  }
  if (res != 0) {
    throw std::runtime_error("Couldn't lock  virtual memory");
  }
}

void configure_process_priority(int process_priority, uint32_t cpu_affinity)
{
  // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
  // deterministic (real-time safe) algorithm, fifo.
  if (process_priority > 0 && process_priority < 99) {
    if (set_this_thread_priority(process_priority, SCHED_FIFO)) {
      throw std::runtime_error("Couldn't set scheduling priority and policy");
    }
  }
  if (cpu_affinity > 0U) {
    if (set_this_thread_cpu_affinity(cpu_affinity)) {
      throw std::runtime_error("Couldn't set cpu affinity");
    }
  }
}

}  // namespace utils

#endif  // PENDULUM_UTILS__PROCESS_SETTINGS_HPP_
