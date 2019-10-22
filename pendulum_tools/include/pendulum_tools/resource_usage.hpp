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

#ifndef PENDULUM_TOOLS__RESOURCE_USAGE_HPP_
#define PENDULUM_TOOLS__RESOURCE_USAGE_HPP_

#include <sys/time.h>  // needed for getrusage
#include <sys/resource.h>  // needed for getrusage

#include <pendulum_msgs_v2/msg/pendulum_stats.hpp>

namespace pendulum
{

class ResourceUsage
{
public:
  ResourceUsage() = default;
  explicit ResourceUsage(int who) : who_(who)
  {}

  bool on_activate()
  {
    return update(false, true);
  }

  bool on_deactivate()
  {
    return update(true, false);
  }

  bool update(bool is_node_active, bool on_activate = false)
  {
    const auto ret = getrusage(who_, &sys_usage_);
    if (ret == 0) {
      if (on_activate) {
        minor_page_faults_on_active_start_ = sys_usage_.ru_minflt;
        major_page_faults_on_active_start_ = sys_usage_.ru_majflt;
      }
      if (is_node_active) {
        minor_pagefaults_in_active_node_ =
          sys_usage_.ru_minflt - minor_page_faults_on_active_start_;
        major_pagefaults_in_active_node_ =
          sys_usage_.ru_majflt - major_page_faults_on_active_start_;
      }
      return true;
    } else {
      return false;
    }
  }

  void update_message(pendulum_msgs_v2::msg::PendulumStats & msg)
  {
    msg.rusage_stats.max_resident_set_size = sys_usage_.ru_maxrss;
    msg.rusage_stats.total_minor_pagefaults = sys_usage_.ru_minflt;
    msg.rusage_stats.total_major_pagefaults = sys_usage_.ru_majflt;
    msg.rusage_stats.voluntary_context_switches = sys_usage_.ru_nvcsw;
    msg.rusage_stats.involuntary_context_switches = sys_usage_.ru_nivcsw;
    msg.rusage_stats.minor_pagefaults_active_node =
      minor_pagefaults_in_active_node_;
    msg.rusage_stats.major_pagefaults_active_node =
      major_pagefaults_in_active_node_;
  }

private:
  rusage sys_usage_;
  int who_ = RUSAGE_SELF;
  uint64_t minor_page_faults_on_active_start_ = 0;
  uint64_t major_page_faults_on_active_start_ = 0;
  uint64_t minor_pagefaults_in_active_node_ = 0;
  uint64_t major_pagefaults_in_active_node_ = 0;
};
}  // namespace pendulum
#endif  // PENDULUM_TOOLS__RESOURCE_USAGE_HPP_
