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

#ifndef PENDULUM_UTILS__LIFECYCLE_AUTOSTART_HPP_
#define PENDULUM_UTILS__LIFECYCLE_AUTOSTART_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

namespace pendulum
{
namespace utils
{
/// \brief Transit a LifecycleNode from inactive to active state
void autostart(rclcpp_lifecycle::LifecycleNode & node)
{
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != node.configure().id()) {
    throw std::runtime_error("Could not configure " + std::string(node.get_name()));
  }
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != node.activate().id()) {
    throw std::runtime_error("Could not activate " + std::string(node.get_name()));
  }
}
}  // namespace utils
}  // namespace pendulum

#endif  // PENDULUM_UTILS__LIFECYCLE_AUTOSTART_HPP_
