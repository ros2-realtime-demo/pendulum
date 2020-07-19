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

#include "pendulum_manager/pendulum_node_manager.hpp"
#include <memory>
#include <string>

namespace pendulum
{

PendulumNodeManager::PendulumNodeManager(
  const std::string & node_name,
  const std::string & controller_node_name,
  const std::string & driver_node_name)
: Node(node_name),
  node_name_(node_name)
{
  auto controller_get_state = this->create_client<lifecycle_msgs::srv::GetState>(
    controller_node_name + "/get_state");
  auto controller_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(
    controller_node_name + "/change_state");
  controller_client_ = std::make_shared<LifecycleServiceClient>(
    controller_node_name,
    controller_get_state, controller_change_state);
  auto driver_get_state = this->create_client<lifecycle_msgs::srv::GetState>(
    driver_node_name + "/get_state");
  auto driver_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(
    driver_node_name + "/change_state");
  driver_client_ = std::make_shared<LifecycleServiceClient>(
    driver_node_name,
    driver_get_state, driver_change_state);
}

}  // namespace pendulum
