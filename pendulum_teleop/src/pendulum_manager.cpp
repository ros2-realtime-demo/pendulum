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

#include "pendulum_teleop/pendulum_manager.hpp"
#include <memory>
#include <string>

namespace pendulum
{

PendulumManager::PendulumManager(
  const std::string & node_name,
  const std::string & controller_node_name,
  const std::string & motor_node_name)
: Node(node_name),
  node_name_(node_name)
{
  auto controller_get_state = this->create_client<lifecycle_msgs::srv::GetState>(
    controller_node_name + "/get_state");
  auto controller_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(
    controller_node_name + "/change_state");
  controller_client_ = std::make_shared<LifecycleServiceClient>(controller_node_name,
      controller_get_state, controller_change_state);
  auto motor_get_state = this->create_client<lifecycle_msgs::srv::GetState>(
    motor_node_name + "/get_state");
  auto motor_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(
    motor_node_name + "/change_state");
  motor_client_ = std::make_shared<LifecycleServiceClient>(motor_node_name,
      motor_get_state, motor_change_state);
}

}  // namespace pendulum
