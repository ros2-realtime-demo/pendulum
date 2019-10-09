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

#ifndef PENDULUM_TELEOP__PENDULUM_MANAGER_HPP_
#define PENDULUM_TELEOP__PENDULUM_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

#include "pendulum_teleop/lifecycle_service_client.hpp"

namespace pendulum
{

class PendulumManager : public rclcpp::Node
{
public:
  PendulumManager(
    const std::string & node_name,
    const std::string & controller_node_name,
    const std::string & motor_node_name);

  void configure_controller() {controller_client_->configure();}
  void activate_controller() {controller_client_->activate();}
  void deactivate_controller() {controller_client_->deactivate();}
  void cleanup_controller() {controller_client_->cleanup();}
  void configure_motor() {motor_client_->configure();}
  void activate_motor() {motor_client_->activate();}
  void deactivate_motor() {motor_client_->deactivate();}
  void cleanup_motor() {motor_client_->cleanup();}

private:
  std::shared_ptr<pendulum::LifecycleServiceClient> controller_client_;
  std::shared_ptr<pendulum::LifecycleServiceClient> motor_client_;
  std::string node_name_;
};

}  // namespace pendulum

#endif  // PENDULUM_TELEOP__PENDULUM_MANAGER_HPP_
