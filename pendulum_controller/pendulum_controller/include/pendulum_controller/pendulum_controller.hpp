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

#ifndef PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_HPP_
#define PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_HPP_

#include "pendulum_ex_msgs/msg/joint_command_ex.hpp"
#include "pendulum_ex_msgs/msg/joint_state_ex.hpp"

namespace pendulum
{

class PendulumController
{
public:
  virtual void update_setpoint_data(const pendulum_ex_msgs::msg::JointCommandEx & msg) = 0;
  virtual void update_sensor_data(const pendulum_ex_msgs::msg::JointStateEx & msg) = 0;
  virtual void update_command_data(pendulum_ex_msgs::msg::JointCommandEx & msg) = 0;
  virtual void update() = 0;
};

}  // namespace pendulum

#endif  // PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_HPP_
