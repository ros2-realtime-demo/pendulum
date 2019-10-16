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

#ifndef PENDULUM_DRIVER__PENDULUM_DRIVER_INTERFACE_HPP_
#define PENDULUM_DRIVER__PENDULUM_DRIVER_INTERFACE_HPP_

#include "pendulum_msgs_v2/msg/pendulum_command.hpp"
#include "pendulum_msgs_v2/msg/pendulum_state.hpp"

namespace pendulum
{

class PendulumDriverInterface
{
public:
  virtual void update_command_data(const pendulum_msgs_v2::msg::PendulumCommand & msg) = 0;
  virtual void update_sensor_data(pendulum_msgs_v2::msg::PendulumState & msg) = 0;
  virtual void update() = 0;
  virtual bool init() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void shutdown() = 0;
};

}  // namespace pendulum

#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_INTERFACE_HPP_
