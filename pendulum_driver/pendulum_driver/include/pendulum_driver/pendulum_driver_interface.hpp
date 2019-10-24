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

/// \file
/// \brief This file provides the inverted pendulum driver or simulation interface.

#ifndef PENDULUM_DRIVER__PENDULUM_DRIVER_INTERFACE_HPP_
#define PENDULUM_DRIVER__PENDULUM_DRIVER_INTERFACE_HPP_

#include "sensor_msgs/msg/joint_state.hpp"
#include "pendulum_msgs_v2/msg/pendulum_command.hpp"
#include "pendulum_msgs_v2/msg/pendulum_state.hpp"

namespace pendulum
{
/// \class This class creates the interface between the driver node and the driver implementation.
class PendulumDriverInterface
{
public:
  /// \brief Updates the command data coming from the controller.
  /// \param[in] msg Command data message.
  virtual void update_command_data(const pendulum_msgs_v2::msg::PendulumCommand & msg) = 0;

  /// \brief Updates the disturbance force data.
  /// \param[in] msg Disturbance data message.
  virtual void update_disturbance_data(const pendulum_msgs_v2::msg::PendulumCommand & msg) = 0;

  /// \brief Updates the status data from the driver implementation.
  /// \param[in,out] msg Status data message.
  virtual void update_status_data(sensor_msgs::msg::JointState & msg) = 0;

  /// \brief Updates the internal state of the driver implementation if necessary.
  virtual void update() = 0;

  /// \brief Initliaze the internal state of the driver implementation.
  virtual bool init() = 0;

  /// \brief Starts communication with the inverted pendulum or the simulation.
  virtual void start() = 0;

  /// \brief Stops communication with the inverted pendulum or the simulation.
  virtual void stop() = 0;

  /// \brief Shuts down communication with the inverted pendulum or the simulation.
  virtual void shutdown() = 0;
};
}  // namespace pendulum

#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_INTERFACE_HPP_
