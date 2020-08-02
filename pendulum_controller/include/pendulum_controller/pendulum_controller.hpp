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
/// \brief This file provides an implementation for a Full State Feedback controller.

#ifndef PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_HPP_
#define PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

#include "pendulum_msgs_v2/msg/pendulum_command.hpp"
#include "pendulum_msgs_v2/msg/pendulum_state.hpp"
#include "pendulum_controller/visibility_control.hpp"

namespace pendulum
{
namespace pendulum_controller
{
/// \class This class implements a <a href="https://en.wikipedia.org/wiki/Full_state_feedback">
///        Full State Feedback controller (FSF)</a>
///
///  The FSF uses the full internal state of the system to control the system in a closed loop.
///  This controller allows to implement both a controller designed using closed loop placement
///  techniques or a Linear Quadratic Regulator (LQR)
class PENDULUM_CONTROLLER_PUBLIC PendulumController
{
public:
  /// dimension of the controller space state array
  static constexpr size_t CONTROLLER_STATE_DIM = 4U;

  class PENDULUM_CONTROLLER_PUBLIC Config
  {
  public:
    /// \brief Constructor
    /// \param[in] feedback matrix
    Config(const std::vector<double> feedback_matrix);

    /// \brief Gets the feedback matrix
    /// \return feedback matrix array
    const std::vector<double> & get_feedback_matrix() const;

  private:
    /// feedback_matrix Feedback matrix values
    std::vector<double> feedback_matrix;
  };

  /// \brief Controller constructor
  /// \param[in] feedback_matrix Feedback matrix values
  explicit PendulumController(const Config & config);

  /// \brief Updates the setpoint data when a setpoint message arrives.
  /// \param[in] msg Setpoint data message.
  virtual void update_setpoint_data(const pendulum_msgs_v2::msg::PendulumCommand & msg);

  /// \brief Updates the sensor data when a status message arrives.
  /// \param[in] msg Setpoint data message.
  virtual void update_status_data(const sensor_msgs::msg::JointState & msg);

  /// \brief Updates the command data from the controller before publishing.
  /// \param[in,out] msg Command data message.
  virtual void update_command_data(pendulum_msgs_v2::msg::PendulumCommand & msg);

  /// \brief Resets the controller internal status and set variables to their default values.
  virtual void reset();

private:
  PENDULUM_CONTROLLER_LOCAL double calculate(
    const std::vector<double> & state,
    const std::vector<double> & reference) const;

  // Controller configuration parameters
  const Config cfg_;

  // Holds the pendulum full state variables
  std::vector<double> state_;

  // Holds the pendulum reference values.
  // Some values may be set by by the user and others are fixed by default
  std::vector<double> reference_;
};
}  // namespace pendulum_controller
}  // namespace pendulum
#endif  // PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_HPP_
