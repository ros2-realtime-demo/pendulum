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

#include <cmath>
#include <vector>

#include "pendulum2_msgs/msg/joint_state.hpp"
#include "pendulum2_msgs/msg/joint_command.hpp"
#include "pendulum2_msgs/msg/pendulum_teleop.hpp"
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
  class PENDULUM_CONTROLLER_PUBLIC Config
  {
public:
    /// \brief Constructor
    /// \param[in] feedback matrix
    explicit Config(std::vector<double> feedback_matrix);

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

  /// \brief Resets the controller internal status and set variables to their default values.
  void reset();

  /// \brief Update controller output command
  void update();

  /// \brief Updates the teleop data when a teleoperation message arrives.
  /// \param[in] cart_pos cart position in m
  /// \param[in] cart_vel cart velocity in m/s
  /// \param[in] pole_pos pole position in radians
  /// \param[in] pole_vel pole velocity in radians/s
  void set_teleop(
    double cart_pos, double cart_vel,
    double pole_pos, double pole_vel);

  /// \brief Updates the teleop data when a teleoperation message arrives.
  /// \param[in] cart_pos cart position in m
  /// \param[in] cart_vel cart velocity in m/s
  void set_teleop(double cart_pos, double cart_vel);

  /// \brief Updates the sensor data when a status message arrives.
  /// \param[in] cart_pos cart position in m
  /// \param[in] cart_vel cart velocity in m/s
  /// \param[in] pole_pos pole position in radians
  /// \param[in] pole_vel pole velocity in radians/s
  void set_state(double cart_pos, double cart_vel, double pole_pos, double pole_vel);

  /// \brief Updates the command data from the controller before publishing.
  /// \param[in] msg Command force in Newton.
  void set_force_command(double cart_force);

  /// \brief Get pendulum teleoperation data
  /// \return Teleoperation data
  const std::vector<double> & get_teleop() const;

  /// \brief Get pendulum state
  /// \return State data
  const std::vector<double> & get_state() const;

  /// \brief Get force command data
  /// \return Force command in Newton
  double get_force_command() const;

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

  // Force command to control the inverted pendulum
  double force_command_;
};
}  // namespace pendulum_controller
}  // namespace pendulum
#endif  // PENDULUM_CONTROLLER__PENDULUM_CONTROLLER_HPP_
