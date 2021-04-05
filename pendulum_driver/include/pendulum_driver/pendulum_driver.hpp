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
/// \brief This file provides an implementation for a simulation of the inverted pendulum.

#ifndef PENDULUM_DRIVER__PENDULUM_DRIVER_HPP_
#define PENDULUM_DRIVER__PENDULUM_DRIVER_HPP_

#include <cmath>
#include <chrono>
#include <vector>
#include <random>

#include "pendulum2_msgs/msg/joint_state.hpp"
#include "pendulum2_msgs/msg/joint_command.hpp"

#include "pendulum_driver/runge_kutta.hpp"
#include "pendulum_driver/visibility_control.hpp"

namespace pendulum
{
namespace pendulum_driver
{
/// \class This class implements a simulation for the inverted pendulum on a cart.
///
///  The simulation is based on the equations used in the
/// <a href="https://www.youtube.com/watch?v=qjhAAQexzLg"> control bootcamp series</a>
class PENDULUM_DRIVER_PUBLIC PendulumDriver
{
public:
  // dimension of the space state array
  static constexpr std::size_t STATE_DIM = 4U;

  /// Struct representing the dynamic/kinematic state of the pendulum.
  struct PendulumState
  {
    // Position of the cart in meters
    double cart_position = 0.0;
    // Velocity of the cart in meters/s
    double cart_velocity = 0.0;
    // Angular position of the pendulum in radians
    // PI is up position
    double pole_angle = M_PI;
    // angular velocity of the pendulum in rad/s
    double pole_velocity = 0.0;
    // total force applied to the cart in Newton
    double cart_force = 0.0;
  };

  class Config
  {
public:
    /// \brief Constructor
    /// \param[in] pendulum_mass pendulum mass
    /// \param[in] cart_mass cart mass
    /// \param[in] pendulum_length pendulum length
    /// \param[in] damping_coefficient damping coefficient
    /// \param[in] gravity gravity
    /// \param[in] max_cart_force maximum cart force
    /// \param[in] physics_update_period physics simulation update period
    Config(
      double pendulum_mass,
      double cart_mass,
      double pendulum_length,
      double damping_coefficient,
      double gravity,
      double max_cart_force,
      double noise_level,
      std::chrono::microseconds physics_update_period);

    /// \brief Gets the pendulum mass
    /// \return Pendulum mass in kilograms
    double get_pendulum_mass() const;

    /// \brief Gets the cart mass
    /// \return Cart mass in kilograms
    double get_cart_mass() const;

    /// \brief Gets the pendulum length
    /// \return Pendulum length in meters
    double get_pendulum_length() const;

    /// \brief Gets the pendulum damping coefficient
    /// \return Damping coefficient
    double get_damping_coefficient() const;

    /// \brief Gets the gravity
    /// \return Gravity in m/s^2
    double get_gravity() const;

    /// \brief Gets the maximum allowed cart force
    /// \return Maximum cart force
    double get_max_cart_force() const;

    /// \brief Gets the simulated noise level
    /// \return Noise level
    double get_noise_level() const;

    /// \brief Gets the physics simulation update period
    /// \return physics simulation update period
    std::chrono::microseconds get_physics_update_period() const;

private:
    /// pendulum mass in kg
    double pendulum_mass = 1.0;
    /// cart mass in Kg
    double cart_mass = 5.0;
    /// pendulum length in m
    double pendulum_length = 2.0;
    /// cart damping coefficient
    double damping_coefficient = 20.0;
    /// gravity, non const, we may want to change it
    double gravity = -9.8;
    /// maximum allowed force applied to the cart
    double max_cart_force = 1000;
    /// Noise level used for simulation
    double noise_level = 1.0;
    /// physics simulation update period
    std::chrono::microseconds physics_update_period;
  };

  explicit PendulumDriver(const Config & config);

  /// \brief Set the pendulum state data
  /// \param[in] cart_pos cart position in m
  /// \param[in] cart_vel cart velocity in m/s
  /// \param[in] pole_pos pole position in radians
  /// \param[in] pole_vel pole velocity in radians/s
  void set_state(double cart_pos, double cart_vel, double pole_pos, double pole_vel);

  /// \brief Sets the applied force by the controller motor
  /// \param[in] force to set in Newton.
  void set_controller_cart_force(double force);

  /// \brief Sets the applied force by a disturbance
  /// \param[in] force to set in Newton.
  void set_disturbance_force(double force);

  /// \brief Get pendulum state
  /// \return State data
  const PendulumState & get_state() const;

  /// \brief Gets the applied force by the controller motor to the cart
  /// \return controller cart applied force in Newton
  double get_controller_cart_force() const;

  /// \brief Gets the applied force by a disturbance
  /// \return cart disturbance force in Newton.
  double get_disturbance_force() const;

  /// \brief Updates the driver simulation.
  void update();

  /// \brief Reset the driver simulation.
  void reset();

private:
  // Pendulum simulation configuration parameters
  const Config cfg_;
  double dt_;
  PendulumState state_;

  RungeKutta ode_solver_;
  // state array for ODE solver
  // X[0]: cart position
  // X[1]: cart velocity
  // X[2]: pole position
  // X[3]: pole velocity
  std::vector<double> X_;

  // force applied by the controller
  // this can be considered as the force applied by the cart motor
  double controller_force_ = 0.0;
  // external disturbance force
  // this can be considered as something pushing the cart
  double disturbance_force_ = 0.0;

  // utils to generate random noise
  std::random_device rd;
  std::mt19937 rand_gen_;
  std::uniform_real_distribution<double> noise_gen_;

  // pointer to the derivative motion functions (ODE)
  derivativeF derivative_function_;
};
}  // namespace pendulum_driver
}  // namespace pendulum
#endif  // PENDULUM_DRIVER__PENDULUM_DRIVER_HPP_
