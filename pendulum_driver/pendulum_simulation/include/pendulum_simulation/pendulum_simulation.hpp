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

#ifndef PENDULUM_SIMULATION__PENDULUM_SIMULATION_HPP_
#define PENDULUM_SIMULATION__PENDULUM_SIMULATION_HPP_

#include <cmath>
#include <chrono>
#include <array>
#include <random>
#include <mutex>

#include "pendulum_driver/pendulum_driver_interface.hpp"
#include "pendulum_simulation/runge_kutta.hpp"

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum
{

/// Struct representing the dynamic/kinematic state of the pendulum.
struct PendulumState
{
  // Position of the cart in meters
  double cart_position = 0.0;
  // Velocity of the cart in meters/s
  double cart_velocity = 0.0;
  // Angular position of the pendulum in radians
  // PI is up position
  double pole_angle = PI;
  // angular velocity of the pendulum in rad/s
  double pole_velocity = 0.0;
  // total force applied to the cart in Newton
  double cart_force = 0.0;
};

/// \class This class implements a simulation for the inverted pendulum on a cart.
///
///  The simulation is based on the equations used in the
/// <a href="https://www.youtube.com/watch?v=qjhAAQexzLg"> control bootcamp series</a>
class PendulumSimulation : public PendulumDriverInterface
{
public:
  explicit PendulumSimulation(std::chrono::microseconds physics_update_period);

  /// \brief Updates the command data coming from the controller.
  /// \param[in] msg Command data message.
  virtual void update_command_data(const pendulum_msgs_v2::msg::PendulumCommand & msg);

  /// \brief Updates the disturbance force data.
  /// \param[in] msg Disturbance data message.
  virtual void update_disturbance_data(const pendulum_msgs_v2::msg::PendulumCommand & msg);

  /// \brief Updates the status data from the driver implementation.
  /// \param[in,out] msg Status data message.
  virtual void update_status_data(sensor_msgs::msg::JointState & msg);

  /// \brief Updates the driver simulation.
  virtual void update();

  /// \brief Initliaze the internal state of the driver implementation.
  virtual bool init();

  /// \brief Starts communication with the inverted pendulum or the simulation.
  virtual void start();

  /// \brief Stops communication with the inverted pendulum or the simulation.
  virtual void stop();

  /// \brief Shuts down communication with the inverted pendulum or the simulation.
  virtual void shutdown();

private:
  std::chrono::microseconds physics_update_period_;
  double dt_;
  PendulumState state_;
  bool done_ = false;
  bool is_active_ = false;

  // dimension of the space state array
  static const std::size_t state_dim = 4;
  RungeKutta<state_dim> ode_solver_;
  // state array for ODE solver
  // X[0]: cart position
  // X[1]: cart velocity
  // X[2]: pole position
  // X[3]: pole velocity
  std::array<double, state_dim> X_;

  // force applied by the controller
  // this can be considered as the force applied by the cart motor
  double controller_force_ = 0.0;
  // external disturbance force
  // this can be considered as something pushing the cart
  double disturbance_force_ = 0.0;

  // pendulum mass in kg
  double m = 1.0;
  // cart mass in Kg
  double M = 5.0;
  // pendulum length in m
  double L = 2.0;
  // cart damping coefficient
  double d = 20.0;
  // gravity, non const, we may want to change it
  double g = -9.8;
  // maximum allowed force applied to the cart
  double max_cart_force_ = 1000;

  // utils to generate random noise
  std::random_device rd;
  std::mt19937 rand_gen_;
  std::uniform_real_distribution<double> noise_gen_;

  // pointer to the derivative motion functions (ODE)
  derivativeF<state_dim> derivative_function_;
};

}  // namespace pendulum
#endif  // PENDULUM_SIMULATION__PENDULUM_SIMULATION_HPP_
