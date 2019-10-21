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
#include <vector>
#include <random>

#include "rttest/rttest.h"
#include "rttest/utils.h"

#include "pendulum_driver/pendulum_driver_interface.hpp"
#include "pendulum_simulation/runge_kutta.hpp"

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum
{

/// Struct representing the dynamic/kinematic state of the pendulum.
struct PendulumState
{
  double cart_position = 0.0;
  double cart_velocity = 0.0;
  double pole_angle = PI;  // Up position
  double pole_velocity = 0.0;
  double cart_force = 0.0;
};

/// \class This class implements a simulation for the inverted pendulum on a cart.
///
///  The simulation is based on the equations used in the
/// <a href="https://www.youtube.com/watch?v=qjhAAQexzLg"> control bootcamp series</a>
class PendulumSimulation : public PendulumDriverInterface
{
public:
  explicit PendulumSimulation(std::chrono::nanoseconds physics_update_period);

  /// \brief Updates the command data coming from the controller.
  /// \param[in] msg Command data message.
  virtual void update_command_data(const pendulum_msgs_v2::msg::PendulumCommand & msg);

  /// \brief Updates the disturbance force data.
  /// \param[in] msg Disturbance data message.
  virtual void update_disturbance_data(const pendulum_msgs_v2::msg::PendulumCommand & msg);

  /// \brief Updates the status data from the driver implementation.
  /// \param[in,out] msg Status data message.
  virtual void update_status_data(sensor_msgs::msg::JointState & msg);

  /// \brief Updates the internal state of the driver implementation if necessary.
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
  static void * physics_update_wrapper(void * args);
  /// \brief Set kinematic and dynamic properties of the pendulum based on state inputs
  void * physics_update();

private:
  std::chrono::nanoseconds physics_update_period_;
  double dt_;
  PendulumState state_;
  bool done_ = false;

  pthread_t physics_update_thread_;
  pthread_attr_t thread_attr_;
  timespec physics_update_timespec_;

  RungeKutta ode_solver_;
  std::vector<double> X_;  // state vector for ODE solver
  double controller_force_ = 0.0;
  double disturbance_force_ = 0.0;

  double m = 1.0;
  double M = 5.0;
  double L = 2.0;
  double g = -9.8;
  double d = 20.0;

  double max_cart_force_ = 1000;

  std::random_device rd;
  std::mt19937 rand_gen_;
  std::uniform_real_distribution<double> noise_gen_;

  derivativeF derivative_function_;
  bool is_active_ = false;
};

}  // namespace pendulum
#endif  // PENDULUM_SIMULATION__PENDULUM_SIMULATION_HPP_
