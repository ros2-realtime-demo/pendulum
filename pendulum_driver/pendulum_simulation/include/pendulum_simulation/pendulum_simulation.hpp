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
  double pole_angle = PI;  // UPvirtual
  double pole_velocity = 0.0;
};

class PendulumSimulation : public PendulumDriverInterface
{
public:
  explicit PendulumSimulation(std::chrono::nanoseconds physics_update_period);

  bool init() override;
  void start() override;
  void stop() override;
  void shutdown() override;
  void update_command_data(const pendulum_msgs_v2::msg::PendulumCommand & msg) override;
  void update_disturbance_data(const pendulum_msgs_v2::msg::PendulumCommand & msg) override;
  void update_sensor_data(pendulum_msgs_v2::msg::PendulumState & msg) override;
  void update() override;

private:
  static void * physics_update_wrapper(void * args);
  // Set kinematic and dynamic properties of the pendulum based on state inputs
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

  std::random_device rd;
  std::mt19937 rand_gen_;
  std::uniform_real_distribution<double> noise_gen_;

  derivativeF derivative_function_;
  bool is_active_ = false;
};

}  // namespace pendulum
#endif  // PENDULUM_SIMULATION__PENDULUM_SIMULATION_HPP_
