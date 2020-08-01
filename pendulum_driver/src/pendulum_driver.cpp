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

#include "pendulum_driver/pendulum_driver.hpp"
#include <array>

namespace pendulum
{
namespace pendulum_driver
{
PendulumDriver::PendulumDriver(std::chrono::microseconds physics_update_period)
: physics_update_period_(physics_update_period), done_(false),
  ode_solver_(), X_{0.0, 0.0, M_PI, 0.0},
  rand_gen_(rd()), noise_gen_(std::uniform_real_distribution<double>(-1.0, 1.0))
{
  // Calculate the controller timestep (for discrete differentiation/integration).
  dt_ = physics_update_period_.count() / (1000.0 * 1000.0);
  if (std::isnan(dt_) || dt_ == 0) {
    throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
  }

  // we use non-linear equations for the simulation
  // linearized equations couls be used if there are issues for real-time execution
  derivative_function_ = [this](const std::array<double, state_dim> & y,
      double u, size_t i) -> double {
      if (i == 0) {
        return y[1];
      } else if (i == 1) {
        double Sy = sin(y[2]);
        double Cy = cos(y[2]);
        double D = m * L * L * (M + m * (1 - Cy * Cy));
        return (1 / D) *
               (-m * m * L * L * g * Cy * Sy + m * L * L * (m * L * y[3] * y[3] * Sy - d * y[1])) +
               m * L * L * (1 / D) * u;
      } else if (i == 2) {
        return y[3];
      } else if (i == 3) {
        double Sy = sin(y[2]);
        double Cy = cos(y[2]);
        double D = m * L * L * (M + m * (1 - Cy * Cy));
        return (1 / D) * ((m + M) * m * g * L * Sy - m * L * Cy * (m * L * y[3] * y[3] * Sy -
               d * y[1])) - m * L * Cy * (1 / D) * u + noise_gen_(rand_gen_);
      } else {
        throw std::invalid_argument("received wrong index");
      }
    };
}

bool PendulumDriver::init()
{
  done_ = false;
  return true;
}

void PendulumDriver::start()
{
  // reset status, use a function!
  X_[0] = 0.0;
  X_[1] = 0.0;
  X_[2] = M_PI;
  X_[3] = 0.0;
  controller_force_ = 0.0;
  disturbance_force_ = 0.0;
  is_active_ = true;
}

void PendulumDriver::stop()
{
  is_active_ = false;
}

void PendulumDriver::shutdown()
{
  done_ = true;
}

void PendulumDriver::update_command_data(const pendulum_msgs_v2::msg::PendulumCommand & msg)
{
  if (msg.cart_force > max_cart_force_) {
    controller_force_ = max_cart_force_;
  } else if (msg.cart_force < -max_cart_force_) {
    controller_force_ = -max_cart_force_;
  } else {
    controller_force_ = msg.cart_force;
  }
}

void PendulumDriver::update_disturbance_data(const pendulum_msgs_v2::msg::PendulumCommand & msg)
{
  disturbance_force_ = msg.cart_force;
}

void PendulumDriver::update_status_data(sensor_msgs::msg::JointState & msg)
{
  msg.position[0] = state_.cart_position;
  msg.velocity[0] = state_.cart_velocity;
  msg.effort[0] = state_.cart_force;
  msg.position[1] = state_.pole_angle;
  msg.velocity[1] = state_.pole_velocity;
}

void PendulumDriver::update()
{
  double cart_force = disturbance_force_ + controller_force_;
  ode_solver_.step(derivative_function_, X_, dt_, cart_force);

  state_.cart_position = X_[0];
  state_.cart_velocity = X_[1];
  state_.cart_force = cart_force;
  state_.pole_angle = X_[2];
  state_.pole_velocity = X_[3];
}
}  // namespace pendulum_driver
}  // namespace pendulum
