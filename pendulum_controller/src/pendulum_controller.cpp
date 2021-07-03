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

#include "pendulum_controller/pendulum_controller.hpp"
#include <utility>
#include <vector>

namespace pendulum
{
namespace pendulum_controller
{
PendulumController::Config::Config(std::vector<double> feedback_matrix)
: feedback_matrix{std::move(feedback_matrix)} {}

const std::vector<double> &
PendulumController::Config::get_feedback_matrix() const
{
  return feedback_matrix;
}

PendulumController::PendulumController(const Config & config)
: cfg_(config)
{
  reset();
}

void PendulumController::reset()
{
  // We reset the controller status to an up pendulum position by default
  set_state(0.0, 0.0, M_PI, 0.0);
  set_teleop(0.0, 0.0, M_PI, 0.0);
}

void PendulumController::update()
{
  const std::array<double, 4> state{
    joint_state_.cart_position,
    joint_state_.cart_velocity,
    joint_state_.pole_angle,
    joint_state_.pole_velocity};

  const std::array<double, 4> reference{
    pendulum_teleop_.cart_position,
    pendulum_teleop_.cart_velocity,
    pendulum_teleop_.pole_angle,
    pendulum_teleop_.pole_velocity};
  set_force_command(calculate(state, reference));
}

void PendulumController::set_teleop(
  double cart_position, double cart_velocity,
  double pole_angle, double pole_velocity)
{
  pendulum_teleop_.cart_position = cart_position;
  pendulum_teleop_.cart_velocity = cart_velocity;
  pendulum_teleop_.pole_angle = pole_angle;
  pendulum_teleop_.pole_velocity = pole_velocity;
}

void PendulumController::set_teleop(double cart_position, double cart_velocity)
{
  pendulum_teleop_.cart_position = cart_position;
  pendulum_teleop_.cart_velocity = cart_velocity;
}

void PendulumController::set_state(
  double cart_position, double cart_velocity,
  double pole_angle, double pole_velocity)
{
  joint_state_.pole_angle = pole_angle;
  joint_state_.pole_velocity = pole_velocity;
  joint_state_.cart_position = cart_position;
  joint_state_.cart_velocity = cart_velocity;
}

void PendulumController::set_force_command(double force)
{
  force_command_ = force;
}

const pendulum2_msgs::msg::PendulumTeleop & PendulumController::get_teleop() const
{
  return pendulum_teleop_;
}

const pendulum2_msgs::msg::JointState & PendulumController::get_state() const
{
  return joint_state_;
}

double PendulumController::get_force_command() const
{
  return force_command_;
}

double PendulumController::calculate(
  const std::array<double, 4> & state,
  const std::array<double, 4> & reference) const
{
  double controller_output = 0.0;
  for (size_t i = 0; i < 4; i++) {
    controller_output += -cfg_.get_feedback_matrix()[i] * (state[i] - reference[i]);
  }
  return controller_output;
}
}  // namespace pendulum_controller
}  // namespace pendulum
