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
: cfg_(config),
  state_{0.0, 0.0, M_PI, 0.0},
  reference_{0.0, 0.0, M_PI, 0.0}
{}

void PendulumController::reset()
{
  // We reset the controller status to an up pendulum position by default
  set_state(0.0, 0.0, M_PI, 0.0);
  set_teleop(0.0, 0.0, M_PI, 0.0);
}

void PendulumController::update()
{
  set_force_command(calculate(state_, reference_));
}

void PendulumController::set_teleop(
  double cart_pos, double cart_vel,
  double pole_pos, double pole_vel)
{
  reference_[0] = cart_pos;
  reference_[1] = cart_vel;
  reference_[2] = pole_pos;
  reference_[3] = pole_vel;
}

void PendulumController::set_teleop(double cart_pos, double cart_vel)
{
  reference_[0] = cart_pos;
  reference_[1] = cart_vel;
}

void PendulumController::set_state(
  double cart_pos, double cart_vel,
  double pole_pos, double pole_vel)
{
  state_ = {cart_pos, cart_vel, pole_pos, pole_vel};
}

void PendulumController::set_force_command(double force)
{
  force_command_ = force;
}

const std::vector<double> & PendulumController::get_teleop() const
{
  return reference_;
}

const std::vector<double> & PendulumController::get_state() const
{
  return state_;
}

double PendulumController::get_force_command() const
{
  return force_command_;
}

double PendulumController::calculate(
  const std::vector<double> & state,
  const std::vector<double> & reference) const
{
  double controller_output = 0.0;
  size_t dim = state.size();
  if ((dim != reference.size()) &&
    (dim != cfg_.get_feedback_matrix().size()))
  {
    throw std::invalid_argument("wrong state size vector");
  }

  for (size_t i = 0; i < dim; i++) {
    controller_output += -cfg_.get_feedback_matrix()[i] * (state[i] - reference[i]);
  }

  return controller_output;
}
}  // namespace pendulum_controller
}  // namespace pendulum
