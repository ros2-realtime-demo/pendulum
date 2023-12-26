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

using utils::PendulumState;

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
  RealtimeTeleopData::ScopedAccess<ThreadType::realtime> teleop(teleop_);
  RealtimeStateData::ScopedAccess<ThreadType::realtime> state(state_);
  const std::array<double, 4> state_array{
    state->cart_position,
    state->cart_velocity,
    state->pole_angle,
    state->pole_velocity};
  const std::array<double, 4> reference_array{
    teleop->cart_position,
    teleop->cart_velocity,
    teleop->pole_angle,
    teleop->pole_velocity};
  set_force_command(calculate(state_array, reference_array));
}

void PendulumController::set_teleop(
  double cart_position, double cart_velocity,
  double pole_angle, double pole_velocity)
{
  RealtimeTeleopData::ScopedAccess<ThreadType::nonRealtime> teleop(teleop_);
  teleop->cart_position = cart_position;
  teleop->cart_velocity = cart_velocity;
  teleop->pole_angle = pole_angle;
  teleop->pole_velocity = pole_velocity;
}

void PendulumController::set_teleop(double cart_position, double cart_velocity)
{
  RealtimeTeleopData::ScopedAccess<ThreadType::nonRealtime> teleop(teleop_);
  teleop->cart_position = cart_position;
  teleop->cart_velocity = cart_velocity;
}

void PendulumController::set_state(
  double cart_position, double cart_velocity,
  double pole_angle, double pole_velocity)
{
  RealtimeStateData::ScopedAccess<ThreadType::realtime> state(state_);
  state->pole_angle = pole_angle;
  state->pole_velocity = pole_velocity;
  state->cart_position = cart_position;
  state->cart_velocity = cart_velocity;
}

void PendulumController::set_force_command(double force)
{
  force_command_.store(force);
}

PendulumState PendulumController::get_teleop()
{
  RealtimeTeleopData::ScopedAccess<ThreadType::realtime> teleop(teleop_);
  return *teleop;
}

PendulumState PendulumController::get_state()
{
  RealtimeStateData::ScopedAccess<ThreadType::nonRealtime> joint_state(state_);
  return *joint_state;
}

double PendulumController::get_force_command() const
{
  return force_command_.load();
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
