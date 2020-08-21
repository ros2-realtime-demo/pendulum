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
#include <vector>

namespace pendulum
{
namespace pendulum_controller
{
PendulumController::Config::Config(std::vector<double> feedback_matrix)
: feedback_matrix{feedback_matrix} {}

const std::vector<double> &
PendulumController::Config::get_feedback_matrix() const
{
  return feedback_matrix;
}

PendulumController::PendulumController(const Config & config)
: cfg_(config),
  state_{0.0, 0.0, M_PI, 0.0},
  reference_{0.0, 0.0, M_PI, 0.0} {}

void PendulumController::update_setpoint_data(
  const pendulum_msgs_v2::msg::PendulumTeleop & msg)
{
  // We only allow to set the cart position and velocity for the moment
  reference_[0] = msg.cart_position;
  reference_[1] = msg.cart_velocity;
}

void PendulumController::update_status_data(
  const sensor_msgs::msg::JointState & msg)
{
  state_[0] = msg.position[0];
  state_[1] = msg.velocity[0];
  state_[2] = msg.position[1];
  state_[3] = msg.velocity[1];
}

void PendulumController::update_command_data(
  pendulum_msgs_v2::msg::JointCommandStamped & msg)
{
  msg.cmd.force = calculate(state_, reference_);
}

void PendulumController::reset()
{
  // We reset the controller status to an up pendulum position by default
  state_[0] = 0.0;
  state_[1] = 0.0;
  state_[2] = M_PI;
  state_[3] = 0.0;
  reference_[0] = 0.0;
  reference_[1] = 0.0;
  reference_[2] = M_PI;
  reference_[3] = 0.0;
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
