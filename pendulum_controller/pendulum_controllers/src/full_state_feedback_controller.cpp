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

#include "pendulum_controllers/full_state_feedback_controller.hpp"
#include <vector>

namespace pendulum
{

FullStateFeedbackController::FullStateFeedbackController(
  std::chrono::nanoseconds period,
  const std::vector<double> & feedback_matrix)
: publish_period_(period), feedback_matrix_(feedback_matrix),
  state_{0.0, 0.0, PI, 0.0}, reference_{0.0, 0.0, PI, 0.0} {}

void FullStateFeedbackController::update_setpoint_data(
  const pendulum_msgs_v2::msg::PendulumCommand & msg)
{
  reference_[1] = msg.cart_position;
  // reference_[2] = msg.cart_velocity;
}

void FullStateFeedbackController::update_sensor_data(
  const sensor_msgs::msg::JointState & msg)
{
  state_[0] = msg.position[0];
  state_[1] = msg.velocity[0];
  state_[2] = msg.position[1];
  state_[3] = msg.velocity[1];
}

void FullStateFeedbackController::update_command_data(
  pendulum_msgs_v2::msg::PendulumCommand & msg)
{
  msg.cart_force = calculate(state_, reference_);
}

void FullStateFeedbackController::update()
{
}

void FullStateFeedbackController::reset()
{
  state_[0] = 0.0;
  state_[1] = 0.0;
  state_[2] = PI;
  state_[3] = 0.0;
  reference_[0] = 0.0;
  reference_[1] = 0.0;
  reference_[2] = PI;
  reference_[3] = 0.0;
}

double FullStateFeedbackController::calculate(
  const std::vector<double> & state,
  const std::vector<double> & reference) const
{
  double controller_output = 0.0;
  size_t dim = state.size();
  if ( (dim != reference.size()) &&
    (dim != feedback_matrix_.size()) )
  {
    throw std::invalid_argument("wrong state size vector");
  }

  for (size_t i = 0; i < dim; i++) {
    controller_output += -feedback_matrix_[i] * (state[i] - reference[i]);
  }

  return controller_output;
}
}  // namespace pendulum
