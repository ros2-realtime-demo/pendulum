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

#ifndef PENDULUM_CONTROLLER__PID_CONTROLLER_HPP_
#define PENDULUM_CONTROLLER__PID_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <vector>

#include "pendulum_controller/pendulum_controller.hpp"

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum
{
  // Implements a Full State Feedback controller
  // https://en.wikipedia.org/wiki/Full_state_feedback
  class FullStateFeedbackController : public PendulumController
  {
  public:
    FullStateFeedbackController(std::chrono::nanoseconds period,
      const std::vector<double> & feedback_matrix)
    : publish_period_(period), feedback_matrix_(feedback_matrix),
      state_{0.0, 0.0, PI, 0.0}, reference_{0.0, 0.0, PI, 0.0} { }

    void update_setpoint_data(const pendulum_msgs_v2::msg::PendulumCommand & msg) override
    {
      reference_[1] = msg.cart_position;
      // reference_[2] = msg.cart_velocity;
    }

    void update_sensor_data(const pendulum_msgs_v2::msg::PendulumState & msg) override
    {
      state_[0] = msg.cart_position;
      state_[1] = msg.cart_velocity;
      state_[2] = msg.pole_angle;
      state_[3] = msg.pole_velocity;
    }

    void update_command_data(pendulum_msgs_v2::msg::PendulumCommand & msg) override
    {
      msg.cart_force = calculate(state_, reference_);
    }

    void update() override
    {

    }

 private:
  double calculate(const std::vector<double> & state,
    const std::vector<double> & reference) const
    {
      double controller_output = 0.0;
      size_t dim = state.size();
      if ( (dim != reference.size()) &&
           (dim != feedback_matrix_.size()) ) {
        throw std::invalid_argument("wrong state size vector");
      }

      for (size_t i = 0; i < dim; i++) {
        controller_output += -feedback_matrix_[i]*(state[i]-reference[i]);
      }

      return controller_output;
    }

 private:
    std::chrono::nanoseconds publish_period_;
    std::vector<double> feedback_matrix_;
    double setpoint_cart_position_;
    std::vector<double> state_;
    std::vector<double> reference_;
  };
}  // namespace pendulum
#endif  // PENDULUM_CONTROLLER__PID_CONTROLLER_HPP_
